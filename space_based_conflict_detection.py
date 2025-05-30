"""
space_based_conflict_detection.py - 基于特征点的空间冲突检测模块
重构版：基于路径特征点的距离+时间冲突检测算法
解决骨干路径重合导致的冲突漏检问题
"""

import math
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Set, Any
from enum import Enum

@dataclass
class SpatialPoint:
    """空间点 - 保持兼容"""
    x: float
    y: float
    z: float = 0.0
    
    def distance_to(self, other: 'SpatialPoint') -> float:
        """计算到另一点的距离"""
        return math.sqrt(
            (self.x - other.x)**2 + 
            (self.y - other.y)**2 + 
            (self.z - other.z)**2
        )
    
    def equals(self, other: 'SpatialPoint', tolerance: float = 2.0) -> bool:
        """检查是否相等（在容差范围内）"""
        return self.distance_to(other) <= tolerance

@dataclass
class PathFeaturePoint:
    """路径特征点"""
    point: SpatialPoint
    timestamp: float  # 车辆到达此点的时间
    path_distance: float  # 从路径起点的累计距离
    point_type: str  # "access" | "backbone" | "interface"
    source_info: Dict = field(default_factory=dict)  # 来源信息

@dataclass
class VehiclePathFeatures:
    """车辆路径特征"""
    vehicle_id: str
    backbone_path_id: str
    feature_points: List[PathFeaturePoint]
    vehicle_priority: int
    total_path_length: float
    original_occupation: Any = None

@dataclass
class FeaturePointConflict:
    """特征点冲突"""
    conflict_id: str
    conflicting_vehicles: List[str]
    conflict_points: List[Tuple[PathFeaturePoint, PathFeaturePoint]]  # (点1, 点2)
    spatial_distance: float
    time_overlap: float
    detection_time: float = field(default_factory=time.time)

# 保持兼容的数据结构
@dataclass
class SpatialSegment:
    """空间段 - 兼容接口"""
    start_point: SpatialPoint
    end_point: SpatialPoint
    spatial_id: str
    tolerance: float = 2.0
    
    def overlaps_with(self, other: 'SpatialSegment') -> bool:
        return self.start_point.equals(other.start_point, self.tolerance)
    
    def contains_point(self, point: SpatialPoint) -> bool:
        return self.start_point.equals(point, self.tolerance)

@dataclass
class SpatialOccupation:
    """空间占用记录 - 兼容接口"""
    vehicle_id: str
    backbone_path_id: str
    spatial_segment: SpatialSegment
    planned_entry_time: float
    planned_exit_time: float
    vehicle_priority: int
    original_occupation: Any = None
    
    def overlaps_time_with(self, other: 'SpatialOccupation') -> bool:
        return not (self.planned_exit_time <= other.planned_entry_time or 
                   other.planned_exit_time <= self.planned_entry_time)
    
    def get_overlap_duration(self, other: 'SpatialOccupation') -> float:
        if not self.overlaps_time_with(other):
            return 0.0
        overlap_start = max(self.planned_entry_time, other.planned_entry_time)
        overlap_end = min(self.planned_exit_time, other.planned_exit_time)
        return max(0.0, overlap_end - overlap_start)

class SpatialConflictType(Enum):
    """空间冲突类型"""
    FEATURE_POINT_CONFLICT = "feature_point_conflict"  # 特征点冲突
    POSITION_OVERLAP = "position_overlap"  # 位置重叠（兼容）
    SEGMENT_OVERLAP = "segment_overlap"    # 段重叠（兼容）
    POINT_CONVERGENCE = "point_convergence" # 点汇聚（兼容）

@dataclass
class SpatialConflict:
    """空间冲突 - 兼容接口"""
    conflict_id: str
    conflict_type: SpatialConflictType
    spatial_segment_id: str
    occupations: List[SpatialOccupation]
    detection_time: float = field(default_factory=time.time)
    
    # 新增：特征点冲突信息
    feature_conflicts: List[FeaturePointConflict] = field(default_factory=list)
    
    @property
    def vehicle_ids(self) -> List[str]:
        return [occ.vehicle_id for occ in self.occupations]
    
    @property
    def backbone_path_ids(self) -> List[str]:
        return list(set(occ.backbone_path_id for occ in self.occupations))
    
    @property
    def total_overlap_duration(self) -> float:
        if self.feature_conflicts:
            return max(fc.time_overlap for fc in self.feature_conflicts)
        
        total = 0.0
        for i in range(len(self.occupations)):
            for j in range(i + 1, len(self.occupations)):
                total += self.occupations[i].get_overlap_duration(self.occupations[j])
        return total

class SpaceBasedConflictDetector:
    """基于特征点的空间冲突检测器"""
    
    def __init__(self, position_tolerance: float = 2.0):
        self.position_tolerance = position_tolerance
        self.time_tolerance = 2.0  # 时间容差（秒）
        
        # 新的数据结构：基于特征点
        self.vehicle_path_features: Dict[str, VehiclePathFeatures] = {}
        self.feature_conflicts: Dict[str, FeaturePointConflict] = {}
        
        # 兼容数据结构
        self.spatial_segments: Dict[str, SpatialSegment] = {}
        self.spatial_occupations: Dict[str, List[SpatialOccupation]] = defaultdict(list)
        self.spatial_conflicts: Dict[str, SpatialConflict] = {}
        self.position_to_spatial_id: Dict[Tuple, str] = {}
        self.vehicle_spatial_occupations: Dict[str, List[str]] = defaultdict(list)
        
        # 配置参数
        self.config = {
            'access_path_feature_spacing': 5.0,  # 接入路径特征点间距（米）
            'backbone_all_nodes': True,           # 骨干路径所有节点作为特征点
            'position_tolerance': position_tolerance,
            'time_tolerance': self.time_tolerance,
            'min_conflict_duration': 5.0,        # 最小冲突持续时间
            'vehicle_safety_radius': 2.0         # 车辆安全半径
        }
        
        # 统计信息
        self.stats = {
            'vehicles_processed': 0,
            'feature_points_extracted': 0,
            'feature_conflicts_detected': 0,
            'spatial_segments_created': 0,
            'spatial_conflicts_detected': 0,
            'position_overlaps_found': 0,
            'segment_overlaps_found': 0
        }
        
        print(f"🌐 初始化基于特征点的空间冲突检测器 (距离容差: {position_tolerance}m, 时间容差: {self.time_tolerance}s)")
    
    def extract_path_feature_points(self, vehicle_id: str, complete_path: List[Tuple],
                                  path_structure: Dict, node_timing_plan: Dict) -> List[PathFeaturePoint]:
        """提取路径特征点 - 核心算法"""
        feature_points = []
        
        if not complete_path:
            return feature_points
        
        print(f"    提取特征点: 车辆 {vehicle_id}, 路径长度 {len(complete_path)}")
        
        # 分析路径结构
        path_type = path_structure.get('type', 'unknown')
        access_path = path_structure.get('access_path', [])
        backbone_path = path_structure.get('backbone_path', [])
        
        current_time = time.time()
        current_distance = 0.0
        
        if path_type == 'interface_assisted' and access_path and backbone_path:
            # 情况1: 有接入路径 + 骨干路径
            print(f"      路径类型: 接入+骨干 (接入:{len(access_path)}, 骨干:{len(backbone_path)})")
            
            # 1. 处理接入路径 - 等间距特征点
            for i in range(len(access_path)):
                point_pos = access_path[i]
                point_time = current_time + current_distance / 1.5  # 假设速度1.5m/s
                
                # 检查是否达到间距要求
                if (i == 0 or i == len(access_path) - 1 or 
                    current_distance >= len(feature_points) * self.config['access_path_feature_spacing']):
                    
                    feature_points.append(PathFeaturePoint(
                        point=SpatialPoint(point_pos[0], point_pos[1], point_pos[2] if len(point_pos) > 2 else 0.0),
                        timestamp=point_time,
                        path_distance=current_distance,
                        point_type="access",
                        source_info={'access_index': i}
                    ))
                
                # 计算到下一点的距离
                if i < len(access_path) - 1:
                    next_pos = access_path[i + 1]
                    distance = math.sqrt(
                        (next_pos[0] - point_pos[0])**2 + 
                        (next_pos[1] - point_pos[1])**2
                    )
                    current_distance += distance
            
            # 2. 处理骨干路径 - 所有节点作为特征点
            access_time_offset = current_distance / 1.5
            for i, point_pos in enumerate(backbone_path):
                if i < len(backbone_path) - 1:
                    next_pos = backbone_path[i + 1]
                    distance = math.sqrt(
                        (next_pos[0] - point_pos[0])**2 + 
                        (next_pos[1] - point_pos[1])**2
                    )
                    segment_time = distance / 1.5
                else:
                    segment_time = 0
                
                point_time = current_time + access_time_offset + current_distance / 1.5
                
                feature_points.append(PathFeaturePoint(
                    point=SpatialPoint(point_pos[0], point_pos[1], point_pos[2] if len(point_pos) > 2 else 0.0),
                    timestamp=point_time,
                    path_distance=current_distance,
                    point_type="backbone",
                    source_info={'backbone_index': i}
                ))
                
                current_distance += distance if i < len(backbone_path) - 1 else 0
        
        elif path_type in ['backbone_only', 'optimized_backbone_only']:
            # 情况2: 纯骨干路径
            print(f"      路径类型: 纯骨干 (长度:{len(complete_path)})")
            
            for i, point_pos in enumerate(complete_path):
                point_time = current_time + current_distance / 1.5
                
                feature_points.append(PathFeaturePoint(
                    point=SpatialPoint(point_pos[0], point_pos[1], point_pos[2] if len(point_pos) > 2 else 0.0),
                    timestamp=point_time,
                    path_distance=current_distance,
                    point_type="backbone",
                    source_info={'backbone_index': i}
                ))
                
                # 计算到下一点的距离
                if i < len(complete_path) - 1:
                    next_pos = complete_path[i + 1]
                    distance = math.sqrt(
                        (next_pos[0] - point_pos[0])**2 + 
                        (next_pos[1] - point_pos[1])**2
                    )
                    current_distance += distance
        
        else:
            # 情况3: 直接路径或其他
            print(f"      路径类型: 直接路径 (长度:{len(complete_path)})")
            
            for i in range(0, len(complete_path), max(1, len(complete_path) // 10)):
                point_pos = complete_path[i]
                point_time = current_time + current_distance / 1.5
                
                feature_points.append(PathFeaturePoint(
                    point=SpatialPoint(point_pos[0], point_pos[1], point_pos[2] if len(point_pos) > 2 else 0.0),
                    timestamp=point_time,
                    path_distance=current_distance,
                    point_type="direct",
                    source_info={'path_index': i}
                ))
                
                if i < len(complete_path) - 1:
                    next_index = min(i + max(1, len(complete_path) // 10), len(complete_path) - 1)
                    next_pos = complete_path[next_index]
                    distance = math.sqrt(
                        (next_pos[0] - point_pos[0])**2 + 
                        (next_pos[1] - point_pos[1])**2
                    )
                    current_distance += distance
        
        # 根据节点时序计划调整时间（如果有）
        if node_timing_plan:
            self._adjust_feature_point_timing(feature_points, node_timing_plan, path_structure)
        
        print(f"      提取完成: {len(feature_points)} 个特征点")
        self.stats['feature_points_extracted'] += len(feature_points)
        
        return feature_points
    
    def _adjust_feature_point_timing(self, feature_points: List[PathFeaturePoint], 
                                   node_timing_plan: Dict, path_structure: Dict):
        """根据节点时序计划调整特征点时间"""
        try:
            # 找到骨干路径的特征点
            backbone_points = [fp for fp in feature_points if fp.point_type == "backbone"]
            
            if not backbone_points or not node_timing_plan:
                return
            
            # 映射节点时序到特征点
            node_indices = sorted(node_timing_plan.keys())
            
            for i, backbone_point in enumerate(backbone_points):
                if i < len(node_indices):
                    node_idx = node_indices[i]
                    if node_idx in node_timing_plan:
                        arrival_time, departure_time = node_timing_plan[node_idx]
                        backbone_point.timestamp = arrival_time
                        
        except Exception as e:
            print(f"      时序调整失败: {e}")
    
    def detect_feature_point_conflicts(self) -> List[FeaturePointConflict]:
        """检测基于特征点的冲突 - 核心算法"""
        conflicts = []
        
        vehicle_features = list(self.vehicle_path_features.values())
        
        print(f"\n🔍 特征点冲突检测: {len(vehicle_features)} 个车辆")
        
        for i in range(len(vehicle_features)):
            for j in range(i + 1, len(vehicle_features)):
                features_1 = vehicle_features[i]
                features_2 = vehicle_features[j]
                
                print(f"  检查车辆对: {features_1.vehicle_id} vs {features_2.vehicle_id}")
                
                # 检测两车的特征点冲突
                conflict_pairs = []
                
                for fp1 in features_1.feature_points:
                    for fp2 in features_2.feature_points:
                        # 计算空间距离
                        spatial_distance = fp1.point.distance_to(fp2.point)
                        
                        # 计算时间差
                        time_diff = abs(fp1.timestamp - fp2.timestamp)
                        
                        # 判断冲突条件
                        if (spatial_distance <= self.config['position_tolerance'] and 
                            time_diff <= self.config['time_tolerance']):
                            
                            conflict_pairs.append((fp1, fp2))
                            
                            print(f"    发现冲突点: 距离{spatial_distance:.1f}m, 时间差{time_diff:.1f}s")
                            print(f"      点1: ({fp1.point.x:.1f}, {fp1.point.y:.1f}) @ {fp1.timestamp:.1f}s")
                            print(f"      点2: ({fp2.point.x:.1f}, {fp2.point.y:.1f}) @ {fp2.timestamp:.1f}s")
                
                # 如果发现冲突点，创建冲突对象
                if conflict_pairs:
                    # 计算最大时间重叠
                    max_time_overlap = max(
                        self.config['time_tolerance'] - abs(fp1.timestamp - fp2.timestamp)
                        for fp1, fp2 in conflict_pairs
                    )
                    
                    # 计算最小空间距离
                    min_spatial_distance = min(
                        fp1.point.distance_to(fp2.point)
                        for fp1, fp2 in conflict_pairs
                    )
                    
                    conflict_id = f"feature_{features_1.vehicle_id}_{features_2.vehicle_id}_{int(time.time() * 1000)}"
                    
                    conflict = FeaturePointConflict(
                        conflict_id=conflict_id,
                        conflicting_vehicles=[features_1.vehicle_id, features_2.vehicle_id],
                        conflict_points=conflict_pairs,
                        spatial_distance=min_spatial_distance,
                        time_overlap=max_time_overlap
                    )
                    
                    conflicts.append(conflict)
                    self.feature_conflicts[conflict_id] = conflict
                    
                    print(f"    ✅ 创建特征点冲突: {conflict_id}")
                    print(f"       冲突点数: {len(conflict_pairs)}")
                    print(f"       最小距离: {min_spatial_distance:.1f}m")
                    print(f"       最大时间重叠: {max_time_overlap:.1f}s")
        
        self.stats['feature_conflicts_detected'] = len(conflicts)
        print(f"🎯 特征点冲突检测完成: {len(conflicts)} 个冲突")
        
        return conflicts
    
    def convert_feature_conflicts_to_spatial_conflicts(self, 
                                                     feature_conflicts: List[FeaturePointConflict]) -> List[SpatialConflict]:
        """将特征点冲突转换为空间冲突（兼容接口）"""
        spatial_conflicts = []
        
        for feature_conflict in feature_conflicts:
            # 创建兼容的空间占用记录
            occupations = []
            
            for vehicle_id in feature_conflict.conflicting_vehicles:
                if vehicle_id in self.vehicle_path_features:
                    features = self.vehicle_path_features[vehicle_id]
                    
                    # 创建虚拟空间段 - 收集属于该车辆的冲突点
                    vehicle_conflict_points = []
                    for pair in feature_conflict.conflict_points:
                        fp1, fp2 = pair
                        # 判断哪个特征点属于当前车辆（通过比较vehicle_id）
                        if hasattr(fp1, 'source_info') or hasattr(fp2, 'source_info'):
                            # 简化：假设第一个点属于第一个车辆，第二个点属于第二个车辆
                            if vehicle_id == feature_conflict.conflicting_vehicles[0]:
                                vehicle_conflict_points.append(fp1)
                            else:
                                vehicle_conflict_points.append(fp2)
                        else:
                            # 备选方案：都添加
                            vehicle_conflict_points.extend([fp1, fp2])
                    
                    if vehicle_conflict_points:
                        first_point = vehicle_conflict_points[0]
                        last_point = vehicle_conflict_points[-1]
                        
                        spatial_segment = SpatialSegment(
                            start_point=first_point.point,
                            end_point=last_point.point,
                            spatial_id=f"feature_{feature_conflict.conflict_id}_{vehicle_id}",
                            tolerance=self.config['position_tolerance']
                        )
                        
                        occupation = SpatialOccupation(
                            vehicle_id=vehicle_id,
                            backbone_path_id=features.backbone_path_id,
                            spatial_segment=spatial_segment,
                            planned_entry_time=first_point.timestamp,
                            planned_exit_time=last_point.timestamp,
                            vehicle_priority=features.vehicle_priority,
                            original_occupation=features.original_occupation
                        )
                        
                        occupations.append(occupation)
            
            # 创建空间冲突
            if len(occupations) >= 2:
                spatial_conflict = SpatialConflict(
                    conflict_id=f"spatial_{feature_conflict.conflict_id}",
                    conflict_type=SpatialConflictType.FEATURE_POINT_CONFLICT,
                    spatial_segment_id=feature_conflict.conflict_id,
                    occupations=occupations,
                    feature_conflicts=[feature_conflict]
                )
                
                spatial_conflicts.append(spatial_conflict)
                self.spatial_conflicts[spatial_conflict.conflict_id] = spatial_conflict
        
        return spatial_conflicts
    
    # ==================== 兼容接口方法 ====================
    
    def process_backbone_occupation(self, backbone_network, vehicle_id: str, 
                                   backbone_path_id: str, node_timing_plan: Dict,
                                   vehicle_priority: int, original_occupations: List = None):
        """处理骨干网络占用，转换为特征点（新实现）"""
        
        print(f"🚗 处理车辆占用: {vehicle_id} -> 路径 {backbone_path_id}")
        
        # 获取完整路径信息（尝试从原始占用获取）
        complete_path = []
        path_structure = {'type': 'backbone_only'}
        
        if original_occupations and len(original_occupations) > 0:
            # 尝试从原始占用重构路径
            first_occ = original_occupations[0]
            if hasattr(first_occ, 'predicted_trajectory') and first_occ.predicted_trajectory:
                complete_path = [(tp.position[0], tp.position[1], tp.position[2]) 
                               for tp in first_occ.predicted_trajectory]
            
        # 如果无法从占用获取，尝试从骨干网络获取
        if not complete_path and backbone_network:
            try:
                if backbone_path_id in backbone_network.bidirectional_paths:
                    path_data = backbone_network.bidirectional_paths[backbone_path_id]
                    complete_path = path_data.forward_path or []
                    print(f"  从骨干网络获取路径: {len(complete_path)} 个点")
            except Exception as e:
                print(f"  骨干网络路径获取失败: {e}")
        
        # 如果仍然没有路径，从节点时序计划推断
        if not complete_path and node_timing_plan:
            interface_spacing = 8  # 假设接口间距
            node_indices = sorted(node_timing_plan.keys())
            
            if backbone_network and backbone_path_id in backbone_network.bidirectional_paths:
                try:
                    path_data = backbone_network.bidirectional_paths[backbone_path_id]
                    backbone_path = path_data.forward_path
                    
                    for node_idx in node_indices:
                        path_idx = node_idx * interface_spacing
                        if path_idx < len(backbone_path):
                            complete_path.append(backbone_path[path_idx])
                    
                    print(f"  从节点索引重构路径: {len(complete_path)} 个点")
                except Exception as e:
                    print(f"  路径重构失败: {e}")
        
        if not complete_path:
            print(f"  ❌ 无法获取车辆 {vehicle_id} 的路径信息")
            return
        
        # 提取特征点
        feature_points = self.extract_path_feature_points(
            vehicle_id, complete_path, path_structure, node_timing_plan
        )
        
        if not feature_points:
            print(f"  ❌ 特征点提取失败")
            return
        
        # 存储车辆路径特征
        total_length = feature_points[-1].path_distance if feature_points else 0.0
        
        self.vehicle_path_features[vehicle_id] = VehiclePathFeatures(
            vehicle_id=vehicle_id,
            backbone_path_id=backbone_path_id,
            feature_points=feature_points,
            vehicle_priority=vehicle_priority,
            total_path_length=total_length,
            original_occupation=original_occupations[0] if original_occupations else None
        )
        
        # 兼容：创建空间占用记录
        segment_id = f"features_{vehicle_id}_{backbone_path_id}"
        
        if feature_points:
            start_point = feature_points[0].point
            end_point = feature_points[-1].point
            
            spatial_segment = SpatialSegment(
                start_point=start_point,
                end_point=end_point,
                spatial_id=segment_id,
                tolerance=self.config['position_tolerance']
            )
            
            occupation = SpatialOccupation(
                vehicle_id=vehicle_id,
                backbone_path_id=backbone_path_id,
                spatial_segment=spatial_segment,
                planned_entry_time=feature_points[0].timestamp,
                planned_exit_time=feature_points[-1].timestamp,
                vehicle_priority=vehicle_priority,
                original_occupation=original_occupations[0] if original_occupations else None
            )
            
            self.spatial_occupations[segment_id].append(occupation)
            self.vehicle_spatial_occupations[vehicle_id].append(segment_id)
            self.spatial_segments[segment_id] = spatial_segment
            self.stats['spatial_segments_created'] += 1
        
        self.stats['vehicles_processed'] += 1
        print(f"  ✅ 处理完成: {len(feature_points)} 个特征点")
    
    def detect_spatial_conflicts(self) -> List[SpatialConflict]:
        """检测空间冲突（主接口）"""
        print(f"\n🌐 开始基于特征点的空间冲突检测")
        
        # 1. 检测特征点冲突
        feature_conflicts = self.detect_feature_point_conflicts()
        
        # 2. 转换为空间冲突格式
        spatial_conflicts = self.convert_feature_conflicts_to_spatial_conflicts(feature_conflicts)
        
        # 3. 更新统计
        self.stats['spatial_conflicts_detected'] = len(spatial_conflicts)
        
        print(f"🎯 空间冲突检测完成: {len(spatial_conflicts)} 个空间冲突")
        
        return spatial_conflicts
    
    def clear_vehicle_spatial_occupations(self, vehicle_id: str):
        """清除车辆的空间占用记录"""
        # 清除特征点记录
        if vehicle_id in self.vehicle_path_features:
            del self.vehicle_path_features[vehicle_id]
        
        # 清除空间占用记录（兼容）
        spatial_segment_ids = self.vehicle_spatial_occupations.get(vehicle_id, [])
        
        for segment_id in spatial_segment_ids:
            if segment_id in self.spatial_occupations:
                self.spatial_occupations[segment_id] = [
                    occ for occ in self.spatial_occupations[segment_id]
                    if occ.vehicle_id != vehicle_id
                ]
                
                if not self.spatial_occupations[segment_id]:
                    del self.spatial_occupations[segment_id]
                    if segment_id in self.spatial_segments:
                        del self.spatial_segments[segment_id]
        
        if vehicle_id in self.vehicle_spatial_occupations:
            del self.vehicle_spatial_occupations[vehicle_id]
        
        # 清除相关冲突
        conflicts_to_remove = []
        for conflict_id, conflict in self.feature_conflicts.items():
            if vehicle_id in conflict.conflicting_vehicles:
                conflicts_to_remove.append(conflict_id)
        
        for conflict_id in conflicts_to_remove:
            del self.feature_conflicts[conflict_id]
        
        print(f"清除车辆 {vehicle_id} 的空间记录")
    
    def get_spatial_conflicts_for_vehicle(self, vehicle_id: str) -> List[SpatialConflict]:
        """获取特定车辆的空间冲突"""
        conflicts = []
        
        for conflict in self.spatial_conflicts.values():
            if vehicle_id in conflict.vehicle_ids:
                conflicts.append(conflict)
        
        return conflicts
    
    # ==================== 兼容方法 ====================
    
    def get_or_create_spatial_id(self, position: Tuple[float, float, float]) -> str:
        """获取或创建空间ID（兼容）"""
        normalized_pos = (
            round(position[0] / self.position_tolerance) * self.position_tolerance,
            round(position[1] / self.position_tolerance) * self.position_tolerance,
            round(position[2] / self.position_tolerance) * self.position_tolerance
        )
        
        if normalized_pos not in self.position_to_spatial_id:
            spatial_id = f"sp_{len(self.position_to_spatial_id)}"
            self.position_to_spatial_id[normalized_pos] = spatial_id
            
        return self.position_to_spatial_id[normalized_pos]
    
    def create_spatial_segment(self, start_pos: Tuple, end_pos: Tuple) -> SpatialSegment:
        """创建空间段（兼容）"""
        start_point = SpatialPoint(start_pos[0], start_pos[1], start_pos[2] if len(start_pos) > 2 else 0.0)
        end_point = SpatialPoint(end_pos[0], end_pos[1], end_pos[2] if len(end_pos) > 2 else 0.0)
        
        start_id = self.get_or_create_spatial_id((start_point.x, start_point.y, start_point.z))
        end_id = self.get_or_create_spatial_id((end_point.x, end_point.y, end_point.z))
        spatial_segment_id = f"{start_id}_to_{end_id}"
        
        segment = SpatialSegment(
            start_point=start_point,
            end_point=end_point,
            spatial_id=spatial_segment_id,
            tolerance=self.position_tolerance
        )
        
        self.spatial_segments[spatial_segment_id] = segment
        return segment
    
    def record_spatial_occupation(self, vehicle_id: str, backbone_path_id: str,
                                 start_pos: Tuple, end_pos: Tuple,
                                 entry_time: float, exit_time: float,
                                 priority: int, original_occupation=None) -> str:
        """记录空间占用（兼容）"""
        spatial_segment = self.create_spatial_segment(start_pos, end_pos)
        spatial_segment_id = spatial_segment.spatial_id
        
        spatial_occupation = SpatialOccupation(
            vehicle_id=vehicle_id,
            backbone_path_id=backbone_path_id,
            spatial_segment=spatial_segment,
            planned_entry_time=entry_time,
            planned_exit_time=exit_time,
            vehicle_priority=priority,
            original_occupation=original_occupation
        )
        
        self.spatial_occupations[spatial_segment_id].append(spatial_occupation)
        self.vehicle_spatial_occupations[vehicle_id].append(spatial_segment_id)
        
        return spatial_segment_id
    
    def extract_backbone_positions(self, backbone_network, backbone_path_id: str,
                                  node_indices: List[int], interface_spacing: int = 8) -> List[Tuple]:
        """从骨干网络提取位置信息（兼容）"""
        positions = []
        
        if not backbone_network or backbone_path_id not in backbone_network.bidirectional_paths:
            return positions
        
        path_data = backbone_network.bidirectional_paths[backbone_path_id]
        backbone_path = path_data.forward_path
        
        if not backbone_path:
            return positions
        
        for node_idx in node_indices:
            path_idx = node_idx * interface_spacing
            if path_idx < len(backbone_path):
                positions.append(backbone_path[path_idx])
        
        return positions
    
    def get_statistics(self) -> Dict:
        """获取统计信息"""
        return {
            'feature_point_stats': {
                'vehicles_processed': self.stats['vehicles_processed'],
                'feature_points_extracted': self.stats['feature_points_extracted'],
                'feature_conflicts_detected': self.stats['feature_conflicts_detected']
            },
            'spatial_stats': {
                'spatial_segments_created': self.stats['spatial_segments_created'],
                'spatial_conflicts_detected': self.stats['spatial_conflicts_detected'],
                'position_overlaps_found': self.stats['position_overlaps_found'],
                'segment_overlaps_found': self.stats['segment_overlaps_found']
            },
            'active_spatial_segments': len(self.spatial_occupations),
            'active_spatial_conflicts': len(self.spatial_conflicts),
            'spatial_occupations_per_segment': {
                segment_id: len(occupations)
                for segment_id, occupations in self.spatial_occupations.items()
                if len(occupations) > 0
            }
        }
    
    def debug_spatial_info(self):
        """调试空间信息"""
        print(f"\n🌐 基于特征点的空间冲突检测器调试信息:")
        print(f"  车辆路径特征: {len(self.vehicle_path_features)}")
        print(f"  特征点冲突: {len(self.feature_conflicts)}")
        print(f"  空间段数量: {len(self.spatial_segments)}")
        print(f"  活跃占用段: {len(self.spatial_occupations)}")
        print(f"  检测到的空间冲突: {len(self.spatial_conflicts)}")
        
        # 显示车辆特征信息
        print(f"\n🚗 车辆路径特征详情:")
        for vehicle_id, features in self.vehicle_path_features.items():
            print(f"  车辆 {vehicle_id}:")
            print(f"    骨干路径: {features.backbone_path_id}")
            print(f"    特征点数: {len(features.feature_points)}")
            print(f"    路径长度: {features.total_path_length:.1f}m")
            print(f"    优先级: {features.vehicle_priority}")
            
            # 显示特征点类型分布
            point_types = {}
            for fp in features.feature_points:
                point_types[fp.point_type] = point_types.get(fp.point_type, 0) + 1
            print(f"    特征点类型: {point_types}")
        
        # 显示特征点冲突详情
        if self.feature_conflicts:
            print(f"\n🚨 特征点冲突详情:")
            for conflict_id, conflict in self.feature_conflicts.items():
                print(f"  冲突 {conflict_id}:")
                print(f"    车辆: {conflict.conflicting_vehicles}")
                print(f"    冲突点数: {len(conflict.conflict_points)}")
                print(f"    最小距离: {conflict.spatial_distance:.1f}m")
                print(f"    时间重叠: {conflict.time_overlap:.1f}s")
        
        print(f"\n📊 统计: {self.stats}")