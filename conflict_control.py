"""
enhanced_conflict_control.py - 优化集成版：逻辑+空间双重冲突检测系统
完美集成空间冲突检测作为逻辑检测的补充，解决骨干路径重合导致的冲突遗漏问题
"""

import math
import time
import threading
from collections import defaultdict, deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Tuple, Optional, Any, Set
import heapq

# 尝试导入空间冲突检测模块
try:
    from space_based_conflict_detection import SpaceBasedConflictDetector
    SPATIAL_DETECTION_AVAILABLE = True
    print("✅ 空间冲突检测模块加载成功")
except ImportError as e:
    print(f"⚠️ 空间冲突检测模块不可用: {e}")
    SPATIAL_DETECTION_AVAILABLE = False
    SpaceBasedConflictDetector = None

class ConflictType(Enum):
    """冲突类型"""
    BACKBONE_SEGMENT_CONFLICT = "backbone_segment_conflict"
    INTERFACE_NODE_CONFLICT = "interface_node_conflict"
    PRIORITY_CONFLICT = "priority_conflict"
    TEMPORAL_DEADLOCK = "temporal_deadlock"
    SPATIAL_CONFLICT = "spatial_conflict"
    PREDICTIVE_CONFLICT = "predictive_conflict"
    FULL_PATH_CONFLICT = "full_path_conflict"

class ConflictSeverity(Enum):
    """冲突严重程度"""
    LOW = 1
    MEDIUM = 2 
    HIGH = 3
    CRITICAL = 4

class ResolutionStrategy(Enum):
    """冲突解决策略"""
    FIRST_COME_FIRST_SERVE = "first_come_first_serve"
    PRIORITY_PREEMPTION = "priority_preemption"
    TEMPORAL_ADJUSTMENT = "temporal_adjustment"
    NEGOTIATED_ADJUSTMENT = "negotiated_adjustment"
    PROGRESSIVE_DELAY = "progressive_delay"
    ALTERNATIVE_BACKBONE_PATH = "alternative_backbone_path"
    ALTERNATIVE_INTERFACE = "alternative_interface"
    PREVENTIVE_RESCHEDULING = "preventive_rescheduling"
    EMERGENCY_STOP = "emergency_stop"

@dataclass
class VehicleTrajectoryPoint:
    """车辆轨迹点"""
    timestamp: float
    position: Tuple[float, float, float]
    velocity: float
    heading: float

@dataclass
class PathOverlapRegion:
    """路径重叠区域"""
    region_id: str
    center_point: Tuple[float, float]
    radius: float
    vehicle1_arrival_time: float
    vehicle2_arrival_time: float
    vehicle1_exit_time: float
    vehicle2_exit_time: float

@dataclass
class VehicleFullPath:
    """车辆完整路径信息"""
    vehicle_id: str
    complete_path: List[Tuple[float, float, float]]
    timing_plan: List[Tuple[float, float]]
    vehicle_priority: int
    safety_radius: float

@dataclass
class SegmentRiskInfo:
    """路段风险信息"""
    gradient: float = 0.0
    width: float = 10.0
    curvature: float = 0.0
    is_main_haul_road: bool = False
    near_excavation_area: bool = False
    weather_risk: float = 1.0

@dataclass
class BackboneSegmentOccupation:
    """骨干路径段占用记录 - 强化版"""
    vehicle_id: str
    backbone_path_id: str
    segment_start_node: int
    segment_end_node: int
    planned_entry_time: float
    planned_exit_time: float
    vehicle_priority: int
    request_time: float
    
    # 新增字段
    segment_id: str = ""
    base_safety_margin: float = 10.0
    segment_risk_info: Optional[SegmentRiskInfo] = None
    predicted_trajectory: List[VehicleTrajectoryPoint] = field(default_factory=list)
    
    def __post_init__(self):
        self.segment_id = f"{self.backbone_path_id}_{self.segment_start_node}_{self.segment_end_node}"
        if self.request_time == 0:
            self.request_time = time.time()
        if self.segment_risk_info is None:
            self.segment_risk_info = SegmentRiskInfo()
    
    def calculate_dynamic_safety_margin(self, vehicle_info: Dict) -> float:
        """计算动态安全边距"""
        base_margin = self.base_safety_margin
        
        vehicle_type_factors = {
            'dump_truck': 2.0,
            'excavator': 1.5, 
            'loader': 1.8,
            'water_truck': 1.3
        }
        
        vehicle_type = vehicle_info.get('vehicle_type', 'dump_truck')
        type_factor = vehicle_type_factors.get(vehicle_type, 1.5)
        
        current_load = vehicle_info.get('current_load', 0)
        max_load = vehicle_info.get('max_load', 100)
        load_factor = 1.0 + (current_load / max(max_load, 1)) * 0.8
        
        risk_info = self.segment_risk_info
        risk_factor = 1.0
        
        if abs(risk_info.gradient) > 0.1:
            risk_factor *= 1.5 if risk_info.gradient > 0 else 2.0
        if risk_info.width < 8:
            risk_factor *= 2.0
        if risk_info.curvature > 0.05:
            risk_factor *= 1.8
        if risk_info.near_excavation_area:
            risk_factor *= 1.5
        
        risk_factor *= risk_info.weather_risk
        
        return base_margin * type_factor * load_factor * risk_factor
    
    def calculate_deceleration_time(self, vehicle_info: Dict) -> float:
        """计算减速时间"""
        current_speed = vehicle_info.get('current_speed', 1.5)
        vehicle_type = vehicle_info.get('vehicle_type', 'dump_truck')
        
        braking_performance = {
            'dump_truck': 0.8,
            'excavator': 1.2,
            'loader': 1.0
        }
        
        decel_rate = braking_performance.get(vehicle_type, 1.0)
        return current_speed / decel_rate
    
    def overlaps_with_enhanced(self, other: 'BackboneSegmentOccupation', 
                              vehicle_info_dict: Dict) -> bool:
        """强化时间重叠检测"""
        self_vehicle_info = vehicle_info_dict.get(self.vehicle_id, {})
        other_vehicle_info = vehicle_info_dict.get(other.vehicle_id, {})
        
        self_margin = self.calculate_dynamic_safety_margin(self_vehicle_info)
        other_margin = other.calculate_dynamic_safety_margin(other_vehicle_info)
        
        self_decel_time = self.calculate_deceleration_time(self_vehicle_info)
        other_decel_time = other.calculate_deceleration_time(other_vehicle_info)
        
        self_start = self.planned_entry_time - self_decel_time
        self_end = self.planned_exit_time + self_margin
        other_start = other.planned_entry_time - other_decel_time
        other_end = other.planned_exit_time + other_margin
        
        overlap = not (self_end <= other_start or other_end <= self_start)
        
        if overlap:
            print(f"    ⚠️ 强化重叠检测: {self.vehicle_id}({self_start:.1f}-{self_end:.1f}) vs {other.vehicle_id}({other_start:.1f}-{other_end:.1f})")
        
        return overlap
    
    def check_spatial_conflict(self, other: 'BackboneSegmentOccupation',
                              vehicle_info_dict: Dict) -> bool:
        """检查空间冲突"""
        if not self.predicted_trajectory or not other.predicted_trajectory:
            return False
        
        self_vehicle_info = vehicle_info_dict.get(self.vehicle_id, {})
        other_vehicle_info = vehicle_info_dict.get(other.vehicle_id, {})
        
        self_length = self_vehicle_info.get('length', 6.0)
        other_length = other_vehicle_info.get('length', 6.0)
        safe_distance = (self_length + other_length) / 2 + 15.0
        
        for t1_point in self.predicted_trajectory:
            for t2_point in other.predicted_trajectory:
                time_diff = abs(t1_point.timestamp - t2_point.timestamp)
                if time_diff < 10.0:
                    pos1 = t1_point.position
                    pos2 = t2_point.position
                    distance = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
                    
                    if distance < safe_distance:
                        return True
        
        return False
    
    # 向后兼容方法
    def overlaps_with(self, other):
        """向后兼容"""
        return self.overlaps_with_enhanced(other, {})
    
    def get_overlap_duration(self, other):
        """向后兼容"""
        if not self.overlaps_with(other):
            return 0.0
        
        overlap_start = max(self.planned_entry_time, other.planned_entry_time)
        overlap_end = min(self.planned_exit_time, other.planned_exit_time)
        return max(0.0, overlap_end - overlap_start)

@dataclass
class BackboneConflict:
    """骨干路径冲突 - 强化版"""
    conflict_id: str
    conflict_type: ConflictType
    severity: ConflictSeverity
    backbone_path_id: str
    segment_id: str
    conflicting_vehicles: List[str]
    occupations: List[BackboneSegmentOccupation]
    
    conflict_time_window: Tuple[float, float]
    overlap_duration: float
    
    suggested_resolution: ResolutionStrategy
    priority_order: List[str]
    
    # 新增字段
    spatial_conflict: bool = False
    predictive_conflict: bool = False
    negotiation_options: Dict = field(default_factory=dict)
    
    detection_time: float = field(default_factory=time.time)
    is_resolved: bool = False
    resolution_time: Optional[float] = None
    resolution_attempts: int = 0
    
    def get_priority_vehicle(self) -> str:
        """获取优先车辆"""
        return self.priority_order[0] if self.priority_order else ""
    
    def get_negotiation_candidates(self) -> List[str]:
        """获取可协商车辆"""
        if len(self.conflicting_vehicles) != 2:
            return []
        
        if len(self.priority_order) >= 2:
            v1, v2 = self.priority_order[0], self.priority_order[1]
            if len(set(occ.vehicle_priority for occ in self.occupations)) <= 1:
                return [v1, v2]
        
        return []

class EnhancedBackboneConflictDetector:
    """完整优化版骨干路径冲突检测器 - 逻辑+空间双重检测"""
    
    def __init__(self, backbone_network=None):
        self.backbone_network = backbone_network
        
        # 核心数据结构
        self.segment_occupations: Dict[str, List[BackboneSegmentOccupation]] = defaultdict(list)
        self.vehicle_occupations: Dict[str, List[str]] = defaultdict(list)
        self.vehicle_info_cache: Dict[str, Dict] = {}
        
        # 轨迹预测缓存
        self.trajectory_cache: Dict[str, List[VehicleTrajectoryPoint]] = {}
        
        # 路径缓存
        self.full_path_cache: Dict[str, VehicleFullPath] = {}
        
        # 冲突记录
        self.detected_conflicts: Dict[str, BackboneConflict] = {}
        self.resolved_conflicts: List[BackboneConflict] = []
        
        # 空间冲突检测器（优化集成）
        self.spatial_detector = None
        if SPATIAL_DETECTION_AVAILABLE:
            try:
                self.spatial_detector = SpaceBasedConflictDetector(position_tolerance=3.0)
                print("✅ 空间冲突检测器已加载")
            except Exception as e:
                print(f"⚠️ 空间冲突检测器初始化失败: {e}")
        
        # 配置参数
        self.config = {
            'base_safety_margin': 10.0,
            'prediction_horizon': 300.0,
            'spatial_check_enabled': True,
            'trajectory_update_interval': 5.0,
            'conflict_detection_interval': 2.0,
            'cleanup_interval': 300.0,
            'max_conflicts_history': 100,
            'negotiation_threshold': 0.8,
            'enable_spatial_detection': SPATIAL_DETECTION_AVAILABLE,  # 自动根据可用性启用
            'spatial_supplement_only': True,  # 只作为补充检测
        }
        
        # 统计信息
        self.stats = {
            'total_occupations_recorded': 0,
            'total_conflicts_detected': 0,
            'logical_conflicts_detected': 0,
            'spatial_conflicts_detected': 0,
            'spatial_supplement_conflicts': 0,  # 补充检测发现的冲突
            'predictive_conflicts_detected': 0,
            'full_path_conflicts_detected': 0,
            'conflicts_by_type': defaultdict(int),
            'conflicts_by_severity': defaultdict(int),
            'negotiated_resolutions': 0,
            'preventive_adjustments': 0,
        }
        
        self.lock = threading.RLock()
        
        print("初始化完整优化版冲突检测器（逻辑+空间双重检测）")
    
    def update_vehicle_info(self, vehicle_id: str, vehicle_info: Dict):
        """更新车辆信息缓存"""
        self.vehicle_info_cache[vehicle_id] = vehicle_info.copy()
    
    def update_vehicle_full_path(self, vehicle_id: str, complete_path: List[Tuple], 
                               timing_plan: List[Tuple], vehicle_priority: int = 2):
        """更新车辆完整路径信息"""
        if not complete_path or not timing_plan:
            return
        
        vehicle_info = self.vehicle_info_cache.get(vehicle_id, {})
        
        length = vehicle_info.get('length', 6.0)
        width = vehicle_info.get('width', 3.0)
        safety_radius = max(length, width) / 2 + 5.0
        
        self.full_path_cache[vehicle_id] = VehicleFullPath(
            vehicle_id=vehicle_id,
            complete_path=complete_path,
            timing_plan=timing_plan,
            vehicle_priority=vehicle_priority,
            safety_radius=safety_radius
        )
    
    def predict_vehicle_trajectory(self, vehicle_id: str, 
                                 time_horizon: float = 300.0) -> List[VehicleTrajectoryPoint]:
        """预测车辆轨迹"""
        if vehicle_id not in self.vehicle_info_cache:
            return []
        
        vehicle_info = self.vehicle_info_cache[vehicle_id]
        current_pos = vehicle_info.get('position', (0, 0, 0))
        current_speed = vehicle_info.get('current_speed', 1.5)
        current_heading = current_pos[2] if len(current_pos) > 2 else 0.0
        
        trajectory = []
        current_time = time.time()
        
        for t in range(0, int(time_horizon), 10):
            timestamp = current_time + t
            
            distance = current_speed * t
            x = current_pos[0] + distance * math.cos(current_heading)
            y = current_pos[1] + distance * math.sin(current_heading)
            
            trajectory.append(VehicleTrajectoryPoint(
                timestamp=timestamp,
                position=(x, y, current_heading),
                velocity=current_speed,
                heading=current_heading
            ))
        
        return trajectory
    
    def record_vehicle_backbone_occupation(self, vehicle_id: str, backbone_path_id: str,
                                         node_timing_plan: Dict[int, Tuple[float, float]],
                                         vehicle_priority: int = 2,
                                         segment_risk_info: Dict = None) -> bool:
        """记录车辆骨干路径占用 - 强化版"""
        with self.lock:
            try:
                print(f"\n🔧 [强化] 记录车辆占用: {vehicle_id}")
                
                if not node_timing_plan:
                    return False
                
                self._clear_vehicle_occupations(vehicle_id)
                
                trajectory = self.predict_vehicle_trajectory(vehicle_id)
                
                node_indices = sorted(node_timing_plan.keys())
                request_time = time.time()
                
                for i in range(len(node_indices) - 1):
                    start_node = node_indices[i]
                    end_node = node_indices[i + 1]
                    
                    start_times = node_timing_plan[start_node]
                    end_times = node_timing_plan[end_node]
                    
                    segment_entry_time = start_times[1]
                    segment_exit_time = end_times[0]
                    
                    if segment_exit_time <= segment_entry_time:
                        segment_exit_time = segment_entry_time + 15.0
                    
                    risk_info = SegmentRiskInfo(**(segment_risk_info or {}))
                    
                    occupation = BackboneSegmentOccupation(
                        vehicle_id=vehicle_id,
                        backbone_path_id=backbone_path_id,
                        segment_start_node=start_node,
                        segment_end_node=end_node,
                        planned_entry_time=segment_entry_time,
                        planned_exit_time=segment_exit_time,
                        vehicle_priority=vehicle_priority,
                        request_time=request_time,
                        base_safety_margin=self.config['base_safety_margin'],
                        segment_risk_info=risk_info,
                        predicted_trajectory=trajectory
                    )
                    
                    segment_id = occupation.segment_id
                    self.segment_occupations[segment_id].append(occupation)
                    self.vehicle_occupations[vehicle_id].append(segment_id)
                
                self.stats['total_occupations_recorded'] += 1
                return True
                
            except Exception as e:
                print(f"❌ 记录占用失败: {e}")
                return False
    
    def detect_backbone_conflicts(self) -> List[BackboneConflict]:
        """检测所有冲突 - 完整优化版（逻辑+空间双重检测）"""
        with self.lock:
            conflicts = []
            
            print(f"\n🔍 [双重检测] 冲突检测开始")
            
            # 1. 逻辑冲突检测
            logical_conflicts = self._detect_logical_conflicts()
            conflicts.extend(logical_conflicts)
            self.stats['logical_conflicts_detected'] = len(logical_conflicts)
            
            print(f"🧠 逻辑检测: {len(logical_conflicts)} 个冲突")
            
            # 2. 空间冲突补充检测（只检测逻辑遗漏的）
            if self.config.get('enable_spatial_detection') and self.spatial_detector:
                print(f"🌐 开始补充空间冲突检测...")
                spatial_supplements = self._detect_supplementary_spatial_conflicts(logical_conflicts)
                if spatial_supplements:
                    conflicts.extend(spatial_supplements)
                    self.stats['spatial_supplement_conflicts'] = len(spatial_supplements)
                    print(f"🌐 补充检测: {len(spatial_supplements)} 个额外空间冲突")
                else:
                    print(f"🌐 补充检测: 无额外空间冲突")
            
            # 更新统计
            for conflict in conflicts:
                if conflict.conflict_id not in self.detected_conflicts:
                    self.detected_conflicts[conflict.conflict_id] = conflict
                    self.stats['conflicts_by_type'][conflict.conflict_type] += 1
                    self.stats['conflicts_by_severity'][conflict.severity] += 1
                    
                    if conflict.conflict_type == ConflictType.SPATIAL_CONFLICT:
                        self.stats['spatial_conflicts_detected'] += 1
                    elif conflict.conflict_type == ConflictType.PREDICTIVE_CONFLICT:
                        self.stats['predictive_conflicts_detected'] += 1
            
            self.stats['total_conflicts_detected'] = len(conflicts)
            
            print(f"🔍 [双重检测] 检测完成: {len(conflicts)} 个总冲突")
            print(f"    逻辑冲突: {self.stats['logical_conflicts_detected']}")
            print(f"    补充空间冲突: {self.stats['spatial_supplement_conflicts']}")
            
            return conflicts
    
    def _detect_logical_conflicts(self) -> List[BackboneConflict]:
        """检测逻辑冲突（原有逻辑）"""
        conflicts = []
        
        # 1. 当前冲突
        current_conflicts = self._detect_current_conflicts()
        conflicts.extend(current_conflicts)
        
        # 2. 预测性冲突
        if self.config['prediction_horizon'] > 0:
            predictive_conflicts = self._detect_predictive_conflicts()
            conflicts.extend(predictive_conflicts)
        
        # 3. 全路径冲突
        full_path_conflicts = self._detect_full_path_conflicts()
        conflicts.extend(full_path_conflicts)
        
        return conflicts
    
    def _detect_supplementary_spatial_conflicts(self, existing_conflicts: List[BackboneConflict]) -> List[BackboneConflict]:
        """补充空间冲突检测 - 检测逻辑层面漏掉的空间冲突"""
        if not self.spatial_detector:
            return []
        
        supplementary_conflicts = []
        
        try:
            # 获取已检测到冲突的车辆对
            existing_vehicle_pairs = set()
            for conflict in existing_conflicts:
                vehicles = conflict.conflicting_vehicles
                if len(vehicles) >= 2:
                    for i in range(len(vehicles)):
                        for j in range(i + 1, len(vehicles)):
                            pair = tuple(sorted([vehicles[i], vehicles[j]]))
                            existing_vehicle_pairs.add(pair)
            
            # 更新空间占用数据
            self._update_spatial_occupations_from_backbone()
            
            # 检测空间冲突
            spatial_conflicts = self.spatial_detector.detect_spatial_conflicts()
            
            # 筛选出新的冲突（逻辑检测未发现的）
            for spatial_conflict in spatial_conflicts:
                vehicles = spatial_conflict.vehicle_ids
                if len(vehicles) >= 2:
                    # 检查是否是新的车辆冲突对
                    has_new_pair = False
                    for i in range(len(vehicles)):
                        for j in range(i + 1, len(vehicles)):
                            pair = tuple(sorted([vehicles[i], vehicles[j]]))
                            if pair not in existing_vehicle_pairs:
                                has_new_pair = True
                                break
                        if has_new_pair:
                            break
                    
                    if has_new_pair:
                        # 转换为BackboneConflict格式
                        backbone_conflict = self._convert_spatial_to_backbone_conflict(spatial_conflict)
                        if backbone_conflict:
                            supplementary_conflicts.append(backbone_conflict)
                            print(f"  🌐 发现补充空间冲突: 车辆 {vehicles} (逻辑检测未发现)")
        
        except Exception as e:
            print(f"  ❌ 补充空间冲突检测异常: {e}")
        
        return supplementary_conflicts
    
    def _update_spatial_occupations_from_backbone(self):
        """从骨干路径占用更新空间占用数据"""
        if not self.spatial_detector:
            return
        
        # 清理旧的空间占用数据
        self.spatial_detector.spatial_occupations.clear()
        self.spatial_detector.vehicle_spatial_occupations.clear()
        
        # 从骨干路径占用记录生成空间占用
        for segment_id, occupations in self.segment_occupations.items():
            for occupation in occupations:
                try:
                    # 解析segment_id获取路径信息
                    parts = segment_id.split('_')
                    if len(parts) >= 3:
                        backbone_path_id = '_'.join(parts[:-2])
                        start_node = int(parts[-2])
                        end_node = int(parts[-1])
                        
                        # 构建简化的节点时序计划用于空间占用处理
                        node_timing_plan = {
                            start_node: (occupation.planned_entry_time, occupation.planned_entry_time + 2.0),
                            end_node: (occupation.planned_exit_time - 2.0, occupation.planned_exit_time)
                        }
                        
                        # 处理为空间占用
                        self.spatial_detector.process_backbone_occupation(
                            self.backbone_network,
                            occupation.vehicle_id,
                            backbone_path_id,
                            node_timing_plan,
                            occupation.vehicle_priority,
                            [occupation]
                        )
                
                except Exception as e:
                    # 静默处理单个占用的错误，继续处理其他
                    continue
    
    def _convert_spatial_to_backbone_conflict(self, spatial_conflict) -> Optional[BackboneConflict]:
        """将空间冲突转换为骨干冲突格式"""
        try:
            vehicle_ids = spatial_conflict.vehicle_ids
            if len(vehicle_ids) < 2:
                return None
            
            # 获取车辆的占用信息来确定优先级顺序
            priority_order = []
            occupations = []
            
            for vehicle_id in vehicle_ids:
                # 查找该车辆的占用记录
                for segment_id, segment_occupations in self.segment_occupations.items():
                    for occ in segment_occupations:
                        if occ.vehicle_id == vehicle_id:
                            occupations.append(occ)
                            break
            
            # 按优先级和请求时间排序
            occupations.sort(key=lambda occ: (-occ.vehicle_priority, occ.request_time))
            priority_order = [occ.vehicle_id for occ in occupations]
            
            # 确定冲突严重程度
            severity = ConflictSeverity.MEDIUM
            if len(vehicle_ids) > 2:
                severity = ConflictSeverity.HIGH
            if spatial_conflict.total_overlap_duration > 60:
                severity = ConflictSeverity.HIGH
            
            # 建议解决策略：空间冲突优先使用时间调整
            suggested_resolution = ResolutionStrategy.TEMPORAL_ADJUSTMENT
            if len(vehicle_ids) > 2:
                suggested_resolution = ResolutionStrategy.ALTERNATIVE_BACKBONE_PATH
            
            # 计算冲突时间窗口
            min_time = min(occ.planned_entry_time for occ in occupations) if occupations else time.time()
            max_time = max(occ.planned_exit_time for occ in occupations) if occupations else time.time() + 60
            
            conflict_id = f"spatial_supplement_{spatial_conflict.spatial_segment_id}_{int(time.time() * 1000)}"
            
            backbone_conflict = BackboneConflict(
                conflict_id=conflict_id,
                conflict_type=ConflictType.SPATIAL_CONFLICT,
                severity=severity,
                backbone_path_id="spatial_multiple",
                segment_id=spatial_conflict.spatial_segment_id,
                conflicting_vehicles=vehicle_ids,
                occupations=occupations,
                conflict_time_window=(min_time, max_time),
                overlap_duration=spatial_conflict.total_overlap_duration,
                suggested_resolution=suggested_resolution,
                priority_order=priority_order,
                spatial_conflict=True
            )
            
            return backbone_conflict
        
        except Exception as e:
            print(f"    转换空间冲突失败: {e}")
            return None
    
    def _detect_current_conflicts(self) -> List[BackboneConflict]:
        """检测当前冲突"""
        conflicts = []
        
        for segment_id, occupations in self.segment_occupations.items():
            if len(occupations) < 2:
                continue
            
            segment_conflicts = self._detect_segment_conflicts_enhanced(segment_id, occupations)
            conflicts.extend(segment_conflicts)
        
        return conflicts
    
    def _detect_predictive_conflicts(self) -> List[BackboneConflict]:
        """检测预测性冲突"""
        current_time = time.time()
        prediction_end = current_time + self.config['prediction_horizon']
        
        predictive_conflicts = []
        
        for segment_id in self.segment_occupations.keys():
            future_occupations = []
            
            for occ in self.segment_occupations[segment_id]:
                if occ.planned_exit_time > current_time:
                    future_occupations.append(occ)
            
            if len(future_occupations) >= 2:
                conflicts = self._detect_segment_conflicts_enhanced(segment_id, future_occupations)
                for conflict in conflicts:
                    conflict.conflict_type = ConflictType.PREDICTIVE_CONFLICT
                    conflict.predictive_conflict = True
                    predictive_conflicts.append(conflict)
        
        return predictive_conflicts
    
    def _detect_full_path_conflicts(self) -> List[BackboneConflict]:
        """检测全路径冲突"""
        conflicts = []
        vehicle_paths = list(self.full_path_cache.values())
        
        if len(vehicle_paths) < 2:
            return conflicts
        
        print(f"      检测全路径冲突: {len(vehicle_paths)} 个车辆路径")
        
        for i in range(len(vehicle_paths)):
            for j in range(i + 1, len(vehicle_paths)):
                path1, path2 = vehicle_paths[i], vehicle_paths[j]
                
                overlap_regions = self._find_path_overlaps(path1, path2)
                
                if overlap_regions:
                    time_conflicts = self._analyze_overlap_timing(overlap_regions)
                    
                    if time_conflicts:
                        conflict = self._create_full_path_conflict(path1, path2, overlap_regions, time_conflicts)
                        if conflict:
                            conflicts.append(conflict)
                            self.stats['full_path_conflicts_detected'] += 1
        
        return conflicts
    
    def _find_path_overlaps(self, path1: VehicleFullPath, path2: VehicleFullPath) -> List[PathOverlapRegion]:
        """查找两个路径的重叠区域"""
        overlap_regions = []
        
        for i in range(len(path1.complete_path) - 1):
            seg1_start = path1.complete_path[i]
            seg1_end = path1.complete_path[i + 1]
            
            for j in range(len(path2.complete_path) - 1):
                seg2_start = path2.complete_path[j]
                seg2_end = path2.complete_path[j + 1]
                
                closest_distance = self._calculate_segment_distance(
                    (seg1_start, seg1_end), (seg2_start, seg2_end)
                )
                
                max_safe_distance = path1.safety_radius + path2.safety_radius
                
                if closest_distance < max_safe_distance:
                    overlap_center = self._find_segments_intersection_point(
                        (seg1_start, seg1_end), (seg2_start, seg2_end)
                    )
                    
                    if overlap_center:
                        path1_time = self._calculate_arrival_time(path1, i, overlap_center)
                        path2_time = self._calculate_arrival_time(path2, j, overlap_center)
                        
                        region = PathOverlapRegion(
                            region_id=f"overlap_{len(overlap_regions)}",
                            center_point=(overlap_center[0], overlap_center[1]),
                            radius=max_safe_distance,
                            vehicle1_arrival_time=path1_time[0],
                            vehicle2_arrival_time=path2_time[0],
                            vehicle1_exit_time=path1_time[1],
                            vehicle2_exit_time=path2_time[1]
                        )
                        
                        overlap_regions.append(region)
        
        return overlap_regions
    
    def _calculate_segment_distance(self, seg1: Tuple, seg2: Tuple) -> float:
        """计算两个线段之间的最短距离"""
        def point_to_segment_distance(point, seg_start, seg_end):
            A, B = seg_start[:2], seg_end[:2]
            P = point[:2]
            
            AB = (B[0] - A[0], B[1] - A[1])
            AP = (P[0] - A[0], P[1] - A[1])
            
            AB_squared = AB[0]**2 + AB[1]**2
            if AB_squared == 0:
                return math.sqrt(AP[0]**2 + AP[1]**2)
            
            t = max(0, min(1, (AP[0]*AB[0] + AP[1]*AB[1]) / AB_squared))
            projection = (A[0] + t*AB[0], A[1] + t*AB[1])
            
            return math.sqrt((P[0] - projection[0])**2 + (P[1] - projection[1])**2)
        
        seg1_start, seg1_end = seg1
        seg2_start, seg2_end = seg2
        
        distances = [
            point_to_segment_distance(seg1_start, seg2_start, seg2_end),
            point_to_segment_distance(seg1_end, seg2_start, seg2_end),
            point_to_segment_distance(seg2_start, seg1_start, seg1_end),
            point_to_segment_distance(seg2_end, seg1_start, seg1_end)
        ]
        
        return min(distances)
    
    def _find_segments_intersection_point(self, seg1: Tuple, seg2: Tuple) -> Optional[Tuple]:
        """查找两个线段的交点或最近点"""
        seg1_start, seg1_end = seg1
        seg2_start, seg2_end = seg2
        
        mid1 = ((seg1_start[0] + seg1_end[0])/2, (seg1_start[1] + seg1_end[1])/2)
        mid2 = ((seg2_start[0] + seg2_end[0])/2, (seg2_start[1] + seg2_end[1])/2)
        
        intersection = ((mid1[0] + mid2[0])/2, (mid1[1] + mid2[1])/2, 0)
        return intersection
    
    def _calculate_arrival_time(self, vehicle_path: VehicleFullPath, 
                              segment_index: int, target_point: Tuple) -> Tuple[float, float]:
        """计算车辆到达目标点的时间"""
        if segment_index >= len(vehicle_path.timing_plan):
            return (0, 0)
        
        base_time = vehicle_path.timing_plan[segment_index][0]
        
        transit_time = vehicle_path.safety_radius * 2 / 1.5
        
        arrival_time = base_time
        exit_time = arrival_time + transit_time
        
        return (arrival_time, exit_time)
    
    def _analyze_overlap_timing(self, overlap_regions: List[PathOverlapRegion]) -> List[PathOverlapRegion]:
        """分析重叠区域的时间冲突"""
        time_conflicts = []
        
        for region in overlap_regions:
            v1_start, v1_end = region.vehicle1_arrival_time, region.vehicle1_exit_time
            v2_start, v2_end = region.vehicle2_arrival_time, region.vehicle2_exit_time
            
            if not (v1_end <= v2_start or v2_end <= v1_start):
                time_conflicts.append(region)
                print(f"        发现路径冲突区域: 车辆时间 {v1_start:.1f}-{v1_end:.1f} vs {v2_start:.1f}-{v2_end:.1f}")
        
        return time_conflicts
    
    def _create_full_path_conflict(self, path1: VehicleFullPath, path2: VehicleFullPath,
                                 overlap_regions: List[PathOverlapRegion],
                                 time_conflicts: List[PathOverlapRegion]) -> Optional[BackboneConflict]:
        """创建全路径冲突对象"""
        if not time_conflicts:
            return None
        
        vehicles = [path1.vehicle_id, path2.vehicle_id]
        
        total_overlap = sum(
            min(region.vehicle1_exit_time, region.vehicle2_exit_time) - 
            max(region.vehicle1_arrival_time, region.vehicle2_arrival_time)
            for region in time_conflicts
        )
        
        min_time = min(region.vehicle1_arrival_time for region in time_conflicts)
        max_time = max(region.vehicle1_exit_time for region in time_conflicts)
        min_time = min(min_time, min(region.vehicle2_arrival_time for region in time_conflicts))
        max_time = max(max_time, max(region.vehicle2_exit_time for region in time_conflicts))
        
        if path1.vehicle_priority != path2.vehicle_priority:
            priority_order = sorted(vehicles, 
                key=lambda v: path1.vehicle_priority if v == path1.vehicle_id else path2.vehicle_priority,
                reverse=True)
            suggested_strategy = ResolutionStrategy.PRIORITY_PREEMPTION
            severity = ConflictSeverity.HIGH
        else:
            priority_order = vehicles
            suggested_strategy = ResolutionStrategy.NEGOTIATED_ADJUSTMENT
            severity = ConflictSeverity.MEDIUM if len(time_conflicts) > 2 else ConflictSeverity.LOW
        
        conflict_id = f"fullpath_{path1.vehicle_id}_{path2.vehicle_id}_{int(time.time())}"
        
        conflict = BackboneConflict(
            conflict_id=conflict_id,
            conflict_type=ConflictType.FULL_PATH_CONFLICT,
            severity=severity,
            backbone_path_id="full_path",
            segment_id="multiple_regions",
            conflicting_vehicles=vehicles,
            occupations=[],
            conflict_time_window=(min_time, max_time),
            overlap_duration=total_overlap,
            suggested_resolution=suggested_strategy,
            priority_order=priority_order
        )
        
        conflict.negotiation_options['overlap_regions'] = [
            {
                'center': region.center_point,
                'radius': region.radius,
                'v1_time': (region.vehicle1_arrival_time, region.vehicle1_exit_time),
                'v2_time': (region.vehicle2_arrival_time, region.vehicle2_exit_time)
            }
            for region in time_conflicts
        ]
        
        return conflict
    
    def _detect_segment_conflicts_enhanced(self, segment_id: str,
                                         occupations: List[BackboneSegmentOccupation]) -> List[BackboneConflict]:
        """强化段冲突检测"""
        conflicts = []
        
        for i in range(len(occupations)):
            for j in range(i + 1, len(occupations)):
                occ1, occ2 = occupations[i], occupations[j]
                
                if occ1.overlaps_with_enhanced(occ2, self.vehicle_info_cache):
                    conflict = self._create_enhanced_conflict(segment_id, [occ1, occ2])
                    if conflict:
                        conflicts.append(conflict)
                
                elif occ1.check_spatial_conflict(occ2, self.vehicle_info_cache):
                    conflict = self._create_enhanced_conflict(segment_id, [occ1, occ2])
                    if conflict:
                        conflict.conflict_type = ConflictType.SPATIAL_CONFLICT
                        conflict.spatial_conflict = True
                        conflicts.append(conflict)
        
        return conflicts
    
    def _create_enhanced_conflict(self, segment_id: str,
                                occupations: List[BackboneSegmentOccupation]) -> Optional[BackboneConflict]:
        """创建强化冲突对象"""
        if len(occupations) < 2:
            return None
        
        vehicles = [occ.vehicle_id for occ in occupations]
        backbone_path_id = occupations[0].backbone_path_id
        
        min_entry = min(occ.planned_entry_time for occ in occupations)
        max_exit = max(occ.planned_exit_time for occ in occupations)
        
        total_overlap = 0.0
        for i in range(len(occupations)):
            for j in range(i + 1, len(occupations)):
                overlap = self._calculate_enhanced_overlap(occupations[i], occupations[j])
                total_overlap += overlap
        
        conflict_type = self._classify_enhanced_conflict_type(occupations)
        severity = self._assess_enhanced_conflict_severity(occupations, total_overlap)
        
        priority_order = self._determine_priority_order_enhanced(occupations)
        suggested_resolution = self._suggest_enhanced_resolution_strategy(occupations, conflict_type, severity)
        
        negotiation_options = {}
        if len(vehicles) == 2 and severity in [ConflictSeverity.LOW, ConflictSeverity.MEDIUM]:
            negotiation_options = self._calculate_negotiation_options(occupations)
        
        conflict_id = f"enhanced_{segment_id}_{int(time.time()*1000)}"
        
        conflict = BackboneConflict(
            conflict_id=conflict_id,
            conflict_type=conflict_type,
            severity=severity,
            backbone_path_id=backbone_path_id,
            segment_id=segment_id,
            conflicting_vehicles=vehicles,
            occupations=occupations,
            conflict_time_window=(min_entry, max_exit),
            overlap_duration=total_overlap,
            suggested_resolution=suggested_resolution,
            priority_order=priority_order,
            negotiation_options=negotiation_options
        )
        
        return conflict
    
    def _calculate_enhanced_overlap(self, occ1: BackboneSegmentOccupation,
                                  occ2: BackboneSegmentOccupation) -> float:
        """计算强化重叠时长"""
        if not occ1.overlaps_with_enhanced(occ2, self.vehicle_info_cache):
            return 0.0
        
        v1_info = self.vehicle_info_cache.get(occ1.vehicle_id, {})
        v2_info = self.vehicle_info_cache.get(occ2.vehicle_id, {})
        
        occ1_margin = occ1.calculate_dynamic_safety_margin(v1_info)
        occ2_margin = occ2.calculate_dynamic_safety_margin(v2_info)
        
        occ1_start = occ1.planned_entry_time - occ1.calculate_deceleration_time(v1_info)
        occ1_end = occ1.planned_exit_time + occ1_margin
        occ2_start = occ2.planned_entry_time - occ2.calculate_deceleration_time(v2_info)
        occ2_end = occ2.planned_exit_time + occ2_margin
        
        overlap_start = max(occ1_start, occ2_start)
        overlap_end = min(occ1_end, occ2_end)
        
        return max(0.0, overlap_end - overlap_start)
    
    def _classify_enhanced_conflict_type(self, occupations: List[BackboneSegmentOccupation]) -> ConflictType:
        """强化冲突分类"""
        priorities = set(occ.vehicle_priority for occ in occupations)
        
        if len(priorities) > 1:
            return ConflictType.PRIORITY_CONFLICT
        
        max_overlap = 0.0
        for i in range(len(occupations)):
            for j in range(i + 1, len(occupations)):
                overlap = self._calculate_enhanced_overlap(occupations[i], occupations[j])
                max_overlap = max(max_overlap, overlap)
        
        if max_overlap > 120:
            return ConflictType.TEMPORAL_DEADLOCK
        
        return ConflictType.BACKBONE_SEGMENT_CONFLICT
    
    def _assess_enhanced_conflict_severity(self, occupations: List[BackboneSegmentOccupation],
                                         total_overlap: float) -> ConflictSeverity:
        """强化严重程度评估"""
        severity_score = 0
        
        priorities = [occ.vehicle_priority for occ in occupations]
        max_priority_diff = max(priorities) - min(priorities)
        severity_score += min(max_priority_diff, 3)
        
        if total_overlap > 120: severity_score += 4
        elif total_overlap > 60: severity_score += 3
        elif total_overlap > 30: severity_score += 2
        elif total_overlap > 15: severity_score += 1
        
        vehicle_count = len(occupations)
        if vehicle_count > 3: severity_score += 3
        elif vehicle_count > 2: severity_score += 2
        
        risk_penalty = 0
        for occ in occupations:
            risk_info = occ.segment_risk_info
            if risk_info.near_excavation_area: risk_penalty += 2
            if risk_info.is_main_haul_road: risk_penalty += 1
            if abs(risk_info.gradient) > 0.15: risk_penalty += 2
            if risk_info.weather_risk > 1.5: risk_penalty += 1
        
        severity_score += min(risk_penalty, 4)
        
        if severity_score >= 8: return ConflictSeverity.CRITICAL
        elif severity_score >= 6: return ConflictSeverity.HIGH
        elif severity_score >= 4: return ConflictSeverity.MEDIUM
        else: return ConflictSeverity.LOW
    
    def _determine_priority_order_enhanced(self, occupations: List[BackboneSegmentOccupation]) -> List[str]:
        """强化优先级排序 - 同优先级按先到先行"""
        def get_priority_score(occ):
            base_priority = occ.vehicle_priority * 1000
            
            # 关键修改：同优先级时，先到者优先（到达时间越早分数越高）
            time_priority = -occ.planned_entry_time  # 负号表示时间越早分数越高
            
            vehicle_info = self.vehicle_info_cache.get(occ.vehicle_id, {})
            load_ratio = vehicle_info.get('current_load', 0) / max(vehicle_info.get('max_load', 100), 1)
            load_bonus = load_ratio * 100
            
            vehicle_type = vehicle_info.get('vehicle_type', 'dump_truck')
            type_bonus = {'excavator': 50, 'emergency': 200}.get(vehicle_type, 0)
            
            return base_priority + time_priority * 0.1 + load_bonus + type_bonus  # 增加时间权重
        
        sorted_occupations = sorted(occupations, key=get_priority_score, reverse=True)
        # 修复：返回不重复的车辆ID列表
        vehicle_order = []
        for occ in sorted_occupations:
            if occ.vehicle_id not in vehicle_order:
                vehicle_order.append(occ.vehicle_id)
        
        return vehicle_order
    
    def _suggest_enhanced_resolution_strategy(self, occupations: List[BackboneSegmentOccupation],
                                            conflict_type: ConflictType,
                                            severity: ConflictSeverity) -> ResolutionStrategy:
        """强化解决策略建议"""
        if severity == ConflictSeverity.CRITICAL:
            return ResolutionStrategy.EMERGENCY_STOP
        
        if conflict_type == ConflictType.PREDICTIVE_CONFLICT:
            return ResolutionStrategy.PREVENTIVE_RESCHEDULING
        
        if conflict_type == ConflictType.SPATIAL_CONFLICT:
            return ResolutionStrategy.NEGOTIATED_ADJUSTMENT
        
        if conflict_type == ConflictType.PRIORITY_CONFLICT:
            return ResolutionStrategy.PROGRESSIVE_DELAY
        
        priorities = set(occ.vehicle_priority for occ in occupations)
        if len(priorities) == 1 and len(occupations) == 2:
            return ResolutionStrategy.NEGOTIATED_ADJUSTMENT
        
        return ResolutionStrategy.FIRST_COME_FIRST_SERVE
    
    def _calculate_negotiation_options(self, occupations: List[BackboneSegmentOccupation]) -> Dict:
        """计算协商选项"""
        if len(occupations) != 2:
            return {}
        
        occ1, occ2 = occupations
        
        overlap = self._calculate_enhanced_overlap(occ1, occ2)
        
        options = {
            'mutual_adjustment': {
                'vehicle1_delay': overlap * 0.3,
                'vehicle2_delay': overlap * 0.7,
                'total_delay': overlap,
                'efficiency_cost': overlap * 0.5
            },
            'minimal_delay': {
                'vehicle1_delay': 0,
                'vehicle2_delay': overlap + 10,
                'total_delay': overlap + 10,
                'efficiency_cost': overlap + 10
            }
        }
        
        return options
    
    def mark_conflict_resolved(self, conflict_id: str, resolution_method: str):
        """标记冲突已解决"""
        with self.lock:
            if conflict_id in self.detected_conflicts:
                conflict = self.detected_conflicts[conflict_id]
                conflict.is_resolved = True
                conflict.resolution_time = time.time()
                
                self.resolved_conflicts.append(conflict)
                del self.detected_conflicts[conflict_id]
                
                if resolution_method == 'negotiated_adjustment':
                    self.stats['negotiated_resolutions'] += 1
                elif resolution_method == 'preventive_rescheduling':
                    self.stats['preventive_adjustments'] += 1
    
    def get_active_conflicts(self) -> List[BackboneConflict]:
        """获取活跃冲突"""
        with self.lock:
            return list(self.detected_conflicts.values())
    
    def get_vehicle_conflicts(self, vehicle_id: str) -> List[BackboneConflict]:
        """获取车辆冲突"""
        with self.lock:
            conflicts = []
            for conflict in self.detected_conflicts.values():
                if vehicle_id in conflict.conflicting_vehicles:
                    conflicts.append(conflict)
            return conflicts
    
    def _clear_vehicle_occupations(self, vehicle_id: str):
        """清除车辆占用记录"""
        for segment_id in self.vehicle_occupations.get(vehicle_id, []):
            if segment_id in self.segment_occupations:
                self.segment_occupations[segment_id] = [
                    occ for occ in self.segment_occupations[segment_id]
                    if occ.vehicle_id != vehicle_id
                ]
                
                if not self.segment_occupations[segment_id]:
                    del self.segment_occupations[segment_id]
        
        if vehicle_id in self.vehicle_occupations:
            del self.vehicle_occupations[vehicle_id]
    
    def get_system_status(self) -> Dict:
        """获取系统状态"""
        with self.lock:
            status = {
                'active_segments': len(self.segment_occupations),
                'active_vehicles': len(self.vehicle_occupations),
                'active_conflicts': len(self.detected_conflicts),
                'resolved_conflicts': len(self.resolved_conflicts),
                'enhanced_statistics': self.stats.copy(),
                'logical_conflicts': self.stats['logical_conflicts_detected'],
                'spatial_conflicts': self.stats['spatial_conflicts_detected'],
                'spatial_supplement_conflicts': self.stats['spatial_supplement_conflicts'],
                'predictive_conflicts': self.stats['predictive_conflicts_detected'],
                'negotiated_resolutions': self.stats['negotiated_resolutions'],
                'spatial_detection_enabled': self.config.get('enable_spatial_detection', False)
            }
            
            if self.spatial_detector:
                try:
                    spatial_stats = self.spatial_detector.get_statistics()
                    status['spatial_detector_stats'] = spatial_stats
                except:
                    pass
            
            return status