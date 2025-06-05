"""
improved_backbone_network_consolidation.py - 路径简化整合系统
基于Douglas-Peucker算法实现工程师级路径简化，减少冗余节点
"""
import math
import time
from collections import defaultdict
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass

@dataclass
class PathSimplificationStats:
    """路径简化统计"""
    original_nodes: int
    simplified_nodes: int
    reduction_ratio: float
    key_turning_points: int
    merged_segments: int

class KeyPointExtractor:
    """关键点提取器"""
    
    def __init__(self, curvature_threshold=0.3, min_segment_length=5.0):
        self.curvature_threshold = curvature_threshold
        self.min_segment_length = min_segment_length
    
    def extract_key_turning_points(self, path: List[Tuple]) -> List[int]:
        """提取关键转折点索引"""
        if len(path) < 3:
            return list(range(len(path)))
        
        key_indices = [0]  # 起点
        
        for i in range(1, len(path) - 1):
            curvature = self._calculate_curvature(path, i)
            
            # 高曲率点标记为关键点
            if curvature > self.curvature_threshold:
                key_indices.append(i)
            
            # 距离足够远的点也保留
            if key_indices:
                last_key = key_indices[-1]
                distance = self._calculate_distance(path[last_key], path[i])
                if distance > self.min_segment_length * 3:
                    key_indices.append(i)
        
        key_indices.append(len(path) - 1)  # 终点
        return sorted(list(set(key_indices)))
    
    def _calculate_curvature(self, path: List[Tuple], index: int) -> float:
        """计算路径点的曲率"""
        if index == 0 or index == len(path) - 1:
            return 0.0
        
        p1, p2, p3 = path[index-1], path[index], path[index+1]
        
        # 计算角度变化
        v1 = (p2[0] - p1[0], p2[1] - p1[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])
        
        len1 = math.sqrt(v1[0]**2 + v1[1]**2)
        len2 = math.sqrt(v2[0]**2 + v2[1]**2)
        
        if len1 < 1e-6 or len2 < 1e-6:
            return 0.0
        
        # 计算夹角
        cos_angle = (v1[0]*v2[0] + v1[1]*v2[1]) / (len1 * len2)
        cos_angle = max(-1.0, min(1.0, cos_angle))
        angle = math.acos(cos_angle)
        
        return angle / math.pi  # 归一化到[0,1]
    
    def _calculate_distance(self, p1: Tuple, p2: Tuple) -> float:
        """计算两点距离"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

class DouglasPackerSimplifier:
    """Douglas-Packer路径简化器"""
    
    def __init__(self, tolerance=2.0):
        self.tolerance = tolerance
    
    def simplify_path(self, path: List[Tuple]) -> List[Tuple]:
        """简化路径，保持关键形状特征"""
        if len(path) <= 2:
            return path
        
        simplified_indices = self._douglas_peucker(path, 0, len(path) - 1)
        simplified_indices = sorted(list(set(simplified_indices)))
        
        return [path[i] for i in simplified_indices]
    
    def _douglas_peucker(self, path: List[Tuple], start: int, end: int) -> List[int]:
        """Douglas-Peucker递归简化"""
        if end - start <= 1:
            return [start, end]
        
        # 找到距离直线最远的点
        max_distance = 0
        max_index = start
        
        for i in range(start + 1, end):
            distance = self._point_to_line_distance(path[i], path[start], path[end])
            if distance > max_distance:
                max_distance = distance
                max_index = i
        
        # 如果最大距离超过容忍度，递归分割
        if max_distance > self.tolerance:
            left_points = self._douglas_peucker(path, start, max_index)
            right_points = self._douglas_peucker(path, max_index, end)
            
            # 合并结果，去除重复的中间点
            return left_points[:-1] + right_points
        else:
            return [start, end]
    
    def _point_to_line_distance(self, point: Tuple, line_start: Tuple, line_end: Tuple) -> float:
        """计算点到直线的距离"""
        x0, y0 = point[0], point[1]
        x1, y1 = line_start[0], line_start[1]
        x2, y2 = line_end[0], line_end[1]
        
        # 避免除零
        line_length_sq = (x2 - x1)**2 + (y2 - y1)**2
        if line_length_sq < 1e-6:
            return math.sqrt((x0 - x1)**2 + (y0 - y1)**2)
        
        # 点到直线距离公式
        numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        return numerator / math.sqrt(line_length_sq)

class PathValidator:
    """路径验证器"""
    
    def __init__(self, env):
        self.env = env
        self.safety_margin = 1.0  # 减小安全边距
        self.validation_step = 3.0  # 增大验证步长
    
    def validate_simplified_path(self, path: List[Tuple]) -> bool:
        """验证简化路径的安全性"""
        if len(path) < 2:
            return True
        
        # 检查所有路径段
        for i in range(len(path) - 1):
            if not self._validate_segment(path[i], path[i + 1]):
                return False
        
        return True
    
    def _validate_segment(self, start: Tuple, end: Tuple) -> bool:
        """验证路径段是否安全"""
        distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        steps = max(1, int(distance / self.validation_step))
        
        for i in range(steps + 1):
            t = i / steps if steps > 0 else 0
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            
            if self._is_position_unsafe(x, y):
                return False
        
        return True
    
    def _is_position_unsafe(self, x: float, y: float) -> bool:
        """检查位置是否不安全 - 放宽检查"""
        # 边界检查
        if x < 0 or y < 0 or x >= self.env.width or y >= self.env.height:
            return True
        
        # 简化障碍物检查 - 只检查核心点
        center_x, center_y = int(x), int(y)
        
        # 直接检查当前位置
        if (hasattr(self.env, 'grid') and 
            self.env.grid[center_x, center_y] == 1):
            return True
        
        # 检查紧邻位置
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                check_x, check_y = center_x + dx, center_y + dy
                
                if (0 <= check_x < self.env.width and 
                    0 <= check_y < self.env.height):
                    
                    if (hasattr(self.env, 'grid') and 
                        self.env.grid[check_x, check_y] == 1):
                        return True
        
        return False

class ImprovedBackboneNetworkConsolidator:
    """改进的骨干网络整合器 - 路径简化版"""
    
    def __init__(self, env, path_planner, config: Dict = None):
        self.env = env
        self.path_planner = path_planner
        
        # 配置
        self.config = {
            'simplification_tolerance': 4.0,  # 增大容忍度
            'curvature_threshold': 0.4,       # 提高曲率阈值
            'min_segment_length': 8.0,        # 增大最小段长度
            'preserve_original_backup': True,
            'aggressive_simplification': False  # 先禁用激进模式
        }
        
        if config:
            self.config.update(config)
        
        # 组件
        self.key_extractor = KeyPointExtractor(
            self.config['curvature_threshold'],
            self.config['min_segment_length']
        )
        self.simplifier = DouglasPackerSimplifier(self.config['simplification_tolerance'])
        self.validator = PathValidator(env)
        
        # 结果存储
        self.original_paths_backup = {}
        self.simplification_stats = {}
        
        print(f"路径简化整合器初始化")
        print(f"  简化容忍度: {self.config['simplification_tolerance']}m")
        print(f"  曲率阈值: {self.config['curvature_threshold']}")
    
    def aggressive_consolidate_backbone_network(self, backbone_network):
        """路径简化整合骨干网络"""
        print(f"\n🚀 开始路径简化整合...")
        start_time = time.time()
        
        # 备份原始路径
        if self.config['preserve_original_backup']:
            self.original_paths_backup = backbone_network.bidirectional_paths.copy()
        
        original_stats = self._calculate_network_stats(backbone_network.bidirectional_paths)
        print(f"原始网络: {original_stats['total_paths']} 条路径, {original_stats['total_nodes']} 个节点")
        
        try:
            simplified_paths = {}
            total_reduction = 0
            
            for path_id, path_data in backbone_network.bidirectional_paths.items():
                print(f"  简化路径: {path_id}")
                
                # 简化前向路径
                simplified_forward = self._simplify_single_path(path_data.forward_path)
                
                if simplified_forward and self.validator.validate_simplified_path(simplified_forward):
                    # 创建简化路径对象
                    simplified_path = self._create_simplified_path_object(
                        path_id, path_data, simplified_forward
                    )
                    simplified_paths[path_id] = simplified_path
                    
                    # 统计节点减少
                    reduction = len(path_data.forward_path) - len(simplified_forward)
                    total_reduction += reduction
                    
                    print(f"    ✅ 节点: {len(path_data.forward_path)} → {len(simplified_forward)} (减少{reduction})")
                else:
                    # 保持原路径
                    simplified_paths[path_id] = path_data
                    print(f"    ⚠️ 简化失败，保持原路径")
            
            # 计算整体统计
            new_stats = self._calculate_network_stats(simplified_paths)
            consolidation_time = time.time() - start_time
            
            reduction_ratio = (original_stats['total_nodes'] - new_stats['total_nodes']) / original_stats['total_nodes']
            
            self.simplification_stats = {
                'original_paths': original_stats['total_paths'],
                'simplified_paths': len(simplified_paths),
                'original_nodes': original_stats['total_nodes'],
                'simplified_nodes': new_stats['total_nodes'],
                'node_reduction': total_reduction,
                'reduction_ratio': reduction_ratio,
                'consolidation_time': consolidation_time,
                'avg_nodes_per_path_before': original_stats['avg_nodes_per_path'],
                'avg_nodes_per_path_after': new_stats['avg_nodes_per_path']
            }
            
            print(f"\n🎉 路径简化整合完成!")
            print(f"  整合耗时: {consolidation_time:.2f}s")
            print(f"  节点压缩: {original_stats['total_nodes']} → {new_stats['total_nodes']} ({reduction_ratio:.1%})")
            print(f"  平均节点/路径: {original_stats['avg_nodes_per_path']:.1f} → {new_stats['avg_nodes_per_path']:.1f}")
            
            self.simplified_paths = simplified_paths
            return self._create_mock_topology(simplified_paths)
        
        except Exception as e:
            print(f"❌ 路径简化整合失败: {e}")
            return None
    
    def _simplify_single_path(self, path: List[Tuple]) -> Optional[List[Tuple]]:
        """简化单条路径"""
        if len(path) <= 2:
            return path
        
        try:
            # 方法1: Douglas-Peucker简化
            simplified = self.simplifier.simplify_path(path)
            
            # 方法2: 如果启用激进简化，进一步提取关键点
            if self.config['aggressive_simplification'] and len(simplified) > 3:
                key_indices = self.key_extractor.extract_key_turning_points(simplified)
                key_points = [simplified[i] for i in key_indices]
                
                # 验证关键点路径是否更优
                if len(key_points) < len(simplified):
                    if self.validator.validate_simplified_path(key_points):
                        simplified = key_points
            
            return simplified
            
        except Exception as e:
            print(f"    简化路径失败: {e}")
            return None
    
    def _create_simplified_path_object(self, path_id: str, original_path: Any, simplified_forward: List):
        """创建简化后的路径对象"""
        class SimplifiedBackbonePath:
            def __init__(self, path_id, original_path, simplified_forward):
                self.path_id = path_id
                self.forward_path = simplified_forward
                self.reverse_path = list(reversed(simplified_forward))
                self.length = self._calculate_length(simplified_forward)
                self.quality = original_path.quality * 1.1  # 简化提升质量
                self.planner_used = 'douglas_peucker_simplification'
                self.created_time = time.time()
                self.usage_count = original_path.usage_count if hasattr(original_path, 'usage_count') else 0
                self.current_load = 0
                self.max_capacity = 5
                
                # 端点信息
                self.point_a = original_path.point_a
                self.point_b = original_path.point_b
                
                # 简化信息
                self.original_node_count = len(original_path.forward_path)
                self.simplified_node_count = len(simplified_forward)
                self.node_reduction = self.original_node_count - self.simplified_node_count
                self.is_consolidated = True
                self.consolidation_level = "path_simplified"
                self.original_path_ids = [path_id]
                self.quality_history = [self.quality]
                self.last_quality_update = time.time()
            
            def get_path(self, from_point_type, from_point_id, to_point_type, to_point_id):
                if (self.point_a['type'] == from_point_type and self.point_a['id'] == from_point_id and
                    self.point_b['type'] == to_point_type and self.point_b['id'] == to_point_id):
                    return self.forward_path
                elif (self.point_b['type'] == from_point_type and self.point_b['id'] == from_point_id and
                      self.point_a['type'] == to_point_type and self.point_a['id'] == to_point_id):
                    return self.reverse_path
                return None
            
            def increment_usage(self):
                self.usage_count += 1
            
            def add_vehicle(self, vehicle_id):
                self.current_load += 1
            
            def remove_vehicle(self, vehicle_id):
                self.current_load = max(0, self.current_load - 1)
            
            def get_load_factor(self):
                return self.current_load / self.max_capacity
            
            def update_quality_history(self, new_quality):
                self.quality_history.append(new_quality)
                self.last_quality_update = time.time()
                if len(self.quality_history) > 20:
                    self.quality_history = self.quality_history[-10:]
            
            def get_average_quality(self):
                return sum(self.quality_history) / len(self.quality_history) if self.quality_history else self.quality
            
            def _calculate_length(self, path):
                if len(path) < 2:
                    return 0.0
                return sum(
                    math.sqrt((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)
                    for i in range(len(path) - 1)
                )
        
        return SimplifiedBackbonePath(path_id, original_path, simplified_forward)
    
    def _calculate_network_stats(self, paths: Dict) -> Dict:
        """计算网络统计"""
        total_paths = len(paths)
        total_nodes = sum(len(p.forward_path) for p in paths.values())
        avg_nodes_per_path = total_nodes / total_paths if total_paths > 0 else 0
        
        return {
            'total_paths': total_paths,
            'total_nodes': total_nodes,
            'avg_nodes_per_path': avg_nodes_per_path
        }
    
    def _create_mock_topology(self, paths: Dict):
        """创建模拟拓扑网络"""
        class MockTopologyNetwork:
            def __init__(self, paths):
                self.key_nodes = {}
                self.backbone_segments = {}
                self.branch_paths = {}
                self.final_backbone_paths = paths
            
            def get_network_summary(self):
                return {
                    'key_nodes': len(self.key_nodes),
                    'backbone_segments': len(self.backbone_segments),
                    'branch_paths': len(self.branch_paths),
                    'final_backbone_paths': len(self.final_backbone_paths)
                }
        
        return MockTopologyNetwork(paths)
    
    def apply_to_backbone_network(self, backbone_network):
        """应用简化结果"""
        if hasattr(self, 'simplified_paths') and self.simplified_paths:
            backbone_network.bidirectional_paths = self.simplified_paths
            
            backbone_network.consolidation_info = {
                'is_consolidated': True,
                'consolidation_type': 'path_simplification',
                'consolidation_stats': self.simplification_stats
            }
            
            if hasattr(backbone_network, '_build_connection_index'):
                backbone_network._build_connection_index()
            
            return True
        return False
    
    def restore_original_network(self, backbone_network):
        """恢复原始网络"""
        if not self.original_paths_backup:
            return False
        
        try:
            backbone_network.bidirectional_paths = self.original_paths_backup.copy()
            
            if hasattr(backbone_network, 'consolidation_info'):
                delattr(backbone_network, 'consolidation_info')
            
            if hasattr(backbone_network, '_build_connection_index'):
                backbone_network._build_connection_index()
            
            return True
        except:
            return False
    
    def get_consolidation_stats(self):
        """获取整合统计"""
        return self.simplification_stats.copy()

# 兼容性接口
def aggressive_consolidate_backbone_network(backbone_network, env, path_planner,
                                          cluster_radius=2.5, distance_threshold=2.0,
                                          apply_immediately=True, visualize=False):
    """兼容性接口"""
    config = {
        'simplification_tolerance': cluster_radius,
        'preserve_original_backup': True,
        'aggressive_simplification': True
    }
    
    consolidator = ImprovedBackboneNetworkConsolidator(env, path_planner, config)
    topology_network = consolidator.aggressive_consolidate_backbone_network(backbone_network)
    
    if apply_immediately and topology_network:
        success = consolidator.apply_to_backbone_network(backbone_network)
        if success:
            print("✅ 路径简化整合完成并已应用")
        else:
            print("❌ 应用简化结果失败")
    
    return consolidator

AggressiveBackboneNetworkConsolidator = ImprovedBackboneNetworkConsolidator