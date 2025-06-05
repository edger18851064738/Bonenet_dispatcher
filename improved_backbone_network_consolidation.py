"""
improved_backbone_network_consolidation.py - 智能继承式节点整合系统
通过智能半径调整和加权质心实现多轮迭代节点继承整合
"""
import math
import time
import random
from collections import defaultdict, OrderedDict
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, field
import threading
try:
    from scipy.interpolate import CubicSpline
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

@dataclass
class NodePathMembership:
    """节点路径归属"""
    path_id: str
    node_index: int
    position: Tuple[float, float, float]

@dataclass
class InheritedNode:
    """继承节点"""
    node_id: str
    position: Tuple[float, float, float]
    memberships: List[NodePathMembership] = field(default_factory=list)
    original_nodes: List[Tuple[float, float, float]] = field(default_factory=list)
    
    def add_membership(self, path_id: str, node_index: int, position: Tuple):
        """添加路径归属"""
        self.memberships.append(NodePathMembership(path_id, node_index, position))
        self.original_nodes.append(position)
    
    def get_path_count(self) -> int:
        """获取路径数量"""
        return len(set(m.path_id for m in self.memberships))

class NodeInheritanceDetector:
    """节点继承检测器 - 智能半径版"""
    
    def __init__(self, base_merge_radius=3.5, env=None):
        self.base_merge_radius = base_merge_radius
        self.env = env
        self.current_radius = base_merge_radius
        
    def detect_mergeable_nodes(self, bidirectional_paths: Dict, round_num: int = 1) -> List[List[NodePathMembership]]:
        """检测可合并的节点组 - 智能半径调整"""
        # 智能半径调整
        self.current_radius = self._calculate_adaptive_radius(bidirectional_paths, round_num)
        
        print(f"\n🔍 [继承检测] 第{round_num}轮，半径: {self.current_radius:.1f}m")
        
        # 建立节点映射
        all_node_memberships = []
        for path_id, path_data in bidirectional_paths.items():
            for i, node in enumerate(path_data.forward_path):
                membership = NodePathMembership(path_id, i, node)
                all_node_memberships.append(membership)
        
        print(f"   总节点数: {len(all_node_memberships)}")
        
        # 检测可合并组
        mergeable_groups = []
        processed_indices = set()
        
        for i, membership in enumerate(all_node_memberships):
            if i in processed_indices:
                continue
                
            conflict_group = [membership]
            conflict_indices = [i]
            
            for j, other_membership in enumerate(all_node_memberships):
                if j <= i or j in processed_indices:
                    continue
                
                if membership.path_id == other_membership.path_id:
                    continue
                
                distance = self._calculate_distance(membership.position, other_membership.position)
                if distance <= self.current_radius:
                    conflict_group.append(other_membership)
                    conflict_indices.append(j)
            
            if len(conflict_group) > 1:
                mergeable_groups.append(conflict_group)
                processed_indices.update(conflict_indices)
        
        print(f"   检测到可合并组: {len(mergeable_groups)}")
        return mergeable_groups
    
    def _calculate_adaptive_radius(self, paths: Dict, round_num: int) -> float:
        """计算自适应半径"""
        # 基础半径随轮次递减
        round_factor = max(0.5, 1.0 - (round_num - 1) * 0.15)
        
        # 根据路径密度调整
        total_nodes = sum(len(p.forward_path) for p in paths.values())
        avg_nodes_per_path = total_nodes / len(paths) if paths else 10
        
        if avg_nodes_per_path > 25:  # 高密度
            density_factor = 1.2
        elif avg_nodes_per_path < 10:  # 低密度
            density_factor = 0.8
        else:
            density_factor = 1.0
        
        # 环境大小自适应
        env_factor = 1.0
        if self.env:
            env_size = max(self.env.width, self.env.height)
            if env_size > 200:
                env_factor = 1.3
            elif env_size < 100:
                env_factor = 0.7
        
        adaptive_radius = self.base_merge_radius * round_factor * density_factor * env_factor
        return max(2.0, min(8.0, adaptive_radius))  # 限制在合理范围
    
    def _calculate_distance(self, pos1: Tuple, pos2: Tuple) -> float:
        """计算两点距离"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

class NodeInheritanceProcessor:
    """节点继承处理器 - 加权质心版"""
    
    def __init__(self, env=None):
        self.inherited_nodes = {}
        self.env = env
        
    def create_inherited_nodes(self, mergeable_groups: List[List[NodePathMembership]], 
                              path_qualities: Dict = None) -> Dict[str, InheritedNode]:
        """创建继承节点 - 加权质心计算"""
        print(f"\n🧬 [节点继承] 开始创建加权继承节点")
        
        inherited_nodes = {}
        
        for group_id, group in enumerate(mergeable_groups):
            # 加权质心计算
            inherit_position = self._calculate_weighted_safe_centroid(group, path_qualities)
            
            inherited_node = InheritedNode(
                node_id=f"inherited_{group_id}",
                position=inherit_position
            )
            
            for membership in group:
                inherited_node.add_membership(
                    membership.path_id, 
                    membership.node_index, 
                    membership.position
                )
            
            inherited_nodes[inherited_node.node_id] = inherited_node
            
            paths = set(m.path_id for m in group)
            print(f"   继承节点 {inherited_node.node_id}: 位置{inherit_position[:2]}, "
                  f"继承{len(group)}节点, 涉及{len(paths)}路径")
        
        self.inherited_nodes = inherited_nodes
        return inherited_nodes
    
    def _calculate_weighted_safe_centroid(self, group: List[NodePathMembership], 
                                        path_qualities: Dict = None) -> Tuple[float, float, float]:
        """计算加权安全质心位置"""
        if not group:
            return (0, 0, 0)
        
        # 计算权重
        weights = []
        for membership in group:
            weight = 1.0  # 基础权重
            
            # 路径质量权重
            if path_qualities and membership.path_id in path_qualities:
                quality = path_qualities[membership.path_id]
                weight *= max(0.5, quality)  # 质量越高权重越大
            
            # 位置安全性权重
            if self._is_position_safe(membership.position):
                weight *= 1.5  # 安全位置加权
            
            weights.append(weight)
        
        # 归一化权重
        total_weight = sum(weights)
        if total_weight == 0:
            weights = [1.0] * len(group)
            total_weight = len(group)
        
        normalized_weights = [w / total_weight for w in weights]
        
        # 加权质心计算
        x_weighted = sum(m.position[0] * w for m, w in zip(group, normalized_weights))
        y_weighted = sum(m.position[1] * w for m, w in zip(group, normalized_weights))
        z_weighted = sum((m.position[2] if len(m.position) > 2 else 0) * w 
                        for m, w in zip(group, normalized_weights))
        
        weighted_centroid = (x_weighted, y_weighted, z_weighted)
        
        # 安全性验证
        if self._is_position_safe(weighted_centroid):
            return weighted_centroid
        else:
            # 选择最安全的高权重节点
            return self._select_safest_weighted_position(group, normalized_weights)
    
    def _select_safest_weighted_position(self, group: List[NodePathMembership], 
                                       weights: List[float]) -> Tuple[float, float, float]:
        """选择最安全的加权位置"""
        # 按权重排序，优先选择高权重的安全位置
        weighted_positions = list(zip(group, weights))
        weighted_positions.sort(key=lambda x: x[1], reverse=True)
        
        for membership, weight in weighted_positions:
            if self._is_position_safe(membership.position):
                return membership.position
        
        # 如果都不安全，返回权重最高的
        return weighted_positions[0][0].position
    
    def _is_position_safe(self, position: Tuple) -> bool:
        """检查位置安全性 - 增强版"""
        x, y = position[0], position[1]
        
        # 边界检查
        if self.env:
            if x < 0 or y < 0 or x >= self.env.width or y >= self.env.height:
                return False
            
            # 障碍物检查
            check_x, check_y = int(x), int(y)
            safety_radius = 2
            
            for dx in range(-safety_radius, safety_radius + 1):
                for dy in range(-safety_radius, safety_radius + 1):
                    cx, cy = check_x + dx, check_y + dy
                    
                    if (0 <= cx < self.env.width and 0 <= cy < self.env.height):
                        if (hasattr(self.env, 'grid') and self.env.grid[cx, cy] == 1):
                            return False
                        
                        if hasattr(self.env, 'obstacle_points'):
                            for obs_x, obs_y in self.env.obstacle_points:
                                if abs(cx - obs_x) <= 1 and abs(cy - obs_y) <= 1:
                                    return False
        else:
            # 无环境时的基础检查
            return not (x < 0 or y < 0 or x > 1000 or y > 1000)
        
        return True

class PathInheritanceApplicator:
    """路径继承应用器"""
    
    def apply_inheritance_to_paths(self, bidirectional_paths: Dict, 
                                 inherited_nodes: Dict[str, InheritedNode]) -> Dict[str, Any]:
        """应用继承到路径"""
        print(f"\n🔄 [路径重建] 开始应用节点继承")
        
        # 建立节点替换映射
        replacement_map = {}  # {(path_id, node_index): inherited_node_id}
        
        for inherited_node in inherited_nodes.values():
            for membership in inherited_node.memberships:
                key = (membership.path_id, membership.node_index)
                replacement_map[key] = inherited_node
        
        # 重建每条路径
        rebuilt_paths = {}
        
        for path_id, path_data in bidirectional_paths.items():
            new_forward_path = []
            
            for i, original_node in enumerate(path_data.forward_path):
                key = (path_id, i)
                
                if key in replacement_map:
                    # 使用继承节点位置
                    inherited_node = replacement_map[key]
                    new_forward_path.append(inherited_node.position)
                else:
                    # 保持原节点
                    new_forward_path.append(original_node)
            
            # 创建新的路径对象
            rebuilt_path = self._create_inherited_backbone_path(
                path_id, path_data, new_forward_path, replacement_map
            )
            rebuilt_paths[path_id] = rebuilt_path
            
            # 计算继承影响
            inherited_count = sum(1 for i in range(len(path_data.forward_path)) 
                                if (path_id, i) in replacement_map)
            print(f"   路径 {path_id}: {inherited_count}/{len(path_data.forward_path)} 节点被继承")
        
        print(f"   ✅ 路径重建完成: {len(rebuilt_paths)} 条路径")
        return rebuilt_paths
    
    def _create_inherited_backbone_path(self, path_id: str, original_path: Any, 
                                      new_forward_path: List, replacement_map: Dict):
        """创建继承后的骨干路径对象"""
        class InheritedBackbonePath:
            def __init__(self, path_id, original_path, new_forward_path, replacement_map):
                self.path_id = path_id
                self.forward_path = new_forward_path
                self.reverse_path = list(reversed(new_forward_path))
                self.length = self._calculate_length(new_forward_path)
                self.quality = original_path.quality * 0.95  # 轻微质量损失
                self.planner_used = 'inheritance_consolidation'
                self.created_time = time.time()
                self.usage_count = original_path.usage_count if hasattr(original_path, 'usage_count') else 0
                self.current_load = 0
                self.max_capacity = 5
                
                # 端点信息
                self.point_a = original_path.point_a
                self.point_b = original_path.point_b
                
                # 继承信息
                self.inherited_node_count = sum(1 for i in range(len(new_forward_path)) 
                                              if (path_id, i) in replacement_map)
                self.is_consolidated = True
                self.consolidation_level = "node_inheritance"
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
        
        return InheritedBackbonePath(path_id, original_path, new_forward_path, replacement_map)

class PathRefitter:
    """层次化骨干网络重构器"""
    
    def __init__(self, env):
        self.env = env
        self.vehicle_width = 3.0
        self.vehicle_length = 6.0
        self.turning_radius = 8.0
        self.safety_margin = 1.5
        
        # 层次化网络结构
        self.backbone_segments = {}  # 主干段
        self.branch_connections = {}  # 枝干连接
        
    def refit_inherited_paths(self, rebuilt_paths: Dict, inherited_nodes: Dict) -> Dict[str, Any]:
        """重构层次化骨干网络"""
        print(f"\n🎯 [层次化重构] 开始构建骨干网络")
        
        # 步骤1: 构建主干路径
        backbone_paths = self._build_backbone_segments(rebuilt_paths, inherited_nodes)
        
        # 步骤2: 构建枝干连接
        branch_paths = self._build_branch_connections(inherited_nodes)
        
        # 步骤3: 整合层次化网络
        hierarchical_network = self._integrate_hierarchical_network(backbone_paths, branch_paths)
        
        print(f"   ✅ 层次化网络构建完成: {len(backbone_paths)}主干 + {len(branch_paths)}枝干")
        
        return hierarchical_network
    
    def _build_backbone_segments(self, rebuilt_paths: Dict, inherited_nodes: Dict) -> Dict:
        """构建主干路径段"""
        print(f"   构建主干路径段...")
        backbone_segments = {}
        
        for path_id, path_obj in rebuilt_paths.items():
            # 识别有序关键节点序列
            key_sequence = self._identify_ordered_key_nodes(path_obj, inherited_nodes)
            
            if len(key_sequence) >= 2:
                # 使用车辆动力学生成主干路径
                backbone_path = self._generate_vehicle_dynamics_path(key_sequence)
                
                if backbone_path:
                    backbone_segment = self._create_backbone_segment(path_id, path_obj, backbone_path, key_sequence)
                    backbone_segments[path_id] = backbone_segment
                    print(f"     主干段 {path_id}: {len(key_sequence)}关键节点 → {len(backbone_path)}路径点")
                else:
                    # 回退到原路径
                    backbone_segments[path_id] = path_obj
        
        self.backbone_segments = backbone_segments
        return backbone_segments
    
    def _build_branch_connections(self, inherited_nodes: Dict) -> Dict:
        """构建枝干连接网络 - 限制连接数"""
        print(f"   构建枝干连接...")
        branch_connections = {}
        connected_pairs = set()  # 避免重复连接
        
        node_positions = {node_id: node.position for node_id, node in inherited_nodes.items()}
        
        for node_id, position in node_positions.items():
            # 找到最近的2-3个节点
            distances = []
            for other_id, other_pos in node_positions.items():
                if other_id != node_id:
                    distance = self._calculate_distance(position, other_pos)
                    if distance <= 15.0:  # 距离限制
                        distances.append((distance, other_id, other_pos))
            
            # 按距离排序，只取最近的2个
            distances.sort()
            closest_nodes = distances[:2]
            
            for distance, nearby_id, nearby_pos in closest_nodes:
                # 避免重复连接（A-B 和 B-A 是同一条）
                pair = tuple(sorted([node_id, nearby_id]))
                if pair not in connected_pairs:
                    connected_pairs.add(pair)
                    
                    branch_id = f"branch_{pair[0]}_to_{pair[1]}"
                    branch_path = self._generate_branch_connection(position, nearby_pos)
                    
                    if branch_path:
                        branch_connections[branch_id] = {
                            'from_node': pair[0],
                            'to_node': pair[1],
                            'path': branch_path,
                            'length': self._calculate_path_length(branch_path),
                            'connection_type': 'branch'
                        }
        
        self.branch_connections = branch_connections
        print(f"     ✅ 生成 {len(branch_connections)} 条枝干连接")
        return branch_connections
    
    def _identify_ordered_key_nodes(self, path_obj, inherited_nodes: Dict) -> List[Tuple]:
        """识别有序关键节点序列"""
        key_sequence = []
        
        # 起点
        key_sequence.append(path_obj.forward_path[0])
        
        # 按路径顺序添加继承节点
        path_nodes = path_obj.forward_path
        for inherited_node in inherited_nodes.values():
            for membership in inherited_node.memberships:
                if membership.path_id == path_obj.path_id:
                    # 找到在原路径中的位置，保持顺序
                    insert_pos = membership.node_index
                    if insert_pos > 0 and insert_pos < len(path_nodes):
                        key_sequence.append(inherited_node.position)
                    break
        
        # 终点
        key_sequence.append(path_obj.forward_path[-1])
        
        # 去重并保持顺序
        unique_sequence = []
        for node in key_sequence:
            if not unique_sequence or self._calculate_distance(node, unique_sequence[-1]) > 2.0:
                unique_sequence.append(node)
        
        return unique_sequence
    
    def _generate_vehicle_dynamics_path(self, key_sequence: List[Tuple]) -> Optional[List]:
        """保持关键节点序列，不生成新节点"""
        if len(key_sequence) < 2:
            return None
        
        # 直接返回关键节点序列，不插值
        return key_sequence
    
    def _generate_branch_connection(self, start: Tuple, end: Tuple) -> Optional[List]:
        """生成最简枝干连接：只有两个端点"""
        return [start, end]
    
    def _integrate_hierarchical_network(self, backbone_paths: Dict, branch_paths: Dict) -> Dict:
        """整合层次化网络"""
        integrated_network = {}
        
        # 添加主干路径
        for path_id, backbone_segment in backbone_paths.items():
            integrated_network[path_id] = backbone_segment
        
        # 添加枝干路径作为特殊路径类型
        for branch_id, branch_data in branch_paths.items():
            branch_path_obj = self._create_branch_path_object(branch_id, branch_data)
            integrated_network[branch_id] = branch_path_obj
        
        return integrated_network
    
    def _create_backbone_segment(self, path_id: str, original_path: Any, backbone_path: List, key_sequence: List):
        """创建主干路径段对象"""
        class BackboneSegment:
            def __init__(self, path_id, original_path, backbone_path, key_sequence):
                self.path_id = path_id
                self.forward_path = backbone_path
                self.reverse_path = list(reversed(backbone_path))
                self.length = self._calculate_length(backbone_path)
                self.quality = original_path.quality * 1.1  # 动力学优化提升质量
                self.planner_used = 'vehicle_dynamics_backbone'
                self.created_time = time.time()
                self.usage_count = getattr(original_path, 'usage_count', 0)
                self.current_load = 0
                self.max_capacity = 8  # 主干容量更大
                
                # 端点信息
                self.point_a = original_path.point_a
                self.point_b = original_path.point_b
                
                # 层次化信息
                self.key_sequence = key_sequence
                self.key_node_count = len(key_sequence)
                self.is_consolidated = True
                self.consolidation_level = "hierarchical_backbone"
                self.network_type = "backbone_segment"
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
            
            def get_average_quality(self):
                return sum(self.quality_history) / len(self.quality_history) if self.quality_history else self.quality
            
            def _calculate_length(self, path):
                if len(path) < 2:
                    return 0.0
                return sum(
                    math.sqrt((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)
                    for i in range(len(path) - 1)
                )
        
        return BackboneSegment(path_id, original_path, backbone_path, key_sequence)
    
    def _create_branch_path_object(self, branch_id: str, branch_data: Dict):
        """创建枝干路径对象"""
        class BranchConnection:
            def __init__(self, branch_id, branch_data):
                self.path_id = branch_id
                self.forward_path = branch_data['path']
                self.reverse_path = list(reversed(branch_data['path']))
                self.length = branch_data['length']
                self.quality = 0.8  # 枝干路径质量
                self.planner_used = 'branch_connection'
                self.created_time = time.time()
                self.usage_count = 0
                self.current_load = 0
                self.max_capacity = 3  # 枝干容量较小
                
                # 枝干特定信息
                self.from_node = branch_data['from_node']
                self.to_node = branch_data['to_node']
                self.connection_type = branch_data['connection_type']
                self.network_type = "branch_connection"
                self.is_consolidated = True
                self.consolidation_level = "branch_network"
                
                # 虚拟端点（用于兼容）
                self.point_a = {'type': 'key_node', 'id': self.from_node}
                self.point_b = {'type': 'key_node', 'id': self.to_node}
            
            def get_path(self, from_point_type, from_point_id, to_point_type, to_point_id):
                return self.forward_path  # 枝干路径简化处理
            
            def increment_usage(self):
                self.usage_count += 1
            
            def add_vehicle(self, vehicle_id):
                self.current_load += 1
            
            def remove_vehicle(self, vehicle_id):
                self.current_load = max(0, self.current_load - 1)
            
            def get_load_factor(self):
                return self.current_load / self.max_capacity
            
            def get_average_quality(self):
                return self.quality
            
            def update_quality_history(self, new_quality):
                """更新质量历史（枝干路径简化实现）"""
                self.quality = new_quality
        
        return BranchConnection(branch_id, branch_data)
    
    def _calculate_distance(self, p1: Tuple, p2: Tuple) -> float:
        """计算两点距离"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def _calculate_path_length(self, path: List[Tuple]) -> float:
        """计算路径长度"""
        if len(path) < 2:
            return 0.0
        return sum(self._calculate_distance(path[i], path[i+1]) for i in range(len(path)-1))

class ImprovedBackboneNetworkConsolidator:
    """改进的骨干网络整合器 - 智能半径+加权质心版"""
    
    def __init__(self, env, path_planner, config: Dict = None):
        self.env = env
        self.path_planner = path_planner
        
        # 配置
        self.config = {
            'base_merge_radius': 3.5,
            'adaptive_radius': True,
            'weighted_centroid': True,
            'preserve_original_backup': True,
            'enable_visualization': False
        }
        
        if config:
            self.config.update(config)
        
        # 组件 - 传入env支持智能功能
        self.detector = NodeInheritanceDetector(
            self.config['base_merge_radius'], 
            env if self.config['adaptive_radius'] else None
        )
        self.processor = NodeInheritanceProcessor(
            env if self.config['weighted_centroid'] else None
        )
        self.applicator = PathInheritanceApplicator()
        self.refitter = PathRefitter(env)
        
        # 结果存储
        self.inherited_nodes = {}
        self.original_paths_backup = {}
        self.consolidation_stats = {}
        
        print(f"智能继承式整合器初始化")
        print(f"  基础半径: {self.config['base_merge_radius']}m")
        print(f"  智能半径: {'✅' if self.config['adaptive_radius'] else '❌'}")
        print(f"  加权质心: {'✅' if self.config['weighted_centroid'] else '❌'}")
    
    def aggressive_consolidate_backbone_network(self, backbone_network):
        """智能多轮迭代整合"""
        print(f"\n🚀 开始智能多轮继承式整合...")
        start_time = time.time()
        
        # 备份原始路径
        if self.config['preserve_original_backup']:
            self.original_paths_backup = backbone_network.bidirectional_paths.copy()
        
        original_count = len(backbone_network.bidirectional_paths)
        original_node_count = sum(len(p.forward_path) for p in backbone_network.bidirectional_paths.values())
        print(f"原始网络: {original_count} 条路径, {original_node_count} 个节点")
        
        try:
            # 提取路径质量信息
            path_qualities = {}
            for path_id, path_data in backbone_network.bidirectional_paths.items():
                path_qualities[path_id] = getattr(path_data, 'quality', 0.7)
            
            # 多轮智能整合
            current_paths = backbone_network.bidirectional_paths
            all_inherited_nodes = {}
            round_num = 0
            max_rounds = 6  # 增加到6轮
            
            while round_num < max_rounds:
                round_num += 1
                print(f"\n🔄 第 {round_num} 轮智能整合:")
                
                # 智能检测可合并节点
                mergeable_groups = self.detector.detect_mergeable_nodes(current_paths, round_num)
                
                if not mergeable_groups:
                    print(f"   第 {round_num} 轮无可合并节点，停止迭代")
                    break
                
                # 加权创建继承节点
                round_inherited = self.processor.create_inherited_nodes(
                    mergeable_groups, path_qualities
                )
                
                # 累积继承节点
                for node_id, node in round_inherited.items():
                    all_inherited_nodes[f"r{round_num}_{node_id}"] = node
                
                # 应用继承
                current_paths = self.applicator.apply_inheritance_to_paths(
                    current_paths, round_inherited
                )
                
                # 统计效果
                current_nodes = sum(len(p.forward_path) for p in current_paths.values())
                print(f"   本轮继承节点: {len(round_inherited)}")
                print(f"   当前总节点: {current_nodes}")
                print(f"   使用半径: {self.detector.current_radius:.1f}m")
                
                # 智能停止条件
                if len(round_inherited) < max(1, len(mergeable_groups) * 0.3):
                    print(f"   改进效率降低，智能停止")
                    break
            
            # 最终路径重拟合
            if all_inherited_nodes:
                print(f"\n🎯 最终智能路径重拟合...")
                final_paths = self.refitter.refit_inherited_paths(current_paths, all_inherited_nodes)
            else:
                final_paths = current_paths
            
            # 详细统计
            consolidation_time = time.time() - start_time
            final_node_count = sum(len(p.forward_path) for p in final_paths.values())
            total_reduction = (original_node_count - final_node_count) / original_node_count if original_node_count > 0 else 0
            avg_key_nodes = sum(getattr(p, 'key_node_count', 0) for p in final_paths.values()) / len(final_paths) if final_paths else 0
            
            self.consolidation_stats = {
                'original_paths': original_count,
                'consolidated_paths': len(final_paths),
                'original_nodes': original_node_count,
                'consolidated_nodes': final_node_count,
                'total_inherited_nodes': len(all_inherited_nodes),
                'consolidation_rounds': round_num,
                'node_reduction_ratio': total_reduction,
                'average_key_nodes_per_path': avg_key_nodes,
                'consolidation_time': consolidation_time,
                'adaptive_radius_used': self.config['adaptive_radius'],
                'weighted_centroid_used': self.config['weighted_centroid'],
                'final_radius': getattr(self.detector, 'current_radius', 0)
            }
            
            print(f"\n🎉 智能多轮整合完成!")
            print(f"  整合轮数: {round_num}")
            print(f"  总继承节点: {len(all_inherited_nodes)}")
            print(f"  节点压缩: {original_node_count} → {final_node_count} ({total_reduction:.1%})")
            print(f"  最终半径: {getattr(self.detector, 'current_radius', 0):.1f}m")
            print(f"  整合耗时: {consolidation_time:.2f}s")
            
            self.inherited_nodes = all_inherited_nodes
            self.rebuilt_paths = final_paths
            return self._create_mock_topology(final_paths, all_inherited_nodes)
        
        except Exception as e:
            print(f"❌ 智能多轮整合失败: {e}")
            return None
    
    def _create_mock_topology(self, paths: Dict, inherited_nodes: Dict):
        """创建模拟拓扑网络"""
        class MockTopologyNetwork:
            def __init__(self, paths, inherited_nodes):
                self.key_nodes = inherited_nodes
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
        
        return MockTopologyNetwork(paths, inherited_nodes)
    
    def apply_to_backbone_network(self, backbone_network):
        """应用继承结果"""
        if hasattr(self, 'rebuilt_paths') and self.rebuilt_paths:
            backbone_network.bidirectional_paths = self.rebuilt_paths
            
            backbone_network.consolidation_info = {
                'is_consolidated': True,
                'consolidation_type': 'smart_node_inheritance',
                'consolidation_stats': self.consolidation_stats,
                'inherited_nodes': self.inherited_nodes
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
        return self.consolidation_stats.copy()

# 兼容性接口
def aggressive_consolidate_backbone_network(backbone_network, env, path_planner,
                                          cluster_radius=3.5, distance_threshold=2.0,
                                          apply_immediately=True, visualize=False):
    """兼容性接口"""
    config = {
        'base_merge_radius': cluster_radius,
        'adaptive_radius': True,
        'weighted_centroid': True,
        'preserve_original_backup': True
    }
    
    consolidator = ImprovedBackboneNetworkConsolidator(env, path_planner, config)
    topology_network = consolidator.aggressive_consolidate_backbone_network(backbone_network)
    
    if apply_immediately and topology_network:
        success = consolidator.apply_to_backbone_network(backbone_network)
        if success:
            print("✅ 智能继承式整合完成并已应用")
        else:
            print("❌ 应用继承结果失败")
    
    return consolidator

AggressiveBackboneNetworkConsolidator = ImprovedBackboneNetworkConsolidator