"""
improved_backbone_network_consolidation.py - 继承式节点整合系统
通过节点继承实现跨路径关键节点整合
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
    """节点继承检测器"""
    
    def __init__(self, merge_radius=3.5):
        self.merge_radius = merge_radius
        
    def detect_mergeable_nodes(self, bidirectional_paths: Dict) -> List[List[NodePathMembership]]:
        """检测可合并的节点组"""
        print(f"\n🔍 [继承检测] 开始检测可合并节点，合并半径: {self.merge_radius}m")
        
        # 1. 建立所有节点的路径归属映射
        all_node_memberships = []
        for path_id, path_data in bidirectional_paths.items():
            for i, node in enumerate(path_data.forward_path):
                membership = NodePathMembership(path_id, i, node)
                all_node_memberships.append(membership)
        
        print(f"   总节点数: {len(all_node_memberships)}")
        
        # 2. 检测距离冲突的节点组
        mergeable_groups = []
        processed_indices = set()
        
        for i, membership in enumerate(all_node_memberships):
            if i in processed_indices:
                continue
                
            # 查找与当前节点距离小于阈值的所有节点
            conflict_group = [membership]
            conflict_indices = [i]
            
            for j, other_membership in enumerate(all_node_memberships):
                if j <= i or j in processed_indices:
                    continue
                
                # 检查是否来自不同路径
                if membership.path_id == other_membership.path_id:
                    continue
                
                distance = self._calculate_distance(membership.position, other_membership.position)
                if distance <= self.merge_radius:
                    conflict_group.append(other_membership)
                    conflict_indices.append(j)
            
            if len(conflict_group) > 1:  # 至少2个不同路径的节点
                mergeable_groups.append(conflict_group)
                processed_indices.update(conflict_indices)
        
        print(f"   检测到可合并组: {len(mergeable_groups)}")
        for i, group in enumerate(mergeable_groups):
            paths = set(m.path_id for m in group)
            print(f"     组{i+1}: {len(group)}节点, 涉及路径: {list(paths)}")
        
        return mergeable_groups
    
    def _calculate_distance(self, pos1: Tuple, pos2: Tuple) -> float:
        """计算两点距离"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

class NodeInheritanceProcessor:
    """节点继承处理器"""
    
    def __init__(self):
        self.inherited_nodes = {}
        
    def create_inherited_nodes(self, mergeable_groups: List[List[NodePathMembership]]) -> Dict[str, InheritedNode]:
        """创建继承节点"""
        print(f"\n🧬 [节点继承] 开始创建继承节点")
        
        inherited_nodes = {}
        
        for group_id, group in enumerate(mergeable_groups):
            # 计算质心位置作为继承节点位置（更安全）
            inherit_position = self._calculate_safe_centroid(group)
            
            # 创建继承节点
            inherited_node = InheritedNode(
                node_id=f"inherited_{group_id}",
                position=inherit_position
            )
            
            # 继承所有成员信息
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
    
    def _calculate_safe_centroid(self, group: List[NodePathMembership]) -> Tuple[float, float, float]:
        """计算安全质心位置"""
        # 计算所有节点的质心
        x_sum = sum(m.position[0] for m in group)
        y_sum = sum(m.position[1] for m in group)
        z_sum = sum(m.position[2] if len(m.position) > 2 else 0 for m in group)
        
        count = len(group)
        centroid = (x_sum / count, y_sum / count, z_sum / count)
        
        # 验证质心是否安全，如果不安全则选择最近的安全节点
        if self._is_position_safe(centroid):
            return centroid
        else:
            # 选择距离质心最近的安全原始节点
            min_distance = float('inf')
            safe_position = group[0].position  # 默认第一个
            
            for membership in group:
                if self._is_position_safe(membership.position):
                    distance = math.sqrt(
                        (membership.position[0] - centroid[0])**2 + 
                        (membership.position[1] - centroid[1])**2
                    )
                    if distance < min_distance:
                        min_distance = distance
                        safe_position = membership.position
            
            return safe_position
    
    def _is_position_safe(self, position: Tuple) -> bool:
        """检查位置是否安全（简单版本，子类可重写）"""
        # 基础检查：确保位置在合理范围内
        x, y = position[0], position[1]
        return not (x < 0 or y < 0 or x > 1000 or y > 1000)  # 假设环境不超过1000x1000

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
    """路径重拟合器"""
    
    def __init__(self, env):
        self.env = env
        self.vehicle_width = 3.0
        self.safety_margin = 1.5
        
    def refit_inherited_paths(self, rebuilt_paths: Dict, inherited_nodes: Dict) -> Dict[str, Any]:
        """重拟合继承后的路径"""
        print(f"\n🎯 [路径拟合] 开始重拟合继承路径")
        
        refitted_paths = {}
        
        for path_id, path_obj in rebuilt_paths.items():
            print(f"   拟合路径: {path_id}")
            
            # 识别关键节点
            key_nodes = self._identify_key_nodes(path_obj, inherited_nodes)
            
            # 路径拟合
            fitted_path = self._fit_path_through_keypoints(path_obj.forward_path, key_nodes)
            
            if fitted_path:
                # 创建拟合后的路径对象
                refitted_path = self._create_refitted_path(path_id, path_obj, fitted_path, key_nodes)
                refitted_paths[path_id] = refitted_path
                print(f"     ✅ 拟合成功: {len(fitted_path)}点, {len(key_nodes)}关键节点")
            else:
                # 保持原路径
                refitted_paths[path_id] = path_obj
                print(f"     ⚠️ 拟合失败，保持原路径")
        
        return refitted_paths
    
    def _identify_key_nodes(self, path_obj, inherited_nodes: Dict) -> List[Tuple]:
        """识别路径上的关键节点"""
        key_nodes = []
        
        # 起点
        key_nodes.append(path_obj.forward_path[0])
        
        # 继承节点
        for inherited_node in inherited_nodes.values():
            for membership in inherited_node.memberships:
                if membership.path_id == path_obj.path_id:
                    key_nodes.append(inherited_node.position)
                    break
        
        # 终点
        key_nodes.append(path_obj.forward_path[-1])
        
        return key_nodes
    
    def _fit_path_through_keypoints(self, original_path: List, key_nodes: List) -> Optional[List]:
        """通过关键节点拟合路径 - 安全优先版本"""
        if len(key_nodes) < 2:
            return original_path
        
        # 首先尝试保守的线性插值
        linear_path = self._linear_interpolate_keypoints(key_nodes)
        if linear_path and self._validate_fitted_path(linear_path):
            print(f"     使用安全线性插值")
            return linear_path
        
        # 如果线性插值失败，尝试样条拟合
        if SCIPY_AVAILABLE and len(key_nodes) >= 3:
            spline_path = self._safe_spline_fit_keypoints(key_nodes)
            if spline_path and self._validate_fitted_path(spline_path):
                print(f"     使用安全样条拟合")
                return spline_path
        
        # 最后回退到原始路径的关键段
        print(f"     回退到原始路径段")
        return self._extract_safe_original_segments(original_path, key_nodes)
    
    def _safe_spline_fit_keypoints(self, key_nodes: List) -> Optional[List]:
        """安全的样条拟合关键节点"""
        try:
            # 验证关键节点都是安全的
            for node in key_nodes:
                if self._is_point_colliding(node):
                    print(f"     关键节点不安全，跳过样条拟合")
                    return None
            
            # 计算累积距离
            distances = [0.0]
            for i in range(1, len(key_nodes)):
                dist = math.sqrt((key_nodes[i][0] - key_nodes[i-1][0])**2 + 
                               (key_nodes[i][1] - key_nodes[i-1][1])**2)
                distances.append(distances[-1] + dist)
            
            x_coords = [kn[0] for kn in key_nodes]
            y_coords = [kn[1] for kn in key_nodes]
            
            # 样条插值
            cs_x = CubicSpline(distances, x_coords, bc_type='natural')
            cs_y = CubicSpline(distances, y_coords, bc_type='natural')
            
            # 生成密集路径，但步长更小以确保安全
            total_dist = distances[-1]
            num_points = max(20, int(total_dist / 1.0))  # 更密集的采样
            t_values = [i * total_dist / (num_points - 1) for i in range(num_points)]
            
            fitted_path = []
            for t in t_values:
                x = float(cs_x(t))
                y = float(cs_y(t))
                
                # 计算朝向
                if t < total_dist - 0.1:
                    dx = float(cs_x(t + 0.1, nu=1))
                    dy = float(cs_y(t + 0.1, nu=1))
                    theta = math.atan2(dy, dx)
                else:
                    theta = key_nodes[-1][2] if len(key_nodes[-1]) > 2 else 0
                
                new_point = (x, y, theta)
                
                # 实时检查每个生成的点
                if self._is_point_colliding(new_point):
                    print(f"     样条拟合产生不安全点，终止")
                    return None
                
                fitted_path.append(new_point)
            
            return fitted_path
            
        except Exception as e:
            print(f"     安全样条拟合失败: {e}")
            return None
    
    def _extract_safe_original_segments(self, original_path: List, key_nodes: List) -> List:
        """提取原始路径的安全段"""
        if not original_path or not key_nodes:
            return original_path
        
        # 找到原始路径中最接近关键节点的点
        key_indices = []
        for key_node in key_nodes:
            min_dist = float('inf')
            closest_idx = 0
            
            for i, orig_point in enumerate(original_path):
                dist = math.sqrt(
                    (orig_point[0] - key_node[0])**2 + 
                    (orig_point[1] - key_node[1])**2
                )
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
            
            key_indices.append(closest_idx)
        
        # 按索引排序
        key_indices.sort()
        
        # 提取路径段
        safe_path = []
        start_idx = key_indices[0]
        end_idx = key_indices[-1]
        
        for i in range(start_idx, end_idx + 1):
            safe_path.append(original_path[i])
        
        return safe_path if safe_path else original_path
    
    def _spline_fit_keypoints(self, key_nodes: List) -> Optional[List]:
        """样条拟合关键节点"""
        try:
            # 计算累积距离
            distances = [0.0]
            for i in range(1, len(key_nodes)):
                dist = math.sqrt((key_nodes[i][0] - key_nodes[i-1][0])**2 + 
                               (key_nodes[i][1] - key_nodes[i-1][1])**2)
                distances.append(distances[-1] + dist)
            
            x_coords = [kn[0] for kn in key_nodes]
            y_coords = [kn[1] for kn in key_nodes]
            
            # 样条插值
            cs_x = CubicSpline(distances, x_coords, bc_type='natural')
            cs_y = CubicSpline(distances, y_coords, bc_type='natural')
            
            # 生成密集路径
            total_dist = distances[-1]
            num_points = max(10, int(total_dist / 2.0))
            t_values = [i * total_dist / (num_points - 1) for i in range(num_points)]
            
            fitted_path = []
            for t in t_values:
                x = float(cs_x(t))
                y = float(cs_y(t))
                
                # 计算朝向
                if t < total_dist - 0.1:
                    dx = float(cs_x(t + 0.1, nu=1))
                    dy = float(cs_y(t + 0.1, nu=1))
                    theta = math.atan2(dy, dx)
                else:
                    theta = key_nodes[-1][2] if len(key_nodes[-1]) > 2 else 0
                
                fitted_path.append((x, y, theta))
            
            return fitted_path
            
        except Exception as e:
            print(f"     样条拟合失败: {e}")
            return None
    
    def _linear_interpolate_keypoints(self, key_nodes: List) -> List:
        """线性插值关键节点"""
        path = []
        
        for i in range(len(key_nodes) - 1):
            start = key_nodes[i]
            end = key_nodes[i + 1]
            
            distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            steps = max(3, int(distance / 2.0))
            
            for j in range(steps + 1):
                if i > 0 and j == 0:
                    continue
                
                t = j / steps
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                theta = start[2] if len(start) > 2 else 0
                
                path.append((x, y, theta))
        
        return path
    
    def _validate_fitted_path(self, path: List) -> bool:
        """验证拟合路径 - 增强安全检查"""
        if not path or len(path) < 2:
            return False
        
        # 密集采样检测 - 每个点都检查，不跳过
        for point in path:
            if self._is_point_colliding(point):
                return False
        
        # 额外检查路径段的中间点
        for i in range(len(path) - 1):
            # 检查每段路径的中点
            mid_x = (path[i][0] + path[i+1][0]) / 2
            mid_y = (path[i][1] + path[i+1][1]) / 2
            mid_theta = path[i][2] if len(path[i]) > 2 else 0
            mid_point = (mid_x, mid_y, mid_theta)
            
            if self._is_point_colliding(mid_point):
                return False
        
        return True
    
    def _is_point_colliding(self, point: Tuple) -> bool:
        """检查点碰撞 - 增强碰撞检测"""
        x, y = int(point[0]), int(point[1])
        
        # 增大安全区域检查
        safety_radius = int((self.vehicle_width + self.safety_margin * 2) / 2)
        
        for dx in range(-safety_radius, safety_radius + 1):
            for dy in range(-safety_radius, safety_radius + 1):
                check_x, check_y = x + dx, y + dy
                
                # 边界检查
                if (check_x < 0 or check_y < 0 or 
                    check_x >= self.env.width or check_y >= self.env.height):
                    return True  # 边界外视为碰撞
                
                # 障碍物检查
                if (hasattr(self.env, 'grid') and
                    self.env.grid[check_x, check_y] == 1):
                    return True
                
                # 如果有obstacle_points，也检查
                if hasattr(self.env, 'obstacle_points'):
                    for obs_x, obs_y in self.env.obstacle_points:
                        if abs(check_x - obs_x) <= 1 and abs(check_y - obs_y) <= 1:
                            return True
        
        return False
    
    def _create_refitted_path(self, path_id: str, original_path_obj, 
                            fitted_path: List, key_nodes: List):
        """创建重拟合路径对象"""
        class RefittedBackbonePath:
            def __init__(self, path_id, original_path_obj, fitted_path, key_nodes):
                self.path_id = path_id
                self.forward_path = fitted_path
                self.reverse_path = list(reversed(fitted_path))
                self.length = self._calculate_length(fitted_path)
                self.quality = original_path_obj.quality * 1.05  # 拟合提升质量
                self.planner_used = 'inheritance_spline_fitting'
                self.created_time = time.time()
                self.usage_count = original_path_obj.usage_count
                self.current_load = 0
                self.max_capacity = 5
                
                # 端点信息
                self.point_a = original_path_obj.point_a
                self.point_b = original_path_obj.point_b
                
                # 拟合信息
                self.key_nodes = key_nodes
                self.key_node_count = len(key_nodes)
                self.inherited_node_count = getattr(original_path_obj, 'inherited_node_count', 0)
                self.is_consolidated = True
                self.consolidation_level = "inheritance_fitted"
                self.original_path_ids = getattr(original_path_obj, 'original_path_ids', [path_id])
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
        
        return RefittedBackbonePath(path_id, original_path_obj, fitted_path, key_nodes)

class ImprovedBackboneNetworkConsolidator:
    """改进的骨干网络整合器 - 继承式"""
    
    def __init__(self, env, path_planner, config: Dict = None):
        self.env = env
        self.path_planner = path_planner
        
        # 配置
        self.config = {
            'merge_radius': 3.5,
            'preserve_original_backup': True,
            'enable_visualization': False
        }
        
        if config:
            self.config.update(config)
        
        # 组件
        self.detector = NodeInheritanceDetector(self.config['merge_radius'])
        self.processor = NodeInheritanceProcessor()
        self.applicator = PathInheritanceApplicator()
        self.refitter = PathRefitter(env)
        
        # 结果存储
        self.inherited_nodes = {}
        self.original_paths_backup = {}
        self.consolidation_stats = {}
        
        print(f"继承式骨干网络整合器初始化")
        print(f"  合并半径: {self.config['merge_radius']}m")
    
    def aggressive_consolidate_backbone_network(self, backbone_network):
            """多轮迭代继承式整合骨干网络"""
            print(f"\n🚀 开始多轮继承式骨干网络整合...")
            start_time = time.time()
            
            # 备份原始路径
            if self.config['preserve_original_backup']:
                self.original_paths_backup = backbone_network.bidirectional_paths.copy()
            
            original_count = len(backbone_network.bidirectional_paths)
            original_node_count = sum(len(p.forward_path) for p in backbone_network.bidirectional_paths.values())
            print(f"原始网络: {original_count} 条路径, {original_node_count} 个节点")
            
            try:
                # 多轮迭代整合
                current_paths = backbone_network.bidirectional_paths
                all_inherited_nodes = {}
                round_num = 0
                max_rounds = 5
                
                while round_num < max_rounds:
                    round_num += 1
                    print(f"\n🔄 第 {round_num} 轮整合:")
                    
                    # 检测可合并节点
                    mergeable_groups = self.detector.detect_mergeable_nodes(current_paths)
                    
                    if not mergeable_groups:
                        print(f"   第 {round_num} 轮无可合并节点，停止迭代")
                        break
                    
                    # 创建继承节点
                    round_inherited = self.processor.create_inherited_nodes(mergeable_groups)
                    
                    # 累积继承节点（添加轮次前缀避免冲突）
                    for node_id, node in round_inherited.items():
                        all_inherited_nodes[f"round{round_num}_{node_id}"] = node
                    
                    # 应用继承
                    current_paths = self.applicator.apply_inheritance_to_paths(
                        current_paths, round_inherited
                    )
                    
                    # 统计本轮效果
                    current_nodes = sum(len(p.forward_path) for p in current_paths.values())
                    print(f"   本轮继承节点: {len(round_inherited)}")
                    print(f"   当前总节点: {current_nodes}")
                    
                    # 如果改进很小，提前停止
                    if len(round_inherited) < 2:
                        print(f"   改进幅度小，提前停止")
                        break
                
                # 最终路径重拟合
                if all_inherited_nodes:
                    print(f"\n🎯 最终路径重拟合...")
                    final_paths = self.refitter.refit_inherited_paths(current_paths, all_inherited_nodes)
                else:
                    print("无继承节点，保持原路径")
                    final_paths = current_paths
                
                # 统计
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
                    'consolidation_time': consolidation_time
                }
                
                print(f"\n🎉 多轮继承式整合完成!")
                print(f"  整合轮数: {round_num}")
                print(f"  总继承节点: {len(all_inherited_nodes)}")
                print(f"  节点压缩: {original_node_count} → {final_node_count} ({total_reduction:.1%})")
                print(f"  整合耗时: {consolidation_time:.2f}s")
                
                self.inherited_nodes = all_inherited_nodes
                self.rebuilt_paths = final_paths
                return self._create_mock_topology(final_paths, all_inherited_nodes)
            
            except Exception as e:
                print(f"❌ 多轮继承式整合失败: {e}")
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
                'consolidation_type': 'node_inheritance',
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
        'merge_radius': cluster_radius,
        'preserve_original_backup': True
    }
    
    consolidator = ImprovedBackboneNetworkConsolidator(env, path_planner, config)
    topology_network = consolidator.aggressive_consolidate_backbone_network(backbone_network)
    
    if apply_immediately and topology_network:
        success = consolidator.apply_to_backbone_network(backbone_network)
        if success:
            print("✅ 继承式整合完成并已应用")
        else:
            print("❌ 应用继承结果失败")
    
    return consolidator

AggressiveBackboneNetworkConsolidator = ImprovedBackboneNetworkConsolidator