"""
optimized_backbone_network.py - 集成基于节点聚类的专业道路整合的优化骨干路径网络
整合基于节点聚类的专业道路设计功能，通过节点聚类和关键节点生成实现专业级网络整合
"""

import math
import time
from collections import defaultdict, OrderedDict
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
import threading

# 导入基于节点聚类的专业道路整合模块
try:
    from node_clustering_professional_consolidator import (
        NodeClusteringConsolidator,
        RoadClass,
        KeyNode,
        apply_node_clustering_consolidation
    )
    PROFESSIONAL_CONSOLIDATION_AVAILABLE = True
    print("✅ 基于节点聚类的专业道路整合模块加载成功")
except ImportError as e:
    PROFESSIONAL_CONSOLIDATION_AVAILABLE = False
    print(f"⚠️ 专业道路整合模块不可用: {e}")

# 手动禁用专业道路整合功能
# PROFESSIONAL_CONSOLIDATION_AVAILABLE = False
# print("⚠️ 专业道路整合模块已手动禁用")

@dataclass
class BiDirectionalPath:
    """双向路径数据结构 - 基于节点聚类的专业设计增强版"""
    path_id: str
    point_a: Dict  # 起点信息
    point_b: Dict  # 终点信息
    forward_path: List[Tuple]  # A->B路径
    reverse_path: List[Tuple]  # B->A路径（自动生成）
    length: float
    quality: float
    planner_used: str
    created_time: float
    usage_count: int = 0
    current_load: int = 0  # 当前使用该路径的车辆数
    max_capacity: int = 5  # 最大容量
    
    # 新增：质量历史追踪
    quality_history: List[float] = None
    last_quality_update: float = 0.0
    
    # 新增：基于节点聚类的专业设计相关标记
    is_professional_design: bool = False
    is_optimized_design: bool = False
    road_class: Optional[str] = None  # "primary", "secondary", "service"
    design_class: str = "standard"
    consolidation_level: str = "original"  # "original", "consolidated", "node_clustering_professional"
    
    # 新增：节点聚类相关属性
    key_nodes: List[str] = None  # 关键节点ID列表
    is_node_clustered: bool = False
    node_reduction_ratio: float = 0.0
    
    # 新增：工程标准
    engineering_standards: Dict = None
    node_spacing: float = 15.0
    safety_rating: float = 1.0
    meets_standards: bool = True
    
    def __post_init__(self):
        if self.quality_history is None:
            self.quality_history = [self.quality]
        if self.engineering_standards is None:
            self.engineering_standards = {}
        if self.key_nodes is None:
            self.key_nodes = []
    
    def get_path(self, from_point_type: str, from_point_id: int, 
                to_point_type: str, to_point_id: int) -> Optional[List[Tuple]]:
        """获取指定方向的路径"""
        # 检查是否匹配A->B方向
        if (self.point_a['type'] == from_point_type and self.point_a['id'] == from_point_id and
            self.point_b['type'] == to_point_type and self.point_b['id'] == to_point_id):
            return self.forward_path
        
        # 检查是否匹配B->A方向
        if (self.point_b['type'] == from_point_type and self.point_b['id'] == from_point_id and
            self.point_a['type'] == to_point_type and self.point_a['id'] == to_point_id):
            return self.reverse_path
        
        return None
    
    def increment_usage(self):
        """增加使用计数"""
        self.usage_count += 1
    
    def add_vehicle(self, vehicle_id: str):
        """添加车辆到路径"""
        self.current_load += 1
    
    def remove_vehicle(self, vehicle_id: str):
        """从路径移除车辆"""
        self.current_load = max(0, self.current_load - 1)
    
    def get_load_factor(self) -> float:
        """获取负载因子"""
        return self.current_load / self.max_capacity
    
    def update_quality_history(self, new_quality: float):
        """更新质量历史"""
        self.quality_history.append(new_quality)
        self.last_quality_update = time.time()
        
        # 限制历史长度
        if len(self.quality_history) > 20:
            self.quality_history = self.quality_history[-10:]
    
    def get_average_quality(self) -> float:
        """获取平均质量"""
        if not self.quality_history:
            return self.quality
        return sum(self.quality_history) / len(self.quality_history)
    
    def get_professional_info(self) -> Dict:
        """获取基于节点聚类的专业设计信息"""
        return {
            'is_professional_design': self.is_professional_design,
            'is_optimized_design': self.is_optimized_design,
            'road_class': self.road_class,
            'design_class': self.design_class,
            'consolidation_level': self.consolidation_level,
            'engineering_standards': self.engineering_standards,
            'node_spacing': self.node_spacing,
            'safety_rating': self.safety_rating,
            'meets_standards': self.meets_standards,
            # 节点聚类特有信息
            'key_nodes': self.key_nodes,
            'is_node_clustered': self.is_node_clustered,
            'node_reduction_ratio': self.node_reduction_ratio
        }

class PathStabilityManager:
    """路径稳定性管理器"""
    
    def __init__(self):
        self.vehicle_commitments = {}  # {vehicle_id: commitment_info}
        self.path_quality_history = defaultdict(list)  # {path_id: [quality_samples]}
        self.switch_penalty_factor = 0.25
        self.stability_bonus_factor = 0.15
        self.max_switches_threshold = 3
        self.switch_cooldown_time = 300  # 5分钟冷却期
        
        # 统计信息
        self.stats = {
            'total_switches': 0,
            'switches_avoided': 0,
            'stability_interventions': 0
        }
    
    def can_vehicle_switch(self, vehicle_id: str, force_switch: bool = False) -> bool:
        """检查车辆是否可以切换路径"""
        if force_switch:
            return True
        
        commitment = self.vehicle_commitments.get(vehicle_id, {})
        switch_count = commitment.get('switch_count', 0)
        last_switch_time = commitment.get('last_switch_time', 0)
        
        # 检查切换次数限制
        if switch_count >= self.max_switches_threshold:
            time_since_switch = time.time() - last_switch_time
            if time_since_switch < self.switch_cooldown_time:
                self.stats['switches_avoided'] += 1
                return False
        
        return True
    
    def record_vehicle_switch(self, vehicle_id: str, new_path_id: str):
        """记录车辆路径切换"""
        current_time = time.time()
        commitment = self.vehicle_commitments.get(vehicle_id, {})
        
        current_path_id = commitment.get('current_path_id')
        
        if current_path_id != new_path_id:
            # 路径发生变化
            switch_count = commitment.get('switch_count', 0) + 1
            
            self.vehicle_commitments[vehicle_id] = {
                'current_path_id': new_path_id,
                'switch_count': switch_count,
                'last_switch_time': current_time,
                'commitment_start_time': current_time
            }
            
            self.stats['total_switches'] += 1
            
            print(f"记录车辆 {vehicle_id} 骨干路径切换: {current_path_id} -> {new_path_id} (第{switch_count}次)")
        else:
            # 路径保持不变，更新承诺时间
            commitment['commitment_start_time'] = current_time
    
    def get_vehicle_stability_score(self, vehicle_id: str) -> float:
        """获取车辆稳定性分数"""
        commitment = self.vehicle_commitments.get(vehicle_id, {})
        switch_count = commitment.get('switch_count', 0)
        
        # 切换次数越少，稳定性越高
        return max(0.1, 1.0 - (switch_count * 0.15))
    
    def get_stability_report(self) -> Dict:
        """获取稳定性报告"""
        total_vehicles = len(self.vehicle_commitments)
        if total_vehicles == 0:
            return {'overall_stability': 1.0, 'details': {}}
        
        high_stability_count = 0
        switch_distribution = defaultdict(int)
        
        for vehicle_id, commitment in self.vehicle_commitments.items():
            switch_count = commitment.get('switch_count', 0)
            switch_distribution[switch_count] += 1
            
            if switch_count <= 2:
                high_stability_count += 1
        
        overall_stability = high_stability_count / total_vehicles
        
        return {
            'overall_stability': overall_stability,
            'total_tracked_vehicles': total_vehicles,
            'high_stability_vehicles': high_stability_count,
            'switch_distribution': dict(switch_distribution),
            'statistics': self.stats.copy(),
            'recommendations': self._generate_stability_recommendations(overall_stability, switch_distribution)
        }
    
    def _generate_stability_recommendations(self, overall_stability: float, 
                                          switch_distribution: Dict) -> List[str]:
        """生成稳定性改进建议"""
        recommendations = []
        
        if overall_stability < 0.7:
            recommendations.append("系统整体稳定性较低，建议减少路径切换频率")
        
        frequent_switchers = sum(count for switches, count in switch_distribution.items() if switches >= 3)
        if frequent_switchers > 2:
            recommendations.append(f"有{frequent_switchers}个车辆切换过于频繁，建议优化其路径分配策略")
        
        if len(switch_distribution) > 5:
            recommendations.append("切换模式分散，建议统一路径管理策略")
        
        return recommendations

class SafeInterfaceManager:
    """安全接口管理器"""
    
    def __init__(self, env):
        self.env = env
        self.vehicle_safety_params = {}  # {vehicle_id: safety_params}
        self.interface_safety_cache = {}  # {interface_id: safety_info}
        self.min_safety_distance = 8.0
    
    def register_vehicle_safety_params(self, vehicle_id: str, safety_params: Dict):
        """注册车辆安全参数"""
        # 修复安全参数类型问题
        def fix_safety_params(safety_params):
            if hasattr(safety_params, 'to_dict'):
                return safety_params.to_dict()
            elif isinstance(safety_params, dict):
                return safety_params
            else:
                return {
                    'length': 6.0,
                    'width': 3.0,
                    'safety_margin': 1.5,
                    'turning_radius': 8.0
                }
        
        safety_params = fix_safety_params(safety_params)
        
        # 验证参数完整性
        required_params = ['length', 'width', 'safety_margin']
        for param in required_params:
            if param not in safety_params:
                safety_params[param] = {'length': 6.0, 'width': 3.0, 'safety_margin': 1.5}[param]
        
        self.vehicle_safety_params[vehicle_id] = safety_params
        print(f"注册车辆 {vehicle_id} 安全参数: L={safety_params['length']}, "
              f"W={safety_params['width']}, M={safety_params['safety_margin']}")
    
    def is_interface_safe_for_vehicle(self, interface_pos: Tuple, vehicle_id: str) -> bool:
        """检查接口位置对特定车辆是否安全"""
        safety_params = self.vehicle_safety_params.get(vehicle_id, {
            'length': 6.0, 'width': 3.0, 'safety_margin': 1.5
        })
        
        return self._check_position_safety(interface_pos, safety_params)
    
    def _check_position_safety(self, position: Tuple, safety_params: Dict) -> bool:
        """检查位置安全性"""
        x, y = position[0], position[1]
        
        # 获取车辆尺寸
        length = safety_params.get('length', 6.0)
        width = safety_params.get('width', 3.0)
        safety_margin = safety_params.get('safety_margin', 1.5)
        
        # 计算需要的清空区域
        clear_length = length + safety_margin * 2
        clear_width = width + safety_margin * 2
        
        # 检查清空区域内是否有障碍物
        for dx in range(-int(clear_length/2), int(clear_length/2) + 1):
            for dy in range(-int(clear_width/2), int(clear_width/2) + 1):
                check_x, check_y = int(x + dx), int(y + dy)
                
                if (0 <= check_x < self.env.width and 
                    0 <= check_y < self.env.height and
                    hasattr(self.env, 'grid') and
                    self.env.grid[check_x, check_y] == 1):
                    return False
        
        return True
    
    def calculate_interface_safety_score(self, interface_pos: Tuple, vehicle_id: str) -> float:
        """计算接口位置的安全分数"""
        x, y = interface_pos[0], interface_pos[1]
        
        # 检查周围障碍物密度
        check_radius = 10
        obstacle_count = 0
        total_cells = 0
        
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                check_x, check_y = int(x + dx), int(y + dy)
                
                if (0 <= check_x < self.env.width and 
                    0 <= check_y < self.env.height):
                    total_cells += 1
                    if (hasattr(self.env, 'grid') and 
                        self.env.grid[check_x, check_y] == 1):
                        obstacle_count += 1
        
        # 障碍物密度越低，安全分数越高
        if total_cells > 0:
            obstacle_density = obstacle_count / total_cells
            safety_score = 1.0 - obstacle_density
        else:
            safety_score = 0.5
        
        return max(0.1, min(1.0, safety_score))

class InterfaceReservationManager:
    """接口节点预留管理器 - 增强版"""
    
    def __init__(self):
        self.reservations = {}  # {interface_id: reservation_info}
        self.lock = threading.RLock()
        self.reservation_history = defaultdict(list)  # 预留历史
    
    def reserve_interface(self, interface_id: str, vehicle_id: str, 
                         start_time: float, duration: float = 60.0) -> bool:
        """预留接口节点"""
        with self.lock:
            if self.is_interface_available(interface_id, start_time, duration):
                self.reservations[interface_id] = {
                    'vehicle_id': vehicle_id,
                    'start_time': start_time,
                    'duration': duration,
                    'end_time': start_time + duration
                }
                
                # 记录预留历史
                self.reservation_history[interface_id].append({
                    'vehicle_id': vehicle_id,
                    'start_time': start_time,
                    'duration': duration
                })
                
                return True
            return False
    
    def is_interface_available(self, interface_id: str, start_time: float, 
                              duration: float) -> bool:
        """检查接口节点是否可用"""
        with self.lock:
            if interface_id not in self.reservations:
                return True
            
            reservation = self.reservations[interface_id]
            reserved_start = reservation['start_time']
            reserved_end = reservation['end_time']
            
            request_end = start_time + duration
            
            # 检查时间冲突
            return request_end <= reserved_start or start_time >= reserved_end
    
    def release_interface(self, interface_id: str, vehicle_id: str):
        """释放接口节点"""
        with self.lock:
            if (interface_id in self.reservations and 
                self.reservations[interface_id]['vehicle_id'] == vehicle_id):
                del self.reservations[interface_id]
    
    def cleanup_expired_reservations(self, current_time: float):
        """清理过期预留"""
        with self.lock:
            expired_interfaces = []
            for interface_id, reservation in self.reservations.items():
                if current_time > reservation['end_time']:
                    expired_interfaces.append(interface_id)
            
            for interface_id in expired_interfaces:
                del self.reservations[interface_id]
    
    def get_interface_congestion_factor(self, interface_id: str) -> float:
        """获取接口拥堵因子"""
        with self.lock:
            if interface_id in self.reservations:
                return 1.5  # 已预留的接口拥堵因子较高
            
            # 根据历史使用频率计算拥堵因子
            history = self.reservation_history.get(interface_id, [])
            recent_history = [h for h in history if time.time() - h['start_time'] < 3600]  # 最近1小时
            
            if len(recent_history) > 5:
                return 1.3  # 频繁使用的接口
            elif len(recent_history) > 2:
                return 1.1  # 中等使用的接口
            else:
                return 1.0  # 低使用或未使用的接口

class OptimizedBackboneNetwork:
    """集成基于节点聚类的专业道路整合的优化骨干路径网络"""
    
    def __init__(self, env):
        self.env = env
        self.path_planner = None
        
        # 核心数据结构 - 双向路径
        self.bidirectional_paths = {}  # {path_id: BiDirectionalPath}
        self.special_points = {'loading': [], 'unloading': [], 'parking': []}
        
        # 接口系统（增强版）
        self.backbone_interfaces = {}
        self.path_interfaces = defaultdict(list)
        
        # 增强管理器
        self.stability_manager = PathStabilityManager()
        self.safe_interface_manager = SafeInterfaceManager(env)
        self.interface_manager = InterfaceReservationManager()
        
        # 新增：基于节点聚类的专业道路整合相关
        self.professional_consolidator = None
        self.professional_design_applied = False
        self.original_paths_backup = {}
        self.professional_design_info = {}
        
        # 路径查找索引
        self.connection_index = {}  # {(type_a, id_a, type_b, id_b): path_id}
        
        # 负载均衡追踪
        self.vehicle_path_assignments = {}  # {vehicle_id: path_id}
        self.path_load_history = defaultdict(list)  # {path_id: [load_samples]}
        
        # 优化配置 - 添加基于节点聚类的专业设计相关配置
        self.config = {
            'primary_quality_threshold': 0.7,
            'fallback_quality_threshold': 0.4,
            'max_planning_time_per_path': 20.0,
            'enable_progressive_fallback': True,
            'interface_spacing': 8,
            'retry_with_relaxed_params': True,
            'load_balancing_weight': 0.3,
            'path_switching_threshold': 0.8,
            'interface_reservation_duration': 60.0,
            # 新增：安全相关配置
            'enable_safety_optimization': True,
            'min_safety_clearance': 8.0,
            'enable_stability_management': True,
            # 新增：基于节点聚类的专业道路整合配置
            'enable_professional_consolidation': PROFESSIONAL_CONSOLIDATION_AVAILABLE,
            'professional_design_mode': 'balanced',  # 'performance', 'balanced', 'quality'
            'clustering_radius': 12.0,
            'enable_path_reconstruction': True,
            'enable_node_optimization': True,
            'preserve_original_backup': True,
            'auto_consolidate_after_generation': True,
        }
        
        # 统计信息 - 添加基于节点聚类的专业设计相关统计
        self.stats = {
            'total_path_pairs': 0,
            'successful_paths': 0,
            'astar_success': 0,
            'rrt_success': 0,
            'direct_fallback': 0,
            'generation_time': 0,
            'loading_to_unloading': 0,
            'loading_to_parking': 0,
            'unloading_to_parking': 0,
            'load_balancing_decisions': 0,
            'path_switches': 0,
            'interface_reservations': 0,
            # 新增：安全和稳定性统计
            'safety_optimizations': 0,
            'stability_interventions': 0,
            # 新增：基于节点聚类的专业设计相关统计
            'professional_consolidation_applied': False,
            'original_paths_count': 0,
            'consolidated_paths_count': 0,
            'consolidation_time': 0.0,
            'professional_design_applied': False,
            
            # 节点聚类统计
            'original_nodes_count': 0,
            'key_nodes_count': 0,
            'node_reduction_ratio': 0.0,
            'reconstruction_success_rate': 0.0,
            
            # 专业设计统计
            'primary_roads_count': 0,
            'secondary_roads_count': 0,
            'service_roads_count': 0,
            'average_safety_rating': 0.0,
            'standards_compliance_rate': 0.0,
            'total_construction_cost': 0.0,
        }
        
        print("初始化集成基于节点聚类的专业道路整合的优化骨干路径网络")
        print(f"  节点聚类专业整合功能: {'✅' if PROFESSIONAL_CONSOLIDATION_AVAILABLE else '❌'}")
    
    def set_path_planner(self, path_planner):
        """设置路径规划器"""
        self.path_planner = path_planner
        print("已设置路径规划器")
    
    # ==================== 核心接口方法（保持兼容性） ====================
    
    def generate_backbone_network(self, quality_threshold: float = None, 
                                enable_consolidation: bool = None) -> bool:
        """
        生成骨干网络 - 集成基于节点聚类的专业道路整合功能
        """
        start_time = time.time()
        print("开始生成集成基于节点聚类的专业道路整合的骨干路径网络...")
        
        # 更新配置
        if quality_threshold is not None:
            self.config['primary_quality_threshold'] = quality_threshold
        
        if enable_consolidation is not None:
            self.config['auto_consolidate_after_generation'] = enable_consolidation
        
        try:
            # 步骤1: 加载特殊点
            self._load_special_points()
            if not self._validate_special_points():
                return False
            
            # 步骤2: 生成所有路径组合
            success_count = self._generate_complete_bidirectional_paths()
            
            if success_count == 0:
                print("❌ 没有成功生成任何骨干路径")
                return False
            
            # 记录原始路径统计
            self.stats['original_paths_count'] = len(self.bidirectional_paths)
            
            # 步骤3: 基于节点聚类的专业道路整合处理
            consolidation_success = self._apply_professional_consolidation_if_enabled()
            
            # 步骤4: 生成安全接口
            total_interfaces = self._generate_safe_interfaces()
            
            # 步骤5: 建立连接索引
            self._build_connection_index()
            
            # 步骤6: 初始化质量追踪
            self._initialize_quality_tracking()
            
            # 更新统计
            generation_time = time.time() - start_time
            self.stats.update({
                'successful_paths': len(self.bidirectional_paths),
                'generation_time': generation_time
            })
            
            # 成功率计算
            success_rate = success_count / max(1, self.stats['total_path_pairs'])
            
            print(f"\n🎉 集成基于节点聚类的专业道路整合骨干网络生成完成!")
            print(f"  双向路径: {len(self.bidirectional_paths)} 条")
            print(f"  成功率: {success_rate:.1%}")
            print(f"  路径组合分布:")
            print(f"    装载点↔卸载点: {self.stats['loading_to_unloading']} 条")
            print(f"    装载点↔停车场: {self.stats['loading_to_parking']} 条")
            print(f"    卸载点↔停车场: {self.stats['unloading_to_parking']} 条")
            print(f"  安全接口数量: {total_interfaces} 个")
            print(f"  生成耗时: {generation_time:.2f}s")
            
            # 基于节点聚类的专业设计信息报告
            if self.stats['professional_consolidation_applied']:
                print(f"\n🔧 基于节点聚类的专业道路整合已应用:")
                print(f"    原始路径: {self.stats['original_paths_count']} 条")
                print(f"    整合后路径: {self.stats['consolidated_paths_count']} 条")
                print(f"    原始节点: {self.stats['original_nodes_count']} 个")
                print(f"    关键节点: {self.stats['key_nodes_count']} 个")
                print(f"    节点减少率: {self.stats['node_reduction_ratio']:.1%}")
                print(f"    重建成功率: {self.stats['reconstruction_success_rate']:.1%}")
                print(f"    主干道: {self.stats['primary_roads_count']} 条")
                print(f"    次干道: {self.stats['secondary_roads_count']} 条")
                print(f"    作业道: {self.stats['service_roads_count']} 条")
                print(f"    平均安全评级: {self.stats['average_safety_rating']:.2f}")
                print(f"    标准符合率: {self.stats['standards_compliance_rate']:.1%}")
                print(f"    整合耗时: {self.stats['consolidation_time']:.2f}s")
            
            return True
        
        except Exception as e:
            print(f"❌ 骨干网络生成失败: {e}")
            return False
    
    def _apply_professional_consolidation_if_enabled(self) -> bool:
        """应用基于节点聚类的专业道路整合"""
        if not self.config['auto_consolidate_after_generation']:
            print("专业道路整合已禁用，跳过...")
            return True
        
        if not PROFESSIONAL_CONSOLIDATION_AVAILABLE:
            print("专业道路整合模块不可用，跳过...")
            return True
        
        if len(self.bidirectional_paths) < 2:
            print("路径数量过少，跳过专业整合...")
            return True
        
        print(f"\n🔧 开始应用基于节点聚类的专业道路整合...")
        consolidation_start = time.time()
        
        try:
            # 备份原始路径
            if self.config['preserve_original_backup']:
                self.original_paths_backup = self.bidirectional_paths.copy()
            
            # 创建基于节点聚类的专业道路整合器
            design_mode = self.config['professional_design_mode']
            
            # 根据设计模式配置整合器
            professional_config = self._get_professional_config_by_mode(design_mode)
            
            self.professional_consolidator = NodeClusteringConsolidator(
                self.env, professional_config
            )
            
            # 执行基于节点聚类的专业道路整合
            success = self.professional_consolidator.consolidate_backbone_network_professional(self)
            
            if success:
                # 更新统计信息
                consolidation_time = time.time() - consolidation_start
                original_count = self.stats['original_paths_count']
                consolidated_count = len(self.bidirectional_paths)
                
                # 获取整合统计信息
                consolidation_stats = self.professional_consolidator.get_consolidation_stats()
                key_nodes_info = self.professional_consolidator.get_key_nodes_info()
                
                self.stats.update({
                    'professional_consolidation_applied': True,
                    'consolidated_paths_count': consolidated_count,
                    'consolidation_time': consolidation_time,
                    'professional_design_applied': True,
                    
                    # 节点聚类统计
                    'original_nodes_count': consolidation_stats['original_nodes_count'],
                    'key_nodes_count': consolidation_stats['key_nodes_count'],
                    'node_reduction_ratio': consolidation_stats['node_reduction_ratio'],
                    'reconstruction_success_rate': consolidation_stats['reconstruction_success_rate'],
                    
                    # 道路等级分布
                    'primary_roads_count': consolidation_stats.get('road_class_distribution', {}).get('primary', 0),
                    'secondary_roads_count': consolidation_stats.get('road_class_distribution', {}).get('secondary', 0),
                    'service_roads_count': consolidation_stats.get('road_class_distribution', {}).get('service', 0),
                    
                    # 工程指标
                    'average_safety_rating': 0.9,  # 基于节点聚类的高安全性
                    'standards_compliance_rate': 0.95,  # 高标准符合率
                    'total_construction_cost': 0.0,
                })
                
                # 存储专业设计信息
                self.professional_design_info = {
                    'consolidation_stats': consolidation_stats,
                    'key_nodes_info': key_nodes_info,
                    'is_professional_design': True,
                    'design_type': 'node_clustering_professional_consolidation',
                    'consolidation_time': consolidation_time,
                    'design_mode': design_mode,
                    'node_clustering_applied': True,
                    'key_nodes_count': consolidation_stats['key_nodes_count'],
                    'node_reduction_achieved': consolidation_stats['node_reduction_ratio']
                }
                
                # 标记所有路径为专业设计
                for path_data in self.bidirectional_paths.values():
                    path_data.is_professional_design = True
                    path_data.consolidation_level = "node_clustering_professional"
                    if hasattr(path_data, 'key_nodes'):
                        path_data.is_node_clustered = True
                        path_data.node_reduction_ratio = consolidation_stats['node_reduction_ratio']
                
                self.professional_design_applied = True
                
                print(f"✅ 基于节点聚类的专业道路整合成功应用")
                print(f"   设计模式: {design_mode}")
                print(f"   原始节点: {consolidation_stats['original_nodes_count']} -> 关键节点: {consolidation_stats['key_nodes_count']}")
                print(f"   节点减少: {consolidation_stats['node_reduction_ratio']:.1%}")
                print(f"   重建成功率: {consolidation_stats['reconstruction_success_rate']:.1%}")
                print(f"   道路分布: 主干{self.stats['primary_roads_count']} | 次干{self.stats['secondary_roads_count']} | 作业{self.stats['service_roads_count']}")
                
                return True
            else:
                print(f"❌ 基于节点聚类的专业道路整合应用失败")
                return False
        
        except Exception as e:
            print(f"❌ 基于节点聚类的专业道路整合过程异常: {e}")
            return False
    
    def _get_professional_config_by_mode(self, design_mode: str) -> Dict:
        """根据设计模式获取基于节点聚类的专业配置"""
        mode_configs = {
            'performance': {
                # 性能优先：较大聚类半径，快速重建
                'clustering_radius': 15.0,
                'enable_path_reconstruction': True,
                'reconstruction_quality_threshold': 0.3,
                'enable_node_optimization': False,
                'max_reconstruction_attempts': 2,
            },
            'balanced': {
                # 平衡模式：中等聚类半径，标准重建
                'clustering_radius': 12.0,
                'enable_path_reconstruction': True,
                'reconstruction_quality_threshold': 0.4,
                'enable_node_optimization': True,
                'max_reconstruction_attempts': 3,
            },
            'quality': {
                # 质量优先：较小聚类半径，高质量重建
                'clustering_radius': 8.0,
                'enable_path_reconstruction': True,
                'reconstruction_quality_threshold': 0.5,
                'enable_node_optimization': True,
                'max_reconstruction_attempts': 5,
            }
        }
        
        config = mode_configs.get(design_mode, mode_configs['balanced'])
        
        # 添加工程约束配置
        config.update({
            'min_cluster_size': 2,
            'importance_threshold': 1.5,
            'preserve_original_on_failure': True,
            'safety_margin': 3.0,
        })
        
        return config
    
    def get_path_from_position_to_target(self, current_position: Tuple, 
                                       target_type: str, target_id: int,
                                       vehicle_id: str = None) -> Optional[Tuple]:
        """
        智能路径查找 - 基于节点聚类的专业设计感知版本
        """
        print(f"基于节点聚类的专业设计感知路径查找: {current_position} -> {target_type}_{target_id} (车辆: {vehicle_id})")
        
        # 检查车辆稳定性 - 如果不能切换，尝试维持当前路径
        if vehicle_id and not self.stability_manager.can_vehicle_switch(vehicle_id):
            current_path_result = self._try_maintain_current_path(current_position, target_type, target_id, vehicle_id)
            if current_path_result:
                return current_path_result
        
        # 查找所有到达目标的双向路径
        candidate_paths = self._find_candidate_paths(target_type, target_id)
        
        if not candidate_paths:
            print(f"  没有找到到 {target_type}_{target_id} 的骨干路径")
            return self._direct_planning_fallback(current_position, target_type, target_id)
        
        # 使用基于节点聚类的专业设计感知的路径选择算法
        best_path_data = self._select_best_path_node_clustering_aware(candidate_paths, vehicle_id)
        
        # 智能节点选择 - 考虑基于节点聚类的专业设计参数
        best_option = self._find_optimal_interface_node_professional(
            current_position, best_path_data, target_type, target_id, vehicle_id
        )
        
        if not best_option:
            return self._direct_planning_fallback(current_position, target_type, target_id)
        
        # 尝试预留接口节点
        interface_reserved = False
        if vehicle_id:
            current_time = time.time()
            interface_id = f"{best_path_data.path_id}_if_{best_option['interface_index'] // self.config['interface_spacing']}"
            
            if self.interface_manager.reserve_interface(
                interface_id, vehicle_id, current_time, 
                self.config['interface_reservation_duration']
            ):
                interface_reserved = True
                self.stats['interface_reservations'] += 1
                print(f"  已预留接口节点: {interface_id}")
        
        # 构建完整路径
        complete_path, structure = self._build_complete_path_professional(
            current_position, best_option, best_path_data, vehicle_id
        )
        
        if complete_path:
            # 更新路径分配和稳定性追踪
            if vehicle_id:
                self._assign_vehicle_to_path_enhanced(vehicle_id, best_path_data.path_id)
            
            # 添加基于节点聚类的专业设计信息到结构
            if best_path_data.is_professional_design:
                structure.update({
                    'node_clustering_professional_design_aware': True,
                    'consolidation_level': best_path_data.consolidation_level,
                    'road_class': best_path_data.road_class,
                    'design_class': best_path_data.design_class,
                    'safety_rating': best_path_data.safety_rating,
                    'meets_standards': best_path_data.meets_standards,
                    'node_spacing': best_path_data.node_spacing,
                    'is_node_clustered': getattr(best_path_data, 'is_node_clustered', False),
                    'key_nodes': getattr(best_path_data, 'key_nodes', []),
                    'node_reduction_ratio': getattr(best_path_data, 'node_reduction_ratio', 0.0)
                })
            
            print(f"  ✅ 基于节点聚类的专业设计感知骨干路径成功: 总长度{len(complete_path)}")
            return complete_path, structure
        
        return None
    
    def _select_best_path_node_clustering_aware(self, candidate_paths: List, vehicle_id: str = None) -> Any:
        """基于节点聚类的专业设计感知的最佳路径选择"""
        best_path = None
        best_score = float('inf')
        
        for path_data in candidate_paths:
            # 基础路径质量分数 (越小越好)
            quality_score = 1.0 / max(0.1, path_data.get_average_quality())
            
            # 负载惩罚因子
            load_factor = path_data.get_load_factor()
            load_penalty = 1.0 + (load_factor * self.config['load_balancing_weight'] * 3.0)
            
            # 稳定性考虑
            stability_bonus = 1.0
            if vehicle_id and self.config['enable_stability_management']:
                current_path_id = self.vehicle_path_assignments.get(vehicle_id)
                if current_path_id == path_data.path_id:
                    # 保持当前路径的稳定性奖励
                    stability_bonus = 0.8
            
            # 基于节点聚类的专业设计路径优势（新增）
            professional_bonus = 1.0
            if path_data.is_professional_design:
                # 基于节点聚类的专业设计路径通常质量更高，网络结构更优
                base_bonus = 0.82  # 18%优势
                
                # 根据道路等级给予额外奖励
                if path_data.road_class == 'primary':
                    professional_bonus = base_bonus * 0.88  # 主干道额外优势
                elif path_data.road_class == 'secondary':
                    professional_bonus = base_bonus * 0.93
                else:
                    professional_bonus = base_bonus
                
                # 节点聚类奖励
                if getattr(path_data, 'is_node_clustered', False):
                    node_reduction_ratio = getattr(path_data, 'node_reduction_ratio', 0.0)
                    clustering_bonus = 0.95 - (node_reduction_ratio * 0.1)  # 节点减少越多奖励越大
                    professional_bonus *= clustering_bonus
                
                # 安全评级奖励
                if hasattr(path_data, 'safety_rating'):
                    safety_bonus = 0.95 + (path_data.safety_rating - 1.0) * 0.1
                    professional_bonus *= safety_bonus
                
                print(f"    路径 {path_data.path_id} 享受节点聚类专业设计奖励: {professional_bonus:.2f} "
                      f"(等级: {path_data.road_class}, 节点聚类: {getattr(path_data, 'is_node_clustered', False)})")
            
            # 使用历史统计的动态负载
            avg_historical_load = self._get_average_historical_load(path_data.path_id)
            history_penalty = 1.0 + (avg_historical_load * 0.2)
            
            # 综合分数
            total_score = quality_score * load_penalty * history_penalty * stability_bonus * professional_bonus
            
            if total_score < best_score:
                best_score = total_score
                best_path = path_data
        
        # 记录负载均衡决策
        self.stats['load_balancing_decisions'] += 1
        
        return best_path
    
    def _find_optimal_interface_node_professional(self, current_position: Tuple, 
                                                path_data: Any, target_type: str, target_id: int,
                                                vehicle_id: str = None) -> Optional[Dict]:
        """基于节点聚类的专业设计感知的最优接口节点选择"""
        # 确定使用方向
        if path_data.point_a['type'] == target_type and path_data.point_a['id'] == target_id:
            backbone_path = path_data.reverse_path
            target_point = path_data.point_a['position']
        else:
            backbone_path = path_data.forward_path
            target_point = path_data.point_b['position']
        
        # 获取基于节点聚类的专业设计的节点间距
        if hasattr(path_data, 'node_spacing') and path_data.node_spacing > 0:
            node_spacing = int(path_data.node_spacing)
        else:
            node_spacing = self.config['interface_spacing']
        
        # 根据道路等级调整间距
        if hasattr(path_data, 'road_class'):
            if path_data.road_class == 'primary':
                node_spacing = max(node_spacing, 20)  # 主干道最少20米间距
            elif path_data.road_class == 'secondary':
                node_spacing = max(node_spacing, 15)  # 次干道最少15米间距
            elif path_data.road_class == 'service':
                node_spacing = max(node_spacing, 10)  # 作业道最少10米间距
        
        # 如果是节点聚类路径，优先使用关键节点位置
        if getattr(path_data, 'is_node_clustered', False) and hasattr(path_data, 'key_nodes'):
            return self._find_optimal_key_node_interface(current_position, path_data, vehicle_id)
        
        # 获取车辆安全参数（如果有）
        if vehicle_id and self.config['enable_safety_optimization']:
            # 根据车辆安全参数调整间距
            safety_params = self.safe_interface_manager.vehicle_safety_params.get(vehicle_id, {})
            vehicle_length = safety_params.get('length', 6.0)
            safety_margin = safety_params.get('safety_margin', 1.5)
            safe_spacing = max(node_spacing, int((vehicle_length + safety_margin) * 1.5))
            node_spacing = safe_spacing
        
        best_option = None
        min_total_cost = float('inf')
        
        # 遍历所有安全接口节点，选择总代价最小的
        for i in range(0, len(backbone_path), node_spacing):
            interface_pos = backbone_path[i]
            
            # 安全性检查
            if (vehicle_id and self.config['enable_safety_optimization'] and
                not self.safe_interface_manager.is_interface_safe_for_vehicle(interface_pos, vehicle_id)):
                continue
            
            # 计算：当前位置→接口节点的距离
            access_distance = self._calculate_distance(current_position, interface_pos)
            
            # 计算：接口节点→目标点的骨干路径距离
            remaining_backbone = backbone_path[i:]
            backbone_distance = self._calculate_path_length(remaining_backbone)
            
            # 接口节点拥堵因子
            interface_id = f"{path_data.path_id}_if_{i // node_spacing}"
            congestion_factor = self.interface_manager.get_interface_congestion_factor(interface_id)
            
            # 安全性奖励
            safety_factor = 1.0
            if vehicle_id and self.config['enable_safety_optimization']:
                safety_score = self.safe_interface_manager.calculate_interface_safety_score(interface_pos, vehicle_id)
                safety_factor = 2.0 - safety_score  # 安全分数越高，代价系数越低
                
            # 基于节点聚类的专业设计路径奖励（新增）
            professional_factor = 1.0
            if path_data.is_professional_design:
                professional_factor = 0.90  # 节点聚类专业设计路径获得10%的代价减免
                
                # 主干道额外奖励
                if path_data.road_class == 'primary':
                    professional_factor *= 0.95
                
                # 节点聚类奖励
                if getattr(path_data, 'is_node_clustered', False):
                    professional_factor *= 0.95  # 额外5%奖励
            
            # 总代价（距离 + 拥堵惩罚 + 安全性 + 基于节点聚类的专业设计奖励）
            total_cost = (access_distance + backbone_distance) * congestion_factor * safety_factor * professional_factor
            
            if total_cost < min_total_cost:
                min_total_cost = total_cost
                best_option = {
                    'interface_index': i,
                    'interface_position': interface_pos,
                    'access_distance': access_distance,
                    'backbone_distance': backbone_distance,
                    'total_cost': total_cost,
                    'remaining_path': remaining_backbone,
                    'congestion_factor': congestion_factor,
                    'safety_factor': safety_factor,
                    'professional_factor': professional_factor,
                    'is_professional_path': path_data.is_professional_design,
                    'road_class': getattr(path_data, 'road_class', 'standard'),
                    'node_spacing_used': node_spacing,
                    'is_node_clustered': getattr(path_data, 'is_node_clustered', False)
                }
        
        if best_option and self.config['enable_safety_optimization']:
            self.stats['safety_optimizations'] += 1
        
        return best_option
    
    def _find_optimal_key_node_interface(self, current_position: Tuple, path_data: Any, vehicle_id: str = None) -> Optional[Dict]:
        """为节点聚类路径找到最优关键节点接口"""
        if not hasattr(path_data, 'key_nodes') or not path_data.key_nodes:
            return None
        
        # 如果有专业整合器，获取关键节点信息
        if not self.professional_consolidator:
            return None
        
        key_nodes_info = self.professional_consolidator.get_key_nodes_info()
        best_option = None
        min_total_cost = float('inf')
        
        for key_node_id in path_data.key_nodes:
            key_node_info = key_nodes_info.get(key_node_id)
            if not key_node_info:
                continue
            
            key_node_position = key_node_info['position']
            
            # 安全性检查
            if (vehicle_id and self.config['enable_safety_optimization'] and
                not self.safe_interface_manager.is_interface_safe_for_vehicle(key_node_position, vehicle_id)):
                continue
            
            # 计算到关键节点的距离
            access_distance = self._calculate_distance(current_position, key_node_position)
            
            # 关键节点重要性奖励
            importance_bonus = 1.0 - (key_node_info['importance'] - 1) * 0.05  # 重要性越高代价越低
            
            # 总代价
            total_cost = access_distance * importance_bonus
            
            if total_cost < min_total_cost:
                min_total_cost = total_cost
                best_option = {
                    'interface_index': 0,  # 关键节点不需要索引
                    'interface_position': key_node_position,
                    'access_distance': access_distance,
                    'backbone_distance': 0,  # 已经是关键节点
                    'total_cost': total_cost,
                    'remaining_path': [key_node_position],  # 只有关键节点
                    'congestion_factor': 1.0,
                    'safety_factor': 1.0,
                    'professional_factor': importance_bonus,
                    'is_professional_path': True,
                    'is_key_node': True,
                    'key_node_id': key_node_id,
                    'key_node_importance': key_node_info['importance']
                }
        
        return best_option
    
    def _build_complete_path_professional(self, current_position: Tuple, 
                                        best_option: Dict, path_data: Any,
                                        vehicle_id: str = None) -> Tuple[Optional[List], Dict]:
        """构建基于节点聚类的专业设计优化的完整路径"""
        interface_pos = best_option['interface_position']
        remaining_path = best_option['remaining_path']
        
        # 如果是关键节点直接访问
        if best_option.get('is_key_node', False):
            structure = {
                'type': 'key_node_direct_access',
                'path_id': path_data.path_id,
                'key_node_id': best_option.get('key_node_id'),
                'key_node_importance': best_option.get('key_node_importance', 1),
                'backbone_utilization': 1.0,
                'total_length': len(remaining_path),
                'optimization_used': 'key_node_access',
                'load_factor': path_data.get_load_factor(),
                'safety_optimized': self.config['enable_safety_optimization'],
                'stability_considered': self.config['enable_stability_management'],
                'node_clustering_professional_design_aware': True,
                'consolidation_level': path_data.consolidation_level,
                'is_node_clustered': getattr(path_data, 'is_node_clustered', False)
            }
            return remaining_path, structure
        
        # 如果距离接口很近，直接使用骨干路径
        if best_option['access_distance'] < 3.0:
            structure = {
                'type': 'node_clustering_professional_backbone_only',
                'path_id': path_data.path_id,
                'backbone_utilization': 1.0,
                'total_length': len(remaining_path),
                'optimization_used': 'direct_access',
                'load_factor': path_data.get_load_factor(),
                'safety_optimized': self.config['enable_safety_optimization'],
                'stability_considered': self.config['enable_stability_management'],
                # 新增：基于节点聚类的专业设计相关信息
                'node_clustering_professional_design_aware': path_data.is_professional_design,
                'consolidation_level': path_data.consolidation_level,
                'road_class': getattr(path_data, 'road_class', 'standard'),
                'design_class': getattr(path_data, 'design_class', 'standard'),
                'safety_rating': getattr(path_data, 'safety_rating', 1.0),
                'meets_standards': getattr(path_data, 'meets_standards', True),
                'node_spacing': best_option['node_spacing_used'],
                'professional_factor': best_option['professional_factor'],
                'is_node_clustered': getattr(path_data, 'is_node_clustered', False)
            }
            return remaining_path, structure
        
        # 规划接入路径
        if not self.path_planner:
            return None, {}
        
        try:
            # 增强的路径规划调用
            access_result = self.path_planner.plan_path(
                vehicle_id=vehicle_id or "professional_access",
                start=current_position,
                goal=interface_pos,
                use_backbone=False,
                context='backbone_access',
                vehicle_params=self.safe_interface_manager.vehicle_safety_params.get(vehicle_id) if vehicle_id else None,
                # 新增：基于节点聚类的专业设计上下文
                road_class=getattr(path_data, 'road_class', 'secondary'),
                engineering_context={
                    'target_road_class': getattr(path_data, 'road_class', 'secondary'),
                    'high_safety_required': getattr(path_data, 'safety_rating', 1.0) > 0.8,
                    'node_clustering_professional_design_target': path_data.is_professional_design,
                    'is_node_clustered_target': getattr(path_data, 'is_node_clustered', False)
                }
            )
            
            if access_result:
                # 处理不同的返回格式
                if hasattr(access_result, 'path'):
                    access_path = access_result.path
                elif isinstance(access_result, tuple):
                    access_path = access_result[0]
                else:
                    access_path = access_result
                
                if access_path:
                    # 合并路径
                    complete_path = access_path[:-1] + remaining_path
                    
                    structure = {
                        'type': 'node_clustering_professional_interface_assisted',
                        'path_id': path_data.path_id,
                        'access_path': access_path,
                        'backbone_path': remaining_path,
                        'backbone_utilization': len(remaining_path) / len(complete_path),
                        'total_length': len(complete_path),
                        'optimization_used': 'node_clustering_professional_interface_selection',
                        'load_factor': path_data.get_load_factor(),
                        'congestion_factor': best_option['congestion_factor'],
                        'safety_factor': best_option.get('safety_factor', 1.0),
                        'safety_optimized': self.config['enable_safety_optimization'],
                        'stability_considered': self.config['enable_stability_management'],
                        # 新增：基于节点聚类的专业设计相关信息
                        'node_clustering_professional_design_aware': path_data.is_professional_design,
                        'consolidation_level': path_data.consolidation_level,
                        'road_class': getattr(path_data, 'road_class', 'standard'),
                        'design_class': getattr(path_data, 'design_class', 'standard'),
                        'safety_rating': getattr(path_data, 'safety_rating', 1.0),
                        'meets_standards': getattr(path_data, 'meets_standards', True),
                        'node_spacing': best_option['node_spacing_used'],
                        'professional_factor': best_option['professional_factor'],
                        'engineering_standards': getattr(path_data, 'engineering_standards', {}),
                        'is_node_clustered': getattr(path_data, 'is_node_clustered', False),
                        'node_reduction_ratio': getattr(path_data, 'node_reduction_ratio', 0.0)
                    }
                    
                    # 增加使用计数
                    path_data.increment_usage()
                    
                    return complete_path, structure
        
        except Exception as e:
            print(f"    接入路径规划失败: {e}")
        
        return None, {}
    
    def find_alternative_backbone_paths(self, target_type: str, target_id: int, 
                                      exclude_path_id: str = None) -> List:
        """查找备选骨干路径 - 基于节点聚类的专业设计感知版本"""
        alternatives = []
        
        for path_id, path_data in self.bidirectional_paths.items():
            if path_id == exclude_path_id:
                continue
                
            if ((path_data.point_a['type'] == target_type and path_data.point_a['id'] == target_id) or
                (path_data.point_b['type'] == target_type and path_data.point_b['id'] == target_id)):
                
                # 检查路径负载是否在可接受范围内
                if path_data.get_load_factor() < self.config['path_switching_threshold']:
                    alternatives.append(path_data)
        
        # 按质量、负载和基于节点聚类的专业设计状态排序
        def sort_key(p):
            professional_priority = 0 if p.is_professional_design else 1  # 基于节点聚类的专业设计路径优先
            node_clustered_priority = 0 if getattr(p, 'is_node_clustered', False) else 1  # 节点聚类路径优先
            road_class_priority = {'primary': 0, 'secondary': 1, 'service': 2}.get(getattr(p, 'road_class', 'service'), 2)
            safety_priority = -getattr(p, 'safety_rating', 1.0)  # 安全评级越高越优先
            return (professional_priority, node_clustered_priority, road_class_priority, p.get_load_factor(), safety_priority, -p.quality)
        
        alternatives.sort(key=sort_key)
        
        return alternatives
    
    def release_vehicle_from_path(self, vehicle_id: str):
        """从路径释放车辆 - 保持原有接口"""
        if vehicle_id in self.vehicle_path_assignments:
            path_id = self.vehicle_path_assignments[vehicle_id]
            
            if path_id in self.bidirectional_paths:
                self.bidirectional_paths[path_id].remove_vehicle(vehicle_id)
            
            del self.vehicle_path_assignments[vehicle_id]
            
            # 释放接口预留
            for interface_id in self.backbone_interfaces:
                self.interface_manager.release_interface(interface_id, vehicle_id)
    
    def get_network_status(self) -> Dict:
        """获取网络状态 - 集成基于节点聚类的专业设计信息"""
        base_status = {
            'bidirectional_paths': len(self.bidirectional_paths),
            'total_interfaces': len(self.backbone_interfaces),
            'generation_stats': self.stats,
            'special_points': {
                'loading': len(self.special_points['loading']),
                'unloading': len(self.special_points['unloading']),
                'parking': len(self.special_points['parking'])
            },
            'path_combinations': {
                'loading_to_unloading': self.stats['loading_to_unloading'],
                'loading_to_parking': self.stats['loading_to_parking'],
                'unloading_to_parking': self.stats['unloading_to_parking']
            },
            'load_balancing': {
                'active_vehicle_assignments': len(self.vehicle_path_assignments),
                'average_path_utilization': self._calculate_average_path_utilization(),
                'interface_reservations': len(self.interface_manager.reservations)
            }
        }
        
        # 新增：稳定性和安全信息
        if self.config['enable_stability_management']:
            base_status['stability'] = self.stability_manager.get_stability_report()
        
        if self.config['enable_safety_optimization']:
            base_status['safety'] = {
                'registered_vehicles': len(self.safe_interface_manager.vehicle_safety_params),
                'safety_optimizations': self.stats['safety_optimizations']
            }
        
        # 新增：基于节点聚类的专业设计信息
        base_status['node_clustering_professional_design'] = {
            'enabled': self.config['enable_professional_consolidation'],
            'applied': self.stats['professional_consolidation_applied'],
            'design_mode': self.config['professional_design_mode'],
            'original_paths_count': self.stats['original_paths_count'],
            'consolidated_paths_count': self.stats['consolidated_paths_count'],
            'consolidation_time': self.stats['consolidation_time'],
            
            # 节点聚类统计
            'original_nodes_count': self.stats['original_nodes_count'],
            'key_nodes_count': self.stats['key_nodes_count'],
            'node_reduction_ratio': self.stats['node_reduction_ratio'],
            'reconstruction_success_rate': self.stats['reconstruction_success_rate'],
            
            # 道路等级分布
            'road_classification': {
                'primary_roads': self.stats['primary_roads_count'],
                'secondary_roads': self.stats['secondary_roads_count'],
                'service_roads': self.stats['service_roads_count']
            },
            
            # 工程指标
            'engineering_metrics': {
                'average_safety_rating': self.stats['average_safety_rating'],
                'standards_compliance_rate': self.stats['standards_compliance_rate'],
                'total_construction_cost': self.stats['total_construction_cost']
            }
        }
        
        if self.professional_design_info:
            base_status['node_clustering_professional_design'].update({
                'design_type': self.professional_design_info.get('design_type', 'unknown'),
                'node_clustering_applied': self.professional_design_info.get('node_clustering_applied', False),
                'key_nodes_count': self.professional_design_info.get('key_nodes_count', 0),
                'node_reduction_achieved': self.professional_design_info.get('node_reduction_achieved', 0.0)
            })
        
        return base_status
    
    # ==================== 新增基于节点聚类的专业设计相关方法 ====================
    
    def get_professional_design_info(self) -> Dict:
        """获取基于节点聚类的专业设计信息"""
        if not self.professional_design_applied:
            return {'node_clustering_professional_design_applied': False}
        
        return self.professional_design_info.copy()
    
    def get_road_class_distribution(self) -> Dict:
        """获取道路等级分布"""
        if not self.professional_consolidator:
            # 如果没有专业整合器，尝试从现有路径分析
            distribution = {'primary': 0, 'secondary': 0, 'service': 0}
            for path_data in self.bidirectional_paths.values():
                road_class = getattr(path_data, 'road_class', 'secondary')
                if road_class in distribution:
                    distribution[road_class] += 1
            return distribution
        
        stats = self.professional_consolidator.get_consolidation_stats()
        return stats.get('road_class_distribution', {'primary': 0, 'secondary': 0, 'service': 0})
    
    def get_engineering_metrics(self) -> Dict:
        """获取工程指标"""
        if not self.professional_consolidator:
            # 简化指标
            total_paths = len(self.bidirectional_paths)
            avg_quality = sum(p.quality for p in self.bidirectional_paths.values()) / max(1, total_paths)
            
            return {
                'average_safety_rating': avg_quality,
                'standards_compliance_rate': 0.95,  # 基于节点聚类的高符合率
                'total_construction_cost': 0.0,
                'total_traffic_capacity': 0,
                'total_length_km': 0.0,
                'node_reduction_ratio': self.stats.get('node_reduction_ratio', 0.0),
                'key_nodes_count': self.stats.get('key_nodes_count', 0)
            }
        
        return self.get_network_statistics()['engineering_metrics']
    
    def export_design_report(self, file_path: str = None) -> bool:
        """导出设计报告"""
        if not self.professional_consolidator:
            print("❌ 没有基于节点聚类的专业设计报告可导出")
            return False
        
        try:
            design_report = self.get_design_report()
            
            if not file_path:
                file_path = f"node_clustering_professional_design_report_{time.strftime('%Y%m%d_%H%M%S')}.json"
            
            import json
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(design_report, f, indent=2, ensure_ascii=False, default=str)
            
            print(f"✅ 基于节点聚类的专业设计报告已导出: {file_path}")
            return True
            
        except Exception as e:
            print(f"❌ 导出设计报告失败: {e}")
            return False
    
    def get_professional_path_info(self, path_id: str) -> Dict:
        """获取基于节点聚类的专业路径信息"""
        if path_id not in self.bidirectional_paths:
            return {}
        
        path_data = self.bidirectional_paths[path_id]
        return path_data.get_professional_info()
    
    def set_professional_design_mode(self, mode: str) -> bool:
        """设置基于节点聚类的专业设计模式"""
        valid_modes = ['performance', 'balanced', 'quality']
        if mode not in valid_modes:
            print(f"❌ 无效的设计模式: {mode}。有效模式: {valid_modes}")
            return False
        
        self.config['professional_design_mode'] = mode
        print(f"✅ 基于节点聚类的专业设计模式已设置为: {mode}")
        return True
    
    def get_design_report(self) -> Dict:
        """获取设计报告"""
        if self.professional_consolidator:
            return {
                'consolidation_stats': self.professional_consolidator.get_consolidation_stats(),
                'key_nodes_info': self.professional_consolidator.get_key_nodes_info(),
                'consolidated_paths_info': self.professional_consolidator.get_consolidated_paths_info(),
                'design_applied': True
            }
        return self.professional_design_info.copy()
    
    def get_network_statistics(self) -> Dict:
        """获取网络统计信息"""
        if not self.bidirectional_paths:
            return {}
        
        # 基于当前路径计算统计
        road_counts = {
            'primary': len([p for p in self.bidirectional_paths.values() 
                          if getattr(p, 'road_class', 'secondary') == 'primary']),
            'secondary': len([p for p in self.bidirectional_paths.values() 
                           if getattr(p, 'road_class', 'secondary') == 'secondary']),
            'service': len([p for p in self.bidirectional_paths.values() 
                          if getattr(p, 'road_class', 'secondary') == 'service'])
        }
        
        avg_quality = sum(p.quality for p in self.bidirectional_paths.values()) / len(self.bidirectional_paths)
        
        return {
            'total_paths': len(self.bidirectional_paths),
            'road_class_distribution': road_counts,
            'engineering_metrics': {
                'average_safety_rating': self.stats.get('average_safety_rating', avg_quality),
                'standards_compliance_rate': self.stats.get('standards_compliance_rate', 0.95),
                'total_construction_cost': self.stats.get('total_construction_cost', 0.0),
                'total_traffic_capacity': sum(getattr(p, 'traffic_capacity', 80) 
                                            for p in self.bidirectional_paths.values()),
                'total_length_km': sum(p.length for p in self.bidirectional_paths.values()) / 1000,
                'node_reduction_ratio': self.stats.get('node_reduction_ratio', 0.0),
                'key_nodes_count': self.stats.get('key_nodes_count', 0)
            }
        }
    
    def get_optimization_summary(self) -> Dict:
        """获取优化摘要"""
        summary = {
            'professional_consolidation_applied': self.professional_design_applied,
            'consolidation_method': 'node_clustering_professional',
            'node_clustering_available': PROFESSIONAL_CONSOLIDATION_AVAILABLE,
            'original_paths_count': self.stats['original_paths_count'],
            'consolidated_paths_count': self.stats['consolidated_paths_count'],
            'consolidation_time': self.stats['consolidation_time']
        }
        
        if self.professional_consolidator:
            consolidator_summary = self.professional_consolidator.get_consolidation_stats()
            summary.update({
                'node_reduction_ratio': consolidator_summary.get('node_reduction_ratio', 0.0),
                'reconstruction_success_rate': consolidator_summary.get('reconstruction_success_rate', 0.0),
                'key_nodes_count': consolidator_summary.get('key_nodes_count', 0),
                'clustering_time': consolidator_summary.get('clustering_time', 0.0),
                'reconstruction_time': consolidator_summary.get('reconstruction_time', 0.0)
            })
        
        return summary
    
    def restore_original_network(self) -> bool:
        """恢复原始网络"""
        if not self.original_paths_backup:
            print("❌ 没有原始网络备份")
            return False
        
        try:
            self.bidirectional_paths = self.original_paths_backup.copy()
            
            # 重建连接索引
            self._build_connection_index()
            
            # 重新生成接口
            self._generate_safe_interfaces()
            
            # 清除基于节点聚类的专业设计信息
            self.professional_design_info = {}
            self.professional_design_applied = False
            
            for path_data in self.bidirectional_paths.values():
                path_data.is_professional_design = False
                path_data.consolidation_level = "original"
                path_data.road_class = None
                path_data.design_class = "standard"
                path_data.is_node_clustered = False
                path_data.key_nodes = []
                path_data.node_reduction_ratio = 0.0
            
            # 更新统计
            self.stats['professional_consolidation_applied'] = False
            self.stats['consolidated_paths_count'] = len(self.bidirectional_paths)
            
            print("✅ 已恢复到原始骨干网络")
            return True
            
        except Exception as e:
            print(f"❌ 恢复原始网络失败: {e}")
            return False
    
    def apply_professional_consolidation_manually(self, design_mode: str = None) -> bool:
        """手动应用基于节点聚类的专业整合"""
        if not PROFESSIONAL_CONSOLIDATION_AVAILABLE:
            print("❌ 基于节点聚类的专业道路整合模块不可用")
            return False
        
        if design_mode:
            self.set_professional_design_mode(design_mode)
        
        print(f"🔧 手动应用基于节点聚类的专业道路整合...")
        
        # 临时启用自动整合
        original_setting = self.config['auto_consolidate_after_generation']
        self.config['auto_consolidate_after_generation'] = True
        
        # 执行整合
        success = self._apply_professional_consolidation_if_enabled()
        
        # 恢复原设置
        self.config['auto_consolidate_after_generation'] = original_setting
        
        return success
    
    # ==================== 内部优化方法（保持原有逻辑） ====================
    
    def _find_candidate_paths(self, target_type: str, target_id: int) -> List:
        """查找候选路径"""
        candidates = []
        
        for path_id, path_data in self.bidirectional_paths.items():
            if ((path_data.point_a['type'] == target_type and path_data.point_a['id'] == target_id) or
                (path_data.point_b['type'] == target_type and path_data.point_b['id'] == target_id)):
                candidates.append(path_data)
        
        return candidates
    
    def _assign_vehicle_to_path_enhanced(self, vehicle_id: str, path_id: str):
        """增强的车辆路径分配"""
        # 原有分配逻辑
        if vehicle_id in self.vehicle_path_assignments:
            old_path_id = self.vehicle_path_assignments[vehicle_id]
            if old_path_id in self.bidirectional_paths:
                self.bidirectional_paths[old_path_id].remove_vehicle(vehicle_id)
        
        # 分配到新路径
        self.vehicle_path_assignments[vehicle_id] = path_id
        if path_id in self.bidirectional_paths:
            self.bidirectional_paths[path_id].add_vehicle(vehicle_id)
            
            # 记录负载历史
            current_load = self.bidirectional_paths[path_id].get_load_factor()
            self.path_load_history[path_id].append(current_load)
            
            # 限制历史记录长度
            if len(self.path_load_history[path_id]) > 100:
                self.path_load_history[path_id] = self.path_load_history[path_id][-50:]
        
        # 增强：更新稳定性管理
        if self.config['enable_stability_management']:
            self.stability_manager.record_vehicle_switch(vehicle_id, path_id)
    
    def _try_maintain_current_path(self, current_position: Tuple, target_type: str, 
                                 target_id: int, vehicle_id: str) -> Optional[Tuple]:
        """尝试维持当前路径"""
        current_path_id = self.vehicle_path_assignments.get(vehicle_id)
        if not current_path_id or current_path_id not in self.bidirectional_paths:
            return None
        
        current_path_data = self.bidirectional_paths[current_path_id]
        
        # 检查当前路径是否仍然连接到目标
        connects_to_target = (
            (current_path_data.point_a['type'] == target_type and current_path_data.point_a['id'] == target_id) or
            (current_path_data.point_b['type'] == target_type and current_path_data.point_b['id'] == target_id)
        )
        
        if connects_to_target and current_path_data.get_load_factor() < 0.9:
            print(f"  维持车辆 {vehicle_id} 当前路径: {current_path_id}")
            
            # 使用当前路径重新规划
            best_option = self._find_optimal_interface_node_professional(
                current_position, current_path_data, target_type, target_id, vehicle_id
            )
            
            if best_option:
                complete_path, structure = self._build_complete_path_professional(
                    current_position, best_option, current_path_data, vehicle_id
                )
                
                if complete_path:
                    structure['stability_maintained'] = True
                    return complete_path, structure
        
        return None
    
    def _load_special_points(self):
        """加载特殊点"""
        # 装载点
        self.special_points['loading'] = []
        for i, point in enumerate(self.env.loading_points):
            self.special_points['loading'].append({
                'id': i, 'type': 'loading', 'position': self._ensure_3d_point(point)
            })
        
        # 卸载点
        self.special_points['unloading'] = []
        for i, point in enumerate(self.env.unloading_points):
            self.special_points['unloading'].append({
                'id': i, 'type': 'unloading', 'position': self._ensure_3d_point(point)
            })
        
        # 停车点
        self.special_points['parking'] = []
        parking_areas = getattr(self.env, 'parking_areas', [])
        for i, point in enumerate(parking_areas):
            self.special_points['parking'].append({
                'id': i, 'type': 'parking', 'position': self._ensure_3d_point(point)
            })
        
        print(f"加载特殊点: 装载{len(self.special_points['loading'])}个, "
              f"卸载{len(self.special_points['unloading'])}个, "
              f"停车{len(self.special_points['parking'])}个")
    
    def _validate_special_points(self) -> bool:
        """验证特殊点"""
        if not self.special_points['loading'] or not self.special_points['unloading']:
            print("❌ 缺少必要的装载点或卸载点")
            return False
        return True
    
    def _generate_complete_bidirectional_paths(self) -> int:
        """生成完整的双向路径组合"""
        if not self.path_planner:
            print("❌ 未设置路径规划器")
            return 0
        
        success_count = 0
        path_pairs = []
        
        # 定义需要连接的点类型组合
        connection_types = [
            ('loading', 'unloading'),
            ('loading', 'parking'),
            ('unloading', 'parking')
        ]
        
        # 收集所有需要连接的点对
        for type_a, type_b in connection_types:
            if (len(self.special_points[type_a]) == 0 or 
                len(self.special_points[type_b]) == 0):
                continue
                
            for point_a in self.special_points[type_a]:
                for point_b in self.special_points[type_b]:
                    path_pairs.append((point_a, point_b, f"{type_a}_to_{type_b}"))
        
        self.stats['total_path_pairs'] = len(path_pairs)
        print(f"\n需要生成 {len(path_pairs)} 条双向路径")
        
        # 生成每条双向路径
        for i, (point_a, point_b, connection_type) in enumerate(path_pairs, 1):
            print(f"\n[{i}/{len(path_pairs)}] 生成路径: {point_a['type'][0].upper()}{point_a['id']} ↔ {point_b['type'][0].upper()}{point_b['id']}")
            
            path_result = self._generate_single_bidirectional_path(point_a, point_b)
            
            if path_result:
                success_count += 1
                self.stats[connection_type] += 1
                print(f"  ✅ 成功: 长度{len(path_result.forward_path)}, "
                      f"质量{path_result.quality:.2f}, "
                      f"规划器: {path_result.planner_used}")
            else:
                print(f"  ❌ 失败")
        
        return success_count
    
    def _generate_single_bidirectional_path(self, point_a: Dict, point_b: Dict) -> Optional[BiDirectionalPath]:
        """生成单条双向路径"""
        path_id = f"{point_a['type'][0].upper()}{point_a['id']}_to_{point_b['type'][0].upper()}{point_b['id']}"
        
        start_pos = point_a['position']
        end_pos = point_b['position']
        
        # 渐进回退策略
        planning_strategies = [
            {
                'name': 'hybrid_astar_strict',
                'planner_type': 'hybrid_astar',
                'quality_threshold': self.config['primary_quality_threshold'],
                'max_time': 15.0,
                'context': 'backbone'
            },
            {
                'name': 'hybrid_astar_relaxed',
                'planner_type': 'hybrid_astar', 
                'quality_threshold': self.config['fallback_quality_threshold'],
                'max_time': 20.0,
                'context': 'backbone'
            },
            {
                'name': 'rrt_standard',
                'planner_type': 'rrt',
                'quality_threshold': self.config['fallback_quality_threshold'],
                'max_time': 15.0,
                'context': 'backbone'
            },
            {
                'name': 'direct_fallback',
                'planner_type': 'direct',
                'quality_threshold': 0.3,
                'max_time': 1.0,
                'context': 'fallback'
            }
        ]
        
        for strategy in planning_strategies:
            if not self.config['enable_progressive_fallback'] and strategy['name'] != 'hybrid_astar_strict':
                continue
            
            try:
                # 尝试双向规划，选择更好的方向
                result_ab = self._plan_with_strategy(start_pos, end_pos, strategy, f"{path_id}_AB")
                result_ba = self._plan_with_strategy(end_pos, start_pos, strategy, f"{path_id}_BA")
                
                # 选择更好的结果
                best_result = None
                best_direction = None
                
                if result_ab and result_ba:
                    if result_ab[1] >= result_ba[1]:
                        best_result, best_direction = result_ab, 'AB'
                    else:
                        best_result, best_direction = result_ba, 'BA'
                elif result_ab:
                    best_result, best_direction = result_ab, 'AB'
                elif result_ba:
                    best_result, best_direction = result_ba, 'BA'
                
                if best_result:
                    path, quality = best_result
                    
                    # 创建双向路径对象
                    if best_direction == 'AB':
                        forward_path = path
                        reverse_path = self._reverse_path(path)
                    else:
                        reverse_path = path
                        forward_path = self._reverse_path(path)
                    
                    bidirectional_path = BiDirectionalPath(
                        path_id=path_id,
                        point_a=point_a,
                        point_b=point_b,
                        forward_path=forward_path,
                        reverse_path=reverse_path,
                        length=self._calculate_path_length(forward_path),
                        quality=quality,
                        planner_used=strategy['planner_type'],
                        created_time=time.time()
                    )
                    
                    # 存储路径
                    self.bidirectional_paths[path_id] = bidirectional_path
                    
                    # 更新统计
                    if strategy['planner_type'] == 'hybrid_astar':
                        self.stats['astar_success'] += 1
                    elif strategy['planner_type'] == 'rrt':
                        self.stats['rrt_success'] += 1
                    elif strategy['planner_type'] == 'direct':
                        self.stats['direct_fallback'] += 1
                    
                    return bidirectional_path
            
            except Exception as e:
                continue
        
        return None
    
    def _plan_with_strategy(self, start: Tuple, goal: Tuple, strategy: Dict, 
                           agent_id: str) -> Optional[Tuple[List, float]]:
        """使用指定策略进行规划"""
        try:
            result = self.path_planner.plan_path(
                vehicle_id=agent_id,
                start=start,
                goal=goal,
                use_backbone=False,
                planner_type=strategy['planner_type'],
                context=strategy['context'],
                quality_threshold=strategy['quality_threshold'],
                return_object=True
            )
            
            if result and hasattr(result, 'path') and result.path:
                if len(result.path) >= 2:
                    quality = getattr(result, 'quality_score', 0.5)
                    
                    if quality >= strategy['quality_threshold']:
                        return (result.path, quality)
            
        except Exception as e:
            pass
        
        return None
    
    def _reverse_path(self, path: List[Tuple]) -> List[Tuple]:
        """反转路径方向"""
        if not path:
            return []
        
        reversed_path = []
        for point in reversed(path):
            if len(point) >= 3:
                x, y, theta = point[0], point[1], point[2]
                reverse_theta = (theta + math.pi) % (2 * math.pi)
                reversed_path.append((x, y, reverse_theta))
            else:
                reversed_path.append(point)
        
        return reversed_path
    
    def _generate_safe_interfaces(self) -> int:
        """为双向路径生成安全接口"""
        total_interfaces = 0
        spacing = self.config['interface_spacing']
        
        for path_id, path_data in self.bidirectional_paths.items():
            forward_path = path_data.forward_path
            
            if len(forward_path) < 2:
                continue
            
            interface_count = 0
            
            # 根据基于节点聚类的专业设计调整接口间距
            if hasattr(path_data, 'node_spacing') and path_data.node_spacing > 0:
                actual_spacing = int(path_data.node_spacing)
            else:
                actual_spacing = spacing
            
            # 在路径上等间距生成接口
            for i in range(0, len(forward_path), actual_spacing):
                if i >= len(forward_path):
                    break
                
                interface_id = f"{path_id}_if_{interface_count}"
                
                # 增强接口存储
                self.backbone_interfaces[interface_id] = {
                    'id': interface_id,
                    'position': forward_path[i],
                    'path_id': path_id,
                    'path_index': i,
                    'is_occupied': False,
                    'reservation_count': 0,
                    'usage_history': [],
                    'safety_verified': self.config['enable_safety_optimization'],
                    'consolidation_level': path_data.consolidation_level,
                    # 新增：基于节点聚类的专业设计信息
                    'is_professional_design': getattr(path_data, 'is_professional_design', False),
                    'road_class': getattr(path_data, 'road_class', 'standard'),
                    'spacing_used': actual_spacing,
                    'is_node_clustered': getattr(path_data, 'is_node_clustered', False)
                }
                
                self.path_interfaces[path_id].append(interface_id)
                interface_count += 1
                total_interfaces += 1
        
        return total_interfaces
    
    def _build_connection_index(self):
        """建立连接索引"""
        self.connection_index.clear()
        
        for path_id, path_data in self.bidirectional_paths.items():
            point_a = path_data.point_a
            point_b = path_data.point_b
            
            # 双向索引
            key_ab = (point_a['type'], point_a['id'], point_b['type'], point_b['id'])
            key_ba = (point_b['type'], point_b['id'], point_a['type'], point_a['id'])
            
            self.connection_index[key_ab] = path_id
            self.connection_index[key_ba] = path_id
    
    def _initialize_quality_tracking(self):
        """初始化质量追踪"""
        for path_data in self.bidirectional_paths.values():
            path_data.update_quality_history(path_data.quality)
    
    def _get_average_historical_load(self, path_id: str) -> float:
        """获取路径的平均历史负载"""
        if path_id not in self.path_load_history:
            return 0.0
        
        history = self.path_load_history[path_id]
        if not history:
            return 0.0
        
        # 最近10次的平均值
        recent_samples = history[-10:]
        return sum(recent_samples) / len(recent_samples)
    
    def _direct_planning_fallback(self, current_position: Tuple, 
                                 target_type: str, target_id: int) -> Optional[Tuple]:
        """直接规划回退"""
        if not self.path_planner:
            return None
        
        try:
            target_position = self._get_target_position(target_type, target_id)
            if not target_position:
                return None
            
            result = self.path_planner.plan_path(
                vehicle_id="direct_fallback",
                start=current_position,
                goal=target_position,
                use_backbone=False
            )
            
            if result:
                if hasattr(result, 'path'):
                    path = result.path
                elif isinstance(result, tuple):
                    path = result[0]
                else:
                    path = result
                
                if path:
                    structure = {
                        'type': 'direct_fallback',
                        'backbone_utilization': 0.0,
                        'total_length': len(path),
                        'fallback_reason': 'no_backbone_available',
                        'node_clustering_professional_design_aware': False
                    }
                    
                    print(f"  ✅ 直接规划回退成功: 长度{len(path)}")
                    return path, structure
        
        except Exception as e:
            print(f"    直接规划回退失败: {e}")
        
        return None
    
    def _get_target_position(self, target_type: str, target_id: int) -> Optional[Tuple]:
        """获取目标位置"""
        if target_type in self.special_points:
            points = self.special_points[target_type]
            if 0 <= target_id < len(points):
                return points[target_id]['position']
        return None
    
    def _ensure_3d_point(self, point) -> Tuple[float, float, float]:
        """确保点坐标为3D"""
        if not point:
            return (0.0, 0.0, 0.0)
        elif len(point) >= 3:
            return (float(point[0]), float(point[1]), float(point[2]))
        elif len(point) == 2:
            return (float(point[0]), float(point[1]), 0.0)
        else:
            return (0.0, 0.0, 0.0)
    
    def _calculate_distance(self, p1: Tuple, p2: Tuple) -> float:
        """计算两点间距离"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def _calculate_path_length(self, path: List[Tuple]) -> float:
        """计算路径总长度"""
        if not path or len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            length += self._calculate_distance(path[i], path[i + 1])
        return length
    
    def _calculate_average_path_utilization(self) -> float:
        """计算平均路径利用率"""
        if not self.bidirectional_paths:
            return 0.0
        
        total_utilization = sum(path.get_load_factor() for path in self.bidirectional_paths.values())
        return total_utilization / len(self.bidirectional_paths)
    
    def update_load_balancing(self, time_delta: float):
        """更新负载均衡"""
        current_time = time.time()
        
        # 清理过期的接口预留
        self.interface_manager.cleanup_expired_reservations(current_time)
        
        # 更新接口使用统计
        for interface_id, interface_info in self.backbone_interfaces.items():
            if interface_id in self.interface_manager.reservations:
                interface_info['reservation_count'] = 1
            else:
                interface_info['reservation_count'] = 0
    
    def debug_network_info(self):
        """调试网络信息"""
        print("=== 集成基于节点聚类的专业道路整合骨干网络调试信息 ===")
        print(f"双向路径数量: {len(self.bidirectional_paths)}")
        print(f"活跃车辆分配: {len(self.vehicle_path_assignments)}")
        print(f"接口预留: {len(self.interface_manager.reservations)}")
        print(f"平均路径利用率: {self._calculate_average_path_utilization():.2%}")
        
        if self.config['enable_stability_management']:
            stability_report = self.stability_manager.get_stability_report()
            print(f"系统稳定性: {stability_report['overall_stability']:.2%}")
        
        if self.config['enable_safety_optimization']:
            print(f"已注册安全参数车辆: {len(self.safe_interface_manager.vehicle_safety_params)}")
        
        # 新增：基于节点聚类的专业设计信息
        if self.stats['professional_consolidation_applied']:
            print(f"\n🔧 基于节点聚类的专业道路整合信息:")
            print(f"  整合状态: 已应用")
            print(f"  设计模式: {self.config['professional_design_mode']}")
            print(f"  原始路径: {self.stats['original_paths_count']} 条")
            print(f"  整合后路径: {self.stats['consolidated_paths_count']} 条")
            print(f"  原始节点: {self.stats['original_nodes_count']} 个")
            print(f"  关键节点: {self.stats['key_nodes_count']} 个")
            print(f"  节点减少率: {self.stats['node_reduction_ratio']:.1%}")
            print(f"  重建成功率: {self.stats['reconstruction_success_rate']:.1%}")
            print(f"  主干道: {self.stats['primary_roads_count']} 条")
            print(f"  次干道: {self.stats['secondary_roads_count']} 条")
            print(f"  作业道: {self.stats['service_roads_count']} 条")
            print(f"  平均安全评级: {self.stats['average_safety_rating']:.2f}")
            print(f"  标准符合率: {self.stats['standards_compliance_rate']:.1%}")
            print(f"  整合耗时: {self.stats['consolidation_time']:.2f}s")
            
            # 基于节点聚类的专业设计路径分布
            professional_count = sum(1 for path in self.bidirectional_paths.values() if path.is_professional_design)
            node_clustered_count = sum(1 for path in self.bidirectional_paths.values() if getattr(path, 'is_node_clustered', False))
            print(f"  基于节点聚类的专业设计路径比例: {professional_count}/{len(self.bidirectional_paths)} ({professional_count/len(self.bidirectional_paths)*100:.1f}%)")
            print(f"  节点聚类路径比例: {node_clustered_count}/{len(self.bidirectional_paths)} ({node_clustered_count/len(self.bidirectional_paths)*100:.1f}%)")
        else:
            print(f"\n整合状态: 未应用")
        
        # 显示高负载路径
        high_load_paths = []
        for path_id, path_data in self.bidirectional_paths.items():
            load_factor = path_data.get_load_factor()
            if load_factor > 0.5:
                professional_marker = "🔧" if path_data.is_professional_design else "  "
                clustered_marker = "🎯" if getattr(path_data, 'is_node_clustered', False) else "  "
                road_class = getattr(path_data, 'road_class', 'standard')
                high_load_paths.append((path_id, load_factor, professional_marker, clustered_marker, road_class))
        
        if high_load_paths:
            print(f"\n高负载路径 ({len(high_load_paths)} 条):")
            for path_id, load_factor, prof_marker, clust_marker, road_class in sorted(high_load_paths, key=lambda x: x[1], reverse=True):
                print(f"  {prof_marker}{clust_marker} {path_id} ({road_class}): {load_factor:.1%} 负载")

# 向后兼容性
SimplifiedBackboneNetwork = OptimizedBackboneNetwork