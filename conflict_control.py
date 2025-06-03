"""
integrated_conflict_control.py - 整合优化版冲突检测控制系统
完美配合网络整理、智能交通管理、多阶段任务调度的双重冲突检测系统
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

# 导入网络拓扑类型
try:
    from integrated_traffic_manager import NetworkTopologyType
    INTEGRATED_TRAFFIC_AVAILABLE = True
except ImportError:
    INTEGRATED_TRAFFIC_AVAILABLE = False
    class NetworkTopologyType(Enum):
        ORIGINAL = "original"
        CONSOLIDATED = "consolidated"
        HIERARCHICAL = "hierarchical"

class ConflictType(Enum):
    """冲突类型 - 扩展版"""
    BACKBONE_SEGMENT_CONFLICT = "backbone_segment_conflict"
    INTERFACE_NODE_CONFLICT = "interface_node_conflict"
    PRIORITY_CONFLICT = "priority_conflict"
    TEMPORAL_DEADLOCK = "temporal_deadlock"
    SPATIAL_CONFLICT = "spatial_conflict"
    PREDICTIVE_CONFLICT = "predictive_conflict"
    FULL_PATH_CONFLICT = "full_path_conflict"
    MULTI_STAGE_CONFLICT = "multi_stage_conflict"
    HIERARCHY_CONFLICT = "hierarchy_conflict"
    CONSOLIDATION_CONFLICT = "consolidation_conflict"

class ConflictSeverity(Enum):
    """冲突严重程度"""
    LOW = 1
    MEDIUM = 2 
    HIGH = 3
    CRITICAL = 4

class ResolutionStrategy(Enum):
    """冲突解决策略 - 扩展版"""
    FIRST_COME_FIRST_SERVE = "first_come_first_serve"
    PRIORITY_PREEMPTION = "priority_preemption"
    TEMPORAL_ADJUSTMENT = "temporal_adjustment"
    NEGOTIATED_ADJUSTMENT = "negotiated_adjustment"
    PROGRESSIVE_DELAY = "progressive_delay"
    ALTERNATIVE_BACKBONE_PATH = "alternative_backbone_path"
    ALTERNATIVE_INTERFACE = "alternative_interface"
    PREVENTIVE_RESCHEDULING = "preventive_rescheduling"
    EMERGENCY_STOP = "emergency_stop"
    HIERARCHY_OPTIMIZATION = "hierarchy_optimization"
    CONSOLIDATION_AWARE_ROUTING = "consolidation_aware_routing"
    MULTI_STAGE_COORDINATION = "multi_stage_coordination"

@dataclass
class NetworkAwareVehicleInfo:
    """网络感知的车辆信息"""
    vehicle_id: str
    position: Tuple[float, float, float]
    current_speed: float
    vehicle_type: str = 'dump_truck'
    priority: int = 2
    
    # 网络感知属性
    network_topology_awareness: str = "original"
    hierarchy_preference: str = "trunk"
    consolidation_aware: bool = False
    
    # 多阶段任务属性
    current_task_stage: str = "transport"
    multi_stage_task: bool = False
    stage_transition_pending: bool = False
    
    # 车辆物理参数
    length: float = 6.0
    width: float = 3.0
    safety_margin: float = 1.5
    turning_radius: float = 8.0
    
    # 动态状态
    current_load: float = 0.0
    max_load: float = 100.0
    passing_status: int = 0  # 0=通行，1=停车

@dataclass
class IntegratedVehicleTrajectoryPoint:
    """集成车辆轨迹点"""
    timestamp: float
    position: Tuple[float, float, float]
    velocity: float
    heading: float
    
    # 网络感知属性
    network_segment_id: Optional[str] = None
    hierarchy_level: str = "trunk"
    spatial_risk_level: float = 0.0

@dataclass
class NetworkAwareSegmentOccupation:
    """网络感知的路段占用记录"""
    vehicle_id: str
    backbone_path_id: str
    segment_start_node: int
    segment_end_node: int
    planned_entry_time: float
    planned_exit_time: float
    vehicle_priority: int
    request_time: float
    
    # 网络感知属性
    segment_id: str = ""
    network_topology_type: str = "original"
    hierarchy_level: str = "trunk"
    consolidation_aware: bool = False
    
    # 多阶段任务属性
    task_stage: str = "transport"
    stage_transition: bool = False
    inter_stage_buffer: float = 0.0
    
    # 空间感知属性
    spatial_conflict_risk: float = 0.0
    predicted_trajectory: List[IntegratedVehicleTrajectoryPoint] = field(default_factory=list)
    
    # 动态安全参数
    base_safety_margin: float = 10.0
    dynamic_safety_margin: float = 10.0
    
    def __post_init__(self):
        self.segment_id = f"{self.backbone_path_id}_{self.segment_start_node}_{self.segment_end_node}"
        if self.request_time == 0:
            self.request_time = time.time()
    
    def calculate_network_aware_safety_margin(self, vehicle_info: NetworkAwareVehicleInfo) -> float:
        """计算网络感知的安全边距"""
        base_margin = self.base_safety_margin
        
        # 车辆类型因子
        type_factors = {
            'dump_truck': 2.0,
            'excavator': 1.5, 
            'loader': 1.8,
            'water_truck': 1.3
        }
        type_factor = type_factors.get(vehicle_info.vehicle_type, 1.5)
        
        # 负载因子
        load_factor = 1.0 + (vehicle_info.current_load / max(vehicle_info.max_load, 1)) * 0.8
        
        # 网络拓扑因子
        topology_factor = 1.0
        if self.network_topology_type == "hierarchical":
            if self.hierarchy_level == "trunk":
                topology_factor = 1.2  # 主干路径需要更大安全边距
            elif self.hierarchy_level == "connector":
                topology_factor = 0.9  # 连接路径可以小一些
        elif self.network_topology_type == "consolidated":
            topology_factor = 0.95  # 整理路径稍微减少
        
        # 多阶段任务因子
        stage_factor = 1.0
        if self.task_stage in ["loading", "unloading"]:
            stage_factor = 1.3  # 装卸阶段需要更大安全边距
        elif self.stage_transition:
            stage_factor = 1.15  # 阶段转换时稍微增加
        
        # 空间冲突风险因子
        spatial_factor = 1.0 + self.spatial_conflict_risk * 0.5
        
        self.dynamic_safety_margin = (
            base_margin * type_factor * load_factor * 
            topology_factor * stage_factor * spatial_factor
        )
        
        return self.dynamic_safety_margin
    
    def overlaps_with_integrated(self, other: 'NetworkAwareSegmentOccupation', 
                                vehicle_info_dict: Dict[str, NetworkAwareVehicleInfo]) -> bool:
        """集成的时间重叠检测"""
        self_vehicle_info = vehicle_info_dict.get(self.vehicle_id)
        other_vehicle_info = vehicle_info_dict.get(other.vehicle_id)
        
        if not self_vehicle_info or not other_vehicle_info:
            return self.overlaps_with_basic(other)
        
        # 计算动态安全边距
        self_margin = self.calculate_network_aware_safety_margin(self_vehicle_info)
        other_margin = other.calculate_network_aware_safety_margin(other_vehicle_info)
        
        # 计算减速时间
        self_decel_time = self._calculate_stage_aware_deceleration_time(self_vehicle_info)
        other_decel_time = other._calculate_stage_aware_deceleration_time(other_vehicle_info)
        
        # 计算实际占用时间窗口
        self_start = self.planned_entry_time - self_decel_time
        self_end = self.planned_exit_time + self_margin + self.inter_stage_buffer
        other_start = other.planned_entry_time - other_decel_time
        other_end = other.planned_exit_time + other_margin + other.inter_stage_buffer
        
        overlap = not (self_end <= other_start or other_end <= self_start)
        
        if overlap:
            print(f"    ⚠️ 集成重叠检测: {self.vehicle_id}({self_start:.1f}-{self_end:.1f}) vs {other.vehicle_id}({other_start:.1f}-{other_end:.1f})")
            print(f"      网络类型: {self.network_topology_type} vs {other.network_topology_type}")
            print(f"      任务阶段: {self.task_stage} vs {other.task_stage}")
        
        return overlap
    
    def _calculate_stage_aware_deceleration_time(self, vehicle_info: NetworkAwareVehicleInfo) -> float:
        """计算阶段感知的减速时间"""
        base_decel_time = vehicle_info.current_speed / 1.0  # 基础减速时间
        
        # 阶段调整
        stage_factors = {
            "loading": 1.5,     # 装载阶段需要更长减速时间
            "unloading": 1.4,   # 卸载阶段
            "transport": 1.0,   # 运输阶段标准
            "parking": 0.8      # 停车阶段可以较快
        }
        
        stage_factor = stage_factors.get(self.task_stage, 1.0)
        
        # 阶段转换调整
        transition_factor = 1.2 if self.stage_transition else 1.0
        
        return base_decel_time * stage_factor * transition_factor
    
    def overlaps_with_basic(self, other: 'NetworkAwareSegmentOccupation') -> bool:
        """基础重叠检测（兼容性）"""
        return not (self.planned_exit_time <= other.planned_entry_time or 
                   other.planned_exit_time <= self.planned_entry_time)

@dataclass
class IntegratedBackboneConflict:
    """集成优化版骨干路径冲突"""
    conflict_id: str
    conflict_type: ConflictType
    severity: ConflictSeverity
    backbone_path_id: str
    segment_id: str
    conflicting_vehicles: List[str]
    occupations: List[NetworkAwareSegmentOccupation]
    
    conflict_time_window: Tuple[float, float]
    overlap_duration: float
    
    suggested_resolution: ResolutionStrategy
    priority_order: List[str]
    
    # 网络感知属性
    network_topology_type: str = "original"
    hierarchy_level: str = "trunk"
    consolidation_aware: bool = False
    
    # 多阶段任务属性
    multi_stage_conflict: bool = False
    affected_task_stages: List[str] = field(default_factory=list)
    stage_transition_involved: bool = False
    
    # 空间感知属性
    spatial_conflict: bool = False
    spatial_overlap_regions: List = field(default_factory=list)
    
    # 预测和协商
    predictive_conflict: bool = False
    negotiation_options: Dict = field(default_factory=dict)
    resolution_context: Dict = field(default_factory=dict)
    
    # 状态跟踪
    detection_time: float = field(default_factory=time.time)
    is_resolved: bool = False
    resolution_time: Optional[float] = None
    resolution_attempts: int = 0
    
    def get_conflict_context(self) -> Dict:
        """获取冲突上下文信息"""
        return {
            'network_topology': self.network_topology_type,
            'hierarchy_level': self.hierarchy_level,
            'consolidation_aware': self.consolidation_aware,
            'multi_stage_conflict': self.multi_stage_conflict,
            'spatial_conflict': self.spatial_conflict,
            'affected_stages': self.affected_task_stages,
            'stage_transition_involved': self.stage_transition_involved,
            'negotiation_feasible': len(self.negotiation_options) > 0,
            'severity_level': self.severity.value,
            'predicted_duration': self.overlap_duration
        }
    
    def get_network_aware_resolution_suggestions(self) -> List[ResolutionStrategy]:
        """获取网络感知的解决策略建议"""
        suggestions = [self.suggested_resolution]
        
        # 基于网络拓扑的策略建议
        if self.network_topology_type == "hierarchical":
            if self.hierarchy_level == "trunk":
                suggestions.append(ResolutionStrategy.HIERARCHY_OPTIMIZATION)
            else:
                suggestions.append(ResolutionStrategy.ALTERNATIVE_INTERFACE)
        
        elif self.network_topology_type == "consolidated":
            suggestions.append(ResolutionStrategy.CONSOLIDATION_AWARE_ROUTING)
        
        # 基于多阶段任务的策略建议
        if self.multi_stage_conflict:
            suggestions.append(ResolutionStrategy.MULTI_STAGE_COORDINATION)
        
        if self.stage_transition_involved:
            suggestions.append(ResolutionStrategy.TEMPORAL_ADJUSTMENT)
        
        # 基于空间冲突的策略建议
        if self.spatial_conflict:
            suggestions.append(ResolutionStrategy.NEGOTIATED_ADJUSTMENT)
        
        # 去重并保持顺序
        unique_suggestions = []
        for strategy in suggestions:
            if strategy not in unique_suggestions:
                unique_suggestions.append(strategy)
        
        return unique_suggestions

class IntegratedBackboneConflictDetector:
    """整合优化版骨干路径冲突检测器"""
    
    def __init__(self, backbone_network=None):
        self.backbone_network = backbone_network
        
        # 核心数据结构 - 网络感知版
        self.segment_occupations: Dict[str, List[NetworkAwareSegmentOccupation]] = defaultdict(list)
        self.vehicle_occupations: Dict[str, List[str]] = defaultdict(list)
        self.vehicle_info_cache: Dict[str, NetworkAwareVehicleInfo] = {}
        
        # 网络拓扑感知
        self.current_network_topology = NetworkTopologyType.ORIGINAL
        self.consolidation_info = {}
        self.topology_change_callbacks = []
        
        # 轨迹预测缓存 - 增强版
        self.trajectory_cache: Dict[str, List[IntegratedVehicleTrajectoryPoint]] = {}
        self.trajectory_update_interval = 5.0
        self.last_trajectory_update = time.time()
        
        # 冲突记录 - 分类存储
        self.detected_conflicts: Dict[str, IntegratedBackboneConflict] = {}
        self.resolved_conflicts: List[IntegratedBackboneConflict] = []
        self.conflict_history: deque = deque(maxlen=1000)
        
        # 空间冲突检测器 - 集成优化版
        self.spatial_detector = None
        if SPATIAL_DETECTION_AVAILABLE:
            try:
                self.spatial_detector = SpaceBasedConflictDetector(position_tolerance=3.0)
                self.spatial_detector_integration = True
                print("✅ 空间冲突检测器已集成")
            except Exception as e:
                print(f"⚠️ 空间冲突检测器初始化失败: {e}")
                self.spatial_detector_integration = False
        else:
            self.spatial_detector_integration = False
        
        # 配置参数 - 整合优化版
        self.config = {
            'base_safety_margin': 10.0,
            'prediction_horizon': 300.0,
            'spatial_check_enabled': True,
            'trajectory_update_interval': 5.0,
            'conflict_detection_interval': 2.0,
            'cleanup_interval': 300.0,
            'max_conflicts_history': 1000,
            'negotiation_threshold': 0.8,
            
            # 网络拓扑感知配置
            'network_topology_adaptation': True,
            'hierarchy_aware_detection': True,
            'consolidation_aware_conflict_detection': True,
            'topology_change_response_time': 30.0,
            
            # 多阶段任务感知配置
            'multi_stage_conflict_detection': True,
            'stage_transition_conflict_prevention': True,
            'inter_stage_buffer_time': 15.0,
            'cross_stage_conflict_analysis': True,
            
            # 空间检测集成配置
            'enable_spatial_detection': SPATIAL_DETECTION_AVAILABLE,
            'spatial_supplement_only': True,
            'spatial_detection_precision': 3.0,
            'spatial_conflict_priority_boost': True,
            
            # 预测和协商配置
            'enable_predictive_detection': True,
            'predictive_conflict_threshold': 60.0,
            'negotiation_conflict_analysis': True,
            'intelligent_resolution_suggestion': True
        }
        
        # 统计信息 - 全面版
        self.stats = {
            'total_occupations_recorded': 0,
            'total_conflicts_detected': 0,
            'logical_conflicts_detected': 0,
            'spatial_conflicts_detected': 0,
            'spatial_supplement_conflicts': 0,
            'predictive_conflicts_detected': 0,
            'full_path_conflicts_detected': 0,
            'conflicts_by_type': defaultdict(int),
            'conflicts_by_severity': defaultdict(int),
            'negotiated_resolutions': 0,
            'preventive_adjustments': 0,
            
            # 网络感知统计
            'network_topology_adaptations': 0,
            'hierarchy_aware_conflicts': 0,
            'consolidation_aware_conflicts': 0,
            'topology_transition_conflicts': 0,
            
            # 多阶段任务统计
            'multi_stage_conflicts_detected': 0,
            'stage_transition_conflicts': 0,
            'inter_stage_conflicts_prevented': 0,
            'cross_stage_optimizations': 0,
            
            # 集成检测统计
            'integrated_detection_cycles': 0,
            'enhanced_resolution_suggestions': 0,
            'context_aware_prioritizations': 0
        }
        
        # 性能监控 - 增强版
        self.performance_monitor = {
            'detection_times': deque(maxlen=100),
            'conflict_resolution_times': deque(maxlen=100),
            'topology_changes': deque(maxlen=50),
            'stage_transitions': deque(maxlen=100),
            'spatial_detection_results': deque(maxlen=200)
        }
        
        self.lock = threading.RLock()
        
        print("初始化整合优化版冲突检测器")
        print(f"  空间检测集成: {'✅' if self.spatial_detector_integration else '❌'}")
        print(f"  网络拓扑感知: ✅")
        print(f"  多阶段任务感知: ✅")
    
    # ==================== 网络拓扑感知方法 ====================
    
    def update_network_topology(self, topology_type: NetworkTopologyType, 
                               consolidation_info: Dict = None):
        """更新网络拓扑感知"""
        if topology_type != self.current_network_topology:
            old_topology = self.current_network_topology
            self.current_network_topology = topology_type
            self.consolidation_info = consolidation_info or {}
            
            # 触发拓扑变化处理
            self._handle_topology_change(old_topology, topology_type)
            
            self.stats['network_topology_adaptations'] += 1
            self.performance_monitor['topology_changes'].append({
                'time': time.time(),
                'old_topology': old_topology.value if hasattr(old_topology, 'value') else str(old_topology),
                'new_topology': topology_type.value,
                'consolidation_info': consolidation_info
            })
            
            print(f"🔄 冲突检测器网络拓扑更新: {old_topology} -> {topology_type.value}")
    
    def _handle_topology_change(self, old_topology, new_topology: NetworkTopologyType):
        """处理拓扑变化"""
        # 重新评估现有占用记录
        self._reevaluate_occupations_for_topology_change(new_topology)
        
        # 更新配置参数
        if new_topology == NetworkTopologyType.HIERARCHICAL:
            self.config['hierarchy_aware_detection'] = True
            self.config['base_safety_margin'] = 12.0  # 层次网络增加安全边距
        elif new_topology == NetworkTopologyType.CONSOLIDATED:
            self.config['consolidation_aware_conflict_detection'] = True
            self.config['base_safety_margin'] = 9.0   # 整理网络可以稍微减少
        else:
            self.config['base_safety_margin'] = 10.0  # 原始网络标准边距
        
        # 通知拓扑变化回调
        for callback in self.topology_change_callbacks:
            try:
                callback(old_topology, new_topology)
            except Exception as e:
                print(f"拓扑变化回调失败: {e}")
    
    def _reevaluate_occupations_for_topology_change(self, new_topology: NetworkTopologyType):
        """重新评估占用记录以适应拓扑变化"""
        for segment_id, occupations in self.segment_occupations.items():
            for occupation in occupations:
                # 更新占用记录的网络类型
                occupation.network_topology_type = new_topology.value
                
                # 根据新拓扑调整层次级别
                if new_topology == NetworkTopologyType.HIERARCHICAL:
                    # 尝试从路径信息推断层次级别
                    occupation.hierarchy_level = self._infer_hierarchy_level(occupation.backbone_path_id)
                elif new_topology == NetworkTopologyType.CONSOLIDATED:
                    occupation.consolidation_aware = True
    
    def _infer_hierarchy_level(self, backbone_path_id: str) -> str:
        """推断路径的层次级别"""
        if not self.backbone_network or not backbone_path_id:
            return "trunk"
        
        if hasattr(self.backbone_network, 'bidirectional_paths'):
            path_data = self.backbone_network.bidirectional_paths.get(backbone_path_id)
            if path_data and hasattr(path_data, 'consolidation_info'):
                return path_data.consolidation_info.get('hierarchy_level', 'trunk')
        
        return "trunk"  # 默认为主干
    
    def register_topology_change_callback(self, callback):
        """注册拓扑变化回调"""
        self.topology_change_callbacks.append(callback)
    
    # ==================== 车辆信息管理 - 网络感知版 ====================
    
    def update_vehicle_info_integrated(self, vehicle_id: str, vehicle_info: Dict):
        """更新集成的车辆信息"""
        # 转换为网络感知的车辆信息
        integrated_info = NetworkAwareVehicleInfo(
            vehicle_id=vehicle_id,
            position=vehicle_info.get('position', (0, 0, 0)),
            current_speed=vehicle_info.get('current_speed', 1.5),
            vehicle_type=vehicle_info.get('vehicle_type', 'dump_truck'),
            priority=vehicle_info.get('priority', 2),
            
            # 网络感知属性
            network_topology_awareness=vehicle_info.get('network_topology_awareness', self.current_network_topology.value),
            hierarchy_preference=vehicle_info.get('hierarchy_preference', 'trunk'),
            consolidation_aware=vehicle_info.get('consolidation_aware', self.current_network_topology != NetworkTopologyType.ORIGINAL),
            
            # 多阶段任务属性
            current_task_stage=vehicle_info.get('current_task_stage', 'transport'),
            multi_stage_task=vehicle_info.get('multi_stage_task', False),
            stage_transition_pending=vehicle_info.get('stage_transition_pending', False),
            
            # 物理参数
            length=vehicle_info.get('length', 6.0),
            width=vehicle_info.get('width', 3.0),
            safety_margin=vehicle_info.get('safety_margin', 1.5),
            turning_radius=vehicle_info.get('turning_radius', 8.0),
            
            # 动态状态
            current_load=vehicle_info.get('current_load', 0.0),
            max_load=vehicle_info.get('max_load', 100.0),
            passing_status=vehicle_info.get('passing_status', 0)
        )
        
        self.vehicle_info_cache[vehicle_id] = integrated_info
    
    def update_vehicle_stage_transition(self, vehicle_id: str, current_stage: str, 
                                      next_stage: str = None):
        """更新车辆阶段转换信息"""
        if vehicle_id in self.vehicle_info_cache:
            vehicle_info = self.vehicle_info_cache[vehicle_id]
            vehicle_info.current_task_stage = current_stage
            vehicle_info.stage_transition_pending = next_stage is not None
            
            # 记录阶段转换
            self.performance_monitor['stage_transitions'].append({
                'vehicle_id': vehicle_id,
                'current_stage': current_stage,
                'next_stage': next_stage,
                'time': time.time()
            })
    
    # ==================== 轨迹预测 - 网络感知版 ====================
    
    def predict_vehicle_trajectory_integrated(self, vehicle_id: str, 
                                            time_horizon: float = 300.0) -> List[IntegratedVehicleTrajectoryPoint]:
        """预测集成的车辆轨迹"""
        if vehicle_id not in self.vehicle_info_cache:
            return []
        
        vehicle_info = self.vehicle_info_cache[vehicle_id]
        current_pos = vehicle_info.position
        current_speed = vehicle_info.current_speed
        current_heading = current_pos[2] if len(current_pos) > 2 else 0.0
        
        trajectory = []
        current_time = time.time()
        
        # 阶段感知的轨迹预测
        stage_speed_factor = self._get_stage_speed_factor(vehicle_info.current_task_stage)
        effective_speed = current_speed * stage_speed_factor
        
        for t in range(0, int(time_horizon), 10):
            timestamp = current_time + t
            
            # 基础位置预测
            distance = effective_speed * t
            x = current_pos[0] + distance * math.cos(current_heading)
            y = current_pos[1] + distance * math.sin(current_heading)
            
            # 空间风险评估
            spatial_risk = self._assess_trajectory_point_spatial_risk(
                (x, y, current_heading), vehicle_info
            )
            
            # 网络段ID推断
            network_segment_id = self._infer_network_segment_at_position((x, y, current_heading))
            
            trajectory_point = IntegratedVehicleTrajectoryPoint(
                timestamp=timestamp,
                position=(x, y, current_heading),
                velocity=effective_speed,
                heading=current_heading,
                network_segment_id=network_segment_id,
                hierarchy_level=vehicle_info.hierarchy_preference,
                spatial_risk_level=spatial_risk
            )
            
            trajectory.append(trajectory_point)
        
        return trajectory
    
    def _get_stage_speed_factor(self, stage: str) -> float:
        """获取阶段速度因子"""
        stage_factors = {
            "loading": 0.3,     # 装载时很慢
            "unloading": 0.4,   # 卸载时较慢
            "transport": 1.0,   # 运输时正常
            "parking": 0.6      # 停车时中等
        }
        return stage_factors.get(stage, 1.0)
    
    def _assess_trajectory_point_spatial_risk(self, position: Tuple, 
                                            vehicle_info: NetworkAwareVehicleInfo) -> float:
        """评估轨迹点的空间风险"""
        # 简化的空间风险评估
        x, y = position[0], position[1]
        
        risk = 0.0
        
        # 检查与其他车辆的距离
        for other_vehicle_id, other_info in self.vehicle_info_cache.items():
            if other_vehicle_id != vehicle_info.vehicle_id:
                other_pos = other_info.position
                distance = math.sqrt((x - other_pos[0])**2 + (y - other_pos[1])**2)
                
                if distance < 20:  # 20米内有风险
                    risk += 0.2
                if distance < 10:  # 10米内高风险
                    risk += 0.3
        
        # 多阶段任务增加风险
        if vehicle_info.multi_stage_task:
            risk += 0.1
        
        # 阶段转换增加风险
        if vehicle_info.stage_transition_pending:
            risk += 0.15
        
        return min(risk, 1.0)
    
    def _infer_network_segment_at_position(self, position: Tuple) -> Optional[str]:
        """推断位置所在的网络段"""
        # 简化实现：基于位置的启发式推断
        x, y = position[0], position[1]
        
        # 这里应该根据实际的网络结构进行推断
        # 简化为基于坐标的分区
        segment_x = int(x / 50)
        segment_y = int(y / 50)
        
        return f"segment_{segment_x}_{segment_y}"
    
    # ==================== 占用记录 - 集成版 ====================
    
    def record_vehicle_backbone_occupation_integrated(self, vehicle_id: str, backbone_path_id: str,
                                                    node_timing_plan: Dict[int, Tuple[float, float]],
                                                    vehicle_priority: int = 2,
                                                    task_context: Dict = None) -> bool:
        """记录集成的车辆骨干路径占用"""
        with self.lock:
            try:
                print(f"\n🔧 [集成占用记录] 车辆: {vehicle_id}")
                print(f"   网络拓扑: {self.current_network_topology.value}")
                
                if not node_timing_plan:
                    return False
                
                # 清除旧的占用记录
                self._clear_vehicle_occupations(vehicle_id)
                
                # 获取车辆信息
                vehicle_info = self.vehicle_info_cache.get(vehicle_id)
                if not vehicle_info:
                    print(f"   ⚠️ 车辆信息不存在，使用默认信息")
                    vehicle_info = NetworkAwareVehicleInfo(vehicle_id=vehicle_id, position=(0, 0, 0), current_speed=1.5)
                
                # 预测轨迹
                trajectory = self.predict_vehicle_trajectory_integrated(vehicle_id)
                
                # 处理任务上下文
                task_context = task_context or {}
                task_stage = task_context.get('current_stage', vehicle_info.current_task_stage)
                stage_transition = task_context.get('stage_transition', vehicle_info.stage_transition_pending)
                inter_stage_buffer = task_context.get('inter_stage_buffer', 0.0)
                
                # 创建网络感知的占用记录
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
                    
                    # 评估空间冲突风险
                    spatial_risk = self._assess_segment_spatial_conflict_risk(
                        backbone_path_id, start_node, end_node, vehicle_info
                    )
                    
                    # 推断层次级别
                    hierarchy_level = self._infer_hierarchy_level(backbone_path_id)
                    
                    occupation = NetworkAwareSegmentOccupation(
                        vehicle_id=vehicle_id,
                        backbone_path_id=backbone_path_id,
                        segment_start_node=start_node,
                        segment_end_node=end_node,
                        planned_entry_time=segment_entry_time,
                        planned_exit_time=segment_exit_time,
                        vehicle_priority=vehicle_priority,
                        request_time=request_time,
                        
                        # 网络感知属性
                        network_topology_type=self.current_network_topology.value,
                        hierarchy_level=hierarchy_level,
                        consolidation_aware=vehicle_info.consolidation_aware,
                        
                        # 多阶段任务属性
                        task_stage=task_stage,
                        stage_transition=stage_transition,
                        inter_stage_buffer=inter_stage_buffer,
                        
                        # 空间属性
                        spatial_conflict_risk=spatial_risk,
                        predicted_trajectory=trajectory,
                        base_safety_margin=self.config['base_safety_margin']
                    )
                    
                    segment_id = occupation.segment_id
                    self.segment_occupations[segment_id].append(occupation)
                    self.vehicle_occupations[vehicle_id].append(segment_id)
                
                self.stats['total_occupations_recorded'] += 1
                print(f"   ✅ 集成占用记录完成: {len(node_indices)-1} 个路段")
                return True
                
            except Exception as e:
                print(f"❌ 集成占用记录失败: {e}")
                return False
    
    def _assess_segment_spatial_conflict_risk(self, backbone_path_id: str, 
                                            start_node: int, end_node: int,
                                            vehicle_info: NetworkAwareVehicleInfo) -> float:
        """评估路段的空间冲突风险"""
        risk = 0.0
        
        # 基于网络拓扑的风险
        if self.current_network_topology == NetworkTopologyType.HIERARCHICAL:
            # 层次网络中主干路径风险较高
            if self._infer_hierarchy_level(backbone_path_id) == "trunk":
                risk += 0.2
        
        # 基于任务阶段的风险
        if vehicle_info.current_task_stage in ["loading", "unloading"]:
            risk += 0.3  # 装卸阶段风险较高
        
        # 基于车辆类型的风险
        if vehicle_info.vehicle_type == "excavator":
            risk += 0.2  # 挖掘机风险较高
        
        # 基于当前负载的风险
        load_ratio = vehicle_info.current_load / max(vehicle_info.max_load, 1)
        risk += load_ratio * 0.15
        
        return min(risk, 1.0)
    
    # ==================== 冲突检测 - 集成优化版 ====================
    
    def detect_backbone_conflicts_integrated(self) -> List[IntegratedBackboneConflict]:
        """集成优化的冲突检测"""
        with self.lock:
            detection_start = time.time()
            conflicts = []
            
            print(f"\n🔍 [集成冲突检测] 开始")
            print(f"   网络拓扑: {self.current_network_topology.value}")
            print(f"   占用段数: {len(self.segment_occupations)}")
            
            self.stats['integrated_detection_cycles'] += 1
            
            # 1. 网络感知的逻辑冲突检测
            logical_conflicts = self._detect_network_aware_logical_conflicts()
            conflicts.extend(logical_conflicts)
            self.stats['logical_conflicts_detected'] = len(logical_conflicts)
            
            print(f"🧠 网络感知逻辑检测: {len(logical_conflicts)} 个冲突")
            
            # 2. 多阶段任务冲突检测
            if self.config.get('multi_stage_conflict_detection', True):
                multi_stage_conflicts = self._detect_multi_stage_conflicts()
                conflicts.extend(multi_stage_conflicts)
                self.stats['multi_stage_conflicts_detected'] = len(multi_stage_conflicts)
                print(f"🔄 多阶段任务检测: {len(multi_stage_conflicts)} 个冲突")
            
            # 3. 空间冲突补充检测
            if self.config.get('enable_spatial_detection') and self.spatial_detector:
                spatial_supplements = self._detect_integrated_spatial_conflicts(logical_conflicts)
                if spatial_supplements:
                    conflicts.extend(spatial_supplements)
                    self.stats['spatial_supplement_conflicts'] = len(spatial_supplements)
                    print(f"🌐 补充空间检测: {len(spatial_supplements)} 个额外冲突")
            
            # 4. 预测性冲突检测
            if self.config.get('enable_predictive_detection', True):
                predictive_conflicts = self._detect_predictive_conflicts_enhanced()
                conflicts.extend(predictive_conflicts)
                self.stats['predictive_conflicts_detected'] = len(predictive_conflicts)
                print(f"🔮 预测性检测: {len(predictive_conflicts)} 个冲突")
            
            # 5. 更新冲突记录和统计
            for conflict in conflicts:
                if conflict.conflict_id not in self.detected_conflicts:
                    self.detected_conflicts[conflict.conflict_id] = conflict
                    self.stats['conflicts_by_type'][conflict.conflict_type] += 1
                    self.stats['conflicts_by_severity'][conflict.severity] += 1
                    
                    # 增强解决策略建议
                    enhanced_suggestions = conflict.get_network_aware_resolution_suggestions()
                    conflict.resolution_context = {
                        'enhanced_suggestions': enhanced_suggestions,
                        'network_context': conflict.get_conflict_context(),
                        'detection_method': 'integrated'
                    }
                    self.stats['enhanced_resolution_suggestions'] += 1
            
            # 更新总统计
            self.stats['total_conflicts_detected'] = len(conflicts)
            detection_time = time.time() - detection_start
            self.performance_monitor['detection_times'].append(detection_time)
            
            print(f"🔍 [集成检测] 完成: {len(conflicts)} 个总冲突 (耗时: {detection_time:.2f}s)")
            print(f"    逻辑: {self.stats['logical_conflicts_detected']}")
            print(f"    多阶段: {self.stats['multi_stage_conflicts_detected']}")
            print(f"    空间补充: {self.stats['spatial_supplement_conflicts']}")
            print(f"    预测性: {self.stats['predictive_conflicts_detected']}")
            
            return conflicts
    
    def _detect_network_aware_logical_conflicts(self) -> List[IntegratedBackboneConflict]:
        """网络感知的逻辑冲突检测"""
        conflicts = []
        
        for segment_id, occupations in self.segment_occupations.items():
            if len(occupations) < 2:
                continue
            
            segment_conflicts = self._detect_segment_conflicts_integrated(segment_id, occupations)
            conflicts.extend(segment_conflicts)
        
        return conflicts
    
    def _detect_multi_stage_conflicts(self) -> List[IntegratedBackboneConflict]:
        """多阶段任务冲突检测"""
        conflicts = []
        
        # 检测阶段转换冲突
        stage_transition_vehicles = []
        for vehicle_id, vehicle_info in self.vehicle_info_cache.items():
            if vehicle_info.stage_transition_pending:
                stage_transition_vehicles.append(vehicle_id)
        
        if len(stage_transition_vehicles) >= 2:
            # 检测阶段转换车辆间的冲突
            transition_conflicts = self._analyze_stage_transition_conflicts(stage_transition_vehicles)
            conflicts.extend(transition_conflicts)
        
        # 检测跨阶段冲突
        if self.config.get('cross_stage_conflict_analysis', True):
            cross_stage_conflicts = self._detect_cross_stage_conflicts()
            conflicts.extend(cross_stage_conflicts)
        
        return conflicts
    
    def _detect_integrated_spatial_conflicts(self, existing_conflicts: List) -> List[IntegratedBackboneConflict]:
        """集成的空间冲突检测"""
        if not self.spatial_detector:
            return []
        
        try:
            # 获取已检测到冲突的车辆对
            existing_pairs = set()
            for conflict in existing_conflicts:
                vehicles = conflict.conflicting_vehicles
                if len(vehicles) >= 2:
                    for i in range(len(vehicles)):
                        for j in range(i + 1, len(vehicles)):
                            pair = tuple(sorted([vehicles[i], vehicles[j]]))
                            existing_pairs.add(pair)
            
            # 更新空间占用数据
            self._update_spatial_occupations_integrated()
            
            # 检测空间冲突
            spatial_conflicts = self.spatial_detector.detect_spatial_conflicts()
            
            # 筛选出新的冲突
            new_spatial_conflicts = []
            for spatial_conflict in spatial_conflicts:
                vehicles = spatial_conflict.vehicle_ids
                if len(vehicles) >= 2:
                    has_new_pair = False
                    for i in range(len(vehicles)):
                        for j in range(i + 1, len(vehicles)):
                            pair = tuple(sorted([vehicles[i], vehicles[j]]))
                            if pair not in existing_pairs:
                                has_new_pair = True
                                break
                        if has_new_pair:
                            break
                    
                    if has_new_pair:
                        backbone_conflict = self._convert_spatial_to_integrated_conflict(spatial_conflict)
                        if backbone_conflict:
                            new_spatial_conflicts.append(backbone_conflict)
                            print(f"  🌐 发现补充空间冲突: 车辆 {vehicles}")
            
            return new_spatial_conflicts
            
        except Exception as e:
            print(f"  ❌ 集成空间冲突检测异常: {e}")
            return []
    
    def _detect_predictive_conflicts_enhanced(self) -> List[IntegratedBackboneConflict]:
        """增强的预测性冲突检测"""
        conflicts = []
        current_time = time.time()
        prediction_end = current_time + self.config['prediction_horizon']
        
        # 基于轨迹预测的冲突检测
        trajectory_conflicts = self._detect_trajectory_based_conflicts(prediction_end)
        conflicts.extend(trajectory_conflicts)
        
        # 基于阶段转换的预测冲突
        stage_prediction_conflicts = self._detect_stage_transition_prediction_conflicts(prediction_end)
        conflicts.extend(stage_prediction_conflicts)
        
        return conflicts
    
    def _detect_segment_conflicts_integrated(self, segment_id: str,
                                           occupations: List[NetworkAwareSegmentOccupation]) -> List[IntegratedBackboneConflict]:
        """集成的路段冲突检测"""
        conflicts = []
        
        for i in range(len(occupations)):
            for j in range(i + 1, len(occupations)):
                occ1, occ2 = occupations[i], occupations[j]
                
                # 使用集成的重叠检测
                if occ1.overlaps_with_integrated(occ2, self.vehicle_info_cache):
                    conflict = self._create_integrated_conflict(segment_id, [occ1, occ2])
                    if conflict:
                        conflicts.append(conflict)
                        
                        # 增强冲突分类
                        self._enhance_conflict_classification(conflict, occ1, occ2)
        
        return conflicts
    
    def _create_integrated_conflict(self, segment_id: str,
                                  occupations: List[NetworkAwareSegmentOccupation]) -> Optional[IntegratedBackboneConflict]:
        """创建集成的冲突对象"""
        if len(occupations) < 2:
            return None
        
        vehicles = [occ.vehicle_id for occ in occupations]
        backbone_path_id = occupations[0].backbone_path_id
        
        min_entry = min(occ.planned_entry_time for occ in occupations)
        max_exit = max(occ.planned_exit_time for occ in occupations)
        
        # 计算增强的重叠时长
        total_overlap = 0.0
        for i in range(len(occupations)):
            for j in range(i + 1, len(occupations)):
                overlap = self._calculate_integrated_overlap(occupations[i], occupations[j])
                total_overlap += overlap
        
        # 网络感知的冲突分类
        conflict_type = self._classify_integrated_conflict_type(occupations)
        severity = self._assess_integrated_conflict_severity(occupations, total_overlap)
        
        # 网络感知的优先级排序
        priority_order = self._determine_integrated_priority_order(occupations)
        suggested_resolution = self._suggest_integrated_resolution_strategy(occupations, conflict_type, severity)
        
        # 收集网络感知属性
        network_topology_type = occupations[0].network_topology_type
        hierarchy_level = occupations[0].hierarchy_level
        consolidation_aware = any(occ.consolidation_aware for occ in occupations)
        
        # 收集多阶段任务属性
        affected_stages = list(set(occ.task_stage for occ in occupations))
        multi_stage_conflict = len(affected_stages) > 1
        stage_transition_involved = any(occ.stage_transition for occ in occupations)
        
        # 协商选项计算
        negotiation_options = {}
        if len(vehicles) == 2 and severity in [ConflictSeverity.LOW, ConflictSeverity.MEDIUM]:
            negotiation_options = self._calculate_integrated_negotiation_options(occupations)
        
        conflict_id = f"integrated_{segment_id}_{int(time.time()*1000)}"
        
        conflict = IntegratedBackboneConflict(
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
            
            # 网络感知属性
            network_topology_type=network_topology_type,
            hierarchy_level=hierarchy_level,
            consolidation_aware=consolidation_aware,
            
            # 多阶段任务属性
            multi_stage_conflict=multi_stage_conflict,
            affected_task_stages=affected_stages,
            stage_transition_involved=stage_transition_involved,
            
            # 协商选项
            negotiation_options=negotiation_options
        )
        
        return conflict
    
    def _enhance_conflict_classification(self, conflict: IntegratedBackboneConflict,
                                       occ1: NetworkAwareSegmentOccupation,
                                       occ2: NetworkAwareSegmentOccupation):
        """增强冲突分类"""
        # 检查是否是层次冲突
        if (occ1.hierarchy_level != occ2.hierarchy_level and 
            conflict.network_topology_type == "hierarchical"):
            conflict.conflict_type = ConflictType.HIERARCHY_CONFLICT
            self.stats['hierarchy_aware_conflicts'] += 1
        
        # 检查是否是整理感知冲突
        if (occ1.consolidation_aware or occ2.consolidation_aware) and conflict.network_topology_type == "consolidated":
            if conflict.conflict_type == ConflictType.BACKBONE_SEGMENT_CONFLICT:
                conflict.conflict_type = ConflictType.CONSOLIDATION_CONFLICT
            self.stats['consolidation_aware_conflicts'] += 1
        
        # 检查是否是多阶段冲突
        if occ1.task_stage != occ2.task_stage or occ1.stage_transition or occ2.stage_transition:
            if conflict.conflict_type == ConflictType.BACKBONE_SEGMENT_CONFLICT:
                conflict.conflict_type = ConflictType.MULTI_STAGE_CONFLICT
        
        # 检查是否是空间冲突
        if occ1.spatial_conflict_risk > 0.5 or occ2.spatial_conflict_risk > 0.5:
            conflict.spatial_conflict = True
    
    # ==================== 辅助方法实现 ====================
    
    def _calculate_integrated_overlap(self, occ1: NetworkAwareSegmentOccupation,
                                    occ2: NetworkAwareSegmentOccupation) -> float:
        """计算集成的重叠时长"""
        if not occ1.overlaps_with_integrated(occ2, self.vehicle_info_cache):
            return 0.0
        
        # 获取车辆信息
        v1_info = self.vehicle_info_cache.get(occ1.vehicle_id)
        v2_info = self.vehicle_info_cache.get(occ2.vehicle_id)
        
        if not v1_info or not v2_info:
            return occ1.get_overlap_duration(occ2) if hasattr(occ1, 'get_overlap_duration') else 0.0
        
        # 计算动态安全边距
        occ1_margin = occ1.calculate_network_aware_safety_margin(v1_info)
        occ2_margin = occ2.calculate_network_aware_safety_margin(v2_info)
        
        # 计算实际时间窗口
        occ1_start = occ1.planned_entry_time - occ1._calculate_stage_aware_deceleration_time(v1_info)
        occ1_end = occ1.planned_exit_time + occ1_margin + occ1.inter_stage_buffer
        occ2_start = occ2.planned_entry_time - occ2._calculate_stage_aware_deceleration_time(v2_info)
        occ2_end = occ2.planned_exit_time + occ2_margin + occ2.inter_stage_buffer
        
        overlap_start = max(occ1_start, occ2_start)
        overlap_end = min(occ1_end, occ2_end)
        
        return max(0.0, overlap_end - overlap_start)
    
    def _classify_integrated_conflict_type(self, occupations: List[NetworkAwareSegmentOccupation]) -> ConflictType:
        """集成的冲突类型分类"""
        # 检查优先级冲突
        priorities = set(occ.vehicle_priority for occ in occupations)
        if len(priorities) > 1:
            return ConflictType.PRIORITY_CONFLICT
        
        # 检查网络拓扑特定冲突
        topology_types = set(occ.network_topology_type for occ in occupations)
        if "hierarchical" in topology_types:
            hierarchy_levels = set(occ.hierarchy_level for occ in occupations)
            if len(hierarchy_levels) > 1:
                return ConflictType.HIERARCHY_CONFLICT
        
        # 检查多阶段任务冲突
        task_stages = set(occ.task_stage for occ in occupations)
        if len(task_stages) > 1 or any(occ.stage_transition for occ in occupations):
            return ConflictType.MULTI_STAGE_CONFLICT
        
        # 检查时间死锁
        max_overlap = 0.0
        for i in range(len(occupations)):
            for j in range(i + 1, len(occupations)):
                overlap = self._calculate_integrated_overlap(occupations[i], occupations[j])
                max_overlap = max(max_overlap, overlap)
        
        if max_overlap > 120:
            return ConflictType.TEMPORAL_DEADLOCK
        
        return ConflictType.BACKBONE_SEGMENT_CONFLICT
    
    def _assess_integrated_conflict_severity(self, occupations: List[NetworkAwareSegmentOccupation],
                                           total_overlap: float) -> ConflictSeverity:
        """集成的冲突严重程度评估"""
        severity_score = 0
        
        # 基础评分
        priorities = [occ.vehicle_priority for occ in occupations]
        max_priority_diff = max(priorities) - min(priorities)
        severity_score += min(max_priority_diff, 3)
        
        # 重叠时长评分
        if total_overlap > 120: severity_score += 4
        elif total_overlap > 60: severity_score += 3
        elif total_overlap > 30: severity_score += 2
        elif total_overlap > 15: severity_score += 1
        
        # 车辆数量评分
        vehicle_count = len(occupations)
        if vehicle_count > 3: severity_score += 3
        elif vehicle_count > 2: severity_score += 2
        
        # 网络拓扑评分
        if any(occ.network_topology_type == "hierarchical" and occ.hierarchy_level == "trunk" for occ in occupations):
            severity_score += 2  # 主干路径冲突更严重
        
        # 多阶段任务评分
        if any(occ.stage_transition for occ in occupations):
            severity_score += 2  # 阶段转换冲突更严重
        
        if len(set(occ.task_stage for occ in occupations)) > 1:
            severity_score += 1  # 跨阶段冲突
        
        # 空间风险评分
        max_spatial_risk = max(occ.spatial_conflict_risk for occ in occupations)
        severity_score += int(max_spatial_risk * 3)
        
        # 返回严重程度
        if severity_score >= 10: return ConflictSeverity.CRITICAL
        elif severity_score >= 7: return ConflictSeverity.HIGH
        elif severity_score >= 4: return ConflictSeverity.MEDIUM
        else: return ConflictSeverity.LOW
    
    def _determine_integrated_priority_order(self, occupations: List[NetworkAwareSegmentOccupation]) -> List[str]:
        """集成的优先级排序"""
        def get_integrated_priority_score(occ):
            # 基础优先级
            base_priority = occ.vehicle_priority * 1000
            
            # 时间优先级（先到先得）
            time_priority = -occ.planned_entry_time * 0.1
            
            # 网络拓扑优先级
            topology_priority = 0
            if occ.network_topology_type == "hierarchical":
                if occ.hierarchy_level == "trunk":
                    topology_priority = 100  # 主干路径优先
                elif occ.hierarchy_level == "branch":
                    topology_priority = 50   # 分支路径次优先
            
            # 多阶段任务优先级
            stage_priority = 0
            stage_priorities = {
                "loading": 80,
                "unloading": 70,
                "transport": 50,
                "parking": 30
            }
            stage_priority = stage_priorities.get(occ.task_stage, 50)
            
            # 阶段转换优先级
            transition_priority = 60 if occ.stage_transition else 0
            
            # 车辆信息优先级
            vehicle_info = self.vehicle_info_cache.get(occ.vehicle_id)
            vehicle_priority = 0
            if vehicle_info:
                load_ratio = vehicle_info.current_load / max(vehicle_info.max_load, 1)
                vehicle_priority = load_ratio * 50
                
                if vehicle_info.vehicle_type == "emergency":
                    vehicle_priority += 200
                elif vehicle_info.vehicle_type == "excavator":
                    vehicle_priority += 50
            
            total_score = (base_priority + time_priority + topology_priority + 
                         stage_priority + transition_priority + vehicle_priority)
            
            return total_score
        
        sorted_occupations = sorted(occupations, key=get_integrated_priority_score, reverse=True)
        
        # 去重
        vehicle_order = []
        for occ in sorted_occupations:
            if occ.vehicle_id not in vehicle_order:
                vehicle_order.append(occ.vehicle_id)
        
        return vehicle_order
    
    def _suggest_integrated_resolution_strategy(self, occupations: List[NetworkAwareSegmentOccupation],
                                              conflict_type: ConflictType,
                                              severity: ConflictSeverity) -> ResolutionStrategy:
        """集成的解决策略建议"""
        # 严重程度优先
        if severity == ConflictSeverity.CRITICAL:
            return ResolutionStrategy.EMERGENCY_STOP
        
        # 网络拓扑感知策略
        if conflict_type == ConflictType.HIERARCHY_CONFLICT:
            return ResolutionStrategy.HIERARCHY_OPTIMIZATION
        
        if conflict_type == ConflictType.CONSOLIDATION_CONFLICT:
            return ResolutionStrategy.CONSOLIDATION_AWARE_ROUTING
        
        # 多阶段任务感知策略
        if conflict_type == ConflictType.MULTI_STAGE_CONFLICT:
            return ResolutionStrategy.MULTI_STAGE_COORDINATION
        
        if any(occ.stage_transition for occ in occupations):
            return ResolutionStrategy.TEMPORAL_ADJUSTMENT
        
        # 空间冲突策略
        max_spatial_risk = max(occ.spatial_conflict_risk for occ in occupations)
        if max_spatial_risk > 0.7:
            return ResolutionStrategy.NEGOTIATED_ADJUSTMENT
        
        # 默认策略
        priorities = set(occ.vehicle_priority for occ in occupations)
        if len(priorities) == 1 and len(occupations) == 2:
            return ResolutionStrategy.NEGOTIATED_ADJUSTMENT
        
        return ResolutionStrategy.FIRST_COME_FIRST_SERVE
    
    def _calculate_integrated_negotiation_options(self, occupations: List[NetworkAwareSegmentOccupation]) -> Dict:
        """计算集成的协商选项"""
        if len(occupations) != 2:
            return {}
        
        occ1, occ2 = occupations
        overlap = self._calculate_integrated_overlap(occ1, occ2)
        
        # 网络感知的协商选项
        options = {
            'network_aware_adjustment': {
                'vehicle1_delay': overlap * 0.3,
                'vehicle2_delay': overlap * 0.7,
                'total_delay': overlap,
                'efficiency_cost': overlap * 0.4,  # 网络感知可以减少成本
                'network_optimization': True
            },
            'stage_aware_coordination': {
                'stage_based_priority': True,
                'inter_stage_buffer_adjustment': True,
                'transition_coordination': occ1.stage_transition or occ2.stage_transition
            }
        }
        
        # 层次网络特殊选项
        if occ1.network_topology_type == "hierarchical":
            options['hierarchy_optimization'] = {
                'trunk_priority': occ1.hierarchy_level == "trunk" or occ2.hierarchy_level == "trunk",
                'level_based_adjustment': True
            }
        
        return options
    
    # ==================== 占位符方法（需要具体实现） ====================
    
    def _update_spatial_occupations_integrated(self):
        """更新集成的空间占用数据"""
        if not self.spatial_detector:
            return
        
        # 清理旧数据
        self.spatial_detector.spatial_occupations.clear()
        self.spatial_detector.vehicle_spatial_occupations.clear()
        
        # 从网络感知占用记录生成空间占用
        for segment_id, occupations in self.segment_occupations.items():
            for occupation in occupations:
                try:
                    # 解析segment_id
                    parts = segment_id.split('_')
                    if len(parts) >= 3:
                        backbone_path_id = '_'.join(parts[:-2])
                        start_node = int(parts[-2])
                        end_node = int(parts[-1])
                        
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
                    continue
    
    def _convert_spatial_to_integrated_conflict(self, spatial_conflict) -> Optional[IntegratedBackboneConflict]:
        """将空间冲突转换为集成冲突格式"""
        try:
            vehicle_ids = spatial_conflict.vehicle_ids
            if len(vehicle_ids) < 2:
                return None
            
            # 获取车辆的占用信息
            occupations = []
            for vehicle_id in vehicle_ids:
                for segment_id, segment_occupations in self.segment_occupations.items():
                    for occ in segment_occupations:
                        if occ.vehicle_id == vehicle_id:
                            occupations.append(occ)
                            break
            
            if len(occupations) < 2:
                return None
            
            # 确定优先级顺序
            priority_order = self._determine_integrated_priority_order(occupations)
            
            # 确定严重程度
            severity = ConflictSeverity.MEDIUM
            if len(vehicle_ids) > 2:
                severity = ConflictSeverity.HIGH
            if spatial_conflict.total_overlap_duration > 60:
                severity = ConflictSeverity.HIGH
            
            # 建议解决策略
            suggested_resolution = ResolutionStrategy.NEGOTIATED_ADJUSTMENT
            if len(vehicle_ids) > 2:
                suggested_resolution = ResolutionStrategy.ALTERNATIVE_BACKBONE_PATH
            
            # 计算时间窗口
            min_time = min(occ.planned_entry_time for occ in occupations)
            max_time = max(occ.planned_exit_time for occ in occupations)
            
            conflict_id = f"spatial_integrated_{spatial_conflict.spatial_segment_id}_{int(time.time() * 1000)}"
            
            conflict = IntegratedBackboneConflict(
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
                spatial_conflict=True,
                network_topology_type=self.current_network_topology.value
            )
            
            return conflict
        
        except Exception as e:
            print(f"    转换空间冲突失败: {e}")
            return None
    
    def _analyze_stage_transition_conflicts(self, transition_vehicles: List[str]) -> List[IntegratedBackboneConflict]:
        """分析阶段转换冲突"""
        conflicts = []
        # 占位符实现
        return conflicts
    
    def _detect_cross_stage_conflicts(self) -> List[IntegratedBackboneConflict]:
        """检测跨阶段冲突"""
        conflicts = []
        # 占位符实现
        return conflicts
    
    def _detect_trajectory_based_conflicts(self, prediction_end: float) -> List[IntegratedBackboneConflict]:
        """基于轨迹的冲突检测"""
        conflicts = []
        # 占位符实现
        return conflicts
    
    def _detect_stage_transition_prediction_conflicts(self, prediction_end: float) -> List[IntegratedBackboneConflict]:
        """阶段转换预测冲突检测"""
        conflicts = []
        # 占位符实现
        return conflicts
    
    # ==================== 兼容性接口 ====================
    
    def update_vehicle_info(self, vehicle_id: str, vehicle_info: Dict):
        """兼容性车辆信息更新方法"""
        self.update_vehicle_info_integrated(vehicle_id, vehicle_info)
    
    def record_vehicle_backbone_occupation(self, vehicle_id: str, backbone_path_id: str,
                                         node_timing_plan: Dict[int, Tuple[float, float]],
                                         vehicle_priority: int = 2,
                                         segment_risk_info: Dict = None) -> bool:
        """兼容性占用记录方法"""
        task_context = segment_risk_info or {}
        return self.record_vehicle_backbone_occupation_integrated(
            vehicle_id, backbone_path_id, node_timing_plan, vehicle_priority, task_context
        )
    
    def detect_backbone_conflicts(self) -> List[IntegratedBackboneConflict]:
        """兼容性冲突检测方法"""
        return self.detect_backbone_conflicts_integrated()
    
    # ==================== 状态管理和清理 ====================
    
    def get_active_conflicts(self) -> List[IntegratedBackboneConflict]:
        """获取活跃冲突"""
        with self.lock:
            return list(self.detected_conflicts.values())
    
    def get_vehicle_conflicts(self, vehicle_id: str) -> List[IntegratedBackboneConflict]:
        """获取车辆冲突"""
        with self.lock:
            conflicts = []
            for conflict in self.detected_conflicts.values():
                if vehicle_id in conflict.conflicting_vehicles:
                    conflicts.append(conflict)
            return conflicts
    
    def mark_conflict_resolved(self, conflict_id: str, resolution_method: str):
        """标记冲突已解决"""
        with self.lock:
            if conflict_id in self.detected_conflicts:
                conflict = self.detected_conflicts[conflict_id]
                conflict.is_resolved = True
                conflict.resolution_time = time.time()
                
                self.resolved_conflicts.append(conflict)
                self.conflict_history.append(conflict)
                del self.detected_conflicts[conflict_id]
                
                # 更新统计
                if resolution_method == 'negotiated_adjustment':
                    self.stats['negotiated_resolutions'] += 1
                elif resolution_method == 'preventive_rescheduling':
                    self.stats['preventive_adjustments'] += 1
    
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
            avg_detection_time = 0.0
            detection_times = list(self.performance_monitor['detection_times'])
            if detection_times:
                avg_detection_time = sum(detection_times) / len(detection_times)
            
            status = {
                'active_segments': len(self.segment_occupations),
                'active_vehicles': len(self.vehicle_occupations),
                'active_conflicts': len(self.detected_conflicts),
                'resolved_conflicts': len(self.resolved_conflicts),
                'current_network_topology': self.current_network_topology.value,
                
                # 集成统计
                'integrated_statistics': self.stats.copy(),
                'performance_metrics': {
                    'average_detection_time': avg_detection_time,
                    'detection_cycles': self.stats['integrated_detection_cycles'],
                    'enhanced_suggestions': self.stats['enhanced_resolution_suggestions']
                },
                
                # 网络感知统计
                'network_awareness': {
                    'topology_adaptations': self.stats['network_topology_adaptations'],
                    'hierarchy_aware_conflicts': self.stats['hierarchy_aware_conflicts'],
                    'consolidation_aware_conflicts': self.stats['consolidation_aware_conflicts']
                },
                
                # 多阶段任务统计
                'multi_stage_awareness': {
                    'multi_stage_conflicts': self.stats['multi_stage_conflicts_detected'],
                    'stage_transition_conflicts': self.stats['stage_transition_conflicts'],
                    'cross_stage_optimizations': self.stats['cross_stage_optimizations']
                },
                
                # 空间检测统计
                'spatial_detection': {
                    'enabled': self.spatial_detector_integration,
                    'spatial_conflicts': self.stats['spatial_conflicts_detected'],
                    'supplement_conflicts': self.stats['spatial_supplement_conflicts']
                }
            }
            
            return status
    
    def shutdown(self):
        """关闭冲突检测器"""
        with self.lock:
            self.segment_occupations.clear()
            self.vehicle_occupations.clear()
            self.vehicle_info_cache.clear()
            self.trajectory_cache.clear()
            self.detected_conflicts.clear()
            self.resolved_conflicts.clear()
            self.conflict_history.clear()
        
        print("整合优化版骨干路径冲突检测器已关闭")


# 兼容性别名
EnhancedBackboneConflictDetector = IntegratedBackboneConflictDetector
BackboneConflict = IntegratedBackboneConflict
BackboneSegmentOccupation = NetworkAwareSegmentOccupation