"""
integrated_vehicle_scheduler.py - 整合优化版车辆调度器
完美配合网络整理、双重冲突检测、智能交通管理的多阶段任务调度系统
"""

import math
import time
import threading
import uuid
from collections import defaultdict, deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Tuple, Optional, Any, Set

# 导入整合组件
try:
    from integrated_traffic_manager import IntegratedBackboneTrafficManager, NetworkTopologyType
    INTEGRATED_TRAFFIC_AVAILABLE = True
except ImportError:
    from traffic_manager import EnhancedBackboneTrafficManager
    INTEGRATED_TRAFFIC_AVAILABLE = False
    NetworkTopologyType = None

try:
    from integrated_planner_config import IntegratedPlannerConfig, TaskStageType
    INTEGRATED_CONFIG_AVAILABLE = True
except ImportError:
    INTEGRATED_CONFIG_AVAILABLE = False
    TaskStageType = None

from conflict_control import EnhancedBackboneConflictDetector

class TaskStatus(Enum):
    """任务状态"""
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    SUSPENDED = "suspended"

class VehicleStatus(Enum):
    """车辆状态"""
    IDLE = "idle"
    MOVING = "moving"
    LOADING = "loading"
    UNLOADING = "unloading"
    WAITING = "waiting"
    PLANNING = "planning"
    CONFLICT_RESOLVING = "conflict_resolving"

class TaskPriority(Enum):
    """任务优先级"""
    LOW = 1
    NORMAL = 2
    HIGH = 3
    URGENT = 4
    CRITICAL = 5

class TaskStage(Enum):
    """任务阶段 - 与配置系统兼容"""
    LOADING = "loading"
    TRANSPORT = "transport"
    UNLOADING = "unloading"
    PARKING = "parking"
    MAINTENANCE = "maintenance"

@dataclass
class NetworkAwareTaskStageInfo:
    """网络感知的任务阶段信息"""
    stage: TaskStage
    location: Tuple[float, float, float]
    target_type: str
    target_id: int
    estimated_duration: float = 60.0
    completed: bool = False
    
    # 网络感知属性
    preferred_hierarchy_level: str = "trunk"  # trunk, branch, connector
    spatial_conflict_risk: float = 0.0
    consolidation_aware: bool = False
    alternative_targets: List[Tuple[str, int]] = field(default_factory=list)

@dataclass
class IntegratedBackboneTask:
    """整合优化版骨干网络任务"""
    task_id: str
    vehicle_id: Optional[str]
    task_type: str
    priority: TaskPriority
    
    # 多阶段信息 - 增强版
    stages: List[NetworkAwareTaskStageInfo] = field(default_factory=list)
    current_stage_index: int = 0
    
    # 兼容性字段
    start_location: Optional[Tuple[float, float, float]] = None
    end_location: Optional[Tuple[float, float, float]] = None
    target_type: Optional[str] = None
    target_id: Optional[int] = None
    
    # 时间约束
    earliest_start_time: float = 0.0
    latest_finish_time: float = 0.0
    estimated_duration: float = 0.0
    
    # 状态信息
    status: TaskStatus = TaskStatus.PENDING
    assigned_time: float = 0.0
    completion_time: float = 0.0
    request_time: float = field(default_factory=time.time)
    
    # 路径信息 - 网络感知版
    assigned_backbone_path_id: Optional[str] = None
    complete_path: List[Tuple] = field(default_factory=list)
    path_structure: Dict = field(default_factory=dict)
    node_timing_plan: Dict[int, Tuple[float, float]] = field(default_factory=dict)
    
    # 网络整理感知
    network_topology_type: str = "original"
    consolidation_info: Dict = field(default_factory=dict)
    hierarchy_aware_planning: bool = False
    spatial_conflict_considered: bool = False
    
    # 重试和优化
    retry_count: int = 0
    path_optimization_attempts: int = 0
    stage_transition_optimization: bool = False

    def is_complete(self) -> bool:
        """检查任务是否完成"""
        return self.status == TaskStatus.COMPLETED

    def is_failed(self) -> bool:
        """检查任务是否失败"""
        return self.status == TaskStatus.FAILED

    def is_active(self) -> bool:
        """检查任务是否处于活跃状态（已分配或进行中）"""
        return self.status in [TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS]

    def is_pending(self) -> bool:
        """检查任务是否等待分配"""
        return self.status == TaskStatus.PENDING

    def is_suspended(self) -> bool:
        """检查任务是否被暂停"""
        return self.status == TaskStatus.SUSPENDED

    def complete_task(self) -> bool:
        """标记任务完成"""
        import time
        
        if self.status not in [TaskStatus.IN_PROGRESS, TaskStatus.ASSIGNED]:
            return False
        
        # 标记所有阶段完成
        for stage in self.stages:
            stage.completed = True
        
        # 更新任务状态
        self.status = TaskStatus.COMPLETED
        self.completion_time = time.time()
        
        # 推进到最后阶段
        self.current_stage_index = len(self.stages) - 1
        
        return True

    def fail_task(self, reason: str = "unknown") -> bool:
        """标记任务失败"""
        import time
        
        self.status = TaskStatus.FAILED
        self.completion_time = time.time()
        
        # 在整理信息中记录失败原因
        if 'failure_info' not in self.consolidation_info:
            self.consolidation_info['failure_info'] = {}
        
        self.consolidation_info['failure_info'] = {
            'reason': reason,
            'failed_at_stage': self.current_stage_index,
            'failure_time': self.completion_time
        }
        
        return True

    def suspend_task(self, reason: str = "conflict_resolution") -> bool:
        """暂停任务"""
        if self.status not in [TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS]:
            return False
        
        self.status = TaskStatus.SUSPENDED
        
        # 记录暂停信息
        if 'suspension_info' not in self.consolidation_info:
            self.consolidation_info['suspension_info'] = {}
        
        self.consolidation_info['suspension_info'] = {
            'reason': reason,
            'suspended_at_stage': self.current_stage_index,
            'suspension_time': time.time()
        }
        
        return True

    def resume_task(self) -> bool:
        """恢复任务"""
        if self.status != TaskStatus.SUSPENDED:
            return False
        
        # 根据之前的状态恢复
        if self.vehicle_id:
            self.status = TaskStatus.IN_PROGRESS if self.assigned_time > 0 else TaskStatus.ASSIGNED
        else:
            self.status = TaskStatus.PENDING
        
        # 清除暂停信息
        if 'suspension_info' in self.consolidation_info:
            self.consolidation_info['suspension_info']['resumed_time'] = time.time()
        
        return True

    def get_current_stage(self) -> Optional['NetworkAwareTaskStageInfo']:
        """获取当前任务阶段"""
        if not self.stages or self.current_stage_index >= len(self.stages):
            return None
        
        return self.stages[self.current_stage_index]

    def get_next_stage(self) -> Optional['NetworkAwareTaskStageInfo']:
        """获取下一个任务阶段"""
        next_index = self.current_stage_index + 1
        if next_index >= len(self.stages):
            return None
        
        return self.stages[next_index]

    def advance_to_next_stage(self) -> bool:
        """推进到下一个阶段"""
        if self.current_stage_index + 1 >= len(self.stages):
            # 已经是最后阶段，标记任务完成
            return self.complete_task()
        
        # 标记当前阶段完成
        current_stage = self.get_current_stage()
        if current_stage:
            current_stage.completed = True
        
        # 推进到下一阶段
        self.current_stage_index += 1
        
        # 更新任务的目标信息
        new_current_stage = self.get_current_stage()
        if new_current_stage:
            self.target_type = new_current_stage.target_type
            self.target_id = new_current_stage.target_id
        
        return True

    def is_multi_stage(self) -> bool:
        """检查是否是多阶段任务"""
        return len(self.stages) > 1

    def get_completion_progress(self) -> float:
        """获取完成进度（0.0-1.0）"""
        if not self.stages:
            return 0.0
        
        if self.status == TaskStatus.COMPLETED:
            return 1.0
        
        if self.status == TaskStatus.FAILED:
            return 0.0
        
        # 计算已完成的阶段
        completed_stages = sum(1 for stage in self.stages if stage.completed)
        
        # 如果当前阶段正在进行，给予部分进度
        if self.status == TaskStatus.IN_PROGRESS and not self.stages[self.current_stage_index].completed:
            completed_stages += 0.5
        
        return min(1.0, completed_stages / len(self.stages))

    def get_remaining_stages(self) -> List['NetworkAwareTaskStageInfo']:
        """获取剩余的任务阶段"""
        if self.current_stage_index >= len(self.stages):
            return []
        
        return self.stages[self.current_stage_index:]

    def get_completed_stages(self) -> List['NetworkAwareTaskStageInfo']:
        """获取已完成的任务阶段"""
        return [stage for stage in self.stages if stage.completed]

    def get_total_estimated_duration(self) -> float:
        """获取总预估持续时间"""
        if hasattr(self, '_cached_total_duration'):
            return self._cached_total_duration
        
        total_duration = 0.0
        
        for i, stage in enumerate(self.stages):
            total_duration += stage.estimated_duration
            
            # 添加阶段间的移动时间
            if i < len(self.stages) - 1:
                next_stage = self.stages[i + 1]
                distance = math.sqrt(
                    (stage.location[0] - next_stage.location[0])**2 +
                    (stage.location[1] - next_stage.location[1])**2
                )
                travel_time = distance / 1.5  # 假设平均速度1.5m/s
                total_duration += travel_time
        
        self._cached_total_duration = total_duration
        return total_duration

    def get_remaining_duration(self) -> float:
        """获取剩余持续时间"""
        remaining_stages = self.get_remaining_stages()
        if not remaining_stages:
            return 0.0
        
        total_remaining = 0.0
        
        for i, stage in enumerate(remaining_stages):
            # 如果是当前阶段且正在进行，只计算剩余部分
            if i == 0 and self.status == TaskStatus.IN_PROGRESS:
                total_remaining += stage.estimated_duration * 0.5  # 假设已完成一半
            else:
                total_remaining += stage.estimated_duration
            
            # 添加阶段间移动时间
            if i < len(remaining_stages) - 1:
                next_stage = remaining_stages[i + 1]
                distance = math.sqrt(
                    (stage.location[0] - next_stage.location[0])**2 +
                    (stage.location[1] - next_stage.location[1])**2
                )
                travel_time = distance / 1.5
                total_remaining += travel_time
        
        return total_remaining

    def get_elapsed_time(self) -> float:
        """获取已经过的时间"""
        import time
        
        if self.status == TaskStatus.PENDING:
            return 0.0
        
        if self.assigned_time == 0:
            return 0.0
        
        end_time = self.completion_time if self.completion_time > 0 else time.time()
        return end_time - self.assigned_time

    def is_overdue(self) -> bool:
        """检查任务是否超期"""
        import time
        
        if self.latest_finish_time == 0:
            return False
        
        current_time = time.time()
        return current_time > self.latest_finish_time and not self.is_complete()

    def get_priority_score(self) -> float:
        """获取优先级分数（考虑紧急程度）"""
        import time
        
        base_score = self.priority.value * 20
        
        # 时间紧急程度
        if self.latest_finish_time > 0:
            current_time = time.time()
            time_remaining = self.latest_finish_time - current_time
            
            if time_remaining < 0:
                # 已超期
                urgency_score = 50
            elif time_remaining < 300:  # 5分钟内
                urgency_score = 30
            elif time_remaining < 900:  # 15分钟内
                urgency_score = 20
            else:
                urgency_score = 0
            
            base_score += urgency_score
        
        # 多阶段任务加权
        if self.is_multi_stage():
            base_score += 10
        
        # 网络拓扑复杂度加权
        if self.network_topology_type == "hierarchical":
            base_score += 5
        elif self.network_topology_type == "consolidated":
            base_score += 3
        
        return base_score

    def needs_replanning(self) -> bool:
        """检查是否需要重新规划"""
        # 检查路径优化尝试次数
        if self.path_optimization_attempts > 3:
            return True
        
        # 检查重试次数
        if self.retry_count > 2:
            return True
        
        # 检查是否有冲突导致的长时间停滞
        if self.status == TaskStatus.SUSPENDED:
            suspension_info = self.consolidation_info.get('suspension_info', {})
            if suspension_info:
                suspension_time = suspension_info.get('suspension_time', 0)
                if time.time() - suspension_time > 300:  # 暂停超过5分钟
                    return True
        
        # 检查是否超期
        if self.is_overdue():
            return True
        
        return False

    def can_be_optimized(self) -> bool:
        """检查是否可以进行优化"""
        # 已完成或失败的任务不能优化
        if self.status in [TaskStatus.COMPLETED, TaskStatus.FAILED]:
            return False
        
        # 优化尝试次数限制
        if self.path_optimization_attempts >= 5:
            return False
        
        # 只有已分配的任务才能优化
        if self.status not in [TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS, TaskStatus.SUSPENDED]:
            return False
        
        return True

    def mark_optimization_attempt(self):
        """标记优化尝试"""
        self.path_optimization_attempts += 1

    def get_task_summary(self) -> Dict:
        """获取任务摘要信息"""
        import time
        
        return {
            'task_id': self.task_id,
            'task_type': self.task_type,
            'status': self.status.value,
            'priority': self.priority.value,
            'vehicle_id': self.vehicle_id,
            'is_complete': self.is_complete(),
            'is_failed': self.is_failed(),
            'is_active': self.is_active(),
            'is_multi_stage': self.is_multi_stage(),
            'is_overdue': self.is_overdue(),
            'needs_replanning': self.needs_replanning(),
            'can_be_optimized': self.can_be_optimized(),
            'completion_progress': self.get_completion_progress(),
            'total_stages': len(self.stages),
            'current_stage_index': self.current_stage_index,
            'completed_stages': len(self.get_completed_stages()),
            'remaining_stages': len(self.get_remaining_stages()),
            'estimated_duration': self.estimated_duration,
            'remaining_duration': self.get_remaining_duration(),
            'elapsed_time': self.get_elapsed_time(),
            'priority_score': self.get_priority_score(),
            'retry_count': self.retry_count,
            'path_optimization_attempts': self.path_optimization_attempts,
            'network_topology': self.network_topology_type,
            'hierarchy_aware': self.hierarchy_aware_planning,
            'spatial_conflict_considered': self.spatial_conflict_considered,
            'stage_transition_optimization': self.stage_transition_optimization,
            'assigned_backbone_path_id': self.assigned_backbone_path_id,
            'request_time': self.request_time,
            'assigned_time': self.assigned_time,
            'completion_time': self.completion_time,
            'latest_finish_time': self.latest_finish_time
        }
@dataclass 
class IntegratedVehicleState:
    """整合优化版车辆状态"""
    vehicle_id: str
    current_position: Tuple[float, float, float]
    current_status: VehicleStatus
    priority: int = 2
    
    # 通行状态管理
    passing_status: int = 0  # 0=通行中，1=停车中
    
    # 任务信息
    current_task_id: Optional[str] = None
    task_queue: List[str] = field(default_factory=list)
    
    # 骨干网络信息 - 网络感知版
    current_backbone_path_id: Optional[str] = None
    backbone_segment_occupations: List[str] = field(default_factory=list)
    network_topology_awareness: str = "original"
    hierarchy_level_preference: str = "trunk"
    
    # 性能参数
    max_speed: float = 1.5
    current_speed: float = 0.0
    last_update_time: float = field(default_factory=time.time)
    
    # 增强统计信息
    total_distance: float = 0.0
    total_tasks_completed: int = 0
    backbone_switches: int = 0
    interface_switches: int = 0
    conflict_involvements: int = 0
    network_topology_adaptations: int = 0
    multi_stage_tasks_completed: int = 0
    consolidation_aware_tasks: int = 0
    
    def update_position(self, new_position: Tuple[float, float, float], time_delta: float):
        """更新车辆位置"""
        if self.current_position:
            distance = math.sqrt(
                (new_position[0] - self.current_position[0])**2 +
                (new_position[1] - self.current_position[1])**2
            )
            self.total_distance += distance
            
            if time_delta > 0:
                self.current_speed = distance / time_delta
        
        self.current_position = new_position
        self.last_update_time = time.time()
    
    def adapt_to_network_topology(self, topology_type: str, consolidation_info: Dict = None):
        """适应网络拓扑变化"""
        if self.network_topology_awareness != topology_type:
            self.network_topology_awareness = topology_type
            self.network_topology_adaptations += 1
            
            # 根据网络类型调整偏好
            if topology_type == "hierarchical":
                self.hierarchy_level_preference = "trunk"
            elif topology_type == "consolidated":
                self.hierarchy_level_preference = "branch"

class IntegratedBackboneVehicleScheduler:
    """整合优化版骨干网络车辆调度器"""
    
    def __init__(self, env, backbone_network, path_planner, conflict_detector, traffic_manager):
        self.env = env
        self.backbone_network = backbone_network
        self.path_planner = path_planner
        self.conflict_detector = conflict_detector
        self.traffic_manager = traffic_manager
        
        # 检测组件集成状态
        self.integrated_traffic_available = INTEGRATED_TRAFFIC_AVAILABLE
        self.integrated_config_available = INTEGRATED_CONFIG_AVAILABLE
        
        # 状态管理 - 整合版
        self.vehicle_states: Dict[str, IntegratedVehicleState] = {}
        self.tasks: Dict[str, IntegratedBackboneTask] = {}
        self.task_queue = deque()
        
        # 网络拓扑感知
        self.current_network_topology = "original"
        self.consolidation_info = {}
        self.topology_change_callbacks = []
        
        # 任务ID生成 - 增强版
        self.task_counter = 0
        self.task_id_lock = threading.Lock()
        
        # 分配策略 - 网络感知版
        self.assignment_strategies = {
            'nearest_vehicle': self._assign_nearest_vehicle,
            'most_efficient': self._assign_most_efficient,
            'backbone_aware': self._assign_backbone_aware_enhanced,
            'conflict_minimal': self._assign_conflict_minimal_enhanced,
            'random_balanced': self._assign_random_balanced,
            'round_robin': self._assign_round_robin,
            'network_topology_aware': self._assign_network_topology_aware,
            'hierarchy_optimized': self._assign_hierarchy_optimized,
            'consolidation_aware': self._assign_consolidation_aware
        }
        
        # 配置参数 - 整合优化版
        self.config = {
            'default_assignment_strategy': 'network_topology_aware',
            'max_tasks_per_vehicle': 3,
            'task_timeout': 3600.0,
            'replanning_threshold': 0.3,
            'interface_spacing': 8,
            'node_stop_time': 2.0,
            'safety_time_margin': 5.0,
            'max_backbone_load_factor': 0.8,
            'enable_proactive_optimization': True,
            'conflict_avoidance_weight': 0.4,
            'backbone_preference_weight': 0.6,
            
            # 网络整理感知配置
            'network_topology_adaptation': True,
            'consolidation_aware_planning': True,
            'hierarchy_aware_assignment': True,
            'spatial_conflict_prevention': True,
            'topology_change_response_time': 30.0,
            
            # 多阶段任务优化
            'multi_stage_optimization': True,
            'inter_stage_planning': True,
            'stage_transition_buffer': 15.0,
            'cross_stage_conflict_prevention': True,
            'stage_aware_path_selection': True,
            
            # 集成交通管理配置
            'integrated_traffic_management': True,
            'intelligent_recovery_support': True,
            'network_aware_conflict_resolution': True
        }
        
        # 轮询分配状态
        self.last_assigned_vehicle_index = 0
        
        # 统计信息 - 全面版
        self.stats = {
            'total_tasks_created': 0,
            'total_tasks_assigned': 0,
            'total_tasks_completed': 0,
            'total_tasks_failed': 0,
            'average_assignment_time': 0.0,
            'average_completion_time': 0.0,
            'backbone_utilization': 0.0,
            'vehicle_efficiency': 0.0,
            'conflict_resolution_success_rate': 0.0,
            'backbone_path_switches': 0,
            'interface_node_switches': 0,
            'temporal_adjustments': 0,
            
            # 网络感知统计
            'network_topology_adaptations': 0,
            'consolidation_aware_assignments': 0,
            'hierarchy_optimized_assignments': 0,
            'spatial_conflict_prevented_assignments': 0,
            
            # 多阶段任务统计
            'multi_stage_tasks_created': 0,
            'multi_stage_tasks_completed': 0,
            'stage_transitions_optimized': 0,
            'inter_stage_conflicts_prevented': 0,
            
            # 集成系统统计
            'integrated_traffic_interactions': 0,
            'intelligent_recovery_assists': 0,
            'config_adaptations': 0
        }
        
        # 性能监控 - 增强版
        self.performance_monitor = {
            'assignment_times': deque(maxlen=100),
            'completion_times': deque(maxlen=100),
            'conflict_events': deque(maxlen=200),
            'topology_changes': deque(maxlen=50),
            'stage_transitions': deque(maxlen=100)
        }
        
        # 线程安全
        self.lock = threading.RLock()
        
        # 设置组件间引用
        if self.traffic_manager:
            self.traffic_manager.set_vehicle_scheduler(self)
        
        print("初始化整合优化版骨干网络车辆调度器")
        print(f"  整合交通管理: {'✅' if self.integrated_traffic_available else '⚠️'}")
        print(f"  整合配置系统: {'✅' if self.integrated_config_available else '⚠️'}")
        print(f"  网络拓扑感知: ✅")
        print(f"  多阶段任务优化: ✅")
    
    # ==================== 网络拓扑感知方法 ====================
    def _assign_nearest_vehicle(self, task: IntegratedBackboneTask) -> Optional[str]:
        """最近车辆分配策略"""
        available_vehicles = self._get_available_vehicles()
        if not available_vehicles:
            return None
        
        current_stage = task.get_current_stage()
        if not current_stage:
            return None
        
        target_location = current_stage.location
        best_vehicle = None
        min_distance = float('inf')
        
        for vehicle_id, vehicle_state in available_vehicles:
            distance = self._calculate_distance(target_location, vehicle_state.current_position)
            
            # 网络感知调整：考虑车辆的网络拓扑适应性
            topology_penalty = 0
            if vehicle_state.network_topology_awareness != self.current_network_topology:
                topology_penalty = distance * 0.1  # 10%的拓扑不匹配惩罚
            
            adjusted_distance = distance + topology_penalty
            
            if adjusted_distance < min_distance:
                min_distance = adjusted_distance
                best_vehicle = vehicle_id
        
        if best_vehicle:
            print(f"最近车辆分配: {best_vehicle} (距离: {min_distance:.1f}m)")
        
        return best_vehicle

    def _assign_most_efficient(self, task: IntegratedBackboneTask) -> Optional[str]:
        """最高效车辆分配策略"""
        available_vehicles = self._get_available_vehicles()
        if not available_vehicles:
            return None
        
        best_vehicle = None
        best_efficiency = -1
        
        for vehicle_id, vehicle_state in available_vehicles:
            # 计算车辆效率分数
            efficiency_score = self._calculate_vehicle_efficiency_score(vehicle_state, task)
            
            if efficiency_score > best_efficiency:
                best_efficiency = efficiency_score
                best_vehicle = vehicle_id
        
        if best_vehicle:
            print(f"最高效车辆分配: {best_vehicle} (效率: {best_efficiency:.2f})")
        
        return best_vehicle

    def _assign_random_balanced(self, task: IntegratedBackboneTask) -> Optional[str]:
        """随机平衡分配策略"""
        import random
        
        available_vehicles = self._get_available_vehicles()
        if not available_vehicles:
            return None
        
        # 根据车辆任务负载进行加权随机选择
        weighted_vehicles = []
        
        for vehicle_id, vehicle_state in available_vehicles:
            # 计算权重：任务越少权重越高
            current_tasks = len(vehicle_state.task_queue)
            weight = max(1, self.config['max_tasks_per_vehicle'] - current_tasks)
            
            # 网络拓扑匹配加权
            if vehicle_state.network_topology_awareness == self.current_network_topology:
                weight *= 1.2
            
            # 车辆历史表现加权
            if vehicle_state.total_tasks_completed > 0:
                success_rate = vehicle_state.total_tasks_completed / max(1, 
                    vehicle_state.total_tasks_completed + vehicle_state.conflict_involvements)
                weight *= (0.5 + success_rate * 0.5)
            
            weighted_vehicles.extend([vehicle_id] * int(weight * 10))
        
        if weighted_vehicles:
            selected_vehicle = random.choice(weighted_vehicles)
            print(f"随机平衡分配: {selected_vehicle}")
            return selected_vehicle
        
        return None

    def _assign_round_robin(self, task: IntegratedBackboneTask) -> Optional[str]:
        """轮询分配策略"""
        available_vehicles = self._get_available_vehicles()
        if not available_vehicles:
            return None
        
        # 按车辆ID排序以确保一致性
        sorted_vehicles = sorted(available_vehicles, key=lambda x: x[0])
        
        if not sorted_vehicles:
            return None
        
        # 使用轮询索引选择车辆
        selected_index = self.last_assigned_vehicle_index % len(sorted_vehicles)
        selected_vehicle_id = sorted_vehicles[selected_index][0]
        
        # 更新轮询索引
        self.last_assigned_vehicle_index = (self.last_assigned_vehicle_index + 1) % len(sorted_vehicles)
        
        print(f"轮询分配: {selected_vehicle_id} (索引: {selected_index})")
        return selected_vehicle_id

    # ==================== 缺失的辅助方法 ====================

    def _calculate_vehicle_efficiency_score(self, vehicle_state: IntegratedVehicleState, 
                                        task: IntegratedBackboneTask) -> float:
        """计算车辆效率分数"""
        base_score = 50.0  # 基础分数
        
        # 任务完成率
        if vehicle_state.total_tasks_completed > 0:
            completion_bonus = min(vehicle_state.total_tasks_completed * 5, 25)
            base_score += completion_bonus
        
        # 冲突参与惩罚
        conflict_penalty = vehicle_state.conflict_involvements * 3
        base_score -= conflict_penalty
        
        # 距离效率
        current_stage = task.get_current_stage()
        if current_stage:
            distance = self._calculate_distance(current_stage.location, vehicle_state.current_position)
            distance_efficiency = max(0, 50 - distance / 2)  # 距离越近效率越高
            base_score += distance_efficiency * 0.3
        
        # 网络拓扑适应性
        if vehicle_state.network_topology_awareness == self.current_network_topology:
            base_score += 10
        
        # 层次偏好匹配
        if current_stage:
            if vehicle_state.hierarchy_level_preference == current_stage.preferred_hierarchy_level:
                base_score += 8
        
        # 多阶段任务经验
        if len(task.stages) > 1 and vehicle_state.multi_stage_tasks_completed > 0:
            base_score += min(vehicle_state.multi_stage_tasks_completed * 3, 15)
        
        # 整理网络经验
        if (task.network_topology_type != "original" and 
            vehicle_state.consolidation_aware_tasks > 0):
            base_score += min(vehicle_state.consolidation_aware_tasks * 2, 10)
        
        return max(0, base_score)

    def get_vehicle_target_info(self, vehicle_id: str) -> Optional[Dict]:
        """获取车辆目标信息"""
        if vehicle_id not in self.vehicle_states:
            return None
        
        vehicle_state = self.vehicle_states[vehicle_id]
        
        if not vehicle_state.current_task_id:
            return None
        
        task = self.tasks.get(vehicle_state.current_task_id)
        if not task:
            return None
        
        current_stage = task.get_current_stage()
        if not current_stage:
            return None
        
        return {
            'target_type': current_stage.target_type,
            'target_id': current_stage.target_id,
            'target_location': current_stage.location,
            'task_id': task.task_id,
            'task_priority': task.priority.value,
            'network_topology': task.network_topology_type,
            'hierarchy_level': current_stage.preferred_hierarchy_level,
            'spatial_risk': current_stage.spatial_conflict_risk
        }

    def switch_vehicle_backbone_path(self, vehicle_id: str, new_path_id: str) -> bool:
        """切换车辆骨干路径"""
        if vehicle_id not in self.vehicle_states:
            print(f"❌ 车辆 {vehicle_id} 不存在")
            return False
        
        vehicle_state = self.vehicle_states[vehicle_id]
        old_path_id = vehicle_state.current_backbone_path_id
        
        try:
            # 检查新路径是否存在
            if self.backbone_network and new_path_id not in self.backbone_network.bidirectional_paths:
                print(f"❌ 骨干路径 {new_path_id} 不存在")
                return False
            
            # 清除旧的占用记录
            if old_path_id and self.conflict_detector:
                self.conflict_detector.release_vehicle_from_path(vehicle_id)
            
            # 更新车辆状态
            vehicle_state.current_backbone_path_id = new_path_id
            vehicle_state.backbone_switches += 1
            
            # 更新任务的路径信息
            if vehicle_state.current_task_id:
                task = self.tasks.get(vehicle_state.current_task_id)
                if task:
                    task.assigned_backbone_path_id = new_path_id
                    
                    # 重新规划路径
                    current_stage = task.get_current_stage()
                    if current_stage:
                        path_result = self._plan_integrated_backbone_path_to_stage(
                            task, vehicle_state, current_stage
                        )
                        
                        if path_result:
                            complete_path, structure = path_result
                            task.complete_path = complete_path
                            task.path_structure = structure
                            
                            # 重新计算时序计划
                            node_timing_plan = self._calculate_integrated_node_timing_plan(
                                complete_path, structure, vehicle_state, task
                            )
                            task.node_timing_plan = node_timing_plan
                            
                            # 重新记录占用
                            success = self._record_integrated_vehicle_occupation(
                                vehicle_id, new_path_id, node_timing_plan, task
                            )
                            
                            if success:
                                self.stats['backbone_path_switches'] += 1
                                print(f"✅ 车辆 {vehicle_id} 路径切换成功: {old_path_id} -> {new_path_id}")
                                return True
            
            print(f"❌ 车辆 {vehicle_id} 路径切换失败")
            return False
            
        except Exception as e:
            print(f"❌ 路径切换异常: {e}")
            return False

    def delay_vehicle_backbone_timing(self, vehicle_id: str, backbone_path_id: str, 
                                    delay_time: float) -> bool:
        """延迟车辆骨干路径时序"""
        if vehicle_id not in self.vehicle_states:
            return False
        
        vehicle_state = self.vehicle_states[vehicle_id]
        
        if not vehicle_state.current_task_id:
            return False
        
        task = self.tasks.get(vehicle_state.current_task_id)
        if not task or task.assigned_backbone_path_id != backbone_path_id:
            return False
        
        try:
            # 更新节点时序计划
            updated_timing_plan = {}
            for node_index, (arrival_time, departure_time) in task.node_timing_plan.items():
                updated_timing_plan[node_index] = (
                    arrival_time + delay_time,
                    departure_time + delay_time
                )
            
            task.node_timing_plan = updated_timing_plan
            
            # 更新任务时间约束
            task.earliest_start_time += delay_time
            task.latest_finish_time += delay_time
            
            # 重新记录占用（如果有冲突检测器）
            if self.conflict_detector:
                # 先清除旧记录
                self.conflict_detector.release_vehicle_from_path(vehicle_id)
                
                # 记录新的占用
                success = self._record_integrated_vehicle_occupation(
                    vehicle_id, backbone_path_id, updated_timing_plan, task
                )
                
                if success:
                    self.stats['temporal_adjustments'] += 1
                    print(f"✅ 车辆 {vehicle_id} 时序延迟 {delay_time:.1f}s")
                    return True
            else:
                # 没有冲突检测器的情况下直接成功
                self.stats['temporal_adjustments'] += 1
                print(f"✅ 车辆 {vehicle_id} 时序延迟 {delay_time:.1f}s")
                return True
            
            return False
            
        except Exception as e:
            print(f"❌ 时序延迟失败: {e}")
            return False    
    def update_network_topology_awareness(self, topology_type: str = None, 
                                        consolidation_info: Dict = None):
        """更新网络拓扑感知"""
        # 自动检测网络拓扑
        if not topology_type and self.backbone_network:
            if hasattr(self.backbone_network, 'get_improved_consolidation_info'):
                info = self.backbone_network.get_improved_consolidation_info()
                if info.get('is_consolidated', False):
                    topology_type = "hierarchical" if info.get('hierarchy_info') else "consolidated"
                else:
                    topology_type = "original"
                consolidation_info = info
        
        if topology_type and topology_type != self.current_network_topology:
            old_topology = self.current_network_topology
            self.current_network_topology = topology_type
            self.consolidation_info = consolidation_info or {}
            
            # 更新所有车辆的拓扑感知
            for vehicle_state in self.vehicle_states.values():
                vehicle_state.adapt_to_network_topology(topology_type, consolidation_info)
            
            # 触发拓扑变化回调
            self._handle_topology_change(old_topology, topology_type)
            
            self.stats['network_topology_adaptations'] += 1
            self.performance_monitor['topology_changes'].append({
                'time': time.time(),
                'old_topology': old_topology,
                'new_topology': topology_type,
                'consolidation_info': consolidation_info
            })
            
            print(f"🔄 网络拓扑感知更新: {old_topology} -> {topology_type}")
    
    def _handle_topology_change(self, old_topology: str, new_topology: str):
        """处理拓扑变化"""
        # 调整默认分配策略
        if new_topology == "hierarchical":
            self.config['default_assignment_strategy'] = 'hierarchy_optimized'
        elif new_topology == "consolidated":
            self.config['default_assignment_strategy'] = 'consolidation_aware'
        else:
            self.config['default_assignment_strategy'] = 'network_topology_aware'
        
        # 重新评估当前任务的路径
        if self.config.get('network_topology_adaptation', True):
            self._reevaluate_active_tasks_for_topology_change(new_topology)
    
    def _reevaluate_active_tasks_for_topology_change(self, new_topology: str):
        """重新评估活跃任务以适应拓扑变化"""
        active_tasks = [task for task in self.tasks.values() 
                       if task.status in [TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS]]
        
        for task in active_tasks:
            if task.vehicle_id and task.vehicle_id in self.vehicle_states:
                # 标记需要路径重新评估
                task.network_topology_type = new_topology
                task.path_optimization_attempts = 0  # 重置优化尝试
                
                # 如果是层次网络，尝试优化到主干路径
                if new_topology == "hierarchical" and task.assigned_backbone_path_id:
                    self._optimize_task_for_hierarchy(task)
    
    def _optimize_task_for_hierarchy(self, task: IntegratedBackboneTask):
        """为层次网络优化任务"""
        if not self.backbone_network:
            return
        
        current_stage = task.get_current_stage()
        if not current_stage:
            return
        
        # 查找主干路径选项
        trunk_paths = []
        if hasattr(self.backbone_network, 'find_alternative_backbone_paths'):
            alternatives = self.backbone_network.find_alternative_backbone_paths(
                current_stage.target_type, current_stage.target_id,
                exclude_path_id=task.assigned_backbone_path_id
            )
            
            # 筛选主干路径
            for path in alternatives:
                if (hasattr(path, 'consolidation_info') and 
                    path.consolidation_info.get('hierarchy_level') == 'trunk'):
                    trunk_paths.append(path)
        
        if trunk_paths:
            # 选择最佳主干路径
            best_trunk = min(trunk_paths, key=lambda p: p.get_load_factor())
            
            if best_trunk.get_load_factor() < 0.7:  # 负载可接受
                # 尝试切换到主干路径
                if self.switch_vehicle_backbone_path(task.vehicle_id, best_trunk.path_id):
                    task.hierarchy_aware_planning = True
                    print(f"  ✅ 任务 {task.task_id} 优化到主干路径 {best_trunk.path_id}")
    
    # ==================== 增强的分配策略 ====================
    
    def _assign_network_topology_aware(self, task: IntegratedBackboneTask) -> Optional[str]:
        """网络拓扑感知分配策略"""
        available_vehicles = self._get_available_vehicles()
        if not available_vehicles:
            return None
        
        # 根据当前网络拓扑选择最佳策略
        if self.current_network_topology == "hierarchical":
            return self._assign_hierarchy_optimized(task)
        elif self.current_network_topology == "consolidated":
            return self._assign_consolidation_aware(task)
        else:
            return self._assign_backbone_aware_enhanced(task)
    
    def _assign_hierarchy_optimized(self, task: IntegratedBackboneTask) -> Optional[str]:
        """层次优化分配策略"""
        available_vehicles = self._get_available_vehicles()
        if not available_vehicles:
            return None
        
        best_vehicle = None
        best_score = float('inf')
        
        for vehicle_id, vehicle_state in available_vehicles:
            # 计算层次感知评分
            score = self._calculate_hierarchy_assignment_score(task, vehicle_state)
            
            if score < best_score:
                best_score = score
                best_vehicle = vehicle_id
        
        if best_vehicle:
            self.stats['hierarchy_optimized_assignments'] += 1
            print(f"层次优化分配: {best_vehicle} (评分: {best_score:.2f})")
        
        return best_vehicle
    
    def _assign_consolidation_aware(self, task: IntegratedBackboneTask) -> Optional[str]:
        """整理感知分配策略"""
        available_vehicles = self._get_available_vehicles()
        if not available_vehicles:
            return None
        
        best_vehicle = None
        best_score = float('inf')
        
        for vehicle_id, vehicle_state in available_vehicles:
            # 计算整理感知评分
            score = self._calculate_consolidation_assignment_score(task, vehicle_state)
            
            if score < best_score:
                best_score = score
                best_vehicle = vehicle_id
        
        if best_vehicle:
            self.stats['consolidation_aware_assignments'] += 1
            print(f"整理感知分配: {best_vehicle} (评分: {best_score:.2f})")
        
        return best_vehicle
    
    def _assign_backbone_aware_enhanced(self, task: IntegratedBackboneTask) -> Optional[str]:
        """增强的骨干网络感知分配"""
        available_vehicles = self._get_available_vehicles()
        if not available_vehicles:
            return None
        
        best_vehicle = None
        best_score = float('inf')
        
        for vehicle_id, vehicle_state in available_vehicles:
            # 增强评分计算，考虑空间冲突风险
            score = self._calculate_enhanced_backbone_assignment_score(task, vehicle_state)
            
            if score < best_score:
                best_score = score
                best_vehicle = vehicle_id
        
        return best_vehicle
    
    def _assign_conflict_minimal_enhanced(self, task: IntegratedBackboneTask) -> Optional[str]:
        """增强的冲突最小化分配"""
        available_vehicles = self._get_available_vehicles()
        if not available_vehicles:
            return None
        
        best_vehicle = None
        min_conflict_risk = float('inf')
        
        for vehicle_id, vehicle_state in available_vehicles:
            # 评估包括空间冲突在内的风险
            conflict_risk = self._assess_enhanced_conflict_risk(task, vehicle_state)
            
            if conflict_risk < min_conflict_risk:
                min_conflict_risk = conflict_risk
                best_vehicle = vehicle_id
        
        if best_vehicle:
            self.stats['spatial_conflict_prevented_assignments'] += 1
        
        return best_vehicle
    
    # ==================== 评分计算方法 ====================
    
    def _calculate_hierarchy_assignment_score(self, task: IntegratedBackboneTask, 
                                            vehicle_state: IntegratedVehicleState) -> float:
        """计算层次感知分配评分"""
        # 基础距离分数
        current_stage = task.get_current_stage()
        if not current_stage:
            return float('inf')
        
        distance = self._calculate_distance(current_stage.location, vehicle_state.current_position)
        distance_score = distance
        
        # 层次偏好匹配
        hierarchy_score = 0
        if vehicle_state.hierarchy_level_preference == current_stage.preferred_hierarchy_level:
            hierarchy_score = -20  # 偏好匹配奖励
        
        # 车辆层次适应性
        adaptation_score = vehicle_state.network_topology_adaptations * 5
        
        # 主干路径可用性
        trunk_availability_score = 0
        if self.backbone_network and current_stage.preferred_hierarchy_level == "trunk":
            trunk_paths = self._find_trunk_paths_to_target(current_stage.target_type, current_stage.target_id)
            if trunk_paths:
                min_load = min(p.get_load_factor() for p in trunk_paths)
                trunk_availability_score = min_load * 50  # 负载越高分数越高（越不好）
        
        total_score = distance_score + hierarchy_score + adaptation_score + trunk_availability_score
        return total_score
    
    def _calculate_consolidation_assignment_score(self, task: IntegratedBackboneTask,
                                                vehicle_state: IntegratedVehicleState) -> float:
        """计算整理感知分配评分"""
        # 基础评分
        base_score = self._calculate_enhanced_backbone_assignment_score(task, vehicle_state)
        
        # 整理网络适应性
        consolidation_bonus = 0
        if vehicle_state.consolidation_aware_tasks > 0:
            consolidation_bonus = -10  # 有整理任务经验的奖励
        
        # 整理路径偏好
        consolidated_path_bonus = 0
        if self.consolidation_info.get('consolidation_performed', False):
            # 检查是否有可用的整理路径
            current_stage = task.get_current_stage()
            if current_stage and self._has_consolidated_paths_to_target(
                current_stage.target_type, current_stage.target_id):
                consolidated_path_bonus = -15
        
        return base_score + consolidation_bonus + consolidated_path_bonus
    
    def _calculate_enhanced_backbone_assignment_score(self, task: IntegratedBackboneTask,
                                                    vehicle_state: IntegratedVehicleState) -> float:
        """计算增强的骨干网络分配评分"""
        current_stage = task.get_current_stage()
        if not current_stage:
            return float('inf')
        
        # 基础距离成本
        distance = self._calculate_distance(current_stage.location, vehicle_state.current_position)
        distance_score = distance
        
        # 骨干网络可用性评分
        backbone_score = 0
        if self.backbone_network:
            candidate_paths = self._find_paths_to_target(current_stage.target_type, current_stage.target_id)
            if candidate_paths:
                min_load = min(p.get_load_factor() for p in candidate_paths)
                backbone_score = min_load * 100
        
        # 车辆历史表现
        performance_score = (
            vehicle_state.conflict_involvements * 10 +
            vehicle_state.backbone_switches * 5 +
            vehicle_state.network_topology_adaptations * 3
        )
        
        # 空间冲突风险评估
        spatial_risk_score = 0
        if current_stage.spatial_conflict_risk > 0:
            spatial_risk_score = current_stage.spatial_conflict_risk * 20
        
        # 优先级调整
        priority_weight = (6 - task.priority.value) * 5
        
        total_score = (
            distance_score * 0.35 +
            backbone_score * 0.25 +
            performance_score * 0.15 +
            spatial_risk_score * 0.15 +
            priority_weight * 0.1
        )
        
        return total_score
    
    def _assess_enhanced_conflict_risk(self, task: IntegratedBackboneTask,
                                     vehicle_state: IntegratedVehicleState) -> float:
        """评估增强的冲突风险"""
        # 基础冲突风险
        historical_risk = vehicle_state.conflict_involvements * 0.2
        
        # 当前骨干路径负载风险
        backbone_risk = 0
        if vehicle_state.current_backbone_path_id and self.backbone_network:
            path_data = self.backbone_network.bidirectional_paths.get(
                vehicle_state.current_backbone_path_id
            )
            if path_data:
                backbone_risk = path_data.get_load_factor() * 10
        
        # 任务优先级风险
        priority_risk = (6 - task.priority.value) * 2
        
        # 空间冲突风险
        spatial_risk = 0
        current_stage = task.get_current_stage()
        if current_stage:
            spatial_risk = current_stage.spatial_conflict_risk * 15
        
        # 网络拓扑适应风险
        topology_risk = 0
        if vehicle_state.network_topology_awareness != self.current_network_topology:
            topology_risk = 5  # 拓扑不匹配的风险
        
        total_risk = historical_risk + backbone_risk + priority_risk + spatial_risk + topology_risk
        return total_risk
    
    # ==================== 任务创建方法 - 网络感知版 ====================
    
    def create_transport_task_integrated(self, start_location: Tuple, end_location: Tuple,
                                       priority: TaskPriority = TaskPriority.NORMAL,
                                       vehicle_id: str = None,
                                       network_aware: bool = True) -> str:
        """创建网络感知的运输任务"""
        task_id = self._generate_unique_task_id("transport")
        
        while task_id in self.tasks:
            task_id = self._generate_unique_task_id("transport")
        
        # 推断起始和结束位置的类型
        start_type, start_id = self._infer_target_from_position(start_location)
        end_type, end_id = self._infer_target_from_position(end_location)
        
        print(f"创建网络感知任务 {task_id}:")
        print(f"  网络拓扑: {self.current_network_topology}")
        print(f"  起始位置: {start_location} -> {start_type}_{start_id}")
        print(f"  结束位置: {end_location} -> {end_type}_{end_id}")
        
        # 创建网络感知的任务阶段
        stages = []
        
        if start_type == "loading":
            loading_stage = NetworkAwareTaskStageInfo(
                stage=TaskStage.LOADING,
                location=start_location,
                target_type=start_type,
                target_id=start_id,
                estimated_duration=60.0,
                preferred_hierarchy_level=self._determine_preferred_hierarchy_level(start_type, start_id),
                spatial_conflict_risk=self._assess_location_spatial_risk(start_location),
                consolidation_aware=network_aware and self.current_network_topology != "original"
            )
            stages.append(loading_stage)
            print(f"  添加装载阶段: L{start_id} (层次偏好: {loading_stage.preferred_hierarchy_level})")
        
        if end_type == "unloading":
            unloading_stage = NetworkAwareTaskStageInfo(
                stage=TaskStage.UNLOADING,
                location=end_location,
                target_type=end_type,
                target_id=end_id,
                estimated_duration=60.0,
                preferred_hierarchy_level=self._determine_preferred_hierarchy_level(end_type, end_id),
                spatial_conflict_risk=self._assess_location_spatial_risk(end_location),
                consolidation_aware=network_aware and self.current_network_topology != "original"
            )
            stages.append(unloading_stage)
            print(f"  添加卸载阶段: U{end_id} (层次偏好: {unloading_stage.preferred_hierarchy_level})")
        
        if end_type == "parking":
            parking_stage = NetworkAwareTaskStageInfo(
                stage=TaskStage.PARKING,
                location=end_location,
                target_type=end_type,
                target_id=end_id,
                estimated_duration=30.0,
                preferred_hierarchy_level="connector",  # 停车通常使用连接路径
                spatial_conflict_risk=self._assess_location_spatial_risk(end_location),
                consolidation_aware=network_aware
            )
            stages.append(parking_stage)
            print(f"  添加停车阶段: P{end_id}")
        
        # 估算总持续时间
        total_duration = sum(stage.estimated_duration for stage in stages)
        if len(stages) > 1:
            for i in range(len(stages) - 1):
                distance = self._calculate_distance(stages[i].location, stages[i+1].location)
                total_duration += distance / 1.5
        
        # 创建整合任务
        task = IntegratedBackboneTask(
            task_id=task_id,
            vehicle_id=vehicle_id,
            task_type="transport",
            priority=priority,
            stages=stages,
            start_location=start_location,
            end_location=end_location,
            earliest_start_time=time.time(),
            latest_finish_time=time.time() + total_duration * 3,
            estimated_duration=total_duration,
            network_topology_type=self.current_network_topology,
            consolidation_info=self.consolidation_info.copy(),
            hierarchy_aware_planning=network_aware and self.current_network_topology == "hierarchical",
            spatial_conflict_considered=network_aware,
            stage_transition_optimization=self.config.get('inter_stage_planning', True)
        )
        
        # 设置当前目标
        current_stage = task.get_current_stage()
        if current_stage:
            task.target_type = current_stage.target_type
            task.target_id = current_stage.target_id
        
        with self.lock:
            self.tasks[task_id] = task
            
            if not vehicle_id:
                self.task_queue.append(task_id)
            
            self.stats['total_tasks_created'] += 1
            if len(stages) > 1:
                self.stats['multi_stage_tasks_created'] += 1
        
        print(f"✅ 创建网络感知任务 {task_id}: {len(stages)}个阶段")
        return task_id
    
    # ==================== 任务分配 - 集成优化版 ====================
    
    def assign_task_integrated(self, task_id: str, vehicle_id: str = None) -> bool:
        """集成优化的任务分配"""
        assignment_start = time.time()
        
        if task_id not in self.tasks:
            print(f"❌ 任务不存在: {task_id}")
            return False
        
        task = self.tasks[task_id]
        
        with self.lock:
            # 更新网络拓扑感知
            self.update_network_topology_awareness()
            
            # 严格检查任务状态和归属
            if task.status != TaskStatus.PENDING:
                print(f"❌ 任务状态不正确: {task.status.value}")
                return False
            
            if task.vehicle_id is not None:
                if vehicle_id is None:
                    vehicle_id = task.vehicle_id
                elif vehicle_id != task.vehicle_id:
                    print(f"❌ 任务已预定给车辆 {task.vehicle_id}")
                    return False
            
            # 车辆选择 - 网络感知版
            if vehicle_id is None:
                strategy_name = self.config['default_assignment_strategy']
                strategy = self.assignment_strategies[strategy_name]
                selected_vehicle = strategy(task)
                
                if not selected_vehicle:
                    print(f"❌ 无法为任务 {task_id} 找到合适车辆")
                    return False
                
                vehicle_id = selected_vehicle
            
            # 检查车辆可用性
            if vehicle_id not in self.vehicle_states:
                print(f"❌ 车辆不存在: {vehicle_id}")
                return False
            
            vehicle_state = self.vehicle_states[vehicle_id]
            
            if vehicle_state.current_task_id is not None:
                print(f"❌ 车辆 {vehicle_id} 已有任务")
                return False
            
            print(f"\n🎯 [集成分配] 任务 {task_id} -> 车辆 {vehicle_id}")
            print(f"   网络拓扑: {self.current_network_topology}")
            print(f"   分配策略: {self.config['default_assignment_strategy']}")
            
            # 获取当前阶段
            current_stage = task.get_current_stage()
            if not current_stage:
                print(f"❌ 任务没有有效阶段")
                return False
            
            print(f"   当前阶段: {current_stage.stage.value} -> {current_stage.target_type}_{current_stage.target_id}")
            print(f"   层次偏好: {current_stage.preferred_hierarchy_level}")
            print(f"   空间风险: {current_stage.spatial_conflict_risk:.2f}")
            
            # 网络感知的路径规划
            path_result = self._plan_integrated_backbone_path_to_stage(task, vehicle_state, current_stage)
            if not path_result:
                print(f"❌ 网络感知路径规划失败")
                return False
            
            complete_path, structure = path_result
            backbone_path_id = structure.get('path_id')
            
            # 计算节点时序计划
            node_timing_plan = self._calculate_integrated_node_timing_plan(
                complete_path, structure, vehicle_state, task
            )
            
            # 记录到冲突检测器 - 增强版
            success = self._record_integrated_vehicle_occupation(
                vehicle_id, backbone_path_id or "direct", node_timing_plan, task
            )
            
            if not success:
                print(f"❌ 占用记录失败")
                return False
            
            # 检测并解决冲突
            self._handle_integrated_conflicts_for_vehicle(vehicle_id, task)
            
            # 原子性更新
            task.vehicle_id = vehicle_id
            task.status = TaskStatus.ASSIGNED
            task.assigned_time = time.time()
            task.assigned_backbone_path_id = backbone_path_id
            task.complete_path = complete_path
            task.path_structure = structure
            task.node_timing_plan = node_timing_plan
            
            # 更新车辆状态
            vehicle_state.current_task_id = task_id
            vehicle_state.task_queue.append(task_id)
            vehicle_state.current_status = VehicleStatus.PLANNING
            vehicle_state.current_backbone_path_id = backbone_path_id
            
            # 网络感知统计更新
            if task.network_topology_type != "original":
                if task.network_topology_type == "consolidated":
                    vehicle_state.consolidation_aware_tasks += 1
                if task.hierarchy_aware_planning:
                    self.stats['hierarchy_optimized_assignments'] += 1
            
            # 统计信息
            assignment_time = time.time() - assignment_start
            self.performance_monitor['assignment_times'].append(assignment_time)
            self.stats['total_tasks_assigned'] += 1
            self._update_average_assignment_time()
            
            print(f"✅ 集成任务分配成功，耗时: {assignment_time:.2f}s")
            print(f"   骨干路径: {backbone_path_id}")
            print(f"   路径长度: {len(complete_path)}")
            
            return True
    
    def _plan_integrated_backbone_path_to_stage(self, task: IntegratedBackboneTask,
                                              vehicle_state: IntegratedVehicleState,
                                              stage: NetworkAwareTaskStageInfo) -> Optional[Tuple]:
        """集成优化的阶段路径规划"""
        try:
            print(f"    [集成规划] 到阶段: {stage.stage.value} ({stage.target_type}_{stage.target_id})")
            print(f"      层次偏好: {stage.preferred_hierarchy_level}")
            print(f"      整理感知: {stage.consolidation_aware}")
            
            # 构建规划上下文
            planning_context = {
                'context': 'navigation',
                'target_type': stage.target_type,
                'target_id': stage.target_id,
                'current_stage': stage.stage.value,
                'multi_stage': len(task.stages) > 1,
                'network_topology': task.network_topology_type,
                'hierarchy_preference': stage.preferred_hierarchy_level,
                'spatial_conflict_risk': stage.spatial_conflict_risk,
                'consolidation_aware': stage.consolidation_aware
            }
            
            # 优先使用骨干网络
            if self.backbone_network:
                backbone_result = self.backbone_network.get_path_from_position_to_target(
                    vehicle_state.current_position,
                    stage.target_type,
                    stage.target_id,
                    vehicle_state.vehicle_id
                )
                
                if backbone_result:
                    print(f"    ✅ 骨干网络路径成功")
                    if isinstance(backbone_result, tuple):
                        path, structure = backbone_result
                        # 增强结构信息
                        structure.update({
                            'network_topology': task.network_topology_type,
                            'hierarchy_aware': task.hierarchy_aware_planning,
                            'consolidation_aware': stage.consolidation_aware,
                            'spatial_conflict_considered': task.spatial_conflict_considered
                        })
                        return path, structure
                    else:
                        return backbone_result, {'type': 'backbone'}
            
            # 回退到集成规划器
            if self.path_planner and hasattr(self.path_planner, 'plan_path'):
                print(f"    回退到集成规划器")
                
                # 使用集成配置的规划器
                direct_result = self.path_planner.plan_path(
                    vehicle_id=vehicle_state.vehicle_id,
                    start=vehicle_state.current_position,
                    goal=stage.location,
                    use_backbone=False,
                    **planning_context
                )
                
                if direct_result:
                    if hasattr(direct_result, 'path'):
                        path = direct_result.path
                        structure = direct_result.structure if hasattr(direct_result, 'structure') else {}
                    else:
                        path = direct_result
                        structure = {'type': 'direct'}
                    
                    # 增强结构信息
                    structure.update({
                        'integrated_planning': True,
                        'network_topology': task.network_topology_type,
                        'stage_aware': True,
                        'total_length': len(path)
                    })
                    
                    print(f"    ✅ 集成规划器成功")
                    return path, structure
            
        except Exception as e:
            print(f"    ❌ 集成规划异常: {e}")
        
        return None
    
    def _record_integrated_vehicle_occupation(self, vehicle_id: str, backbone_path_id: str,
                                            node_timing_plan: Dict, task: IntegratedBackboneTask) -> bool:
        """记录集成的车辆占用"""
        if not self.conflict_detector:
            return True
        
        try:
            # 构建网络感知的风险信息
            segment_risk_info = {
                'network_topology': task.network_topology_type,
                'hierarchy_aware': task.hierarchy_aware_planning,
                'consolidation_aware': getattr(task.get_current_stage(), 'consolidation_aware', False),
                'spatial_conflict_considered': task.spatial_conflict_considered
            }
            
            # 更新车辆信息到冲突检测器
            if hasattr(self.conflict_detector, 'update_vehicle_info'):
                vehicle_state = self.vehicle_states[vehicle_id]
                vehicle_info = {
                    'position': vehicle_state.current_position,
                    'current_speed': vehicle_state.current_speed,
                    'vehicle_type': 'dump_truck',  # 默认类型
                    'priority': vehicle_state.priority,
                    'network_topology_awareness': vehicle_state.network_topology_awareness,
                    'hierarchy_preference': vehicle_state.hierarchy_level_preference
                }
                self.conflict_detector.update_vehicle_info(vehicle_id, vehicle_info)
            
            # 记录占用
            success = self.conflict_detector.record_vehicle_backbone_occupation(
                vehicle_id, backbone_path_id, node_timing_plan, 
                task.priority.value, segment_risk_info
            )
            
            if success:
                self.stats['integrated_traffic_interactions'] += 1
            
            return success
            
        except Exception as e:
            print(f"    ❌ 集成占用记录失败: {e}")
            return False
    
    def _handle_integrated_conflicts_for_vehicle(self, vehicle_id: str, task: IntegratedBackboneTask):
        """处理车辆的集成冲突"""
        conflicts = self.conflict_detector.get_vehicle_conflicts(vehicle_id)
        
        if conflicts:
            print(f"🚨 车辆 {vehicle_id} 涉及 {len(conflicts)} 个冲突")
            
            vehicle_state = self.vehicle_states[vehicle_id]
            vehicle_state.current_status = VehicleStatus.CONFLICT_RESOLVING
            vehicle_state.conflict_involvements += len(conflicts)
            
            # 记录冲突事件
            for conflict in conflicts:
                self.performance_monitor['conflict_events'].append({
                    'vehicle_id': vehicle_id,
                    'conflict_id': conflict.conflict_id,
                    'conflict_type': conflict.conflict_type.value,
                    'network_topology': task.network_topology_type,
                    'spatial_conflict': getattr(conflict, 'spatial_conflict', False),
                    'time': time.time()
                })
            
            # 委托集成交通管理器处理
            if self.traffic_manager:
                results = self.traffic_manager.process_conflicts(conflicts)
                
                # 更新车辆状态
                all_resolved = all(
                    result.value == 'success' for result in results.values()
                )
                
                if all_resolved:
                    vehicle_state.current_status = VehicleStatus.PLANNING
                    print(f"✅ 车辆 {vehicle_id} 所有冲突已解决")
                else:
                    print(f"⚠️ 车辆 {vehicle_id} 部分冲突未解决")
                    
                    # 尝试智能恢复
                    if (self.integrated_traffic_available and 
                        hasattr(self.traffic_manager, 'check_and_recover_vehicles_enhanced')):
                        self.traffic_manager.check_and_recover_vehicles_enhanced()
                        self.stats['intelligent_recovery_assists'] += 1
    
    # ==================== 辅助方法 ====================
    
    def _generate_unique_task_id(self, task_type: str = "task") -> str:
        """生成唯一任务ID"""
        with self.task_id_lock:
            self.task_counter += 1
            timestamp = int(time.time() * 1000000)
            random_suffix = str(uuid.uuid4())[:8]
            return f"{task_type}_{timestamp}_{self.task_counter}_{random_suffix}"
    
    def _get_available_vehicles(self) -> List[Tuple[str, IntegratedVehicleState]]:
        """获取可用车辆列表"""
        available = []
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            if (vehicle_state.current_status in [VehicleStatus.IDLE, VehicleStatus.PLANNING] and
                len(vehicle_state.task_queue) < self.config['max_tasks_per_vehicle']):
                available.append((vehicle_id, vehicle_state))
        return available
    
    def _determine_preferred_hierarchy_level(self, target_type: str, target_id: int) -> str:
        """确定偏好的层次级别"""
        # 根据目标类型和网络拓扑确定偏好
        if self.current_network_topology == "hierarchical":
            if target_type in ["loading", "unloading"]:
                return "trunk"  # 主要操作使用主干路径
            elif target_type == "parking":
                return "connector"  # 停车使用连接路径
            else:
                return "branch"  # 其他使用分支路径
        elif self.current_network_topology == "consolidated":
            return "branch"  # 整理网络主要使用分支
        else:
            return "trunk"  # 原始网络使用主干
    
    def _assess_location_spatial_risk(self, location: Tuple) -> float:
        """评估位置的空间冲突风险"""
        # 简化实现：基于位置的启发式评估
        x, y = location[0], location[1]
        
        # 检查是否靠近其他重要位置
        risk = 0.0
        
        # 检查与装载点的距离
        for loading_point in self.env.loading_points:
            distance = self._calculate_distance(location, loading_point)
            if distance < 20:
                risk += 0.3
        
        # 检查与卸载点的距离
        for unloading_point in self.env.unloading_points:
            distance = self._calculate_distance(location, unloading_point)
            if distance < 20:
                risk += 0.3
        
        # 基于坐标的简单风险评估
        if abs(x - 50) < 10 or abs(y - 50) < 10:  # 假设(50,50)是高风险区域
            risk += 0.4
        
        return min(risk, 1.0)
    
    def _find_trunk_paths_to_target(self, target_type: str, target_id: int) -> List:
        """查找到目标的主干路径"""
        if not self.backbone_network:
            return []
        
        trunk_paths = []
        for path_id, path_data in self.backbone_network.bidirectional_paths.items():
            # 检查是否连接到目标
            connects_to_target = (
                (path_data.point_a['type'] == target_type and path_data.point_a['id'] == target_id) or
                (path_data.point_b['type'] == target_type and path_data.point_b['id'] == target_id)
            )
            
            if connects_to_target:
                # 检查是否是主干路径
                if (hasattr(path_data, 'consolidation_info') and 
                    path_data.consolidation_info.get('hierarchy_level') == 'trunk'):
                    trunk_paths.append(path_data)
                elif not hasattr(path_data, 'consolidation_info'):
                    # 没有层次信息的默认为主干
                    trunk_paths.append(path_data)
        
        return trunk_paths
    
    def _has_consolidated_paths_to_target(self, target_type: str, target_id: int) -> bool:
        """检查是否有到目标的整理路径"""
        if not self.backbone_network:
            return False
        
        for path_id, path_data in self.backbone_network.bidirectional_paths.items():
            connects_to_target = (
                (path_data.point_a['type'] == target_type and path_data.point_a['id'] == target_id) or
                (path_data.point_b['type'] == target_type and path_data.point_b['id'] == target_id)
            )
            
            if connects_to_target and hasattr(path_data, 'path_type'):
                if path_data.path_type == "merged":
                    return True
        
        return False
    
    def _find_paths_to_target(self, target_type: str, target_id: int) -> List:
        """查找到目标的所有路径"""
        if not self.backbone_network:
            return []
        
        paths = []
        for path_id, path_data in self.backbone_network.bidirectional_paths.items():
            connects_to_target = (
                (path_data.point_a['type'] == target_type and path_data.point_a['id'] == target_id) or
                (path_data.point_b['type'] == target_type and path_data.point_b['id'] == target_id)
            )
            
            if connects_to_target:
                paths.append(path_data)
        
        return paths
    
    def _calculate_distance(self, pos1: Tuple, pos2: Tuple) -> float:
        """计算两点间距离"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def _calculate_integrated_node_timing_plan(self, complete_path: List, structure: Dict,
                                             vehicle_state: IntegratedVehicleState,
                                             task: IntegratedBackboneTask) -> Dict[int, Tuple[float, float]]:
        """计算集成的节点时序计划"""
        timing_plan = {}
        
        if not complete_path:
            return timing_plan
        
        # 获取骨干路径部分
        backbone_path = structure.get('backbone_path', complete_path)
        
        # 参数设置 - 网络感知
        vehicle_speed = vehicle_state.max_speed
        node_stop_time = self.config['node_stop_time']
        interface_spacing = self.config['interface_spacing']
        
        # 网络拓扑调整
        if task.network_topology_type == "hierarchical":
            # 层次网络可能需要更长的停车时间
            node_stop_time *= 1.2
        elif task.network_topology_type == "consolidated":
            # 整理网络可以稍微减少停车时间
            node_stop_time *= 0.9
        
        # 空间冲突风险调整
        current_stage = task.get_current_stage()
        if current_stage and current_stage.spatial_conflict_risk > 0.5:
            node_stop_time *= (1 + current_stage.spatial_conflict_risk * 0.5)
        
        # 从当前时间开始计算
        current_time = time.time()
        travel_time = current_time
        
        # 计算接入路径的时间
        if 'access_path' in structure:
            access_path = structure['access_path']
            for i in range(len(access_path) - 1):
                p1, p2 = access_path[i], access_path[i + 1]
                distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                travel_time += distance / vehicle_speed
        
        # 计算骨干路径上节点的时序
        node_index = 0
        
        for i in range(0, len(backbone_path), interface_spacing):
            if i >= len(backbone_path):
                break
            
            node_pos = backbone_path[i]
            
            # 计算到达当前节点的时间
            if node_index > 0:
                prev_index = max(0, i - interface_spacing)
                if prev_index < len(backbone_path):
                    prev_pos = backbone_path[prev_index]
                    distance = math.sqrt(
                        (node_pos[0] - prev_pos[0])**2 + 
                        (node_pos[1] - prev_pos[1])**2
                    )
                    travel_time += distance / vehicle_speed
            
            arrival_time = travel_time
            departure_time = arrival_time + node_stop_time
            
            timing_plan[node_index] = (arrival_time, departure_time)
            
            travel_time = departure_time
            node_index += 1
        
        print(f"   集成时序计划: {len(timing_plan)} 个节点 (停车时间: {node_stop_time:.1f}s)")
        
        return timing_plan
    
    # ==================== 兼容性方法 ====================
    
    def create_transport_task(self, start_location: Tuple, end_location: Tuple,
                             priority: TaskPriority = TaskPriority.NORMAL,
                             vehicle_id: str = None) -> str:
        """兼容性运输任务创建方法"""
        return self.create_transport_task_integrated(start_location, end_location, priority, vehicle_id)
    
    def assign_task_with_corridors(self, task_id: str, vehicle_id: str = None) -> bool:
        """兼容性任务分配方法"""
        return self.assign_task_integrated(task_id, vehicle_id)
    
    def assign_task(self, task_id: str, vehicle_id: str = None) -> bool:
        """兼容性任务分配方法"""
        return self.assign_task_integrated(task_id, vehicle_id)
    
    # ==================== 其他原有方法保持不变 ====================
    
    def initialize_vehicles(self):
        """初始化车辆状态 - 集成版"""
        with self.lock:
            self.vehicle_states.clear()
            
            if not self.env or not hasattr(self.env, 'vehicles'):
                return
            
            for vehicle_id, vehicle_info in self.env.vehicles.items():
                # 兼容不同的车辆信息格式
                if hasattr(vehicle_info, 'position'):
                    position = vehicle_info.position
                elif hasattr(vehicle_info, '__getitem__'):
                    position = vehicle_info.get('position', (0, 0, 0))
                else:
                    position = (0, 0, 0)
                
                # 确保位置是3D坐标
                if len(position) < 3:
                    position = (*position, 0.0)
                
                self.vehicle_states[vehicle_id] = IntegratedVehicleState(
                    vehicle_id=vehicle_id,
                    current_position=position,
                    current_status=VehicleStatus.IDLE,
                    network_topology_awareness=self.current_network_topology,
                    hierarchy_level_preference=self._determine_preferred_hierarchy_level("loading", 0)
                )
        
        print(f"初始化 {len(self.vehicle_states)} 个集成车辆状态")
    
    def get_comprehensive_stats(self) -> Dict:
        """获取综合统计信息"""
        base_stats = {
            'scheduler_stats': self.stats.copy(),
            'vehicle_count': len(self.vehicle_states),
            'active_tasks': len([t for t in self.tasks.values() 
                               if t.status in [TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS]]),
            'pending_tasks': len(self.task_queue),
            'task_distribution': {
                'pending': len([t for t in self.tasks.values() if t.status == TaskStatus.PENDING]),
                'assigned': len([t for t in self.tasks.values() if t.status == TaskStatus.ASSIGNED]),
                'in_progress': len([t for t in self.tasks.values() if t.status == TaskStatus.IN_PROGRESS]),
                'completed': len([t for t in self.tasks.values() if t.status == TaskStatus.COMPLETED]),
                'failed': len([t for t in self.tasks.values() if t.status == TaskStatus.FAILED])
            }
        }
        
        # 网络感知统计
        base_stats['network_awareness'] = {
            'current_topology': self.current_network_topology,
            'topology_adaptations': self.stats['network_topology_adaptations'],
            'consolidation_aware_assignments': self.stats['consolidation_aware_assignments'],
            'hierarchy_optimized_assignments': self.stats['hierarchy_optimized_assignments'],
            'spatial_conflict_prevented_assignments': self.stats['spatial_conflict_prevented_assignments']
        }
        
        # 多阶段任务统计
        base_stats['multi_stage_performance'] = {
            'multi_stage_tasks_created': self.stats['multi_stage_tasks_created'],
            'multi_stage_tasks_completed': self.stats['multi_stage_tasks_completed'],
            'stage_transitions_optimized': self.stats['stage_transitions_optimized'],
            'inter_stage_conflicts_prevented': self.stats['inter_stage_conflicts_prevented']
        }
        
        # 车辆表现统计
        base_stats['vehicle_performance'] = {}
        for vehicle_id, vs in self.vehicle_states.items():
            base_stats['vehicle_performance'][vehicle_id] = {
                'tasks_completed': vs.total_tasks_completed,
                'total_distance': vs.total_distance,
                'conflict_involvements': vs.conflict_involvements,
                'backbone_switches': vs.backbone_switches,
                'interface_switches': vs.interface_switches,
                'network_topology_adaptations': vs.network_topology_adaptations,
                'multi_stage_tasks_completed': vs.multi_stage_tasks_completed,
                'consolidation_aware_tasks': vs.consolidation_aware_tasks,
                'hierarchy_preference': vs.hierarchy_level_preference,
                'current_topology_awareness': vs.network_topology_awareness
            }
        
        return base_stats
    
    def update(self, time_delta: float):
        """更新调度器状态 - 集成版"""
        # 定期更新网络拓扑感知
        if self.config.get('network_topology_adaptation', True):
            self.update_network_topology_awareness()
        
        # 更新车辆状态
        self._update_vehicle_states(time_delta)
        
        # 处理任务队列
        self._process_task_queue()
        
        # 监控任务进度
        self._monitor_task_progress()
        
        # 更新统计信息
        self._update_statistics()
    
    def shutdown(self):
        """关闭调度器"""
        with self.lock:
            self.vehicle_states.clear()
            self.tasks.clear()
            self.task_queue.clear()
        
        print("整合优化版骨干网络车辆调度器已关闭")
    
    # 省略其他方法的具体实现...
    def _update_vehicle_states(self, time_delta: float):
        """更新车辆状态"""
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            if vehicle_id in self.env.vehicles:
                env_vehicle = self.env.vehicles[vehicle_id]
                
                if hasattr(env_vehicle, 'position'):
                    new_position = env_vehicle.position
                elif hasattr(env_vehicle, '__getitem__'):
                    new_position = env_vehicle.get('position', vehicle_state.current_position)
                else:
                    new_position = vehicle_state.current_position
                
                vehicle_state.update_position(new_position, time_delta)
    
    def _process_task_queue(self):
        """处理任务队列"""
        processed_count = 0
        max_process_per_cycle = 5
        
        while self.task_queue and processed_count < max_process_per_cycle:
            task_id = self.task_queue.popleft()
            task = self.tasks.get(task_id)
            
            if not task or task.status != TaskStatus.PENDING:
                processed_count += 1
                continue
            
            if self.assign_task_integrated(task_id):
                print(f"✅ 队列任务分配成功: {task_id}")
            else:
                task.retry_count += 1
                if task.retry_count < 3:
                    self.task_queue.append(task_id)
                else:
                    task.status = TaskStatus.FAILED
                    self.stats['total_tasks_failed'] += 1
                    print(f"❌ 任务分配失败，已放弃: {task_id}")
            
            processed_count += 1
    
    def _monitor_task_progress(self):
        """监控任务进度"""
        current_time = time.time()
        
        for task_id, task in self.tasks.items():
            if task.status == TaskStatus.IN_PROGRESS:
                if current_time > task.latest_finish_time:
                    task.status = TaskStatus.FAILED
                    self.stats['total_tasks_failed'] += 1
                    print(f"⏰ 任务超时失败: {task_id}")
                    
                    if task.vehicle_id and task.vehicle_id in self.vehicle_states:
                        vehicle_state = self.vehicle_states[task.vehicle_id]
                        if vehicle_state.current_task_id == task_id:
                            vehicle_state.current_task_id = None
                            vehicle_state.current_status = VehicleStatus.IDLE
            
            elif task.status == TaskStatus.ASSIGNED:
                if current_time >= task.earliest_start_time:
                    task.status = TaskStatus.IN_PROGRESS
                    
                    if task.vehicle_id and task.vehicle_id in self.vehicle_states:
                        vehicle_state = self.vehicle_states[task.vehicle_id]
                        vehicle_state.current_status = VehicleStatus.MOVING
    
    def _update_statistics(self):
        """更新统计信息"""
        self._update_average_assignment_time()
        self._update_average_completion_time()
        self._calculate_backbone_utilization()
        self._calculate_vehicle_efficiency()
        self._calculate_conflict_resolution_rate()
    
    def _update_average_assignment_time(self):
        """更新平均分配时间"""
        assignment_times = list(self.performance_monitor['assignment_times'])
        if assignment_times:
            self.stats['average_assignment_time'] = sum(assignment_times) / len(assignment_times)
    
    def _update_average_completion_time(self):
        """更新平均完成时间"""
        completion_times = list(self.performance_monitor['completion_times'])
        if completion_times:
            self.stats['average_completion_time'] = sum(completion_times) / len(completion_times)
    
    def _calculate_backbone_utilization(self):
        """计算骨干网络利用率"""
        if not self.backbone_network:
            return
        
        total_paths = len(self.backbone_network.bidirectional_paths)
        if total_paths == 0:
            return
        
        utilized_paths = set()
        for vehicle_state in self.vehicle_states.values():
            if vehicle_state.current_backbone_path_id:
                utilized_paths.add(vehicle_state.current_backbone_path_id)
        
        self.stats['backbone_utilization'] = len(utilized_paths) / total_paths
    
    def _calculate_vehicle_efficiency(self):
        """计算车辆效率"""
        if not self.vehicle_states:
            return
        
        total_efficiency = 0
        active_vehicles = 0
        
        for vehicle_state in self.vehicle_states.values():
            if vehicle_state.total_tasks_completed > 0:
                efficiency = vehicle_state.total_tasks_completed / (
                    vehicle_state.total_distance / 100 + 
                    vehicle_state.conflict_involvements + 1
                )
                total_efficiency += efficiency
                active_vehicles += 1
        
        if active_vehicles > 0:
            self.stats['vehicle_efficiency'] = total_efficiency / active_vehicles
    
    def _calculate_conflict_resolution_rate(self):
        """计算冲突解决成功率"""
        if self.traffic_manager:
            traffic_stats = self.traffic_manager.get_system_status()
            self.stats['conflict_resolution_success_rate'] = traffic_stats.get('resolution_success_rate', 0.0)
    
    # 其他占位符方法
    def _infer_target_from_position(self, position: Tuple) -> Tuple[str, int]:
        """从位置推断目标类型"""
        min_distance = float('inf')
        best_match = ('loading', 0)
        
        for i, loading_point in enumerate(self.env.loading_points):
            distance = self._calculate_distance(position, loading_point)
            if distance < min_distance:
                min_distance = distance
                best_match = ('loading', i)
        
        for i, unloading_point in enumerate(self.env.unloading_points):
            distance = self._calculate_distance(position, unloading_point)
            if distance < min_distance:
                min_distance = distance
                best_match = ('unloading', i)
        
        if hasattr(self.env, 'parking_areas'):
            for i, parking_area in enumerate(self.env.parking_areas):
                distance = self._calculate_distance(position, parking_area)
                if distance < min_distance:
                    min_distance = distance
                    best_match = ('parking', i)
        
        return best_match if min_distance < 20 else ('loading', 0)

    def create_random_cycle_task(self, vehicle_id: str, start_location: Tuple, 
                            loading_location: Tuple, unloading_location: Tuple,
                            parking_location: Tuple, priority: TaskPriority = TaskPriority.NORMAL,
                            enable_cycle: bool = True) -> str:
        """创建随机循环任务"""
        
        task_id = self._generate_unique_task_id("random_cycle")
        
        while task_id in self.tasks:
            task_id = self._generate_unique_task_id("random_cycle")
        
        print(f"\n🔄 创建随机循环任务 {task_id}:")
        print(f"  车辆: {vehicle_id}")
        print(f"  起点: ({start_location[0]:.1f}, {start_location[1]:.1f})")
        print(f"  装载点: ({loading_location[0]:.1f}, {loading_location[1]:.1f})")
        print(f"  卸载点: ({unloading_location[0]:.1f}, {unloading_location[1]:.1f})")
        print(f"  停车点: ({parking_location[0]:.1f}, {parking_location[1]:.1f})")
        print(f"  循环模式: {'开启' if enable_cycle else '关闭'}")
        
        # 推断目标点的类型和ID
        loading_type, loading_id = self._infer_target_from_position(loading_location)
        unloading_type, unloading_id = self._infer_target_from_position(unloading_location)
        parking_type, parking_id = self._infer_target_from_position(parking_location)
        
        # 创建4阶段任务
        stages = []
        
        # 阶段1: 移动到装载点
        move_to_loading_stage = NetworkAwareTaskStageInfo(
            stage=TaskStage.TRANSPORT,
            location=loading_location,
            target_type=loading_type,
            target_id=loading_id,
            estimated_duration=self._calculate_travel_time(start_location, loading_location),
            preferred_hierarchy_level=self._determine_preferred_hierarchy_level(loading_type, loading_id),
            spatial_conflict_risk=self._assess_location_spatial_risk(loading_location),
            consolidation_aware=self.current_network_topology != "original"
        )
        stages.append(move_to_loading_stage)
        print(f"  阶段1: 移动到装载点 L{loading_id}")
        
        # 阶段2: 装载操作
        loading_stage = NetworkAwareTaskStageInfo(
            stage=TaskStage.LOADING,
            location=loading_location,
            target_type=loading_type,
            target_id=loading_id,
            estimated_duration=60.0,  # 装载时间
            preferred_hierarchy_level=self._determine_preferred_hierarchy_level(loading_type, loading_id),
            spatial_conflict_risk=self._assess_location_spatial_risk(loading_location),
            consolidation_aware=self.current_network_topology != "original"
        )
        stages.append(loading_stage)
        print(f"  阶段2: 装载操作 L{loading_id}")
        
        # 阶段3: 卸载操作
        unloading_stage = NetworkAwareTaskStageInfo(
            stage=TaskStage.UNLOADING,
            location=unloading_location,
            target_type=unloading_type,
            target_id=unloading_id,
            estimated_duration=60.0,  # 卸载时间
            preferred_hierarchy_level=self._determine_preferred_hierarchy_level(unloading_type, unloading_id),
            spatial_conflict_risk=self._assess_location_spatial_risk(unloading_location),
            consolidation_aware=self.current_network_topology != "original"
        )
        stages.append(unloading_stage)
        print(f"  阶段3: 卸载操作 U{unloading_id}")
        
        # 阶段4: 停车
        parking_stage = NetworkAwareTaskStageInfo(
            stage=TaskStage.PARKING,
            location=parking_location,
            target_type=parking_type,
            target_id=parking_id,
            estimated_duration=30.0,  # 停车时间
            preferred_hierarchy_level="connector",  # 停车通常使用连接路径
            spatial_conflict_risk=self._assess_location_spatial_risk(parking_location),
            consolidation_aware=self.current_network_topology != "original"
        )
        stages.append(parking_stage)
        print(f"  阶段4: 停车 P{parking_id}")
        
        # 计算总时间
        total_duration = sum(stage.estimated_duration for stage in stages)
        
        # 添加移动时间
        for i in range(len(stages) - 1):
            distance = self._calculate_distance(stages[i].location, stages[i+1].location)
            total_duration += distance / 1.5  # 假设平均速度1.5m/s
        
        # 创建任务对象
        task = IntegratedBackboneTask(
            task_id=task_id,
            vehicle_id=vehicle_id,
            task_type="random_cycle",
            priority=priority,
            stages=stages,
            start_location=start_location,
            end_location=parking_location,  # 最终到达停车点
            earliest_start_time=time.time(),
            latest_finish_time=time.time() + total_duration * 3,
            estimated_duration=total_duration,
            network_topology_type=self.current_network_topology,
            consolidation_info=self.consolidation_info.copy(),
            hierarchy_aware_planning=self.current_network_topology == "hierarchical",
            spatial_conflict_considered=True,
            stage_transition_optimization=True
        )
        
        # 设置循环标记
        if enable_cycle:
            if 'cycle_info' not in task.consolidation_info:
                task.consolidation_info['cycle_info'] = {}
            
            task.consolidation_info['cycle_info'] = {
                'is_cycle_task': True,
                'cycle_count': 0,
                'max_cycles': 10,  # 最多循环10次
                'original_start_location': start_location,
                'loading_location': loading_location,
                'unloading_location': unloading_location,
                'parking_location': parking_location,
                'cycle_enabled': True,
                'randomize_targets': True  # 每轮随机化目标
            }
            print(f"  ♻️ 循环设置: 最多 {task.consolidation_info['cycle_info']['max_cycles']} 次")
        
        # 设置当前目标为第一阶段
        first_stage = task.get_current_stage()
        if first_stage:
            task.target_type = first_stage.target_type
            task.target_id = first_stage.target_id
        
        with self.lock:
            self.tasks[task_id] = task
            
            # 不自动加入队列，等待手动分配
            if not vehicle_id:
                self.task_queue.append(task_id)
            
            self.stats['total_tasks_created'] += 1
            self.stats['multi_stage_tasks_created'] += 1
        
        print(f"✅ 随机循环任务创建完成: {task_id} ({len(stages)}阶段)")
        return task_id

    def _calculate_travel_time(self, start: Tuple, end: Tuple, speed: float = 1.5) -> float:
        """计算移动时间"""
        distance = self._calculate_distance(start, end)
        return distance / speed

    def handle_cycle_task_completion(self, task_id: str) -> bool:
        """处理循环任务完成"""
        if task_id not in self.tasks:
            return False
        
        task = self.tasks[task_id]
        
        # 检查是否是循环任务
        cycle_info = task.consolidation_info.get('cycle_info', {})
        if not cycle_info.get('is_cycle_task', False) or not cycle_info.get('cycle_enabled', False):
            return False
        
        print(f"\n♻️ 处理循环任务完成: {task_id}")
        
        # 检查循环次数
        current_cycles = cycle_info.get('cycle_count', 0)
        max_cycles = cycle_info.get('max_cycles', 10)
        
        print(f"   当前循环: {current_cycles}/{max_cycles}")
        
        if current_cycles >= max_cycles:
            print(f"   🏁 达到最大循环次数，任务结束")
            return False
        
        # 增加循环计数
        cycle_info['cycle_count'] = current_cycles + 1
        
        # 获取循环位置信息
        start_location = cycle_info['original_start_location']
        loading_location = cycle_info['loading_location']
        unloading_location = cycle_info['unloading_location']
        parking_location = cycle_info['parking_location']
        
        # 随机化下一轮的目标点
        if cycle_info.get('randomize_targets', True):
            loading_location = self._get_random_loading_location()
            unloading_location = self._get_random_unloading_location()
            parking_location = self._get_random_parking_location()
            
            # 更新循环信息
            cycle_info['loading_location'] = loading_location
            cycle_info['unloading_location'] = unloading_location
            cycle_info['parking_location'] = parking_location
            
            print(f"   🎲 随机化下一轮目标点")
        
        try:
            # 创建下一轮循环任务
            next_task_id = self.create_random_cycle_task(
                vehicle_id=task.vehicle_id,
                start_location=parking_location,  # 从停车位置开始
                loading_location=loading_location,
                unloading_location=unloading_location,
                parking_location=parking_location,
                priority=task.priority,
                enable_cycle=True
            )
            
            if next_task_id:
                # 继承循环信息
                next_task = self.tasks[next_task_id]
                next_task.consolidation_info['cycle_info'] = cycle_info
                
                # 自动分配给同一车辆
                success = self.assign_task_integrated(next_task_id, task.vehicle_id)
                
                if success:
                    print(f"   ✅ 第 {cycle_info['cycle_count']} 轮循环任务已创建并分配: {next_task_id}")
                    return True
                else:
                    print(f"   ❌ 循环任务分配失败")
                    return False
            else:
                print(f"   ❌ 创建下一轮循环任务失败")
                return False
                
        except Exception as e:
            print(f"   ❌ 处理循环任务异常: {e}")
            return False

    def _get_random_loading_location(self) -> Tuple:
        """获取随机装载位置"""
        import random
        if self.env.loading_points:
            point = random.choice(self.env.loading_points)
            return self._ensure_3d_point(point)
        return (0, 0, 0)

    def _get_random_unloading_location(self) -> Tuple:
        """获取随机卸载位置"""
        import random
        if self.env.unloading_points:
            point = random.choice(self.env.unloading_points)
            return self._ensure_3d_point(point)
        return (0, 0, 0)

    def _get_random_parking_location(self) -> Tuple:
        """获取随机停车位置"""
        import random
        parking_areas = getattr(self.env, 'parking_areas', [])
        if parking_areas:
            point = random.choice(parking_areas)
            return self._ensure_3d_point(point)
        else:
            # 如果没有停车区，在装载点附近生成
            if self.env.loading_points:
                loading_point = random.choice(self.env.loading_points)
                loading_location = self._ensure_3d_point(loading_point)
                return self._generate_parking_near_loading(loading_location)
        return (0, 0, 0)

    def _ensure_3d_point(self, point) -> Tuple[float, float, float]:
        """确保点是3D坐标"""
        if not point:
            return (0.0, 0.0, 0.0)
        elif len(point) >= 3:
            return (float(point[0]), float(point[1]), float(point[2]))
        elif len(point) == 2:
            return (float(point[0]), float(point[1]), 0.0)
        else:
            return (0.0, 0.0, 0.0)

    def _generate_parking_near_loading(self, loading_location: Tuple) -> Tuple[float, float, float]:
        """在装载点附近生成停车位置"""
        import random
        x, y, z = loading_location
        offset_x = random.uniform(-20, 20)
        offset_y = random.uniform(-20, 20)
        return (x + offset_x, y + offset_y, z)
    def advance_task_stage(self, task_id: str) -> bool:
        """推进任务到下一阶段（修复版 - 解决阶段卡住问题）"""
        if task_id not in self.tasks:
            print(f"❌ 任务不存在: {task_id}")
            return False
        
        task = self.tasks[task_id]
        
        with self.lock:
            try:
                print(f"\n🔄 [阶段推进] 任务: {task_id}")
                print(f"   当前阶段: {task.current_stage_index}/{len(task.stages)}")
                print(f"   任务状态: {task.status.value}")
                
                # 修复1: 更宽松的状态检查 - 允许更多状态进行阶段推进
                valid_statuses = [TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS, TaskStatus.SUSPENDED]
                if task.status not in valid_statuses:
                    print(f"❌ 任务状态不允许阶段推进: {task.status.value}")
                    return False
                
                # 修复2: 检查车辆状态
                if not task.vehicle_id or task.vehicle_id not in self.vehicle_states:
                    print(f"❌ 任务没有分配车辆或车辆不存在: {task.vehicle_id}")
                    return False
                
                vehicle_state = self.vehicle_states[task.vehicle_id]
                print(f"   车辆状态: {vehicle_state.current_status.value}")
                print(f"   车辆位置: ({vehicle_state.current_position[0]:.1f}, {vehicle_state.current_position[1]:.1f})")
                
                # 获取当前阶段
                current_stage = task.get_current_stage()
                if not current_stage:
                    print(f"❌ 无法获取当前阶段")
                    return False
                
                print(f"   当前阶段: {current_stage.stage.value} -> {current_stage.target_type}_{current_stage.target_id}")
                
                # 检查是否还有下一阶段
                if task.current_stage_index + 1 >= len(task.stages):
                    print(f"🏁 任务所有阶段已完成，标记为完成")
                    return self._complete_task_fully(task)
                
                # 修复3: 确保当前阶段被正确标记为完成
                if not current_stage.completed:
                    current_stage.completed = True
                    print(f"   ✅ 标记阶段 {task.current_stage_index} 完成: {current_stage.stage.value}")
                
                # 保存旧阶段信息用于回滚
                old_stage_index = task.current_stage_index
                old_target_type = task.target_type
                old_target_id = task.target_id
                old_path = task.complete_path.copy() if task.complete_path else []
                old_backbone_path_id = task.assigned_backbone_path_id
                
                # 推进到下一阶段
                advance_success = task.advance_to_next_stage()
                if not advance_success:
                    print(f"❌ 任务内部阶段推进失败")
                    return False
                
                print(f"   ➡️ 阶段推进: {old_stage_index} -> {task.current_stage_index}")
                
                # 再次检查任务是否完全完成
                if task.is_complete():
                    print(f"🎉 任务 {task_id} 所有阶段完成")
                    return self._complete_task_fully(task)
                
                # 获取新阶段信息
                new_stage = task.get_current_stage()
                if not new_stage:
                    print(f"❌ 无法获取新阶段信息")
                    # 回滚
                    task.current_stage_index = old_stage_index
                    task.target_type = old_target_type
                    task.target_id = old_target_id
                    current_stage.completed = False
                    return False
                
                print(f"   🎯 新阶段: {new_stage.stage.value} -> {new_stage.target_type}_{new_stage.target_id}")
                print(f"   层次偏好: {new_stage.preferred_hierarchy_level}")
                
                # 修复4: 强制更新任务状态为进行中
                if task.status != TaskStatus.IN_PROGRESS:
                    task.status = TaskStatus.IN_PROGRESS
                    print(f"   📝 强制更新任务状态为 IN_PROGRESS")
                
                # 修复5: 清理旧的资源占用
                self._cleanup_old_stage_resources(task, vehicle_state)
                
                # 修复6: 重新规划新阶段的路径
                replan_success = self._replan_for_new_stage_enhanced(task, vehicle_state, new_stage)
                
                if not replan_success:
                    print(f"❌ 新阶段路径规划失败，回滚到上一阶段")
                    # 完整回滚
                    task.current_stage_index = old_stage_index
                    task.target_type = old_target_type
                    task.target_id = old_target_id
                    task.complete_path = old_path
                    task.assigned_backbone_path_id = old_backbone_path_id
                    current_stage.completed = False
                    return False
                
                # 修复7: 重置路径进度和车辆状态
                self._reset_path_progress_for_new_stage(task, vehicle_state)
                
                # 修复8: 处理新阶段的冲突
                self._handle_new_stage_conflicts(task, vehicle_state)
                
                # 记录阶段转换统计
                self._record_stage_transition(task, old_stage_index)
                
                print(f"✅ 任务 {task_id} 阶段推进成功: {old_stage_index} -> {task.current_stage_index}")
                print(f"   新路径长度: {len(task.complete_path) if task.complete_path else 0}")
                print(f"   新骨干路径: {task.assigned_backbone_path_id}")
                
                return True
                
            except Exception as e:
                print(f"❌ 阶段推进异常: {e}")
                import traceback
                traceback.print_exc()
                return False

    def _complete_task_fully(self, task) -> bool:
        """完整处理任务完成（修改版 - 支持循环任务）"""
        print(f"🎉 [任务完成] {task.task_id}")
        
        try:
            # 检查是否是循环任务
            if task.task_type == "random_cycle":
                cycle_handled = self.handle_cycle_task_completion(task.task_id)
                
                if cycle_handled:
                    # 循环任务已处理，当前任务标记为完成但不清理车辆状态
                    task.status = TaskStatus.COMPLETED
                    task.completion_time = time.time()
                    
                    # 只更新统计，不清理车辆状态（因为车辆会继续下一轮）
                    self.stats['total_tasks_completed'] += 1
                    if len(task.stages) > 1:
                        self.stats['multi_stage_tasks_completed'] += 1
                    
                    print(f"   ♻️ 循环任务继续下一轮")
                    return True
                else:
                    print(f"   🏁 循环任务结束")
                    # 循环结束，按正常流程处理
            
            # 标记任务完成
            task.status = TaskStatus.COMPLETED
            task.completion_time = time.time()
            
            # 标记所有阶段完成
            for stage in task.stages:
                stage.completed = True
            
            # 更新车辆状态
            if task.vehicle_id and task.vehicle_id in self.vehicle_states:
                vehicle_state = self.vehicle_states[task.vehicle_id]
                
                # 清理车辆任务信息
                vehicle_state.current_task_id = None
                vehicle_state.current_status = VehicleStatus.IDLE
                vehicle_state.current_backbone_path_id = None
                vehicle_state.total_tasks_completed += 1
                
                # 多阶段任务统计
                if len(task.stages) > 1:
                    vehicle_state.multi_stage_tasks_completed += 1
                
                print(f"   🚗 车辆 {task.vehicle_id} 状态更新为 IDLE")
            
            # 释放骨干网络资源
            if self.backbone_network and task.assigned_backbone_path_id:
                self.backbone_network.release_vehicle_from_path(task.vehicle_id)
                print(f"   🛤️ 释放骨干路径: {task.assigned_backbone_path_id}")
            
            # 释放冲突检测器资源
            if self.conflict_detector and task.vehicle_id:
                self.conflict_detector.release_vehicle_from_path(task.vehicle_id)
                print(f"   🚫 释放冲突检测器资源")
            
            # 更新统计
            self.stats['total_tasks_completed'] += 1
            if len(task.stages) > 1:
                self.stats['multi_stage_tasks_completed'] += 1
                self.stats['stage_transitions_optimized'] += 1
            
            # 记录完成时间
            if task.assigned_time > 0:
                completion_time = time.time() - task.assigned_time
                self.performance_monitor['completion_times'].append(completion_time)
                print(f"   ⏱️ 任务耗时: {completion_time:.1f}s")
            
            return True
            
        except Exception as e:
            print(f"❌ 任务完成处理异常: {e}")
            return False

    def _cleanup_old_stage_resources(self, task, vehicle_state):
        """清理旧阶段的资源"""
        try:
            print(f"   🧹 清理旧阶段资源...")
            
            # 清除旧的骨干路径占用
            if self.conflict_detector and task.assigned_backbone_path_id:
                self.conflict_detector.release_vehicle_from_path(task.vehicle_id)
                print(f"     - 释放冲突检测器占用: {task.assigned_backbone_path_id}")
            
            # 清除旧的骨干网络占用
            if self.backbone_network and task.assigned_backbone_path_id:
                self.backbone_network.release_vehicle_from_path(task.vehicle_id)
                print(f"     - 释放骨干网络占用: {task.assigned_backbone_path_id}")
            
            # 清理车辆的旧路径信息
            vehicle_state.current_backbone_path_id = None
            vehicle_state.backbone_segment_occupations.clear()
            
        except Exception as e:
            print(f"   ⚠️ 清理旧资源时出现异常: {e}")

    def _replan_for_new_stage_enhanced(self, task, vehicle_state, new_stage) -> bool:
        """为新阶段重新规划路径（增强版）"""
        try:
            print(f"   🗺️ 为新阶段重新规划路径...")
            print(f"     目标: {new_stage.target_type}_{new_stage.target_id}")
            print(f"     位置: ({new_stage.location[0]:.1f}, {new_stage.location[1]:.1f})")
            
            # 更新任务目标信息
            task.target_type = new_stage.target_type
            task.target_id = new_stage.target_id
            
            # 尝试多种规划方法
            path_result = None
            
            # 方法1: 使用集成骨干路径规划
            if hasattr(self, '_plan_integrated_backbone_path_to_stage'):
                try:
                    print(f"     尝试集成骨干路径规划...")
                    path_result = self._plan_integrated_backbone_path_to_stage(task, vehicle_state, new_stage)
                    if path_result:
                        print(f"     ✅ 集成骨干路径规划成功")
                    else:
                        print(f"     ❌ 集成骨干路径规划失败")
                except Exception as e:
                    print(f"     ❌ 集成骨干路径规划异常: {e}")
            
            # 方法2: 直接使用骨干网络
            if not path_result and self.backbone_network:
                try:
                    print(f"     尝试直接骨干网络规划...")
                    backbone_result = self.backbone_network.get_path_from_position_to_target(
                        vehicle_state.current_position,
                        new_stage.target_type,
                        new_stage.target_id,
                        vehicle_state.vehicle_id
                    )
                    
                    if backbone_result:
                        if isinstance(backbone_result, tuple):
                            path_result = backbone_result
                        else:
                            path_result = (backbone_result, {'type': 'backbone'})
                        print(f"     ✅ 直接骨干网络规划成功")
                    else:
                        print(f"     ❌ 直接骨干网络规划失败")
                except Exception as e:
                    print(f"     ❌ 直接骨干网络规划异常: {e}")
            
            # 方法3: 使用路径规划器
            if not path_result and self.path_planner:
                try:
                    print(f"     尝试路径规划器...")
                    direct_result = self.path_planner.plan_path(
                        vehicle_id=vehicle_state.vehicle_id,
                        start=vehicle_state.current_position,
                        goal=new_stage.location,
                        use_backbone=True,
                        context='navigation',
                        target_type=new_stage.target_type,
                        target_id=new_stage.target_id
                    )
                    
                    if direct_result:
                        if hasattr(direct_result, 'path'):
                            path = direct_result.path
                            structure = getattr(direct_result, 'structure', {'type': 'direct'})
                        else:
                            path = direct_result
                            structure = {'type': 'direct'}
                        
                        path_result = (path, structure)
                        print(f"     ✅ 路径规划器成功")
                    else:
                        print(f"     ❌ 路径规划器失败")
                except Exception as e:
                    print(f"     ❌ 路径规划器异常: {e}")
            
            # 检查规划结果
            if not path_result:
                print(f"     ❌ 所有路径规划方法都失败")
                return False
            
            complete_path, structure = path_result
            new_backbone_path_id = structure.get('path_id')
            
            if not complete_path or len(complete_path) < 2:
                print(f"     ❌ 路径无效或太短: {len(complete_path) if complete_path else 0}")
                return False
            
            print(f"     ✅ 路径规划成功: 长度 {len(complete_path)}")
            
            # 计算新的节点时序计划
            node_timing_plan = {}
            if hasattr(self, '_calculate_integrated_node_timing_plan'):
                try:
                    node_timing_plan = self._calculate_integrated_node_timing_plan(
                        complete_path, structure, vehicle_state, task
                    )
                    print(f"     ✅ 时序计划计算完成: {len(node_timing_plan)} 个节点")
                except Exception as e:
                    print(f"     ⚠️ 时序计划计算失败: {e}")
            
            # 记录新的占用
            occupation_success = True
            if hasattr(self, '_record_integrated_vehicle_occupation'):
                try:
                    occupation_success = self._record_integrated_vehicle_occupation(
                        task.vehicle_id, new_backbone_path_id or "direct", node_timing_plan, task
                    )
                    if occupation_success:
                        print(f"     ✅ 占用记录成功")
                    else:
                        print(f"     ⚠️ 占用记录失败，但继续执行")
                except Exception as e:
                    print(f"     ⚠️ 占用记录异常: {e}")
            
            # 更新任务路径信息
            task.assigned_backbone_path_id = new_backbone_path_id
            task.complete_path = complete_path
            task.path_structure = structure
            task.node_timing_plan = node_timing_plan
            
            # 更新车辆骨干路径
            vehicle_state.current_backbone_path_id = new_backbone_path_id
            
            return True
            
        except Exception as e:
            print(f"     ❌ 新阶段路径规划异常: {e}")
            return False

    def _reset_path_progress_for_new_stage(self, task, vehicle_state):
        """为新阶段重置路径进度"""
        try:
            print(f"   🔄 重置新阶段路径进度...")
            
            # 重置任务路径进度
            if hasattr(task, 'path_progress'):
                task.path_progress = 0.0
                print(f"     - 任务路径进度重置为 0.0")
            
            if hasattr(task, 'current_path_index'):
                task.current_path_index = 0
                print(f"     - 路径索引重置为 0")
            
            # 重置车辆移动状态
            vehicle_state.current_status = VehicleStatus.MOVING
            print(f"     - 车辆状态设为 MOVING")
            
            # 清理车辆的旧占用记录
            vehicle_state.backbone_segment_occupations.clear()
            
        except Exception as e:
            print(f"   ⚠️ 重置路径进度异常: {e}")

    def _handle_new_stage_conflicts(self, task, vehicle_state):
        """处理新阶段的冲突"""
        try:
            if not self.conflict_detector:
                return
            
            print(f"   🚨 检查新阶段冲突...")
            
            # 获取车辆相关冲突
            conflicts = self.conflict_detector.get_vehicle_conflicts(vehicle_state.vehicle_id)
            
            if conflicts:
                print(f"     发现 {len(conflicts)} 个冲突")
                vehicle_state.conflict_involvements += len(conflicts)
                
                # 临时设置为冲突解决状态
                old_status = vehicle_state.current_status
                vehicle_state.current_status = VehicleStatus.CONFLICT_RESOLVING
                
                # 委托交通管理器处理
                if self.traffic_manager:
                    try:
                        results = self.traffic_manager.process_conflicts(conflicts)
                        
                        all_resolved = all(
                            result.value == 'success' for result in results.values()
                        )
                        
                        if all_resolved:
                            vehicle_state.current_status = old_status
                            print(f"     ✅ 所有冲突已解决")
                        else:
                            print(f"     ⚠️ 部分冲突未解决")
                            # 保持冲突解决状态，让系统后续处理
                    
                    except Exception as e:
                        print(f"     ❌ 冲突处理异常: {e}")
                        vehicle_state.current_status = old_status
                else:
                    print(f"     ⚠️ 没有交通管理器处理冲突")
                    vehicle_state.current_status = old_status
            else:
                print(f"     ✅ 无冲突")
        
        except Exception as e:
            print(f"   ⚠️ 冲突处理异常: {e}")

    def _record_stage_transition(self, task, old_stage_index):
        """记录阶段转换统计"""
        try:
            new_stage = task.get_current_stage()
            
            transition_record = {
                'task_id': task.task_id,
                'vehicle_id': task.vehicle_id,
                'old_stage': old_stage_index,
                'new_stage': task.current_stage_index,
                'stage_type': new_stage.stage.value if new_stage else 'unknown',
                'target': f"{new_stage.target_type}_{new_stage.target_id}" if new_stage else 'unknown',
                'time': time.time(),
                'network_topology': task.network_topology_type
            }
            
            self.performance_monitor['stage_transitions'].append(transition_record)
            self.stats['stage_transitions_optimized'] += 1
            
        except Exception as e:
            print(f"   ⚠️ 记录阶段转换统计异常: {e}")

    def monitor_task_execution(self) -> Dict:
        """监控任务执行状态（新增调试方法）"""
        """返回当前任务执行的详细状态，特别关注多阶段任务"""
        
        execution_status = {
            'total_tasks': len(self.tasks),
            'active_tasks': [],
            'multi_stage_tasks': [],
            'stage_progression_issues': [],
            'vehicle_task_mapping': {},
            'summary': {}
        }
        
        # 分析所有任务
        for task_id, task in self.tasks.items():
            task_info = {
                'task_id': task_id,
                'status': task.status.value,
                'vehicle_id': task.vehicle_id,
                'is_multi_stage': task.is_multi_stage(),
                'current_stage': task.current_stage_index,
                'total_stages': len(task.stages),
                'completion_progress': task.get_completion_progress(),
                'has_path': len(task.complete_path) > 0 if task.complete_path else False,
                'backbone_path': task.assigned_backbone_path_id
            }
            
            # 检查是否是活跃任务
            if task.status in [TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS]:
                execution_status['active_tasks'].append(task_info)
                
                # 检查车辆映射
                if task.vehicle_id:
                    execution_status['vehicle_task_mapping'][task.vehicle_id] = task_id
                    
                    # 检查车辆状态一致性
                    if task.vehicle_id in self.vehicle_states:
                        vehicle_state = self.vehicle_states[task.vehicle_id]
                        if vehicle_state.current_task_id != task_id:
                            execution_status['stage_progression_issues'].append({
                                'type': 'vehicle_task_mismatch',
                                'task_id': task_id,
                                'vehicle_id': task.vehicle_id,
                                'vehicle_current_task': vehicle_state.current_task_id
                            })
            
            # 检查多阶段任务
            if task.is_multi_stage():
                stage_details = []
                for i, stage in enumerate(task.stages):
                    stage_details.append({
                        'index': i,
                        'type': stage.stage.value,
                        'target': f"{stage.target_type}_{stage.target_id}",
                        'completed': stage.completed,
                        'is_current': i == task.current_stage_index
                    })
                
                multi_stage_info = task_info.copy()
                multi_stage_info['stages'] = stage_details
                execution_status['multi_stage_tasks'].append(multi_stage_info)
                
                # 检查阶段推进问题
                if task.status == TaskStatus.IN_PROGRESS:
                    current_stage = task.get_current_stage()
                    if current_stage and current_stage.completed:
                        # 当前阶段已完成但任务仍在进行，可能有推进问题
                        if task.current_stage_index + 1 < len(task.stages):
                            execution_status['stage_progression_issues'].append({
                                'type': 'stage_progression_stuck',
                                'task_id': task_id,
                                'current_stage': task.current_stage_index,
                                'stage_completed': True,
                                'has_next_stage': True
                            })
        
        # 生成摘要
        execution_status['summary'] = {
            'total_tasks': len(self.tasks),
            'active_tasks_count': len(execution_status['active_tasks']),
            'multi_stage_tasks_count': len(execution_status['multi_stage_tasks']),
            'progression_issues_count': len(execution_status['stage_progression_issues']),
            'vehicles_with_tasks': len(execution_status['vehicle_task_mapping'])
        }
        
        return execution_status

    def debug_multi_stage_execution(self, task_id: str = None):
        """调试多阶段任务执行（专用方法）"""
        print("\n" + "="*70)
        print("🔧 多阶段任务执行调试")
        print("="*70)
        
        if task_id:
            # 调试特定任务
            if task_id not in self.tasks:
                print(f"❌ 任务 {task_id} 不存在")
                return
            
            task = self.tasks[task_id]
            self._debug_single_multi_stage_task(task)
        else:
            # 调试所有多阶段任务
            multi_stage_tasks = [
                (tid, task) for tid, task in self.tasks.items() 
                if task.is_multi_stage() and task.status in [TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS]
            ]
            
            print(f"📊 发现 {len(multi_stage_tasks)} 个活跃的多阶段任务")
            
            for task_id, task in multi_stage_tasks:
                print(f"\n{'-'*50}")
                self._debug_single_multi_stage_task(task)
        
        # 显示系统级别的诊断
        print(f"\n{'-'*50}")
        print("🔍 系统级别诊断:")
        
        execution_status = self.monitor_task_execution()
        summary = execution_status['summary']
        
        print(f"活跃任务: {summary['active_tasks_count']}/{summary['total_tasks']}")
        print(f"多阶段任务: {summary['multi_stage_tasks_count']}")
        print(f"推进问题: {summary['progression_issues_count']}")
        
        if execution_status['stage_progression_issues']:
            print(f"\n⚠️ 发现的推进问题:")
            for issue in execution_status['stage_progression_issues']:
                print(f"  - {issue['type']}: 任务 {issue['task_id']}")
        
        print("="*70)

    def _debug_single_multi_stage_task(self, task):
        """调试单个多阶段任务的详细信息"""
        print(f"🔍 任务: {task.task_id}")
        print(f"   状态: {task.status.value}")
        print(f"   车辆: {task.vehicle_id}")
        print(f"   网络拓扑: {task.network_topology_type}")
        print(f"   完成进度: {task.get_completion_progress():.1%}")
        
        # 路径信息
        if hasattr(task, 'path_progress'):
            print(f"   路径进度: {getattr(task, 'path_progress', 'N/A')}")
        
        print(f"   路径长度: {len(task.complete_path) if task.complete_path else 0}")
        print(f"   骨干路径: {task.assigned_backbone_path_id}")
        
        # 阶段详情
        print(f"   阶段详情 ({task.current_stage_index + 1}/{len(task.stages)}):")
        for i, stage in enumerate(task.stages):
            marker = "👉" if i == task.current_stage_index else "  "
            status = "✅" if stage.completed else "⏳"
            risk_indicator = "🚨" if stage.spatial_conflict_risk > 0.5 else ""
            
            print(f"   {marker} [{i}] {stage.stage.value} -> {stage.target_type}_{stage.target_id} {status} {risk_indicator}")
            print(f"        位置: ({stage.location[0]:.1f}, {stage.location[1]:.1f})")
            print(f"        层次: {stage.preferred_hierarchy_level}, 风险: {stage.spatial_conflict_risk:.2f}")
        
        # 车辆状态
        if task.vehicle_id and task.vehicle_id in self.vehicle_states:
            vehicle_state = self.vehicle_states[task.vehicle_id]
            print(f"   车辆状态:")
            print(f"     状态: {vehicle_state.current_status.value}")
            print(f"     位置: ({vehicle_state.current_position[0]:.1f}, {vehicle_state.current_position[1]:.1f})")
            print(f"     当前任务ID: {vehicle_state.current_task_id}")
            print(f"     骨干路径: {vehicle_state.current_backbone_path_id}")
            
            # 检查一致性
            if vehicle_state.current_task_id != task.task_id:
                print(f"   ⚠️ 车辆任务ID不匹配!")
        
        # 诊断建议
        issues = []
        
        current_stage = task.get_current_stage()
        if current_stage and current_stage.completed and task.current_stage_index + 1 < len(task.stages):
            issues.append("当前阶段已完成但未推进到下一阶段")
        
        if not task.complete_path:
            issues.append("任务没有完整路径")
        
        if task.vehicle_id and task.vehicle_id in self.vehicle_states:
            vehicle_state = self.vehicle_states[task.vehicle_id]
            if vehicle_state.current_task_id != task.task_id:
                issues.append("车辆与任务的关联不一致")
        
        if issues:
            print(f"   🚨 发现问题:")
            for issue in issues:
                print(f"     - {issue}")
        else:
            print(f"   ✅ 未发现明显问题")

    # 修复任务状态检查方法
    def is_task_ready_for_stage_advancement(self, task_id: str) -> Tuple[bool, str]:
        """检查任务是否准备好进行阶段推进"""
        if task_id not in self.tasks:
            return False, "任务不存在"
        
        task = self.tasks[task_id]
        
        # 检查任务状态
        if task.status not in [TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS]:
            return False, f"任务状态不正确: {task.status.value}"
        
        # 检查是否是多阶段任务
        if not task.is_multi_stage():
            return False, "不是多阶段任务"
        
        # 检查是否还有下一阶段
        if task.current_stage_index + 1 >= len(task.stages):
            return True, "任务可以完成"  # 这种情况下应该完成任务而不是推进阶段
        
        # 检查当前阶段是否完成
        current_stage = task.get_current_stage()
        if not current_stage:
            return False, "无法获取当前阶段"
        
        if not current_stage.completed:
            return False, "当前阶段尚未完成"
        
        # 检查车辆状态
        if not task.vehicle_id or task.vehicle_id not in self.vehicle_states:
            return False, "车辆不存在或未分配"
        
        vehicle_state = self.vehicle_states[task.vehicle_id]
        if vehicle_state.current_task_id != task.task_id:
            return False, "车辆任务关联不一致"
        
        return True, "准备就绪"

    # 添加强制阶段推进方法（用于调试）
    def force_advance_task_stage(self, task_id: str) -> bool:
        """强制推进任务阶段（调试用）"""
        print(f"🔧 [强制推进] 任务 {task_id}")
        
        if task_id not in self.tasks:
            print(f"❌ 任务不存在")
            return False
        
        task = self.tasks[task_id]
        
        # 强制标记当前阶段完成
        current_stage = task.get_current_stage()
        if current_stage:
            current_stage.completed = True
            print(f"   ✅ 强制标记阶段 {task.current_stage_index} 完成")
        
        # 确保任务状态正确
        if task.status != TaskStatus.IN_PROGRESS:
            task.status = TaskStatus.IN_PROGRESS
            print(f"   📝 强制设置任务状态为 IN_PROGRESS")
        
        # 调用正常的阶段推进
        return self.advance_task_stage(task_id)

    def get_assignment_strategy_info(self) -> Dict:
        """获取分配策略信息"""
        return {
            'current_strategy': self.config['default_assignment_strategy'],
            'available_strategies': list(self.assignment_strategies.keys()),
            'last_assigned_vehicle_index': self.last_assigned_vehicle_index,
            'network_topology': self.current_network_topology,
            'consolidation_info': self.consolidation_info
        }

    def debug_vehicle_assignment_status(self) -> Dict:
        """调试车辆分配状态"""
        debug_info = {
            'assignment_summary': {
                'total_vehicles': len(self.vehicle_states),
                'idle_vehicles': 0,
                'busy_vehicles': 0,
                'available_for_assignment': 0
            },
            'vehicles': {}
        }
        
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            is_available = (
                vehicle_state.current_status in [VehicleStatus.IDLE, VehicleStatus.PLANNING] and
                len(vehicle_state.task_queue) < self.config['max_tasks_per_vehicle']
            )
            
            debug_info['vehicles'][vehicle_id] = {
                'status': vehicle_state.current_status.value,
                'position': vehicle_state.current_position,
                'current_task': vehicle_state.current_task_id,
                'task_queue_size': len(vehicle_state.task_queue),
                'total_tasks_completed': vehicle_state.total_tasks_completed,
                'conflict_involvements': vehicle_state.conflict_involvements,
                'available_for_assignment': is_available,
                'network_topology_awareness': vehicle_state.network_topology_awareness,
                'hierarchy_preference': vehicle_state.hierarchy_level_preference
            }
            
            if vehicle_state.current_status == VehicleStatus.IDLE:
                debug_info['assignment_summary']['idle_vehicles'] += 1
            else:
                debug_info['assignment_summary']['busy_vehicles'] += 1
            
            if is_available:
                debug_info['assignment_summary']['available_for_assignment'] += 1
        
        return debug_info

# 兼容性别名
EnhancedBackboneVehicleScheduler = IntegratedBackboneVehicleScheduler