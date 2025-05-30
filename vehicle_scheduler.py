"""
enhanced_vehicle_scheduler.py - 完整修复版：支持多阶段任务的车辆调度器
修复问题：
1. 任务ID重复导致多车共享任务
2. 任务归属验证错误
3. 线程安全问题
4. 支持装载→卸载的多阶段任务
5. 均衡分配策略，避免任务集中在特定车辆
"""

import math
import time
import threading
import uuid
from collections import defaultdict, deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Tuple, Optional, Any, Set

from conflict_control import EnhancedBackboneConflictDetector
from traffic_manager import EnhancedBackboneTrafficManager

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
    """任务阶段"""
    LOADING = "loading"
    TRANSPORT = "transport"
    UNLOADING = "unloading"
    PARKING = "parking"

@dataclass
class TaskStageInfo:
    """任务阶段信息"""
    stage: TaskStage
    location: Tuple[float, float, float]
    target_type: str  # "loading", "unloading", "parking"
    target_id: int
    estimated_duration: float = 60.0  # 默认60秒
    completed: bool = False

@dataclass
class BackboneTask:
    """增强的骨干网络任务 - 支持多阶段"""
    task_id: str
    vehicle_id: Optional[str]
    task_type: str  # "transport", "loading_only", "unloading_only", "maintenance", "repositioning"
    priority: TaskPriority
    
    # 多阶段信息
    stages: List[TaskStageInfo] = field(default_factory=list)
    current_stage_index: int = 0
    
    # 兼容性：保留原有字段
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
    
    # 路径信息
    assigned_backbone_path_id: Optional[str] = None
    complete_path: List[Tuple] = field(default_factory=list)
    path_structure: Dict = field(default_factory=dict)
    node_timing_plan: Dict[int, Tuple[float, float]] = field(default_factory=dict)
    
    # 重试计数
    retry_count: int = 0
    
    def __post_init__(self):
        """初始化后处理"""
        # 如果没有阶段但有起始和结束位置，自动创建阶段
        if not self.stages and self.start_location and self.end_location:
            self._create_stages_from_locations()
    
    def _create_stages_from_locations(self):
        """从起始和结束位置创建任务阶段"""
        if self.task_type == "transport":
            # 运输任务：装载 -> 卸载
            start_type, start_id = self._infer_location_type(self.start_location)
            end_type, end_id = self._infer_location_type(self.end_location)
            
            if start_type == "loading":
                self.stages.append(TaskStageInfo(
                    stage=TaskStage.LOADING,
                    location=self.start_location,
                    target_type=start_type,
                    target_id=start_id,
                    estimated_duration=60.0
                ))
            
            if end_type == "unloading":
                self.stages.append(TaskStageInfo(
                    stage=TaskStage.UNLOADING,
                    location=self.end_location,
                    target_type=end_type,
                    target_id=end_id,
                    estimated_duration=60.0
                ))
        
        elif self.task_type == "loading_only":
            # 只装载
            start_type, start_id = self._infer_location_type(self.start_location)
            if start_type == "loading":
                self.stages.append(TaskStageInfo(
                    stage=TaskStage.LOADING,
                    location=self.start_location,
                    target_type=start_type,
                    target_id=start_id,
                    estimated_duration=60.0
                ))
        
        elif self.task_type == "unloading_only":
            # 只卸载
            end_type, end_id = self._infer_location_type(self.end_location)
            if end_type == "unloading":
                self.stages.append(TaskStageInfo(
                    stage=TaskStage.UNLOADING,
                    location=self.end_location,
                    target_type=end_type,
                    target_id=end_id,
                    estimated_duration=60.0
                ))
        
        # 更新兼容性字段
        if self.stages:
            current_stage = self.stages[self.current_stage_index]
            self.target_type = current_stage.target_type
            self.target_id = current_stage.target_id
    
    def _infer_location_type(self, location: Tuple) -> Tuple[str, int]:
        """从位置推断类型（简化版，实际应该从环境获取）"""
        # 这里简化处理，实际应该从调度器的环境中查找最近的点
        return "loading", 0  # 占位符
    
    def get_current_stage(self) -> Optional[TaskStageInfo]:
        """获取当前阶段"""
        if 0 <= self.current_stage_index < len(self.stages):
            return self.stages[self.current_stage_index]
        return None
    
    def advance_to_next_stage(self) -> bool:
        """前进到下一阶段"""
        if self.current_stage_index < len(self.stages) - 1:
            # 标记当前阶段完成
            if self.current_stage_index < len(self.stages):
                self.stages[self.current_stage_index].completed = True
            
            self.current_stage_index += 1
            
            # 更新兼容性字段
            current_stage = self.get_current_stage()
            if current_stage:
                self.target_type = current_stage.target_type
                self.target_id = current_stage.target_id
            
            return True
        return False
    
    def is_complete(self) -> bool:
        """检查任务是否完成"""
        return self.current_stage_index >= len(self.stages)

@dataclass 
class VehicleState:
    """车辆状态 - 新增passing_status"""
    vehicle_id: str
    current_position: Tuple[float, float, float]
    current_status: VehicleStatus
    priority: int = 2
    
    # === 新增：通行状态 ===
    passing_status: int = 0  # 0=通行中，1=停车中
    
    # 任务信息
    current_task_id: Optional[str] = None
    task_queue: List[str] = field(default_factory=list)
    
    # 骨干网络信息
    current_backbone_path_id: Optional[str] = None
    backbone_segment_occupations: List[str] = field(default_factory=list)
    
    # 性能参数
    max_speed: float = 1.5
    current_speed: float = 0.0
    last_update_time: float = field(default_factory=time.time)
    
    # 统计信息
    total_distance: float = 0.0
    total_tasks_completed: int = 0
    backbone_switches: int = 0
    interface_switches: int = 0
    conflict_involvements: int = 0
    
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
    
    def is_passing(self) -> bool:
        """检查是否在通行中"""
        return self.passing_status == 0
    
    def is_stopped(self) -> bool:
        """检查是否被停车"""
        return self.passing_status == 1
    
    def set_passing(self):
        """设置为通行状态"""
        self.passing_status = 0
    
    def set_stopped(self):
        """设置为停车状态"""
        self.passing_status = 1

class EnhancedBackboneVehicleScheduler:
    """完整修复版骨干网络车辆调度器 - 支持多阶段任务和均衡分配"""
    
    def __init__(self, env, backbone_network, path_planner, conflict_detector, traffic_manager):
        self.env = env
        self.backbone_network = backbone_network
        self.path_planner = path_planner
        self.conflict_detector = conflict_detector
        self.traffic_manager = traffic_manager
        
        # 状态管理
        self.vehicle_states: Dict[str, VehicleState] = {}
        self.tasks: Dict[str, BackboneTask] = {}
        self.task_queue = deque()
        
        # 修复：任务ID生成
        self.task_counter = 0
        self.task_id_lock = threading.Lock()
        
        # 分配策略（新增均衡策略）
        self.assignment_strategies = {
            'nearest_vehicle': self._assign_nearest_vehicle,
            'most_efficient': self._assign_most_efficient,
            'backbone_aware': self._assign_backbone_aware,
            'conflict_minimal': self._assign_conflict_minimal,
            'random_balanced': self._assign_random_balanced,
            'round_robin': self._assign_round_robin,
        }
        
        # 配置参数
        self.config = {
            'default_assignment_strategy': 'random_balanced',
            'max_tasks_per_vehicle': 3,
            'task_timeout': 3600.0,
            'replanning_threshold': 0.3,
            'interface_spacing': 8,
            'node_stop_time': 2.0,
            'safety_time_margin': 5.0,
            'max_backbone_load_factor': 0.8,
            'enable_proactive_optimization': True,
            'conflict_avoidance_weight': 0.4,
            'backbone_preference_weight': 0.6
        }
        
        # 轮询分配状态
        self.last_assigned_vehicle_index = 0
        
        # 统计信息
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
            'temporal_adjustments': 0
        }
        
        # 性能监控
        self.performance_monitor = {
            'assignment_times': deque(maxlen=100),
            'completion_times': deque(maxlen=100),
            'conflict_events': deque(maxlen=200)
        }
        
        # 线程安全
        self.lock = threading.RLock()
        
        # 设置组件间引用
        if self.traffic_manager:
            self.traffic_manager.set_vehicle_scheduler(self)
        
        print("初始化完整修复版骨干网络车辆调度器（支持多阶段任务和均衡分配）")
    
    # ==================== 修复：唯一任务ID生成 ====================
    
    def _generate_unique_task_id(self, task_type: str = "task") -> str:
        """生成唯一任务ID"""
        with self.task_id_lock:
            self.task_counter += 1
            # 使用微秒级时间戳 + 计数器 + 随机数确保唯一性
            timestamp = int(time.time() * 1000000)  # 微秒级时间戳
            random_suffix = str(uuid.uuid4())[:8]  # 8位随机字符
            return f"{task_type}_{timestamp}_{self.task_counter}_{random_suffix}"
    
    # ==================== 调试和策略管理方法 ====================
    
    def debug_vehicle_assignment_status(self) -> Dict:
        """调试车辆分配状态"""
        debug_info = {
            'vehicles': {},
            'assignment_summary': {
                'total_vehicles': len(self.vehicle_states),
                'idle_vehicles': 0,
                'busy_vehicles': 0,
                'available_for_assignment': 0
            }
        }
        
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            # 检查车辆是否可分配
            is_available = (
                vehicle_state.current_status in [VehicleStatus.IDLE, VehicleStatus.PLANNING] and
                len(vehicle_state.task_queue) < self.config['max_tasks_per_vehicle']
            )
            
            debug_info['vehicles'][vehicle_id] = {
                'status': vehicle_state.current_status.value,
                'position': vehicle_state.current_position,
                'task_queue_size': len(vehicle_state.task_queue),
                'current_task': vehicle_state.current_task_id,
                'available_for_assignment': is_available,
                'total_tasks_completed': vehicle_state.total_tasks_completed,
                'conflict_involvements': vehicle_state.conflict_involvements
            }
            
            # 统计
            if vehicle_state.current_status == VehicleStatus.IDLE:
                debug_info['assignment_summary']['idle_vehicles'] += 1
            else:
                debug_info['assignment_summary']['busy_vehicles'] += 1
            
            if is_available:
                debug_info['assignment_summary']['available_for_assignment'] += 1
        
        return debug_info
    
    def set_assignment_strategy(self, strategy_name: str):
        """动态设置分配策略"""
        if strategy_name in self.assignment_strategies:
            self.config['default_assignment_strategy'] = strategy_name
            print(f"✅ 分配策略已更改为: {strategy_name}")
        else:
            available = list(self.assignment_strategies.keys())
            print(f"❌ 无效策略: {strategy_name}")
            print(f"   可用策略: {available}")
    
    def get_assignment_strategy_info(self) -> Dict:
        """获取分配策略信息"""
        return {
            'current_strategy': self.config['default_assignment_strategy'],
            'available_strategies': {
                'nearest_vehicle': '最近车辆优先',
                'most_efficient': '最高效车辆优先',
                'backbone_aware': '骨干网络感知（智能评分）',
                'conflict_minimal': '冲突最小化',
                'random_balanced': '随机均衡（推荐）',
                'round_robin': '轮询分配'
            },
            'last_assigned_vehicle_index': self.last_assigned_vehicle_index
        }
    
    # ==================== 新增分配策略 ====================
    
    def _assign_random_balanced(self, task: BackboneTask) -> Optional[str]:
        """随机均衡分配策略"""
        import random
        
        # 获取所有可用车辆
        available_vehicles = []
        
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            if (vehicle_state.current_status in [VehicleStatus.IDLE, VehicleStatus.PLANNING] and
                len(vehicle_state.task_queue) < self.config['max_tasks_per_vehicle']):
                available_vehicles.append((vehicle_id, vehicle_state))
        
        if not available_vehicles:
            print("❌ 没有可用车辆")
            return None
        
        # 按任务负载分组（优先选择任务少的车辆）
        task_groups = {}
        for vehicle_id, vehicle_state in available_vehicles:
            task_count = len(vehicle_state.task_queue)
            if task_count not in task_groups:
                task_groups[task_count] = []
            task_groups[task_count].append(vehicle_id)
        
        # 选择任务最少的组
        min_task_count = min(task_groups.keys())
        best_group = task_groups[min_task_count]
        
        # 在最佳组内随机选择
        selected_vehicle = random.choice(best_group)
        
        print(f"随机均衡分配: 从 {len(best_group)} 个候选车辆中选择 {selected_vehicle} (任务数: {min_task_count})")
        return selected_vehicle
    
    def _assign_round_robin(self, task: BackboneTask) -> Optional[str]:
        """轮询分配策略"""
        # 获取所有可用车辆
        available_vehicles = []
        
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            if (vehicle_state.current_status in [VehicleStatus.IDLE, VehicleStatus.PLANNING] and
                len(vehicle_state.task_queue) < self.config['max_tasks_per_vehicle']):
                available_vehicles.append(vehicle_id)
        
        if not available_vehicles:
            return None
        
        # 按车辆ID排序确保一致性
        available_vehicles.sort()
        
        # 轮询选择
        if self.last_assigned_vehicle_index >= len(available_vehicles):
            self.last_assigned_vehicle_index = 0
        
        selected_vehicle = available_vehicles[self.last_assigned_vehicle_index]
        self.last_assigned_vehicle_index = (self.last_assigned_vehicle_index + 1) % len(available_vehicles)
        
        print(f"轮询分配: 选择车辆 {selected_vehicle} (索引: {self.last_assigned_vehicle_index-1}/{len(available_vehicles)-1})")
        return selected_vehicle
    
    def _assign_nearest_vehicle(self, task: BackboneTask) -> Optional[str]:
        """分配最近车辆"""
        best_vehicle = None
        min_distance = float('inf')
        
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            if (vehicle_state.current_status == VehicleStatus.IDLE and
                len(vehicle_state.task_queue) < self.config['max_tasks_per_vehicle']):
                
                distance = math.sqrt(
                    (task.start_location[0] - vehicle_state.current_position[0])**2 +
                    (task.start_location[1] - vehicle_state.current_position[1])**2
                )
                
                if distance < min_distance:
                    min_distance = distance
                    best_vehicle = vehicle_id
        
        return best_vehicle
    
    def _assign_most_efficient(self, task: BackboneTask) -> Optional[str]:
        """分配最高效车辆"""
        best_vehicle = None
        best_efficiency = 0.0
        
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            if (vehicle_state.current_status == VehicleStatus.IDLE and
                len(vehicle_state.task_queue) < self.config['max_tasks_per_vehicle']):
                
                # 计算效率分数
                task_completion_rate = vehicle_state.total_tasks_completed
                distance_efficiency = vehicle_state.total_distance / max(1, vehicle_state.total_tasks_completed)
                conflict_penalty = vehicle_state.conflict_involvements * 0.1
                
                efficiency = task_completion_rate - distance_efficiency * 0.01 - conflict_penalty
                
                if efficiency > best_efficiency:
                    best_efficiency = efficiency
                    best_vehicle = vehicle_id
        
        return best_vehicle
    
    def _assign_backbone_aware(self, task: BackboneTask) -> Optional[str]:
        """骨干网络感知分配"""
        best_vehicle = None
        best_score = float('inf')
        
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            if (vehicle_state.current_status not in [VehicleStatus.IDLE, VehicleStatus.PLANNING] or
                len(vehicle_state.task_queue) >= self.config['max_tasks_per_vehicle']):
                continue
            
            # 计算综合评分
            score = self._calculate_backbone_assignment_score(task, vehicle_state)
            
            if score < best_score:
                best_score = score
                best_vehicle = vehicle_id
        
        return best_vehicle
    
    def _assign_conflict_minimal(self, task: BackboneTask) -> Optional[str]:
        """冲突最小化分配"""
        best_vehicle = None
        min_conflict_risk = float('inf')
        
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            if (vehicle_state.current_status not in [VehicleStatus.IDLE, VehicleStatus.PLANNING] or
                len(vehicle_state.task_queue) >= self.config['max_tasks_per_vehicle']):
                continue
            
            # 评估冲突风险
            conflict_risk = self._assess_conflict_risk(task, vehicle_state)
            
            if conflict_risk < min_conflict_risk:
                min_conflict_risk = conflict_risk
                best_vehicle = vehicle_id
        
        return best_vehicle
    
    def _calculate_backbone_assignment_score(self, task: BackboneTask, 
                                           vehicle_state: VehicleState) -> float:
        """计算骨干网络感知的分配评分"""
        # 基础距离成本
        distance = math.sqrt(
            (task.start_location[0] - vehicle_state.current_position[0])**2 +
            (task.start_location[1] - vehicle_state.current_position[1])**2
        )
        distance_score = distance
        
        # 骨干网络可用性评分
        backbone_score = 0
        if self.backbone_network:
            # 查找到达目标的骨干路径
            candidate_paths = []
            for path_id, path_data in self.backbone_network.bidirectional_paths.items():
                if ((path_data.point_a['type'] == task.target_type and 
                     path_data.point_a['id'] == task.target_id) or
                    (path_data.point_b['type'] == task.target_type and 
                     path_data.point_b['id'] == task.target_id)):
                    candidate_paths.append(path_data)
            
            if candidate_paths:
                # 选择负载最低的路径评分
                min_load = min(p.get_load_factor() for p in candidate_paths)
                backbone_score = min_load * 100  # 负载越高评分越差
        
        # 车辆历史表现
        performance_score = (
            vehicle_state.conflict_involvements * 10 +  # 冲突经历惩罚
            vehicle_state.backbone_switches * 5        # 切换历史惩罚
        )
        
        # 优先级调整
        priority_weight = (6 - task.priority.value) * 5  # 优先级越高权重越小
        
        total_score = (
            distance_score * 0.4 +
            backbone_score * 0.3 +
            performance_score * 0.2 +
            priority_weight * 0.1
        )
        
        return total_score
    
    def _assess_conflict_risk(self, task: BackboneTask, vehicle_state: VehicleState) -> float:
        """评估冲突风险"""
        # 基于车辆历史冲突次数
        historical_risk = vehicle_state.conflict_involvements * 0.2
        
        # 基于当前骨干路径负载
        backbone_risk = 0
        if vehicle_state.current_backbone_path_id and self.backbone_network:
            path_data = self.backbone_network.bidirectional_paths.get(
                vehicle_state.current_backbone_path_id
            )
            if path_data:
                backbone_risk = path_data.get_load_factor() * 10
        
        # 基于任务优先级
        priority_risk = (6 - task.priority.value) * 2
        
        return historical_risk + backbone_risk + priority_risk
    
    # ==================== 车辆初始化 ====================
    
    def initialize_vehicles(self):
        """初始化车辆状态"""
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
                
                self.vehicle_states[vehicle_id] = VehicleState(
                    vehicle_id=vehicle_id,
                    current_position=position,
                    current_status=VehicleStatus.IDLE
                )
        
        print(f"初始化 {len(self.vehicle_states)} 个车辆状态")
    
    # ==================== 任务创建方法（修复版） ====================
    
    def create_transport_task(self, start_location: Tuple, end_location: Tuple,
                             priority: TaskPriority = TaskPriority.NORMAL,
                             vehicle_id: str = None) -> str:
        """创建运输任务（修复版：唯一ID + 多阶段）"""
        # 使用新的ID生成方法
        task_id = self._generate_unique_task_id("transport")
        
        # 确保ID唯一性（双重检查）
        while task_id in self.tasks:
            task_id = self._generate_unique_task_id("transport")
        
        # 推断起始和结束位置的类型
        start_type, start_id = self._infer_target_from_position(start_location)
        end_type, end_id = self._infer_target_from_position(end_location)
        
        print(f"创建唯一任务 {task_id}:")
        print(f"  指定车辆: {vehicle_id}")
        print(f"  起始位置: {start_location} -> {start_type}_{start_id}")
        print(f"  结束位置: {end_location} -> {end_type}_{end_id}")
        
        # 创建任务阶段
        stages = []
        task_type = "transport"
        
        # 如果起始位置是装载点，添加装载阶段
        if start_type == "loading":
            stages.append(TaskStageInfo(
                stage=TaskStage.LOADING,
                location=start_location,
                target_type=start_type,
                target_id=start_id,
                estimated_duration=60.0
            ))
            print(f"  添加装载阶段: L{start_id}")
        
        # 如果结束位置是卸载点，添加卸载阶段
        if end_type == "unloading":
            stages.append(TaskStageInfo(
                stage=TaskStage.UNLOADING,
                location=end_location,
                target_type=end_type,
                target_id=end_id,
                estimated_duration=60.0
            ))
            print(f"  添加卸载阶段: U{end_id}")
        
        # 如果结束位置是停车区，添加停车阶段
        if end_type == "parking":
            stages.append(TaskStageInfo(
                stage=TaskStage.PARKING,
                location=end_location,
                target_type=end_type,
                target_id=end_id,
                estimated_duration=30.0
            ))
            print(f"  添加停车阶段: P{end_id}")
        
        # 根据阶段数量确定任务类型
        if len(stages) == 0:
            print(f"  ⚠️ 无法创建有效阶段，使用默认卸载")
            stages.append(TaskStageInfo(
                stage=TaskStage.UNLOADING,
                location=end_location,
                target_type="unloading",
                target_id=0,
                estimated_duration=60.0
            ))
        elif len(stages) == 1:
            stage = stages[0]
            if stage.stage == TaskStage.LOADING:
                task_type = "loading_only"
            elif stage.stage == TaskStage.UNLOADING:
                task_type = "unloading_only"
        
        # 估算总持续时间
        total_duration = sum(stage.estimated_duration for stage in stages)
        if len(stages) > 1:
            # 添加移动时间
            for i in range(len(stages) - 1):
                distance = math.sqrt(
                    (stages[i+1].location[0] - stages[i].location[0])**2 +
                    (stages[i+1].location[1] - stages[i].location[1])**2
                )
                total_duration += distance / 1.5  # 假设速度1.5m/s
        
        # 创建任务 - 明确设置归属
        task = BackboneTask(
            task_id=task_id,
            vehicle_id=vehicle_id,  # 明确设置归属车辆
            task_type=task_type,
            priority=priority,
            stages=stages,
            start_location=start_location,
            end_location=end_location,
            earliest_start_time=time.time(),
            latest_finish_time=time.time() + total_duration * 3,
            estimated_duration=total_duration
        )
        
        # 设置当前目标为第一个阶段
        current_stage = task.get_current_stage()
        if current_stage:
            task.target_type = current_stage.target_type
            task.target_id = current_stage.target_id
        
        with self.lock:
            # 再次确保ID唯一
            if task_id in self.tasks:
                print(f"❌ 任务ID冲突: {task_id}")
                return ""
            
            self.tasks[task_id] = task
            
            # 如果指定了车辆，不放入队列，直接标记为特定车辆任务
            if not vehicle_id:
                self.task_queue.append(task_id)
            
            self.stats['total_tasks_created'] += 1
        
        print(f"✅ 创建任务 {task_id}: {task_type}, {len(stages)}个阶段, 归属车辆: {vehicle_id}")
        return task_id
    
    def create_loading_only_task(self, loading_location: Tuple,
                                priority: TaskPriority = TaskPriority.NORMAL,
                                vehicle_id: str = None) -> str:
        """创建纯装载任务（修复版）"""
        task_id = self._generate_unique_task_id("load")
        
        while task_id in self.tasks:
            task_id = self._generate_unique_task_id("load")
        
        loading_type, loading_id = self._infer_target_from_position(loading_location)
        
        if loading_type != "loading":
            print(f"⚠️ 指定位置不是装载点: {loading_location}")
            return ""
        
        stages = [TaskStageInfo(
            stage=TaskStage.LOADING,
            location=loading_location,
            target_type=loading_type,
            target_id=loading_id,
            estimated_duration=60.0
        )]
        
        task = BackboneTask(
            task_id=task_id,
            vehicle_id=vehicle_id,
            task_type="loading_only",
            priority=priority,
            stages=stages,
            start_location=loading_location,
            end_location=loading_location,
            earliest_start_time=time.time(),
            latest_finish_time=time.time() + 300,
            estimated_duration=60.0,
            target_type=loading_type,
            target_id=loading_id
        )
        
        with self.lock:
            self.tasks[task_id] = task
            if not vehicle_id:
                self.task_queue.append(task_id)
            self.stats['total_tasks_created'] += 1
        
        print(f"✅ 创建装载任务 {task_id}: L{loading_id}, 归属车辆: {vehicle_id}")
        return task_id
    
    def create_unloading_only_task(self, unloading_location: Tuple,
                                  priority: TaskPriority = TaskPriority.NORMAL,
                                  vehicle_id: str = None) -> str:
        """创建纯卸载任务（修复版）"""
        task_id = self._generate_unique_task_id("unload")
        
        while task_id in self.tasks:
            task_id = self._generate_unique_task_id("unload")
        
        unloading_type, unloading_id = self._infer_target_from_position(unloading_location)
        
        if unloading_type != "unloading":
            print(f"⚠️ 指定位置不是卸载点: {unloading_location}")
            return ""
        
        stages = [TaskStageInfo(
            stage=TaskStage.UNLOADING,
            location=unloading_location,
            target_type=unloading_type,
            target_id=unloading_id,
            estimated_duration=60.0
        )]
        
        task = BackboneTask(
            task_id=task_id,
            vehicle_id=vehicle_id,
            task_type="unloading_only",
            priority=priority,
            stages=stages,
            start_location=unloading_location,
            end_location=unloading_location,
            earliest_start_time=time.time(),
            latest_finish_time=time.time() + 300,
            estimated_duration=60.0,
            target_type=unloading_type,
            target_id=unloading_id
        )
        
        with self.lock:
            self.tasks[task_id] = task
            if not vehicle_id:
                self.task_queue.append(task_id)
            self.stats['total_tasks_created'] += 1
        
        print(f"✅ 创建卸载任务 {task_id}: U{unloading_id}, 归属车辆: {vehicle_id}")
        return task_id
    
    # ==================== 任务分配方法（修复版） ====================
    
    def assign_task_with_corridors(self, task_id: str, vehicle_id: str = None) -> bool:
        """分配任务（GUI兼容接口）"""
        return self.assign_task(task_id, vehicle_id)
    
    def assign_task(self, task_id: str, vehicle_id: str = None) -> bool:
        """分配任务到车辆（修复版 - 严格检查归属）"""
        assignment_start = time.time()
        
        if task_id not in self.tasks:
            print(f"❌ 任务不存在: {task_id}")
            return False
        
        task = self.tasks[task_id]
        
        with self.lock:
            # 修复：严格检查任务状态和归属
            if task.status != TaskStatus.PENDING:
                print(f"❌ 任务状态不正确: {task.status.value} (任务 {task_id})")
                return False
            
            # 如果任务已指定车辆，必须匹配
            if task.vehicle_id is not None:
                if vehicle_id is None:
                    vehicle_id = task.vehicle_id
                elif vehicle_id != task.vehicle_id:
                    print(f"❌ 任务 {task_id} 已预定给车辆 {task.vehicle_id}，不能分配给 {vehicle_id}")
                    return False
            
            # 调试：打印车辆状态
            debug_info = self.debug_vehicle_assignment_status()
            print(f"\n📊 车辆分配状态调试:")
            print(f"   总车辆: {debug_info['assignment_summary']['total_vehicles']}")
            print(f"   空闲车辆: {debug_info['assignment_summary']['idle_vehicles']}")
            print(f"   可分配车辆: {debug_info['assignment_summary']['available_for_assignment']}")
            
            for vid, vinfo in debug_info['vehicles'].items():
                status_marker = "✅" if vinfo['available_for_assignment'] else "❌"
                print(f"   {status_marker} 车辆{vid}: {vinfo['status']}, 队列: {vinfo['task_queue_size']}")
            
            # 如果没有指定车辆，选择合适的车辆
            if vehicle_id is None:
                strategy_name = self.config['default_assignment_strategy']
                strategy = self.assignment_strategies[strategy_name]
                selected_vehicle = strategy(task)
                
                if not selected_vehicle:
                    print(f"❌ 无法为任务 {task_id} 找到合适车辆")
                    print(f"   使用策略: {strategy_name}")
                    return False
                
                vehicle_id = selected_vehicle
            
            # 检查车辆是否存在且可用
            if vehicle_id not in self.vehicle_states:
                print(f"❌ 车辆不存在: {vehicle_id}")
                return False
            
            vehicle_state = self.vehicle_states[vehicle_id]
            
            # 检查车辆是否已有任务
            if vehicle_state.current_task_id is not None:
                print(f"❌ 车辆 {vehicle_id} 已有任务: {vehicle_state.current_task_id}")
                return False
            
            print(f"\n🎯 分配多阶段任务 {task_id} 给车辆 {vehicle_id}")
            print(f"   分配策略: {self.config['default_assignment_strategy']}")
            print(f"   任务类型: {task.task_type}")
            print(f"   阶段数量: {len(task.stages)}")
            
            # 获取当前阶段
            current_stage = task.get_current_stage()
            if not current_stage:
                print(f"❌ 任务没有有效阶段")
                return False
            
            print(f"   当前阶段: {current_stage.stage.value} -> {current_stage.target_type}_{current_stage.target_id}")
            
            # 规划到当前阶段目标的路径
            path_result = self._plan_backbone_path_to_stage(task, vehicle_state, current_stage)
            if not path_result:
                print(f"❌ 当前阶段路径规划失败")
                return False
            
            complete_path, structure = path_result
            backbone_path_id = structure.get('path_id')
            
            # 计算节点时序计划
            node_timing_plan = self._calculate_node_timing_plan(
                complete_path, structure, vehicle_state
            )
            
            # 记录到冲突检测器
            success = self.conflict_detector.record_vehicle_backbone_occupation(
                vehicle_id, backbone_path_id or "direct", 
                node_timing_plan, task.priority.value
            )
            
            if not success:
                print(f"❌ 占用记录失败")
                return False
            
            # 检测并解决冲突
            self._handle_conflicts_for_vehicle(vehicle_id)
            
            # 原子性更新：先更新任务，再更新车辆
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
            
            # 统计信息
            assignment_time = time.time() - assignment_start
            self.performance_monitor['assignment_times'].append(assignment_time)
            self.stats['total_tasks_assigned'] += 1
            self._update_average_assignment_time()
            
            print(f"✅ 多阶段任务分配成功，耗时: {assignment_time:.2f}s")
            print(f"   骨干路径: {backbone_path_id}")
            print(f"   路径长度: {len(complete_path)}")
            
            return True
    
    def _plan_backbone_path_to_stage(self, task: BackboneTask, vehicle_state: VehicleState, 
                                    stage: TaskStageInfo) -> Optional[Tuple]:
        """规划到特定阶段的骨干路径"""
        try:
            print(f"    规划路径到阶段: {stage.stage.value} ({stage.target_type}_{stage.target_id})")
            
            # 优先使用骨干网络
            if self.backbone_network:
                backbone_result = self.backbone_network.get_path_from_position_to_target(
                    vehicle_state.current_position,
                    stage.target_type,  # 目标类型
                    stage.target_id,    # 目标ID
                    vehicle_state.vehicle_id
                )
                
                if backbone_result:
                    if isinstance(backbone_result, tuple):
                        print(f"    ✅ 骨干网络路径成功")
                        return backbone_result
                    else:
                        return backbone_result, {'type': 'backbone'}
            
            # 回退到直接路径规划
            if self.path_planner:
                print(f"    回退到直接路径规划")
                direct_result = self.path_planner.plan_path(
                    vehicle_id=vehicle_state.vehicle_id,
                    start=vehicle_state.current_position,
                    goal=stage.location,
                    use_backbone=False,
                    context='navigation',
                    target_type=stage.target_type,
                    target_id=stage.target_id
                )
                
                if direct_result:
                    if hasattr(direct_result, 'path'):
                        path = direct_result.path
                    else:
                        path = direct_result
                    
                    structure = {
                        'type': 'direct',
                        'backbone_utilization': 0.0,
                        'total_length': len(path)
                    }
                    
                    print(f"    ✅ 直接路径成功")
                    return path, structure
            
        except Exception as e:
            print(f"    ❌ 路径规划异常: {e}")
        
        return None
    
    def advance_task_stage(self, task_id: str) -> bool:
        """推进任务到下一阶段 - 修复版"""
        if task_id not in self.tasks:
            return False
        
        task = self.tasks[task_id]
        vehicle_id = task.vehicle_id
        
        if not vehicle_id or vehicle_id not in self.vehicle_states:
            return False
        
        vehicle_state = self.vehicle_states[vehicle_id]
        
        print(f"\n⏭️ [修复] 推进任务 {task_id} 阶段")
        print(f"   当前阶段索引: {task.current_stage_index}/{len(task.stages)}")
        
        # 关键修复：获取当前位置（从环境中获取最新位置）
        if vehicle_id in self.env.vehicles:
            env_vehicle = self.env.vehicles[vehicle_id]
            if hasattr(env_vehicle, 'position'):
                current_position = env_vehicle.position
            else:
                current_position = env_vehicle.get('position', vehicle_state.current_position)
            
            # 更新车辆状态中的位置
            vehicle_state.current_position = current_position
            print(f"   当前位置: ({current_position[0]:.1f}, {current_position[1]:.1f})")
        
        # 标记当前阶段完成
        current_stage = task.get_current_stage()
        if current_stage:
            current_stage.completed = True
            print(f"   完成阶段: {current_stage.stage.value} @ {current_stage.target_type}_{current_stage.target_id}")
        
        # 前进到下一阶段
        if task.advance_to_next_stage():
            next_stage = task.get_current_stage()
            print(f"   下一阶段: {next_stage.stage.value} @ {next_stage.target_type}_{next_stage.target_id}")
            print(f"   目标位置: ({next_stage.location[0]:.1f}, {next_stage.location[1]:.1f})")
            
            # 关键修复：从当前位置（而非原始位置）规划到下一阶段
            path_result = self._plan_backbone_path_to_stage(task, vehicle_state, next_stage)
            if path_result:
                complete_path, structure = path_result
                backbone_path_id = structure.get('path_id')
                
                print(f"   ✅ 新路径规划成功: {len(complete_path)} 个点")
                print(f"   起点: ({complete_path[0][0]:.1f}, {complete_path[0][1]:.1f})")
                print(f"   终点: ({complete_path[-1][0]:.1f}, {complete_path[-1][1]:.1f})")
                
                # 更新任务路径信息
                task.assigned_backbone_path_id = backbone_path_id
                task.complete_path = complete_path
                task.path_structure = structure
                
                # 重新计算时序 - 从当前时间开始
                node_timing_plan = self._calculate_node_timing_plan(
                    complete_path, structure, vehicle_state
                )
                task.node_timing_plan = node_timing_plan
                
                # 关键修复：重置路径进度
                task.path_progress = 0.0
                task.current_path_index = 0
                
                # 更新冲突检测器
                self.conflict_detector.record_vehicle_backbone_occupation(
                    vehicle_id, backbone_path_id or "direct", 
                    node_timing_plan, task.priority.value
                )
                
                print(f"   ✅ 阶段推进成功")
                return True
            else:
                print(f"   ❌ 新阶段路径规划失败")
                return False
        else:
            # 任务完成
            print(f"   🎉 所有阶段完成！")
            task.status = TaskStatus.COMPLETED
            task.completion_time = time.time()
            vehicle_state.current_task_id = None
            vehicle_state.current_status = VehicleStatus.IDLE
            vehicle_state.total_tasks_completed += 1
            
            self.stats['total_tasks_completed'] += 1
            return True

    
    # ==================== 辅助方法 ====================
    
    def _infer_target_from_position(self, position: Tuple) -> Tuple[Optional[str], Optional[int]]:
        """从位置推断目标类型"""
        min_distance = float('inf')
        best_match = (None, None)
        
        # 检查装载点
        for i, loading_point in enumerate(self.env.loading_points):
            distance = math.sqrt(
                (position[0] - loading_point[0])**2 +
                (position[1] - loading_point[1])**2
            )
            if distance < min_distance:
                min_distance = distance
                best_match = ('loading', i)
        
        # 检查卸载点
        for i, unloading_point in enumerate(self.env.unloading_points):
            distance = math.sqrt(
                (position[0] - unloading_point[0])**2 +
                (position[1] - unloading_point[1])**2
            )
            if distance < min_distance:
                min_distance = distance
                best_match = ('unloading', i)
        
        # 检查停车区
        if hasattr(self.env, 'parking_areas'):
            for i, parking_area in enumerate(self.env.parking_areas):
                distance = math.sqrt(
                    (position[0] - parking_area[0])**2 +
                    (position[1] - parking_area[1])**2
                )
                if distance < min_distance:
                    min_distance = distance
                    best_match = ('parking', i)
        
        # 距离阈值检查
        if min_distance > 20:
            return (None, None)
        
        return best_match
    
    def _calculate_node_timing_plan(self, complete_path: List, structure: Dict, 
                                vehicle_state) -> Dict[int, Tuple[float, float]]:
        """计算节点时序计划 - 修复版"""
        timing_plan = {}
        
        if not complete_path:
            return timing_plan
        
        # 获取骨干路径部分
        backbone_path = structure.get('backbone_path', [])
        if not backbone_path:
            backbone_path = complete_path
        
        # 参数设置
        vehicle_speed = vehicle_state.max_speed
        node_stop_time = self.config['node_stop_time']
        interface_spacing = self.config['interface_spacing']
        
        # 修复：从当前时间开始计算，而不是历史时间
        current_time = time.time()
        travel_time = current_time
        
        # 计算接入路径的时间（如果有）
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
        
        print(f"   时序计划: {len(timing_plan)} 个节点")
        for node_idx, (arr, dep) in timing_plan.items():
            print(f"     节点{node_idx}: {arr:.1f}-{dep:.1f}s")
        
        return timing_plan
    
    def _handle_conflicts_for_vehicle(self, vehicle_id: str):
        """处理特定车辆的冲突"""
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
                    'time': time.time()
                })
            
            # 委托交通管理器处理
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
    
    # ==================== 冲突解决接口方法 ====================
    
    def get_vehicle_priority(self, vehicle_id: str) -> int:
        """获取车辆优先级"""
        vehicle_state = self.vehicle_states.get(vehicle_id)
        if vehicle_state:
            return vehicle_state.priority
        return 2  # 默认优先级
    
    def get_vehicle_target_info(self, vehicle_id: str) -> Optional[Dict]:
        """获取车辆目标信息"""
        vehicle_state = self.vehicle_states.get(vehicle_id)
        if not vehicle_state or not vehicle_state.current_task_id:
            return None
        
        task = self.tasks.get(vehicle_state.current_task_id)
        if task:
            current_stage = task.get_current_stage()
            if current_stage:
                return {
                    'target_type': current_stage.target_type,
                    'target_id': current_stage.target_id,
                    'target_location': current_stage.location
                }
        return None
    
    def delay_vehicle_backbone_timing(self, vehicle_id: str, backbone_path_id: str, 
                                    delay_seconds: float) -> bool:
        """延迟车辆骨干路径时序"""
        vehicle_state = self.vehicle_states.get(vehicle_id)
        if not vehicle_state or not vehicle_state.current_task_id:
            return False
        
        task = self.tasks.get(vehicle_state.current_task_id)
        if not task or task.assigned_backbone_path_id != backbone_path_id:
            return False
        
        # 调整节点时序计划
        old_timing = task.node_timing_plan.copy()
        new_timing = {}
        
        for node_idx, (arrival, departure) in old_timing.items():
            new_timing[node_idx] = (arrival + delay_seconds, departure + delay_seconds)
        
        task.node_timing_plan = new_timing
        
        # 更新冲突检测器中的记录
        success = self.conflict_detector.record_vehicle_backbone_occupation(
            vehicle_id, backbone_path_id, new_timing, task.priority.value
        )
        
        if success:
            self.stats['temporal_adjustments'] += 1
            print(f"车辆 {vehicle_id} 延迟 {delay_seconds:.1f}s")
        
        return success
    
    def switch_vehicle_backbone_path(self, vehicle_id: str, new_backbone_path_id: str) -> bool:
        """切换车辆骨干路径"""
        vehicle_state = self.vehicle_states.get(vehicle_id)
        if not vehicle_state or not vehicle_state.current_task_id:
            return False
        
        task = self.tasks.get(vehicle_state.current_task_id)
        if not task:
            return False
        
        # 获取目标信息
        target_info = self.get_vehicle_target_info(vehicle_id)
        if not target_info:
            return False
        
        try:
            # 强制使用指定骨干路径重新规划
            alternative_result = self.backbone_network.force_vehicle_path_switch(
                vehicle_id, vehicle_state.current_position,
                target_info['target_type'], target_info['target_id']
            )
            
            if alternative_result:
                if isinstance(alternative_result, tuple):
                    new_path, structure = alternative_result
                else:
                    new_path = alternative_result
                    structure = {'path_id': new_backbone_path_id}
                
                # 重新计算时序计划
                new_timing = self._calculate_node_timing_plan(new_path, structure, vehicle_state)
                
                # 更新任务信息
                task.assigned_backbone_path_id = new_backbone_path_id
                task.complete_path = new_path
                task.path_structure = structure
                task.node_timing_plan = new_timing
                
                # 更新车辆状态
                vehicle_state.current_backbone_path_id = new_backbone_path_id
                vehicle_state.backbone_switches += 1
                
                # 更新冲突检测器
                success = self.conflict_detector.record_vehicle_backbone_occupation(
                    vehicle_id, new_backbone_path_id, new_timing, task.priority.value
                )
                
                if success:
                    self.stats['backbone_path_switches'] += 1
                    print(f"车辆 {vehicle_id} 切换到骨干路径 {new_backbone_path_id}")
                
                return success
            
        except Exception as e:
            print(f"骨干路径切换失败: {e}")
        
        return False
    
    def try_alternative_interface_node(self, vehicle_id: str, backbone_path_id: str) -> bool:
        """尝试备选接入节点"""
        vehicle_state = self.vehicle_states.get(vehicle_id)
        if not vehicle_state or not vehicle_state.current_task_id:
            return False
        
        task = self.tasks.get(vehicle_state.current_task_id)
        if not task or task.assigned_backbone_path_id != backbone_path_id:
            return False
        
        # 获取目标信息
        target_info = self.get_vehicle_target_info(vehicle_id)
        if not target_info:
            return False
        
        try:
            # 重新规划路径，可能选择不同的接入节点
            backbone_result = self.backbone_network.get_path_from_position_to_target(
                vehicle_state.current_position,
                target_info['target_type'],
                target_info['target_id'],
                vehicle_id
            )
            
            if backbone_result:
                if isinstance(backbone_result, tuple):
                    new_path, structure = backbone_result
                else:
                    new_path = backbone_result
                    structure = {}
                
                # 检查是否确实改变了接入点
                if len(new_path) != len(task.complete_path):
                    # 重新计算时序
                    new_timing = self._calculate_node_timing_plan(new_path, structure, vehicle_state)
                    
                    # 更新任务信息
                    task.complete_path = new_path
                    task.path_structure = structure
                    task.node_timing_plan = new_timing
                    
                    # 更新车辆状态
                    vehicle_state.interface_switches += 1
                    
                    # 更新冲突检测器
                    success = self.conflict_detector.record_vehicle_backbone_occupation(
                        vehicle_id, backbone_path_id, new_timing, task.priority.value
                    )
                    
                    if success:
                        self.stats['interface_node_switches'] += 1
                        print(f"车辆 {vehicle_id} 切换接入节点成功")
                    
                    return success
        
        except Exception as e:
            print(f"接入节点切换失败: {e}")
        
        return False
    
    def emergency_stop_vehicle(self, vehicle_id: str) -> bool:
        """紧急停车（修改版 - 使用passing_status）"""
        vehicle_state = self.vehicle_states.get(vehicle_id)
        if not vehicle_state:
            return False
        
        # 设置停车状态
        vehicle_state.set_stopped()  # passing_status = 1
        vehicle_state.current_status = VehicleStatus.WAITING
        vehicle_state.current_speed = 0.0
        
        # 如果有当前任务，暂停任务
        if vehicle_state.current_task_id:
            task = self.tasks.get(vehicle_state.current_task_id)
            if task:
                task.status = TaskStatus.SUSPENDED
        
        print(f"🛑 车辆 {vehicle_id} 紧急停车 (passing_status=1)")
        return True
    def resume_vehicle_passing(self, vehicle_id: str) -> bool:
        """恢复车辆通行"""
        vehicle_state = self.vehicle_states.get(vehicle_id)
        if not vehicle_state:
            return False
        
        # 恢复通行状态
        vehicle_state.set_passing()  # passing_status = 0
        vehicle_state.current_status = VehicleStatus.MOVING
        vehicle_state.current_speed = vehicle_state.max_speed
        
        # 恢复任务
        if vehicle_state.current_task_id:
            task = self.tasks.get(vehicle_state.current_task_id)
            if task and task.status == TaskStatus.SUSPENDED:
                task.status = TaskStatus.IN_PROGRESS
        
        print(f"🚀 车辆 {vehicle_id} 恢复通行 (passing_status=0)")
        return True    
    # ==================== 更新和统计方法 ====================
    
    def update(self, time_delta: float):
        """更新调度器状态"""
        # 更新车辆状态
        self._update_vehicle_states(time_delta)
        
        # 处理任务队列
        self._process_task_queue()
        
        # 监控任务进度
        self._monitor_task_progress()
        
        # 更新统计信息
        self._update_statistics()
    
    def _update_vehicle_states(self, time_delta: float):
        """更新车辆状态"""
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            if vehicle_id in self.env.vehicles:
                env_vehicle = self.env.vehicles[vehicle_id]
                
                # 兼容不同的数据格式
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
        max_process_per_cycle = 5  # 限制每次处理的任务数
        
        while self.task_queue and processed_count < max_process_per_cycle:
            task_id = self.task_queue.popleft()
            task = self.tasks.get(task_id)
            
            if not task or task.status != TaskStatus.PENDING:
                processed_count += 1
                continue
            
            if self.assign_task(task_id):
                print(f"✅ 队列任务分配成功: {task_id}")
            else:
                # 重新排队，但限制重试次数
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
                # 检查超时
                if current_time > task.latest_finish_time:
                    task.status = TaskStatus.FAILED
                    self.stats['total_tasks_failed'] += 1
                    print(f"⏰ 任务超时失败: {task_id}")
                    
                    # 清理车辆状态
                    if task.vehicle_id and task.vehicle_id in self.vehicle_states:
                        vehicle_state = self.vehicle_states[task.vehicle_id]
                        if vehicle_state.current_task_id == task_id:
                            vehicle_state.current_task_id = None
                            vehicle_state.current_status = VehicleStatus.IDLE
            
            elif task.status == TaskStatus.ASSIGNED:
                # 检查是否可以开始执行
                if current_time >= task.earliest_start_time:
                    task.status = TaskStatus.IN_PROGRESS
                    
                    if task.vehicle_id and task.vehicle_id in self.vehicle_states:
                        vehicle_state = self.vehicle_states[task.vehicle_id]
                        vehicle_state.current_status = VehicleStatus.MOVING
    
    def _update_statistics(self):
        """更新统计信息"""
        # 更新平均分配时间
        self._update_average_assignment_time()
        
        # 更新平均完成时间
        self._update_average_completion_time()
        
        # 计算骨干网络利用率
        self._calculate_backbone_utilization()
        
        # 计算车辆效率
        self._calculate_vehicle_efficiency()
        
        # 计算冲突解决成功率
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
                # 简单效率计算：任务完成数 / (距离 + 冲突次数)
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
    
    def get_comprehensive_stats(self) -> Dict:
        """获取综合统计信息（GUI兼容接口）"""
        return {
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
            },
            'vehicle_performance': {
                vehicle_id: {
                    'tasks_completed': vs.total_tasks_completed,
                    'total_distance': vs.total_distance,
                    'conflict_involvements': vs.conflict_involvements,
                    'backbone_switches': vs.backbone_switches,
                    'interface_switches': vs.interface_switches
                }
                for vehicle_id, vs in self.vehicle_states.items()
            }
        }
    
    def shutdown(self):
        """关闭调度器"""
        # 清理所有状态
        with self.lock:
            self.vehicle_states.clear()
            self.tasks.clear()
            self.task_queue.clear()
        
        print("完整修复版骨干网络车辆调度器已关闭")


# ==================== 系统集成接口 ====================

def create_integrated_backbone_system(env, backbone_network, path_planner):
    """创建集成的骨干网络调度系统"""
    
    # 1. 创建冲突检测器
    conflict_detector = EnhancedBackboneConflictDetector(backbone_network)
    
    # 2. 创建交通管理器
    traffic_manager = EnhancedBackboneTrafficManager(env, backbone_network, conflict_detector)
    
    # 3. 创建车辆调度器
    vehicle_scheduler = EnhancedBackboneVehicleScheduler(
        env, backbone_network, path_planner, conflict_detector, traffic_manager
    )
    
    return {
        'conflict_detector': conflict_detector,
        'traffic_manager': traffic_manager,
        'vehicle_scheduler': vehicle_scheduler
    }

# GUI兼容性接口
ISTCsVehicleScheduler = EnhancedBackboneVehicleScheduler
ISTCsTrafficManager = EnhancedBackboneTrafficManager