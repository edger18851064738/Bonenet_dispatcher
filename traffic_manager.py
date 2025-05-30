"""
traffic_manager.py - 集成恢复功能的修复版
直接在交通管理器中集成车辆恢复逻辑，添加passing_status状态管理
"""

import time
import threading
from collections import defaultdict, deque
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Tuple, Optional, Any, Set

from conflict_control import (
    BackboneConflict, ConflictType, ConflictSeverity, ResolutionStrategy,
    EnhancedBackboneConflictDetector
)

# 重新定义枚举避免循环导入
class VehicleStatus(Enum):
    """车辆状态"""
    IDLE = "idle"
    MOVING = "moving"
    LOADING = "loading"
    UNLOADING = "unloading"
    WAITING = "waiting"
    PLANNING = "planning"
    CONFLICT_RESOLVING = "conflict_resolving"

class TaskStatus(Enum):
    """任务状态"""
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    SUSPENDED = "suspended"

class ResolutionResult(Enum):
    """解决结果"""
    SUCCESS = "success"
    PARTIAL_SUCCESS = "partial_success" 
    FAILURE = "failure"
    RETRY_REQUIRED = "retry_required"

@dataclass
class ResolutionAttempt:
    """解决尝试记录"""
    conflict_id: str
    strategy: ResolutionStrategy
    attempt_time: float
    result: ResolutionResult
    vehicles_affected: List[str]
    details: Dict = None
    
    def __post_init__(self):
        if self.details is None:
            self.details = {}

@dataclass 
class VehicleStopRecord:
    """车辆停车记录"""
    vehicle_id: str
    conflict_id: str
    stop_time: float
    stop_reason: str
    min_stop_duration: float = 30.0  # 最小停车时间（秒）
    recovery_attempts: int = 0
    last_check_time: float = 0.0

class EnhancedBackboneTrafficManager:
    """增强版骨干网络交通管理器 - 集成恢复功能"""
    
    def __init__(self, env, backbone_network, conflict_detector):
        self.env = env
        self.backbone_network = backbone_network
        self.conflict_detector = conflict_detector
        self.vehicle_scheduler = None  # 后续注入
        
        # 解决策略映射
        self.resolution_strategies = {
            ResolutionStrategy.FIRST_COME_FIRST_SERVE: self._resolve_first_come_first_serve,
            ResolutionStrategy.PRIORITY_PREEMPTION: self._resolve_priority_preemption,
            ResolutionStrategy.TEMPORAL_ADJUSTMENT: self._resolve_temporal_adjustment,
            ResolutionStrategy.ALTERNATIVE_BACKBONE_PATH: self._resolve_alternative_backbone,
            ResolutionStrategy.ALTERNATIVE_INTERFACE: self._resolve_alternative_interface,
            ResolutionStrategy.EMERGENCY_STOP: self._resolve_emergency_stop_with_recovery
        }
        
        # === 新增：恢复管理 ===
        self.stopped_vehicles: Dict[str, VehicleStopRecord] = {}  # 停车记录
        self.recovery_check_interval = 5.0  # 恢复检查间隔（秒）
        self.last_recovery_check = time.time()
        self.max_stop_duration = 120.0  # 最大停车时间（秒）
        
        # 解决历史和统计
        self.resolution_history: List[ResolutionAttempt] = []
        self.active_resolutions: Dict[str, ResolutionAttempt] = {}
        
        # 配置参数
        self.config = {
            'max_resolution_attempts': 3,
            'resolution_timeout': 30.0,
            'temporal_adjustment_step': 10.0,
            'max_temporal_delay': 120.0,
            'priority_override_threshold': 2,
            'emergency_threshold': 5,
            'cleanup_interval': 300.0,
            'enable_proactive_resolution': True,
            'backbone_switching_cost': 0.3,
            # 新增恢复配置
            'enable_auto_recovery': True,
            'recovery_check_interval': 5.0,
            'force_recovery_timeout': 120.0,
        }
        
        # 统计信息
        self.stats = {
            'total_conflicts_processed': 0,
            'conflicts_resolved': 0,
            'resolution_success_rate': 0.0,
            'average_resolution_time': 0.0,
            'strategy_usage': defaultdict(int),
            'strategy_success_rates': defaultdict(lambda: {'success': 0, 'total': 0}),
            'first_come_resolutions': 0,
            'priority_resolutions': 0,
            'backbone_switches': 0,
            'interface_switches': 0,
            'temporal_adjustments': 0,
            'emergency_stops': 0,
            # 新增恢复统计
            'vehicles_stopped': 0,
            'vehicles_recovered': 0,
            'auto_recoveries': 0,
            'force_recoveries': 0
        }
        
        # 性能监控
        self.performance_monitor = {
            'resolution_times': deque(maxlen=100),
            'successful_strategies': deque(maxlen=200),
            'last_cleanup_time': time.time()
        }
        
        # 线程安全
        self.lock = threading.RLock()
        
        print("初始化增强版骨干网络交通管理器（集成恢复功能）")
    
    def set_vehicle_scheduler(self, scheduler):
        """注入车辆调度器引用"""
        self.vehicle_scheduler = scheduler
        print("交通管理器已连接车辆调度器")
    
    # === 新增：车辆停车和恢复管理 ===
    
    def stop_vehicle(self, vehicle_id: str, conflict_id: str, reason: str = "conflict_resolution", 
                    conflict_vehicle_count: int = 2) -> bool:
        """停车车辆并记录（增加最小停车时间计算）"""
        if not self.vehicle_scheduler:
            return False
        
        try:
            vehicle_state = self.vehicle_scheduler.vehicle_states.get(vehicle_id)
            if not vehicle_state:
                return False
            
            # 设置passing_status = 1 (停车)
            if not hasattr(vehicle_state, 'passing_status'):
                vehicle_state.passing_status = 0  # 初始化为通行状态
            
            vehicle_state.passing_status = 1  # 停车状态
            vehicle_state.current_status = VehicleStatus.WAITING
            vehicle_state.current_speed = 0.0
            
            # 暂停任务
            if vehicle_state.current_task_id:
                task = self.vehicle_scheduler.tasks.get(vehicle_state.current_task_id)
                if task:
                    task.status = TaskStatus.SUSPENDED
            
            # === 计算最小停车时间 ===
            base_stop_time = 30.0  # 基础停车时间30秒
            
            # 根据冲突车辆数量调整停车时间
            vehicle_factor = min(conflict_vehicle_count - 1, 5) * 15.0  # 每增加1车辆增加15秒，最多5车辆
            
            # 根据车辆优先级调整（优先级低的停车时间长）
            priority_factor = max(0, 3 - vehicle_state.priority) * 10.0
            
            min_stop_duration = base_stop_time + vehicle_factor + priority_factor
            
            # 记录停车信息
            stop_record = VehicleStopRecord(
                vehicle_id=vehicle_id,
                conflict_id=conflict_id,
                stop_time=time.time(),
                stop_reason=reason,
                min_stop_duration=min_stop_duration
            )
            
            with self.lock:
                self.stopped_vehicles[vehicle_id] = stop_record
                self.stats['vehicles_stopped'] += 1
            
            print(f"🛑 车辆 {vehicle_id} 已停车 (冲突: {conflict_id}, 最小停车时间: {min_stop_duration:.1f}s)")
            return True
            
        except Exception as e:
            print(f"❌ 停车车辆 {vehicle_id} 失败: {e}")
            return False
    
    def recover_vehicle(self, vehicle_id: str, force: bool = False) -> bool:
        """恢复车辆通行"""
        if not self.vehicle_scheduler:
            return False
        
        try:
            vehicle_state = self.vehicle_scheduler.vehicle_states.get(vehicle_id)
            if not vehicle_state:
                return False
            
            # 检查是否在停车列表中
            if vehicle_id not in self.stopped_vehicles:
                return False
            
            stop_record = self.stopped_vehicles[vehicle_id]
            
            # 如果不是强制恢复，检查冲突状态
            if not force:
                if self._has_blocking_conflicts(vehicle_id):
                    print(f"    车辆 {vehicle_id} 仍有冲突，不能恢复")
                    return False
            
            # 恢复车辆状态
            vehicle_state.passing_status = 0  # 恢复通行状态
            vehicle_state.current_status = VehicleStatus.MOVING
            vehicle_state.current_speed = vehicle_state.max_speed
            
            # 恢复任务
            if vehicle_state.current_task_id:
                task = self.vehicle_scheduler.tasks.get(vehicle_state.current_task_id)
                if task and task.status == TaskStatus.SUSPENDED:
                    task.status = TaskStatus.IN_PROGRESS
            
            # 清除停车记录
            with self.lock:
                del self.stopped_vehicles[vehicle_id]
                self.stats['vehicles_recovered'] += 1
                if force:
                    self.stats['force_recoveries'] += 1
                else:
                    self.stats['auto_recoveries'] += 1
            
            stop_duration = time.time() - stop_record.stop_time
            recovery_type = "强制" if force else "自动"
            print(f"🚀 车辆 {vehicle_id} {recovery_type}恢复通行 (停车 {stop_duration:.1f}s)")
            return True
            
        except Exception as e:
            print(f"❌ 恢复车辆 {vehicle_id} 失败: {e}")
            return False
    
    def check_and_recover_vehicles(self) -> int:
        """检查并恢复被停车辆（分批恢复避免新冲突）"""
        if not self.config['enable_auto_recovery']:
            return 0
        
        current_time = time.time()
        
        if current_time - self.last_recovery_check < self.recovery_check_interval:
            return 0
        
        recovered_count = 0
        
        with self.lock:
            if not self.stopped_vehicles:
                self.last_recovery_check = current_time
                return 0
            
            print(f"\n🔄 [分批恢复检查] 被停车辆: {len(self.stopped_vehicles)} 个")
            
            # === 分批恢复：一次只恢复一个车辆 ===
            candidates = []  # 候选恢复车辆
            
            for vehicle_id, stop_record in self.stopped_vehicles.items():
                stop_duration = current_time - stop_record.stop_time
                min_required = stop_record.min_stop_duration
                
                print(f"  检查车辆 {vehicle_id} (停车 {stop_duration:.1f}s / 最少 {min_required:.1f}s)")
                
                # 必须满足最小停车时间
                if stop_duration < min_required:
                    remaining_time = min_required - stop_duration
                    print(f"    ⏳ 未满最小停车时间，还需等待 {remaining_time:.1f}s")
                    continue
                
                # 检查原冲突是否已解决
                if self._is_conflict_resolved(stop_record.conflict_id):
                    print(f"    ✅ 原冲突已解决且已满停车时间")
                    candidates.append((vehicle_id, False, stop_record.stop_time))  # 按停车时间排序
                elif not self._has_blocking_conflicts(vehicle_id):
                    print(f"    ✅ 无阻塞冲突且已满停车时间") 
                    candidates.append((vehicle_id, False, stop_record.stop_time))
                elif stop_duration > self.max_stop_duration:
                    print(f"    🚨 强制恢复候选 (超过最大时间)")
                    candidates.append((vehicle_id, True, stop_record.stop_time))
                else:
                    print(f"    ⏳ 继续等待（仍有冲突）")
            
            # === 关键修改：按停车时间排序，优先恢复停车最久的 ===
            if candidates:
                candidates.sort(key=lambda x: x[2])  # 按停车时间排序
                
                # 一次只恢复一个车辆
                vehicle_id, force, _ = candidates[0]
                
                print(f"  🎯 选择恢复车辆: {vehicle_id} ({'强制' if force else '正常'})")
                
                if self.recover_vehicle(vehicle_id, force):
                    recovered_count = 1
                    
                    # === 恢复后立即重新检测冲突 ===
                    self._recheck_conflicts_after_recovery()
                    
        self.last_recovery_check = current_time
        
        if recovered_count > 0:
            print(f"🎉 分批恢复完成: {recovered_count} 个车辆恢复通行")
        
    def _recheck_conflicts_after_recovery(self):
        """恢复车辆后重新检测冲突"""
        try:
            print(f"    🔍 恢复后重新检测冲突...")
            
            # 检测当前所有冲突
            new_conflicts = self.conflict_detector.detect_backbone_conflicts()
            
            if new_conflicts:
                print(f"    ⚠️ 发现新冲突: {len(new_conflicts)} 个")
                
                # 处理新检测到的冲突
                for conflict in new_conflicts:
                    print(f"      新冲突: {conflict.conflict_id} 涉及车辆 {conflict.conflicting_vehicles}")
                    
                    # 如果冲突涉及刚恢复的车辆和其他停车车辆，需要重新停车
                    conflicted_stopped_vehicles = []
                    for vehicle_id in conflict.conflicting_vehicles:
                        if vehicle_id in self.stopped_vehicles:
                            conflicted_stopped_vehicles.append(vehicle_id)
                    
                    if conflicted_stopped_vehicles:
                        print(f"      影响已停车辆: {conflicted_stopped_vehicles}")
                        # 延长这些车辆的停车时间，避免立即恢复
                        for vehicle_id in conflicted_stopped_vehicles:
                            stop_record = self.stopped_vehicles[vehicle_id]
                            # 重置停车时间，延长停车
                            stop_record.stop_time = time.time()
                            stop_record.min_stop_duration += 15.0  # 额外增加15秒
                            print(f"        延长车辆 {vehicle_id} 停车时间 +15s")
                
                # 尝试解决新冲突
                self.process_conflicts(new_conflicts)
            else:
                print(f"    ✅ 无新冲突")
                
        except Exception as e:
            print(f"    ❌ 重新检测冲突失败: {e}")
    
    def _is_conflict_resolved(self, conflict_id: str) -> bool:
        """检查冲突是否已解决"""
        try:
            active_conflicts = self.conflict_detector.get_active_conflicts()
            for conflict in active_conflicts:
                if conflict.conflict_id == conflict_id:
                    return False
            return True
        except:
            return True
    
    def _has_blocking_conflicts(self, vehicle_id: str) -> bool:
        """检查车辆是否有阻塞冲突"""
        try:
            vehicle_conflicts = self.conflict_detector.get_vehicle_conflicts(vehicle_id)
            return len(vehicle_conflicts) > 0
        except:
            return False
    
    def _fix_priority_order(self, conflict: BackboneConflict):
        """修复优先级顺序重复问题"""
        if not hasattr(conflict, 'priority_order') or not conflict.priority_order:
            return
        
        # 去重并保持顺序
        seen = set()
        fixed_order = []
        
        for vehicle_id in conflict.priority_order:
            if vehicle_id not in seen:
                fixed_order.append(vehicle_id)
                seen.add(vehicle_id)
        
        conflict.priority_order = fixed_order
        print(f"🔧 修复优先级顺序: {len(conflict.priority_order)} 个唯一车辆")
    
    # === 修改：增强的紧急停车策略 ===
    
    def _resolve_emergency_stop_with_recovery(self, conflict: BackboneConflict) -> bool:
        """增强的紧急停车策略（集成恢复）"""
        print(f"      🚨 执行增强紧急停车策略...")
        
        if not self.vehicle_scheduler:
            return False
        
        # 修复优先级顺序
        self._fix_priority_order(conflict)
        
        if not conflict.priority_order or len(conflict.priority_order) < 2:
            print(f"        ❌ 优先级顺序无效")
            return False
        
        print(f"        📋 修复后优先级顺序: {conflict.priority_order}")
        
        # 确定通行和停车车辆
        first_vehicle = conflict.priority_order[0]  # 第一优先级
        vehicles_to_stop = [v for v in conflict.priority_order[1:] if v != first_vehicle]
        
        print(f"        🚦 先到先行: {first_vehicle} 继续通行")
        print(f"        🛑 需要停车: {vehicles_to_stop}")
        
        if not vehicles_to_stop:
            print(f"        ✅ 无需停车，先行车辆已确定")
            return True
        
        success_count = 0
        
        # 确保先行车辆处于通行状态
        if first_vehicle in self.vehicle_scheduler.vehicle_states:
            first_vehicle_state = self.vehicle_scheduler.vehicle_states[first_vehicle]
            if not hasattr(first_vehicle_state, 'passing_status'):
                first_vehicle_state.passing_status = 0
            
            if first_vehicle_state.passing_status == 1:  # 如果先行车辆被停车了
                first_vehicle_state.passing_status = 0  # 恢复通行
                first_vehicle_state.current_status = VehicleStatus.MOVING
                first_vehicle_state.current_speed = first_vehicle_state.max_speed
                print(f"        🚀 恢复先行车辆 {first_vehicle} 通行状态")
        
        # 停车其他车辆
        for vehicle_id in vehicles_to_stop:
            if self.stop_vehicle(vehicle_id, conflict.conflict_id, "emergency_stop_priority", 
                               conflict_vehicle_count=len(conflict.conflicting_vehicles)):
                success_count += 1
        
        # 更新统计
        if success_count > 0:
            self.stats['emergency_stops'] += 1
        
        print(f"        📊 停车结果: {success_count}/{len(vehicles_to_stop)} 成功")
        return success_count > 0
    
    # === 其他冲突解决策略实现 ===
    
    def _resolve_priority_preemption(self, conflict: BackboneConflict) -> bool:
        """优先级抢占解决策略"""
        print(f"      ⚖️ 执行优先级抢占策略...")
        
        if not self.vehicle_scheduler or not conflict.priority_order:
            return False
        
        self._fix_priority_order(conflict)
        
        # 获取最高优先级车辆
        priority_vehicle = conflict.priority_order[0]
        preempted_vehicles = conflict.priority_order[1:]
        
        print(f"        优先车辆: {priority_vehicle}")
        print(f"        被抢占车辆: {preempted_vehicles}")
        
        # 处理被抢占的车辆
        success_count = 0
        
        for vehicle_id in preempted_vehicles:
            # 获取车辆优先级
            vehicle_priority = self.vehicle_scheduler.get_vehicle_priority(vehicle_id)
            priority_vehicle_priority = self.vehicle_scheduler.get_vehicle_priority(priority_vehicle)
            
            priority_diff = priority_vehicle_priority - vehicle_priority
            
            if priority_diff >= self.config['priority_override_threshold']:
                # 优先级差异足够大，强制延迟
                delay_time = self.config['temporal_adjustment_step'] * priority_diff
                
                success = self.vehicle_scheduler.delay_vehicle_backbone_timing(
                    vehicle_id, conflict.backbone_path_id, delay_time
                )
            else:
                # 优先级差异不大，尝试重新路由
                success = self.vehicle_scheduler.try_alternative_interface_node(
                    vehicle_id, conflict.backbone_path_id
                )
            
            if success:
                success_count += 1
                print(f"        ✅ 车辆 {vehicle_id} 处理成功")
            else:
                print(f"        ❌ 车辆 {vehicle_id} 处理失败")
        
        # 更新统计
        if success_count > 0:
            self.stats['priority_resolutions'] += 1
        
        return success_count == len(preempted_vehicles)
    
    def _resolve_temporal_adjustment(self, conflict: BackboneConflict) -> bool:
        """时间调整解决策略"""
        print(f"      ⏰ 执行时间调整策略...")
        
        if not self.vehicle_scheduler or not conflict.priority_order:
            return False
        
        self._fix_priority_order(conflict)
        
        # 选择优先级较低的车辆进行时间调整
        vehicles_by_priority = conflict.priority_order.copy()
        vehicles_by_priority.reverse()  # 从低优先级开始
        
        success_count = 0
        adjustment_step = self.config['temporal_adjustment_step']
        
        # 尝试调整每个车辆的时间
        for i, vehicle_id in enumerate(vehicles_by_priority[:-1]):  # 保留最高优先级车辆不调整
            # 计算调整时间
            delay_time = adjustment_step * (i + 1)
            
            if delay_time > self.config['max_temporal_delay']:
                print(f"        ⚠️ 车辆 {vehicle_id} 延迟时间超过限制")
                break
            
            success = self.vehicle_scheduler.delay_vehicle_backbone_timing(
                vehicle_id, conflict.backbone_path_id, delay_time
            )
            
            if success:
                success_count += 1
                print(f"        ✅ 车辆 {vehicle_id} 时间调整 +{delay_time:.1f}s")
            else:
                print(f"        ❌ 车辆 {vehicle_id} 时间调整失败")
        
        # 更新统计
        if success_count > 0:
            self.stats['temporal_adjustments'] += 1
        
        return success_count > 0
    
    def _resolve_alternative_backbone(self, conflict: BackboneConflict) -> bool:
        """备选骨干路径解决策略"""
        print(f"      🛤️ 执行备选骨干路径策略...")
        
        if not self.vehicle_scheduler or not self.backbone_network:
            return False
        
        self._fix_priority_order(conflict)
        
        # 选择优先级较低的车辆切换骨干路径
        vehicles_by_priority = conflict.priority_order.copy()
        vehicles_by_priority.reverse()  # 从低优先级开始
        
        success_count = 0
        
        for vehicle_id in vehicles_by_priority:
            # 获取车辆目标信息
            target_info = self.vehicle_scheduler.get_vehicle_target_info(vehicle_id)
            if not target_info:
                continue
            
            # 查找备选骨干路径
            alternative_paths = self.backbone_network.find_alternative_backbone_paths(
                target_info['target_type'], 
                target_info['target_id'],
                exclude_path_id=conflict.backbone_path_id
            )
            
            if not alternative_paths:
                print(f"        ⚠️ 车辆 {vehicle_id} 无备选骨干路径")
                continue
            
            # 选择负载最低的备选路径
            best_alternative = min(alternative_paths, key=lambda p: p.get_load_factor())
            
            # 检查负载是否可接受
            if best_alternative.get_load_factor() > 0.8:
                print(f"        ⚠️ 最佳备选路径负载过高: {best_alternative.get_load_factor():.1%}")
                continue
            
            # 尝试切换
            success = self.vehicle_scheduler.switch_vehicle_backbone_path(
                vehicle_id, best_alternative.path_id
            )
            
            if success:
                success_count += 1
                print(f"        ✅ 车辆 {vehicle_id} 切换到路径 {best_alternative.path_id}")
                break  # 只需要一个车辆成功切换即可解决冲突
            else:
                print(f"        ❌ 车辆 {vehicle_id} 路径切换失败")
        
        # 更新统计
        if success_count > 0:
            self.stats['backbone_switches'] += 1
        
        return success_count > 0
    
    def _resolve_alternative_interface(self, conflict: BackboneConflict) -> bool:
        """备选接入点解决策略"""
        print(f"      🔗 执行备选接入点策略...")
        
        if not self.vehicle_scheduler:
            return False
        
        self._fix_priority_order(conflict)
        
        # 选择优先级较低的车辆尝试不同接入点
        vehicles_by_priority = conflict.priority_order.copy()
        vehicles_by_priority.reverse()  # 从低优先级开始
        
        success_count = 0
        
        for vehicle_id in vehicles_by_priority:
            success = self.vehicle_scheduler.try_alternative_interface_node(
                vehicle_id, conflict.backbone_path_id
            )
            
            if success:
                success_count += 1
                print(f"        ✅ 车辆 {vehicle_id} 接入点切换成功")
                break  # 只需要一个车辆成功即可
            else:
                print(f"        ❌ 车辆 {vehicle_id} 接入点切换失败")
        
        # 更新统计
        if success_count > 0:
            self.stats['interface_switches'] += 1
        
        return success_count > 0

    # === 原有冲突处理方法保持不变 ===
    
    def process_conflicts(self, conflicts: List[BackboneConflict]) -> Dict[str, ResolutionResult]:
        """批量处理冲突"""
        with self.lock:
            results = {}
            
            if not conflicts:
                return results
            
            # 按严重程度和优先级排序冲突
            sorted_conflicts = self._prioritize_conflicts(conflicts)
            
            print(f"\n🚦 开始处理 {len(conflicts)} 个冲突")
            
            for conflict in sorted_conflicts:
                if conflict.conflict_id not in results:
                    result = self.resolve_backbone_conflict(conflict)
                    results[conflict.conflict_id] = result
                    
                    # 更新统计
                    self.stats['total_conflicts_processed'] += 1
                    if result == ResolutionResult.SUCCESS:
                        self.stats['conflicts_resolved'] += 1
            
            # 更新成功率
            if self.stats['total_conflicts_processed'] > 0:
                self.stats['resolution_success_rate'] = (
                    self.stats['conflicts_resolved'] / self.stats['total_conflicts_processed']
                )
            
            print(f"🎯 冲突处理完成，成功率: {self.stats['resolution_success_rate']:.1%}")
            return results
    
    def resolve_backbone_conflict(self, conflict: BackboneConflict) -> ResolutionResult:
        """解决单个骨干路径冲突"""
        print(f"\n🔧 解决冲突: {conflict.conflict_id}")
        print(f"   类型: {conflict.conflict_type.value}")
        print(f"   严重程度: {conflict.severity.value}")
        print(f"   涉及车辆: {conflict.conflicting_vehicles}")
        print(f"   优先级顺序: {conflict.priority_order}")
        print(f"   建议策略: {conflict.suggested_resolution.value}")
        
        resolution_start = time.time()
        
        # 执行解决策略
        strategy_func = self.resolution_strategies.get(conflict.suggested_resolution)
        if not strategy_func:
            print(f"   ❌ 未知解决策略: {conflict.suggested_resolution}")
            return ResolutionResult.FAILURE
        
        # 尝试解决
        for attempt in range(self.config['max_resolution_attempts']):
            print(f"   🔄 第 {attempt + 1} 次尝试...")
            
            try:
                success = strategy_func(conflict)
                
                # 记录尝试
                attempt_record = ResolutionAttempt(
                    conflict_id=conflict.conflict_id,
                    strategy=conflict.suggested_resolution,
                    attempt_time=time.time(),
                    result=ResolutionResult.SUCCESS if success else ResolutionResult.FAILURE,
                    vehicles_affected=conflict.conflicting_vehicles.copy()
                )
                
                self.resolution_history.append(attempt_record)
                
                if success:
                    resolution_time = time.time() - resolution_start
                    
                    # 更新统计
                    self._update_resolution_stats(conflict.suggested_resolution, resolution_time)
                    
                    # 标记冲突已解决
                    self.conflict_detector.mark_conflict_resolved(
                        conflict.conflict_id, 
                        conflict.suggested_resolution.value
                    )
                    
                    print(f"   ✅ 冲突解决成功，耗时: {resolution_time:.2f}s")
                    return ResolutionResult.SUCCESS
                else:
                    print(f"   ❌ 第 {attempt + 1} 次尝试失败")
                    
                    # 如果不是最后一次尝试，尝试备选策略
                    if attempt < self.config['max_resolution_attempts'] - 1:
                        conflict.suggested_resolution = self._get_alternative_strategy(
                            conflict, conflict.suggested_resolution
                        )
                        strategy_func = self.resolution_strategies.get(conflict.suggested_resolution)
                        print(f"   🔄 切换到备选策略: {conflict.suggested_resolution.value}")
            
            except Exception as e:
                print(f"   ⚠️ 解决尝试异常: {e}")
                continue
        
        print(f"   ❌ 冲突解决失败，已达到最大尝试次数")
        return ResolutionResult.FAILURE
    
    def update(self, time_delta: float):
        """更新交通管理器（新增恢复检查）"""
        current_time = time.time()
        
        # 检测冲突
        conflicts = self.conflict_detector.detect_backbone_conflicts()
        
        if conflicts:
            # 处理新冲突
            self.process_conflicts(conflicts)
        
        # === 新增：恢复检查 ===
        self.check_and_recover_vehicles()
        
        # 定期清理
        if current_time - self.performance_monitor['last_cleanup_time'] > self.config['cleanup_interval']:
            self._cleanup_resolution_history()
            self.performance_monitor['last_cleanup_time'] = current_time
    
    def get_system_status(self) -> Dict:
        """获取系统状态（新增恢复信息）"""
        with self.lock:
            # 计算策略成功率
            strategy_success_rates = {}
            for strategy, stats in self.stats['strategy_success_rates'].items():
                if stats['total'] > 0:
                    strategy_success_rates[strategy] = stats['success'] / stats['total']
                else:
                    strategy_success_rates[strategy] = 0.0
            
            return {
                'total_conflicts_processed': self.stats['total_conflicts_processed'],
                'conflicts_resolved': self.stats['conflicts_resolved'],
                'resolution_success_rate': self.stats['resolution_success_rate'],
                'average_resolution_time': self.stats['average_resolution_time'],
                'active_conflicts': len(self.conflict_detector.get_active_conflicts()),
                'strategy_usage': dict(self.stats['strategy_usage']),
                'strategy_success_rates': strategy_success_rates,
                'performance_metrics': {
                    'first_come_resolutions': self.stats['first_come_resolutions'],
                    'priority_resolutions': self.stats['priority_resolutions'],
                    'backbone_switches': self.stats['backbone_switches'],
                    'interface_switches': self.stats['interface_switches'],
                    'temporal_adjustments': self.stats['temporal_adjustments'],
                    'emergency_stops': self.stats['emergency_stops']
                },
                # === 新增：恢复状态信息 ===
                'recovery_status': {
                    'stopped_vehicles_count': len(self.stopped_vehicles),
                    'total_vehicles_stopped': self.stats['vehicles_stopped'],
                    'total_vehicles_recovered': self.stats['vehicles_recovered'],
                    'auto_recoveries': self.stats['auto_recoveries'],
                    'force_recoveries': self.stats['force_recoveries'],
                    'stopped_vehicles': {
                        vehicle_id: {
                            'conflict_id': record.conflict_id,
                            'stop_duration': time.time() - record.stop_time,
                            'stop_reason': record.stop_reason,
                            'recovery_attempts': record.recovery_attempts
                        }
                        for vehicle_id, record in self.stopped_vehicles.items()
                    }
                }
            }
    
    # === 手动控制接口 ===
    
    def manual_recover_vehicle(self, vehicle_id: str) -> bool:
        """手动恢复指定车辆"""
        return self.recover_vehicle(vehicle_id, force=True)
    
    def manual_recover_all_vehicles(self) -> int:
        """手动恢复所有停车车辆"""
        stopped_vehicles = list(self.stopped_vehicles.keys())
        recovered_count = 0
        
        for vehicle_id in stopped_vehicles:
            if self.recover_vehicle(vehicle_id, force=True):
                recovered_count += 1
        
        print(f"🚀 手动恢复完成: {recovered_count}/{len(stopped_vehicles)} 个车辆")
        return recovered_count
    
    def get_stopped_vehicles_info(self) -> Dict:
        """获取停车车辆信息"""
        with self.lock:
            current_time = time.time()
            return {
                vehicle_id: {
                    'conflict_id': record.conflict_id,
                    'stop_duration': current_time - record.stop_time,
                    'stop_reason': record.stop_reason,
                    'recovery_attempts': record.recovery_attempts
                }
                for vehicle_id, record in self.stopped_vehicles.items()
            }
    
    # === 原有方法保持不变（省略其他代码...） ===
    
    def _resolve_first_come_first_serve(self, conflict: BackboneConflict) -> bool:
        """先到先行解决策略"""
        print(f"      📅 执行先到先行策略...")
        
        if not self.vehicle_scheduler or not conflict.priority_order:
            return False
        
        self._fix_priority_order(conflict)
        
        if len(conflict.priority_order) < 2:
            return False
        
        priority_vehicle = conflict.priority_order[0]
        delayed_vehicles = conflict.priority_order[1:]
        
        print(f"        🚦 先到优先: {priority_vehicle}")
        print(f"        ⏳ 需要延迟: {delayed_vehicles}")
        
        success_count = 0
        base_delay = self.config['temporal_adjustment_step']
        
        for i, vehicle_id in enumerate(delayed_vehicles):
            delay_time = base_delay * (i + 1)
            
            success = self.vehicle_scheduler.delay_vehicle_backbone_timing(
                vehicle_id, conflict.backbone_path_id, delay_time
            )
            
            if success:
                success_count += 1
                print(f"        ✅ 车辆 {vehicle_id} 延迟 {delay_time:.1f}s")
            else:
                print(f"        ❌ 车辆 {vehicle_id} 延迟失败")
        
        if success_count > 0:
            self.stats['first_come_resolutions'] += 1
        
        return success_count > 0
    
    def _prioritize_conflicts(self, conflicts: List[BackboneConflict]) -> List[BackboneConflict]:
        """优先级排序冲突"""
        def conflict_priority(conflict):
            severity_score = conflict.severity.value * 10
            vehicle_count_score = len(conflict.conflicting_vehicles) * 2
            
            current_time = time.time()
            time_to_conflict = conflict.conflict_time_window[0] - current_time
            urgency_score = max(0, 20 - time_to_conflict / 10)
            
            return severity_score + vehicle_count_score + urgency_score
        
        return sorted(conflicts, key=conflict_priority, reverse=True)
    
    def _get_alternative_strategy(self, conflict: BackboneConflict, 
                                failed_strategy: ResolutionStrategy) -> ResolutionStrategy:
        """获取备选解决策略（修改版：停车作为最后手段）"""
        fallback_strategies = {
            ResolutionStrategy.FIRST_COME_FIRST_SERVE: ResolutionStrategy.TEMPORAL_ADJUSTMENT,
            ResolutionStrategy.PRIORITY_PREEMPTION: ResolutionStrategy.TEMPORAL_ADJUSTMENT,
            ResolutionStrategy.TEMPORAL_ADJUSTMENT: ResolutionStrategy.ALTERNATIVE_INTERFACE,
            ResolutionStrategy.ALTERNATIVE_INTERFACE: ResolutionStrategy.ALTERNATIVE_BACKBONE_PATH,
            ResolutionStrategy.ALTERNATIVE_BACKBONE_PATH: ResolutionStrategy.EMERGENCY_STOP,
            ResolutionStrategy.EMERGENCY_STOP: ResolutionStrategy.EMERGENCY_STOP
        }
        
        next_strategy = fallback_strategies.get(failed_strategy, ResolutionStrategy.EMERGENCY_STOP)
        
        print(f"        🔄 策略降级: {failed_strategy.value} -> {next_strategy.value}")
        return next_strategy
    
    def _update_resolution_stats(self, strategy: ResolutionStrategy, resolution_time: float):
        """更新解决统计"""
        self.stats['strategy_usage'][strategy.value] += 1
        
        strategy_stats = self.stats['strategy_success_rates'][strategy.value]
        strategy_stats['success'] += 1
        strategy_stats['total'] += 1
        
        self.performance_monitor['resolution_times'].append(resolution_time)
        self.performance_monitor['successful_strategies'].append(strategy.value)
        
        resolution_times = list(self.performance_monitor['resolution_times'])
        if resolution_times:
            self.stats['average_resolution_time'] = sum(resolution_times) / len(resolution_times)
    
    def _cleanup_resolution_history(self):
        """清理解决历史"""
        current_time = time.time()
        cleanup_threshold = current_time - self.config['cleanup_interval']
        
        self.resolution_history = [
            attempt for attempt in self.resolution_history
            if attempt.attempt_time > cleanup_threshold
        ]
        
        if len(self.resolution_history) > 500:
            self.resolution_history = self.resolution_history[-250:]
    
    def emergency_clear_all_conflicts(self):
        """紧急清除所有冲突"""
        conflicts = self.conflict_detector.get_active_conflicts()
        
        print(f"🚨 紧急清除所有冲突: {len(conflicts)} 个")
        
        for conflict in conflicts:
            conflict.suggested_resolution = ResolutionStrategy.EMERGENCY_STOP
            self.resolve_backbone_conflict(conflict)
        
        print("🚨 紧急冲突清除完成")
    
    def shutdown(self):
        """关闭交通管理器"""
        with self.lock:
            self.resolution_history.clear()
            self.active_resolutions.clear()
            self.stopped_vehicles.clear()
        
        print("增强版骨干网络交通管理器已关闭")