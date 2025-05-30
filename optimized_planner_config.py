"""
optimized_planner_config.py - 优化的路径规划器配置
解决混合A*参数过于严格的问题，提供渐进式参数调整策略
"""

import math
import threading
from typing import Dict, Any

class OptimizedPlannerConfig:
    """优化的规划器配置类"""
    
    def __init__(self):
        # 基础配置
        self.base_config = {
            'max_planning_time': 20.0,
            'quality_threshold': 0.6,
            'cache_size': 200,
            'enable_fallback': True,
            'timeout_retry_count': 3
        }
        
        # 混合A*配置 - 分层参数策略
        self.astar_configs = {
            'strict': {
                'vehicle_length': 6.0,
                'vehicle_width': 3.0,
                'turning_radius': 8.0,
                'step_size': 2.0,
                'angle_resolution': 30,
                'max_iterations': 20000,
                'rs_fitting_radius': 20.0,
                'quality_threshold': 0.7,
                'timeout': 15.0
            },
            'standard': {
                'vehicle_length': 6.0,
                'vehicle_width': 3.0,
                'turning_radius': 7.0,
                'step_size': 2.5,
                'angle_resolution': 36,
                'max_iterations': 20000,
                'rs_fitting_radius': 25.0,
                'quality_threshold': 0.5,
                'timeout': 18.0
            },
            'relaxed': {
                'vehicle_length': 6.0,
                'vehicle_width': 3.0,
                'turning_radius': 6.0,
                'step_size': 3.0,
                'angle_resolution': 45,
                'max_iterations': 20000,
                'rs_fitting_radius': 30.0,
                'quality_threshold': 0.4,
                'timeout': 20.0
            }
        }
        
        # RRT配置 - 分层参数策略
        self.rrt_configs = {
            'standard': {
                'vehicle_length': 5.0,
                'vehicle_width': 2.0,
                'turning_radius': 5.0,
                'step_size': 0.8,
                'max_nodes': 6000,
                'goal_bias': 0.15,
                'quality_threshold': 0.5,
                'enable_dynamics_optimization': True
            },
            'fast': {
                'vehicle_length': 5.0,
                'vehicle_width': 2.0,
                'turning_radius': 4.0,
                'step_size': 1.0,
                'max_nodes': 4000,
                'goal_bias': 0.20,
                'quality_threshold': 0.4,
                'enable_dynamics_optimization': False
            }
        }
        
        # 渐进式回退策略
        self.fallback_strategies = [
            {
                'name': 'astar_strict',
                'planner': 'hybrid_astar',
                'config': 'strict',
                'max_time': 12.0,
                'priority': 1
            },
            {
                'name': 'astar_standard',
                'planner': 'hybrid_astar',
                'config': 'standard',
                'max_time': 15.0,
                'priority': 2
            },
            {
                'name': 'rrt_standard',
                'planner': 'rrt',
                'config': 'standard',
                'max_time': 12.0,
                'priority': 3
            },
            {
                'name': 'astar_relaxed',
                'planner': 'hybrid_astar',
                'config': 'relaxed',
                'max_time': 18.0,
                'priority': 4
            },
            {
                'name': 'rrt_fast',
                'planner': 'rrt',
                'config': 'fast',
                'max_time': 10.0,
                'priority': 5
            },
            {
                'name': 'direct_fallback',
                'planner': 'direct',
                'config': None,
                'max_time': 1.0,
                'priority': 6
            }
        ]
    
    def get_astar_config(self, level: str = 'standard') -> Dict[str, Any]:
        """获取混合A*配置"""
        return self.astar_configs.get(level, self.astar_configs['standard']).copy()
    
    def get_rrt_config(self, level: str = 'standard') -> Dict[str, Any]:
        """获取RRT配置"""
        return self.rrt_configs.get(level, self.rrt_configs['standard']).copy()
    
    def get_context_optimized_config(self, context: str, planner_type: str) -> Dict[str, Any]:
        """根据上下文获取优化配置"""
        if context == 'backbone':
            # 骨干路径生成：要求高质量，允许更多时间
            if planner_type == 'hybrid_astar':
                config = self.get_astar_config('strict')
                config.update({
                    'max_iterations': 8000,
                    'timeout': 20.0,
                    'quality_threshold': 0.6
                })
                return config
            elif planner_type == 'rrt':
                config = self.get_rrt_config('standard')
                config.update({
                    'max_nodes': 4000,
                    'enable_dynamics_optimization': True
                })
                return config
        
        elif context == 'navigation':
            # 导航路径：平衡质量和速度
            if planner_type == 'hybrid_astar':
                config = self.get_astar_config('standard')
                config.update({
                    'timeout': 12.0,
                    'quality_threshold': 0.5
                })
                return config
            elif planner_type == 'rrt':
                config = self.get_rrt_config('fast')
                return config
        
        elif context == 'emergency':
            # 紧急情况：优先速度
            if planner_type == 'hybrid_astar':
                config = self.get_astar_config('relaxed')
                config.update({
                    'timeout': 8.0,
                    'quality_threshold': 0.3
                })
                return config
            elif planner_type == 'rrt':
                config = self.get_rrt_config('fast')
                config.update({
                    'max_nodes': 1500,
                    'enable_dynamics_optimization': False
                })
                return config
        
        # 默认配置
        if planner_type == 'hybrid_astar':
            return self.get_astar_config('standard')
        elif planner_type == 'rrt':
            return self.get_rrt_config('standard')
        
        return {}
    
    def get_progressive_fallback_sequence(self, context: str = 'normal') -> list:
        """获取渐进式回退序列"""
        if context == 'backbone':
            # 骨干路径：更保守的回退
            return [
                self.fallback_strategies[0],  # astar_strict
                self.fallback_strategies[1],  # astar_standard
                self.fallback_strategies[3],  # astar_relaxed
                self.fallback_strategies[2],  # rrt_standard
                self.fallback_strategies[5]   # direct_fallback
            ]
        
        elif context == 'navigation':
            # 导航路径：平衡的回退
            return [
                self.fallback_strategies[1],  # astar_standard
                self.fallback_strategies[2],  # rrt_standard
                self.fallback_strategies[3],  # astar_relaxed
                self.fallback_strategies[5]   # direct_fallback
            ]
        
        elif context == 'emergency':
            # 紧急情况：快速回退
            return [
                self.fallback_strategies[4],  # rrt_fast
                self.fallback_strategies[3],  # astar_relaxed
                self.fallback_strategies[5]   # direct_fallback
            ]
        
        # 默认回退序列
        return self.fallback_strategies

class EnhancedPathPlannerWithConfig:
    """增强的路径规划器（集成优化配置）"""
    
    def __init__(self, env, backbone_network=None, traffic_manager=None):
        self.env = env
        self.backbone_network = backbone_network
        self.traffic_manager = traffic_manager
        
        # 加载优化配置
        self.config_manager = OptimizedPlannerConfig()
        
        # 规划器实例
        self.planners = {}
        self._initialize_planners_with_config()
        
        # 简化缓存（向后兼容）
        self.cache = {}
        self.cache_lock = threading.RLock() if 'threading' in globals() else None
        
        # 性能统计
        self.stats = {
            'strategy_success_rates': {},
            'average_planning_times': {},
            'fallback_usage': {},
            'total_requests': 0,
            'successful_plans': 0,
            'cache_hits': 0
        }
        
        print("初始化增强路径规划器（集成优化配置）")
    
    def _initialize_planners_with_config(self):
        """使用优化配置初始化规划器"""
        try:
            # 初始化混合A*（使用标准配置）
            from astar import HybridAStarPlanner
            astar_config = self.config_manager.get_astar_config('standard')
            
            self.planners['hybrid_astar'] = HybridAStarPlanner(
                self.env,
                vehicle_length=astar_config['vehicle_length'],
                vehicle_width=astar_config['vehicle_width'],
                turning_radius=astar_config['turning_radius'],
                step_size=astar_config['step_size'],
                angle_resolution=astar_config['angle_resolution']
            )
            print("✅ 混合A*规划器初始化完成")
        
        except ImportError:
            print("⚠️ 混合A*规划器不可用")
        
        try:
            # 初始化RRT（使用标准配置）
            from RRT import SimplifiedRRTPlanner
            rrt_config = self.config_manager.get_rrt_config('standard')
            
            self.planners['rrt'] = SimplifiedRRTPlanner(
                self.env,
                vehicle_length=rrt_config['vehicle_length'],
                vehicle_width=rrt_config['vehicle_width'],
                turning_radius=rrt_config['turning_radius'],
                step_size=rrt_config['step_size']
            )
            print("✅ RRT规划器初始化完成")
        
        except ImportError:
            print("⚠️ RRT规划器不可用")
    
    def plan_path(self, vehicle_id: str, start: tuple, goal: tuple,
                use_backbone: bool = True, check_conflicts: bool = True,
                planner_type: str = "auto", context: str = "normal",
                return_object: bool = False, **kwargs) -> Any:
        """
        标准路径规划接口（向后兼容）
        内部使用渐进式回退策略
        """
        
        # ===== 新增：骨干网络优先逻辑 =====
        if use_backbone and self.backbone_network and context != 'backbone':
            print(f"[EnhancedPathPlannerWithConfig] 尝试使用骨干网络: {vehicle_id}")
            
            # 尝试从kwargs或推断目标信息
            target_type = kwargs.get('target_type')
            target_id = kwargs.get('target_id')
            
            if target_type and target_id is not None:
                try:
                    backbone_result = self.backbone_network.get_path_from_position_to_target(
                        start, target_type, target_id, vehicle_id
                    )
                    
                    if backbone_result:
                        print(f"  ✅ 骨干网络路径成功!")
                        
                        # 处理返回结果
                        if isinstance(backbone_result, tuple) and len(backbone_result) >= 2:
                            path, structure = backbone_result
                            
                            # 创建兼容的结果对象
                            result = self._create_result_object(
                                path, 'backbone_network', len(path)
                            )
                            result.structure = structure
                            
                            if return_object:
                                return result
                            else:
                                return path
                        else:
                            return backbone_result
                except Exception as e:
                    print(f"  骨干网络尝试失败: {e}")
        
        # ===== 原有逻辑：渐进式回退 =====
        result = self.plan_path_with_progressive_fallback(
            vehicle_id, start, goal, context, **kwargs
        )
        
        # 根据return_object参数返回不同格式
        if return_object and result:
            return result
        elif result and hasattr(result, 'path'):
            return result.path
        else:
            return result
    

    def plan_path_with_progressive_fallback(self, vehicle_id: str, start: tuple, goal: tuple,
                                          context: str = 'normal', **kwargs) -> Any:
        """
        使用渐进式回退策略的路径规划
        """
        import time
        
        self.stats['total_requests'] += 1
        planning_start = time.time()
        fallback_sequence = self.config_manager.get_progressive_fallback_sequence(context)
        

        
        for i, strategy in enumerate(fallback_sequence, 1):
            strategy_name = strategy['name']
            planner_type = strategy['planner']
            max_time = strategy['max_time']
            
            print(f"  [{i}/{len(fallback_sequence)}] 尝试策略: {strategy_name}")
            
            # 获取策略特定的配置
            if strategy['config']:
                if planner_type == 'hybrid_astar':
                    strategy_config = self.config_manager.get_astar_config(strategy['config'])
                elif planner_type == 'rrt':
                    strategy_config = self.config_manager.get_rrt_config(strategy['config'])
                else:
                    strategy_config = {}
            else:
                strategy_config = {}
            
            # 执行规划
            strategy_start = time.time()
            result = self._execute_strategy(
                vehicle_id, start, goal, planner_type, 
                strategy_config, max_time, context
            )
            strategy_time = time.time() - strategy_start
            
            # 更新统计
            if strategy_name not in self.stats['strategy_success_rates']:
                self.stats['strategy_success_rates'][strategy_name] = {'success': 0, 'total': 0}
            
            self.stats['strategy_success_rates'][strategy_name]['total'] += 1
            
            if result:
                self.stats['strategy_success_rates'][strategy_name]['success'] += 1
                self.stats['successful_plans'] += 1
                
                # 记录平均规划时间
                if strategy_name not in self.stats['average_planning_times']:
                    self.stats['average_planning_times'][strategy_name] = []
                self.stats['average_planning_times'][strategy_name].append(strategy_time)
                
                total_time = time.time() - planning_start
                #print(f"    ✅ 策略成功! 耗时: {strategy_time:.2f}s, 总耗时: {total_time:.2f}s")
                
                return result
            else:
                print(f"    ❌ 策略失败, 耗时: {strategy_time:.2f}s")
        
        print("  ❌ 所有回退策略均失败")
        return None
    
    def _execute_strategy(self, vehicle_id: str, start: tuple, goal: tuple,
                         planner_type: str, config: dict, max_time: float, context: str):
        """执行具体的规划策略"""
        try:
            if planner_type == 'hybrid_astar' and 'hybrid_astar' in self.planners:
                return self._plan_with_astar(vehicle_id, start, goal, config, max_time)
            
            elif planner_type == 'rrt' and 'rrt' in self.planners:
                return self._plan_with_rrt(vehicle_id, start, goal, config, max_time)
            
            elif planner_type == 'direct':
                return self._plan_direct_path(start, goal)
            
            return None
        
        except Exception as e:
            print(f"      策略执行异常: {e}")
            return None
    
    def _plan_with_astar(self, vehicle_id: str, start: tuple, goal: tuple, 
                        config: dict, max_time: float):
        """使用混合A*规划"""
        planner = self.planners['hybrid_astar']
        
        # 动态更新配置
        if config:
            if hasattr(planner, 'config'):
                planner.config.update({
                    'max_iterations': config.get('max_iterations', 8000),
                    'timeout': min(max_time, config.get('timeout', 15.0)),
                    'quality_threshold': config.get('quality_threshold', 0.6),
                    'rs_fitting_radius': config.get('rs_fitting_radius', 25.0)
                })
        
        path = planner.plan_path(
            start, goal,
            agent_id=vehicle_id,
            max_iterations=config.get('max_iterations'),
            quality_threshold=config.get('quality_threshold', 0.6)
        )
        
        if path:
            return self._create_result_object(path, 'hybrid_astar', len(path))
        
        return None
    
    def _plan_with_rrt(self, vehicle_id: str, start: tuple, goal: tuple,
                      config: dict, max_time: float):
        """使用RRT规划"""
        planner = self.planners['rrt']
        
        path = planner.plan_path(
            start, goal,
            agent_id=vehicle_id,
            max_iterations=config.get('max_nodes', 3000),
            quality_threshold=config.get('quality_threshold', 0.5),
            enable_dynamics_optimization=config.get('enable_dynamics_optimization', True)
        )
        
        if path:
            return self._create_result_object(path, 'rrt', len(path))
        
        return None
    
    def _plan_direct_path(self, start: tuple, goal: tuple):
        """直线路径回退"""
        import math
        
        # 简单直线路径
        distance = math.sqrt((goal[0] - start[0])**2 + (goal[1] - start[1])**2)
        steps = max(3, int(distance / 2.0))
        
        path = []
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * (goal[0] - start[0])
            y = start[1] + t * (goal[1] - start[1])
            theta = start[2] if len(start) > 2 else 0
            path.append((x, y, theta))
        
        return self._create_result_object(path, 'direct', len(path))
    
    def _create_result_object(self, path: list, planner_used: str, path_length: int):
        """创建结果对象"""
        class PlanningResult:
            def __init__(self, path, planner_used, path_length):
                self.path = path
                self.planner_used = planner_used
                self.quality_score = 0.8 if planner_used != 'direct' else 0.5
                self.planning_time = 0.0  # 将由调用者设置
                self.structure = {
                    'type': planner_used,
                    'total_length': path_length,
                    'planner_used': planner_used,
                    'backbone_utilization': 0.0
                }
        
        return PlanningResult(path, planner_used, path_length)
    
    def get_performance_statistics(self) -> dict:
        """获取性能统计"""
        stats = {}
        
        # 策略成功率
        for strategy, data in self.stats['strategy_success_rates'].items():
            if data['total'] > 0:
                success_rate = data['success'] / data['total']
                stats[f'{strategy}_success_rate'] = success_rate
        
        # 平均规划时间
        for strategy, times in self.stats['average_planning_times'].items():
            if times:
                avg_time = sum(times) / len(times)
                stats[f'{strategy}_avg_time'] = avg_time
        
        return stats
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
        for planner in self.planners.values():
            if hasattr(planner, 'set_backbone_network'):
                planner.set_backbone_network(backbone_network)
    
    def get_statistics(self):
        """获取统计信息（向后兼容）"""
        return self.get_performance_statistics()
    
    def clear_cache(self):
        """清除缓存（向后兼容）"""
        if self.cache_lock:
            with self.cache_lock:
                self.cache.clear()
        else:
            self.cache.clear()
        
        # 重置缓存相关统计
        self.stats['cache_hits'] = 0
        print("路径规划器缓存已清理")
    
    def shutdown(self):
        """关闭规划器"""
        # 清理缓存
        self.clear_cache()
        
        # 关闭子规划器
        for planner_name, planner in self.planners.items():
            if hasattr(planner, 'shutdown'):
                try:
                    planner.shutdown()
                except Exception as e:
                    print(f"关闭规划器 {planner_name} 失败: {e}")
        
        self.planners.clear()
        print("路径规划器已关闭")