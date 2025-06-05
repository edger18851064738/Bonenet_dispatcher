"""
integrated_planner_config.py - 整合优化版路径规划器配置
完美适应网络整理、双重冲突检测、多阶段任务调度的规划器配置系统
使用RS曲线避障连接替换RRT规划器
"""

import math
import time
import threading
from typing import Dict, Any, List, Tuple, Optional
from enum import Enum
from dataclasses import dataclass

# 导入网络整理相关模块
try:
    from improved_backbone_network_consolidation import ImprovedBackboneNetworkConsolidator
    CONSOLIDATION_AVAILABLE = True
except ImportError:
    CONSOLIDATION_AVAILABLE = False

# 导入RS曲线模块
try:
    from eastar import MiningOptimizedReedShepp
    RS_CURVES_AVAILABLE = True
except ImportError:
    RS_CURVES_AVAILABLE = False

class NetworkTopologyType(Enum):
    """网络拓扑类型"""
    ORIGINAL = "original"
    CONSOLIDATED = "consolidated"
    HIERARCHICAL = "hierarchical"

class ConflictAwarenessLevel(Enum):
    """冲突感知级别"""
    BASIC = "basic"
    ENHANCED = "enhanced"
    SPATIAL_AWARE = "spatial_aware"
    PREDICTIVE = "predictive"

class TaskStageType(Enum):
    """任务阶段类型"""
    LOADING = "loading"
    TRANSPORT = "transport"
    UNLOADING = "unloading"
    PARKING = "parking"
    MAINTENANCE = "maintenance"

@dataclass
class NetworkAwareConfig:
    """网络感知配置"""
    topology_type: NetworkTopologyType = NetworkTopologyType.ORIGINAL
    consolidation_aware: bool = False
    hierarchy_level_preference: str = "trunk"  # trunk, branch, connector
    spatial_conflict_avoidance: bool = True
    alternative_path_exploration_depth: int = 3
    
@dataclass
class ConflictAwareConfig:
    """冲突感知配置"""
    awareness_level: ConflictAwarenessLevel = ConflictAwarenessLevel.ENHANCED
    spatial_detection_enabled: bool = True
    predictive_horizon: float = 300.0
    safety_margin_multiplier: float = 1.2
    conflict_avoidance_weight: float = 0.3

@dataclass
class MultiStageTaskConfig:
    """多阶段任务配置"""
    stage_aware_planning: bool = True
    inter_stage_optimization: bool = True
    stage_transition_buffer: float = 15.0
    cross_stage_conflict_prevention: bool = True
    stage_priority_weighting: Dict[str, float] = None
    
    def __post_init__(self):
        if self.stage_priority_weighting is None:
            self.stage_priority_weighting = {
                "loading": 1.2,
                "transport": 1.0,
                "unloading": 1.1,
                "parking": 0.8,
                "maintenance": 0.9
            }

class IntegratedPlannerConfig:
    """整合优化版规划器配置类"""
    
    def __init__(self):
        # 基础配置 - 优化版
        self.base_config = {
            'max_planning_time': 25.0,  # 增加规划时间适应复杂网络
            'quality_threshold': 0.55,  # 降低质量阈值提高成功率
            'cache_size': 300,  # 增加缓存大小
            'enable_fallback': True,
            'timeout_retry_count': 3,
            'enable_progressive_optimization': True,
            'network_topology_adaptation': True
        }
        
        # 网络感知配置
        self.network_aware_config = NetworkAwareConfig()
        
        # 冲突感知配置  
        self.conflict_aware_config = ConflictAwareConfig()
        
        # 多阶段任务配置
        self.multi_stage_config = MultiStageTaskConfig()
        
        # 混合A*配置 - 网络整理适应版
        self.astar_configs = {
            'strict_original': {
                'vehicle_length': 6.0,
                'vehicle_width': 3.0,
                'turning_radius': 8.0,
                'step_size': 1.8,  # 稍小的步长提高精度
                'angle_resolution': 24,  # 更精细的角度分辨率
                'max_iterations': 25000,  # 增加迭代次数
                'rs_fitting_radius': 18.0,
                'quality_threshold': 0.75,
                'timeout': 18.0,
                'spatial_awareness': False
            },
            'standard_consolidated': {
                'vehicle_length': 6.0,
                'vehicle_width': 3.0,
                'turning_radius': 7.5,
                'step_size': 2.0,
                'angle_resolution': 30,
                'max_iterations': 22000,
                'rs_fitting_radius': 22.0,
                'quality_threshold': 0.6,
                'timeout': 20.0,
                'spatial_awareness': True,
                'consolidation_bonus': 0.1  # 整理路径质量加权
            },
            'relaxed_hierarchical': {
                'vehicle_length': 6.0,
                'vehicle_width': 3.0,
                'turning_radius': 7.0,
                'step_size': 2.2,
                'angle_resolution': 36,
                'max_iterations': 20000,
                'rs_fitting_radius': 25.0,
                'quality_threshold': 0.45,
                'timeout': 22.0,
                'spatial_awareness': True,
                'hierarchy_aware': True,
                'trunk_preference_bonus': 0.15
            },
            'emergency_fallback': {
                'vehicle_length': 6.0,
                'vehicle_width': 3.0,
                'turning_radius': 6.5,
                'step_size': 2.5,
                'angle_resolution': 45,
                'max_iterations': 15000,
                'rs_fitting_radius': 30.0,
                'quality_threshold': 0.3,
                'timeout': 15.0,
                'spatial_awareness': False,
                'aggressive_mode': True
            }
        }
        
        # RS曲线配置 - 替换原RRT配置，多阶段任务适应版
        self.rs_curve_configs = {
            'standard_multi_stage': {
                'vehicle_length': 6.0,
                'vehicle_width': 3.0,
                'turning_radius': 8.0,
                'step_size': 0.8,
                'quality_threshold': 0.6,
                'max_curve_attempts': 5,
                'smoothness_preference': 0.8,
                'enable_obstacle_avoidance': True,
                'stage_aware_curves': True,
                'inter_stage_optimization': True,
                'curve_complexity_limit': 3.0  # 限制曲线复杂度
            },
            'fast_consolidated': {
                'vehicle_length': 6.0,
                'vehicle_width': 3.0,
                'turning_radius': 7.5,
                'step_size': 1.0,
                'quality_threshold': 0.5,
                'max_curve_attempts': 3,
                'smoothness_preference': 0.7,
                'enable_obstacle_avoidance': True,
                'consolidation_aware': True,
                'prefer_trunk_paths': True,
                'curve_complexity_limit': 2.5
            },
            'spatial_aware': {
                'vehicle_length': 6.0,
                'vehicle_width': 3.0,
                'turning_radius': 8.5,
                'step_size': 0.6,
                'quality_threshold': 0.65,
                'max_curve_attempts': 6,
                'smoothness_preference': 0.9,
                'enable_obstacle_avoidance': True,
                'spatial_conflict_avoidance': True,
                'safety_margin_boost': 1.3,
                'curve_complexity_limit': 3.5,
                'precision_mode': True
            },
            'aggressive_fallback': {
                'vehicle_length': 6.0,
                'vehicle_width': 3.0,
                'turning_radius': 6.0,
                'step_size': 1.2,
                'quality_threshold': 0.35,
                'max_curve_attempts': 2,
                'smoothness_preference': 0.5,
                'enable_obstacle_avoidance': False,
                'aggressive_mode': True,
                'curve_complexity_limit': 2.0,
                'allow_sharp_turns': True
            }
        }
        
        # 网络感知的渐进式回退策略 - 更新为使用RS曲线
        self.network_aware_fallback_strategies = {
            NetworkTopologyType.ORIGINAL: [
                {
                    'name': 'astar_strict_original',
                    'planner': 'hybrid_astar',
                    'config': 'strict_original',
                    'max_time': 15.0,
                    'priority': 1,
                    'network_specific': True
                },
                {
                    'name': 'astar_standard_consolidated',
                    'planner': 'hybrid_astar',
                    'config': 'standard_consolidated',
                    'max_time': 18.0,
                    'priority': 2,
                    'network_specific': False
                },
                {
                    'name': 'rs_standard_multi_stage',
                    'planner': 'rs_curves',
                    'config': 'standard_multi_stage',
                    'max_time': 12.0,
                    'priority': 3,
                    'network_specific': False
                },
                {
                    'name': 'astar_emergency',
                    'planner': 'hybrid_astar',
                    'config': 'emergency_fallback',
                    'max_time': 12.0,
                    'priority': 4,
                    'network_specific': False
                },
                {
                    'name': 'rs_aggressive_fallback',
                    'planner': 'rs_curves',
                    'config': 'aggressive_fallback',
                    'max_time': 8.0,
                    'priority': 5,
                    'network_specific': False
                },
                {
                    'name': 'direct_fallback',
                    'planner': 'direct',
                    'config': None,
                    'max_time': 1.0,
                    'priority': 6,
                    'network_specific': False
                }
            ],
            
            NetworkTopologyType.CONSOLIDATED: [
                {
                    'name': 'astar_standard_consolidated',
                    'planner': 'hybrid_astar',
                    'config': 'standard_consolidated',
                    'max_time': 18.0,
                    'priority': 1,
                    'network_specific': True
                },
                {
                    'name': 'rs_fast_consolidated',
                    'planner': 'rs_curves',
                    'config': 'fast_consolidated',
                    'max_time': 10.0,
                    'priority': 2,
                    'network_specific': True
                },
                {
                    'name': 'astar_relaxed_hierarchical',
                    'planner': 'hybrid_astar',
                    'config': 'relaxed_hierarchical',
                    'max_time': 20.0,
                    'priority': 3,
                    'network_specific': False
                },
                {
                    'name': 'rs_spatial_aware',
                    'planner': 'rs_curves',
                    'config': 'spatial_aware',
                    'max_time': 13.0,
                    'priority': 4,
                    'network_specific': False
                },
                {
                    'name': 'rs_aggressive_fallback',
                    'planner': 'rs_curves',
                    'config': 'aggressive_fallback',
                    'max_time': 8.0,
                    'priority': 5,
                    'network_specific': False
                },
                {
                    'name': 'direct_fallback',
                    'planner': 'direct',
                    'config': None,
                    'max_time': 1.0,
                    'priority': 6,
                    'network_specific': False
                }
            ],
            
            NetworkTopologyType.HIERARCHICAL: [
                {
                    'name': 'astar_relaxed_hierarchical',
                    'planner': 'hybrid_astar',
                    'config': 'relaxed_hierarchical',
                    'max_time': 20.0,
                    'priority': 1,
                    'network_specific': True
                },
                {
                    'name': 'rs_spatial_aware',
                    'planner': 'rs_curves',
                    'config': 'spatial_aware',
                    'max_time': 13.0,
                    'priority': 2,
                    'network_specific': True
                },
                {
                    'name': 'astar_standard_consolidated',
                    'planner': 'hybrid_astar',
                    'config': 'standard_consolidated',
                    'max_time': 18.0,
                    'priority': 3,
                    'network_specific': False
                },
                {
                    'name': 'rs_standard_multi_stage',
                    'planner': 'rs_curves',
                    'config': 'standard_multi_stage',
                    'max_time': 12.0,
                    'priority': 4,
                    'network_specific': False
                },
                {
                    'name': 'astar_emergency',
                    'planner': 'hybrid_astar',
                    'config': 'emergency_fallback',
                    'max_time': 12.0,
                    'priority': 5,
                    'network_specific': False
                },
                {
                    'name': 'rs_aggressive_fallback',
                    'planner': 'rs_curves',
                    'config': 'aggressive_fallback',
                    'max_time': 8.0,
                    'priority': 6,
                    'network_specific': False
                },
                {
                    'name': 'direct_fallback',
                    'planner': 'direct',
                    'config': None,
                    'max_time': 1.0,
                    'priority': 7,
                    'network_specific': False
                }
            ]
        }
        
        # 任务阶段感知的配置调整
        self.stage_aware_adjustments = {
            TaskStageType.LOADING: {
                'quality_threshold_boost': 0.1,
                'safety_margin_multiplier': 1.3,
                'preferred_planners': ['hybrid_astar', 'rs_curves'],
                'timeout_extension': 1.2
            },
            TaskStageType.TRANSPORT: {
                'quality_threshold_boost': 0.0,
                'safety_margin_multiplier': 1.0,
                'preferred_planners': ['hybrid_astar', 'rs_curves'],
                'timeout_extension': 1.0
            },
            TaskStageType.UNLOADING: {
                'quality_threshold_boost': 0.08,
                'safety_margin_multiplier': 1.2,
                'preferred_planners': ['hybrid_astar', 'rs_curves'],
                'timeout_extension': 1.15
            },
            TaskStageType.PARKING: {
                'quality_threshold_boost': -0.05,
                'safety_margin_multiplier': 0.9,
                'preferred_planners': ['rs_curves'],
                'timeout_extension': 0.8
            }
        }
        self.professional_road_config = {
            'enable_road_class_awareness': True,
            'prefer_higher_class_roads': True,
            'engineering_standards_compliance': True,
            'safety_rating_weight': 0.25,
            'construction_cost_awareness': True,
        }
        
        # 道路等级感知的规划参数调整
        self.road_class_adjustments = {
            'primary': {
                'quality_threshold_boost': 0.15,
                'safety_margin_multiplier': 1.1,
                'preferred_planners': ['hybrid_astar'],
                'timeout_extension': 1.3,
                'priority_bonus': 0.2
            },
            'secondary': {
                'quality_threshold_boost': 0.08,
                'safety_margin_multiplier': 1.0,
                'preferred_planners': ['hybrid_astar', 'rs_curves'],
                'timeout_extension': 1.0,
                'priority_bonus': 0.1
            },
            'service': {
                'quality_threshold_boost': 0.0,
                'safety_margin_multiplier': 0.9,
                'preferred_planners': ['rs_curves'],
                'timeout_extension': 0.8,
                'priority_bonus': 0.0
            }
        }
    
    def update_network_topology(self, topology_type: NetworkTopologyType, 
                               consolidation_info: Dict = None):
        """更新网络拓扑配置"""
        self.network_aware_config.topology_type = topology_type
        
        if topology_type in [NetworkTopologyType.CONSOLIDATED, NetworkTopologyType.HIERARCHICAL]:
            self.network_aware_config.consolidation_aware = True
            
            if consolidation_info:
                # 根据整理信息调整配置
                self._adapt_config_to_consolidation(consolidation_info)
        
        print(f"✅ 网络拓扑配置已更新: {topology_type.value}")
    
    def _adapt_config_to_consolidation(self, consolidation_info: Dict):
        """根据整理信息适应配置"""
        # 获取整理统计
        stats = consolidation_info.get('consolidation_stats', {})
        
        if 'summary' in stats:
            summary = stats['summary']
            reduction_ratio = summary.get('reduction_ratio', 0.0)
            
            # 根据路径减少比例调整配置
            if reduction_ratio > 0.4:  # 高度整理
                self.base_config['quality_threshold'] *= 0.9  # 降低质量要求
                self.network_aware_config.alternative_path_exploration_depth = 2
            elif reduction_ratio > 0.2:  # 中度整理
                self.base_config['quality_threshold'] *= 0.95
                self.network_aware_config.alternative_path_exploration_depth = 3
        
        # 调整冲突感知参数
        if consolidation_info.get('hierarchy_info'):
            self.conflict_aware_config.awareness_level = ConflictAwarenessLevel.SPATIAL_AWARE
            self.conflict_aware_config.spatial_detection_enabled = True
    
    def get_astar_config(self, level: str = 'standard_consolidated', 
                        network_context: Dict = None) -> Dict[str, Any]:
        """获取网络感知的混合A*配置"""
        base_config = self.astar_configs.get(level, self.astar_configs['standard_consolidated']).copy()
        
        # 网络感知调整
        if network_context:
            base_config = self._apply_network_context_to_config(base_config, network_context)
        
        # 冲突感知调整
        if self.conflict_aware_config.spatial_detection_enabled:
            base_config = self._apply_spatial_awareness_to_config(base_config)
        
        return base_config
    
    def get_rs_curve_config(self, level: str = 'standard_multi_stage',
                           task_context: Dict = None) -> Dict[str, Any]:
        """获取任务感知的RS曲线配置"""
        base_config = self.rs_curve_configs.get(level, self.rs_curve_configs['standard_multi_stage']).copy()
        
        # 任务阶段调整
        if task_context and 'current_stage' in task_context:
            base_config = self._apply_task_stage_adjustments(base_config, task_context)
        
        # 网络整理感知调整
        if self.network_aware_config.consolidation_aware:
            base_config = self._apply_consolidation_awareness_to_config(base_config)
        
        return base_config
    
    def get_context_optimized_config(self, context: str, planner_type: str, 
                                   network_info: Dict = None, 
                                   task_info: Dict = None) -> Dict[str, Any]:
        """根据上下文获取优化配置"""
        # 向后兼容：RRT重定向到RS曲线
        if planner_type == 'rrt':
            planner_type = 'rs_curves'
            print(f"配置获取：RRT重定向到RS曲线")
        
        if context == 'backbone':
            # 骨干路径生成：高质量要求，网络感知
            if planner_type == 'hybrid_astar':
                if self.network_aware_config.topology_type == NetworkTopologyType.HIERARCHICAL:
                    config = self.get_astar_config('relaxed_hierarchical', network_info)
                elif self.network_aware_config.consolidation_aware:
                    config = self.get_astar_config('standard_consolidated', network_info)
                else:
                    config = self.get_astar_config('strict_original', network_info)
                
                config.update({
                    'max_iterations': min(config.get('max_iterations', 20000), 30000),
                    'timeout': min(config.get('timeout', 20.0), 25.0),
                    'quality_threshold': max(config.get('quality_threshold', 0.6), 0.5)
                })
                return config
                
            elif planner_type == 'rs_curves':
                config = self.get_rs_curve_config('spatial_aware', task_info)
                config.update({
                    'max_curve_attempts': min(config.get('max_curve_attempts', 6), 8),
                    'enable_obstacle_avoidance': True,
                    'precision_mode': True
                })
                return config
        
        elif context == 'navigation':
            # 导航路径：平衡质量和速度，多阶段感知
            if planner_type == 'hybrid_astar':
                if self.network_aware_config.consolidation_aware:
                    config = self.get_astar_config('standard_consolidated', network_info)
                else:
                    config = self.get_astar_config('strict_original', network_info)
                
                config.update({
                    'timeout': min(config.get('timeout', 15.0), 18.0),
                    'quality_threshold': config.get('quality_threshold', 0.55) * 0.9
                })
                return config
                
            elif planner_type == 'rs_curves':
                config = self.get_rs_curve_config('standard_multi_stage', task_info)
                return config
        
        elif context == 'emergency':
            # 紧急情况：优先速度，降低质量要求
            if planner_type == 'hybrid_astar':
                config = self.get_astar_config('emergency_fallback', network_info)
                config.update({
                    'timeout': min(config.get('timeout', 12.0), 10.0),
                    'quality_threshold': max(config.get('quality_threshold', 0.3), 0.2),
                    'aggressive_mode': True
                })
                return config
                
            elif planner_type == 'rs_curves':
                config = self.get_rs_curve_config('aggressive_fallback', task_info)
                config.update({
                    'max_curve_attempts': min(config.get('max_curve_attempts', 2), 1),
                    'enable_obstacle_avoidance': False,
                    'aggressive_mode': True
                })
                return config
        
        # 默认配置 - 向后兼容处理
        if planner_type == 'hybrid_astar':
            return self.get_astar_config('standard_consolidated', network_info)
        elif planner_type == 'rs_curves':
            return self.get_rs_curve_config('standard_multi_stage', task_info)
        
        return {}
    
    def get_network_aware_fallback_sequence(self, context: str = 'normal', 
                                          network_info: Dict = None) -> List:
        """获取网络感知的渐进式回退序列"""
        # 检测网络拓扑
        topology = self.network_aware_config.topology_type
        if network_info and 'topology_type' in network_info:
            topology = NetworkTopologyType(network_info['topology_type'])
        
        # 获取对应的回退策略
        fallback_sequence = self.network_aware_fallback_strategies.get(
            topology, 
            self.network_aware_fallback_strategies[NetworkTopologyType.ORIGINAL]
        ).copy()
        
        # 根据上下文调整序列
        if context == 'backbone':
            # 骨干路径生成：更保守的回退，优先高质量策略
            fallback_sequence = [s for s in fallback_sequence if s['priority'] <= 4]
            
        elif context == 'emergency':
            # 紧急情况：快速回退序列，优先RS曲线
            emergency_sequence = []
            for strategy in fallback_sequence:
                if ('emergency' in strategy['name'] or 
                    strategy['planner'] == 'rs_curves' or
                    'aggressive' in strategy['name']):
                    emergency_sequence.append(strategy)
                if len(emergency_sequence) >= 3:
                    break
            emergency_sequence.append({
                'name': 'direct_fallback',
                'planner': 'direct',
                'config': None,
                'max_time': 1.0,
                'priority': 999
            })
            fallback_sequence = emergency_sequence
            
        elif context == 'multi_stage':
            # 多阶段任务：优先多阶段感知策略，RS曲线很适合
            multi_stage_sequence = []
            for strategy in fallback_sequence:
                config_name = strategy.get('config', '')
                if ('multi_stage' in config_name or 
                    'consolidated' in config_name or 
                    'hierarchical' in config_name or
                    strategy['planner'] == 'rs_curves'):
                    multi_stage_sequence.append(strategy)
            
            # 补充其他策略
            for strategy in fallback_sequence:
                if strategy not in multi_stage_sequence:
                    multi_stage_sequence.append(strategy)
                    
            fallback_sequence = multi_stage_sequence
        
        # 应用网络特定优化
        if network_info:
            fallback_sequence = self._optimize_fallback_for_network(fallback_sequence, network_info)
        
        return fallback_sequence
    
    def _apply_network_context_to_config(self, config: Dict, network_context: Dict) -> Dict:
        """应用网络上下文到配置"""
        enhanced_config = config.copy()
        
        # 整理网络的特殊处理
        if network_context.get('is_consolidated', False):
            if 'consolidation_bonus' in config:
                quality_threshold = enhanced_config.get('quality_threshold', 0.6)
                enhanced_config['quality_threshold'] = quality_threshold + config['consolidation_bonus']
        
        # 层次网络的特殊处理
        if network_context.get('hierarchy_level') == 'trunk':
            if 'trunk_preference_bonus' in config:
                quality_threshold = enhanced_config.get('quality_threshold', 0.6)
                enhanced_config['quality_threshold'] = quality_threshold + config['trunk_preference_bonus']
        
        # 空间冲突感知
        if network_context.get('spatial_conflicts_detected', False):
            enhanced_config['step_size'] = enhanced_config.get('step_size', 2.0) * 0.9
            enhanced_config['safety_margin_boost'] = enhanced_config.get('safety_margin_boost', 1.0) * 1.2
        
        return enhanced_config
    
    def _apply_spatial_awareness_to_config(self, config: Dict) -> Dict:
        """应用空间感知到配置"""
        spatial_config = config.copy()
        
        # 增强安全边距
        if self.conflict_aware_config.safety_margin_multiplier != 1.0:
            if 'turning_radius' in spatial_config:
                spatial_config['turning_radius'] *= self.conflict_aware_config.safety_margin_multiplier
        
        # 降低步长提高精度
        if 'step_size' in spatial_config:
            spatial_config['step_size'] *= 0.95
        
        # 增加角度分辨率
        if 'angle_resolution' in spatial_config:
            spatial_config['angle_resolution'] = max(24, int(spatial_config['angle_resolution'] * 0.8))
        
        return spatial_config
    
    def _apply_task_stage_adjustments(self, config: Dict, task_context: Dict) -> Dict:
        """应用任务阶段调整"""
        stage_config = config.copy()
        current_stage = task_context.get('current_stage', 'transport')
        
        try:
            stage_type = TaskStageType(current_stage)
        except ValueError:
            stage_type = TaskStageType.TRANSPORT
        
        adjustments = self.stage_aware_adjustments.get(stage_type, {})
        
        # 质量阈值调整
        if 'quality_threshold_boost' in adjustments:
            quality_threshold = stage_config.get('quality_threshold', 0.5)
            stage_config['quality_threshold'] = quality_threshold + adjustments['quality_threshold_boost']
        
        # 安全边距调整
        if 'safety_margin_multiplier' in adjustments:
            if 'turning_radius' in stage_config:
                stage_config['turning_radius'] *= adjustments['safety_margin_multiplier']
        
        # 超时调整
        if 'timeout_extension' in adjustments and 'timeout' in stage_config:
            stage_config['timeout'] *= adjustments['timeout_extension']
        
        # 多阶段特殊配置 - RS曲线优化
        if self.multi_stage_config.stage_aware_planning:
            if 'stage_aware_curves' not in stage_config:
                stage_config['stage_aware_curves'] = True
            
            if stage_type in [TaskStageType.LOADING, TaskStageType.UNLOADING]:
                stage_config['precision_mode'] = True
                if 'smoothness_preference' in stage_config:
                    stage_config['smoothness_preference'] *= 1.1  # 增强平滑性
        
        return stage_config
    
    def _apply_consolidation_awareness_to_config(self, config: Dict) -> Dict:
        """应用整理感知到配置"""
        consolidation_config = config.copy()
        
        # 整理网络的优化
        if 'prefer_trunk_paths' in config and config['prefer_trunk_paths']:
            consolidation_config['trunk_path_bonus'] = 0.1
        
        if 'consolidation_aware' in config and config['consolidation_aware']:
            # 适应整理后的路径结构 - RS曲线调整
            if 'step_size' in consolidation_config:
                consolidation_config['step_size'] *= 1.05  # 稍大的步长适应整理路径
            
            if 'max_curve_attempts' in consolidation_config:
                consolidation_config['max_curve_attempts'] = max(2, consolidation_config['max_curve_attempts'] - 1)  # 减少尝试次数
        
        return consolidation_config
    
    def _optimize_fallback_for_network(self, fallback_sequence: List, network_info: Dict) -> List:
        """为特定网络优化回退序列"""
        optimized_sequence = []
        
        # 网络特性
        is_consolidated = network_info.get('is_consolidated', False)
        has_spatial_conflicts = network_info.get('spatial_conflicts_detected', False)
        alternative_paths_count = network_info.get('alternative_paths_available', 0)
        
        for strategy in fallback_sequence:
            optimized_strategy = strategy.copy()
            
            # 根据网络特性调整超时
            if is_consolidated:
                # 整理网络可以稍微延长规划时间
                optimized_strategy['max_time'] *= 1.1
            
            if has_spatial_conflicts:
                # 空间冲突网络需要更精细的规划，RS曲线很适合
                if 'rs_curves' in optimized_strategy['name']:
                    optimized_strategy['max_time'] *= 1.2  # RS曲线获得更多时间
                elif 'astar' in optimized_strategy['name']:
                    optimized_strategy['max_time'] *= 1.15
            
            if alternative_paths_count < 2:
                # 备选路径少时需要更高质量的规划
                optimized_strategy['max_time'] *= 1.2
                optimized_strategy['quality_boost'] = True
            
            optimized_sequence.append(optimized_strategy)
        
        return optimized_sequence
    
    def get_stage_aware_planning_config(self, current_stage: str, next_stage: str = None) -> Dict:
        """获取阶段感知的规划配置"""
        config = {
            'current_stage': current_stage,
            'stage_transition_planning': next_stage is not None,
            'inter_stage_buffer': self.multi_stage_config.stage_transition_buffer
        }
        
        if next_stage:
            config['next_stage'] = next_stage
            config['cross_stage_optimization'] = self.multi_stage_config.inter_stage_optimization
            
            # 阶段间优化参数 - RS曲线优化
            if current_stage == 'loading' and next_stage == 'transport':
                config['loading_to_transport_optimization'] = True
                config['curve_continuity_weight'] = 0.3  # RS曲线连续性
            elif current_stage == 'transport' and next_stage == 'unloading':
                config['transport_to_unloading_optimization'] = True
                config['precision_approach_planning'] = True
        
        return config
    
    def adapt_to_conflict_detection_results(self, conflict_info: Dict):
        """根据冲突检测结果适应配置"""
        if not conflict_info:
            return
        
        # 更新冲突感知级别
        spatial_conflicts = conflict_info.get('spatial_conflicts_detected', 0)
        logical_conflicts = conflict_info.get('logical_conflicts_detected', 0)
        
        if spatial_conflicts > logical_conflicts * 0.5:
            # 空间冲突较多，提升空间感知，RS曲线很适合处理空间冲突
            self.conflict_aware_config.awareness_level = ConflictAwarenessLevel.SPATIAL_AWARE
            self.conflict_aware_config.spatial_detection_enabled = True
            self.conflict_aware_config.safety_margin_multiplier = 1.3
        
        # 调整预测时域
        total_conflicts = spatial_conflicts + logical_conflicts
        if total_conflicts > 10:
            # 冲突较多，增加预测时域
            self.conflict_aware_config.predictive_horizon = min(
                self.conflict_aware_config.predictive_horizon * 1.2, 450.0
            )
        
        print(f"✅ 配置已根据冲突信息调整: 空间冲突{spatial_conflicts}, 逻辑冲突{logical_conflicts}")


class IntegratedPathPlannerWithOptimizedConfig:
    """集成优化配置的增强路径规划器"""
    
    def __init__(self, env, backbone_network=None, traffic_manager=None):
        self.env = env
        self.backbone_network = backbone_network
        self.traffic_manager = traffic_manager
        
        # 加载集成优化配置
        self.config_manager = IntegratedPlannerConfig()
        
        # 规划器实例
        self.planners = {}
        self._initialize_planners_with_integrated_config()
        
        # 网络拓扑监控
        self.current_network_topology = NetworkTopologyType.ORIGINAL
        self.last_topology_check = time.time()
        self.topology_check_interval = 30.0  # 30秒检查一次
        
        # 简化缓存
        self.cache = {}
        self.cache_lock = threading.RLock() if 'threading' in globals() else None
        
        # 性能统计 - 增强版
        self.stats = {
            'strategy_success_rates': {},
            'average_planning_times': {},
            'fallback_usage': {},
            'total_requests': 0,
            'successful_plans': 0,
            'cache_hits': 0,
            
            # 网络感知统计
            'network_topology_adaptations': 0,
            'consolidation_aware_plans': 0,
            'spatial_conflict_aware_plans': 0,
            'multi_stage_optimized_plans': 0,
            'hierarchy_aware_plans': 0,
            'rs_curve_plans': 0,  # 新增RS曲线统计
            
            # 配置优化统计
            'config_adaptations': 0,
            'fallback_sequence_optimizations': 0,
            'stage_aware_adjustments': 0
        }
        
        print("初始化集成优化配置的增强路径规划器")
        print(f"  网络整理感知: {'✅' if CONSOLIDATION_AVAILABLE else '❌'}")
        print(f"  多阶段任务感知: ✅")
        print(f"  空间冲突感知: ✅")
        print(f"  RS曲线规划器: {'✅' if RS_CURVES_AVAILABLE else '❌'}")
    
    def _initialize_planners_with_integrated_config(self):
        """使用集成优化配置初始化规划器"""
        try:
            # 初始化混合A*
            from eastar import HybridAStarPlanner
            astar_config = self.config_manager.get_astar_config('standard_consolidated')
            
            self.planners['hybrid_astar'] = HybridAStarPlanner(
                self.env,
                vehicle_length=astar_config['vehicle_length'],
                vehicle_width=astar_config['vehicle_width'],
                turning_radius=astar_config['turning_radius'],
                step_size=astar_config['step_size'],
                angle_resolution=astar_config['angle_resolution']
            )
            print("✅ 网络感知混合A*规划器初始化完成")
        
        except ImportError:
            print("⚠️ 混合A*规划器不可用")
        
        # 初始化RS曲线规划器并添加兼容性包装
        if RS_CURVES_AVAILABLE:
            try:
                rs_config = self.config_manager.get_rs_curve_config('standard_multi_stage')
                base_rs_planner = MiningOptimizedReedShepp(
                    turning_radius=rs_config['turning_radius'],
                    step_size=rs_config['step_size']
                )
                
                # 创建兼容性包装器
                class RSPlannerWrapper:
                    def __init__(self, rs_planner, env):
                        self.rs_planner = rs_planner
                        self.env = env
                    
                    def plan_path(self, start, goal, **kwargs):
                        """兼容RRT接口的plan_path方法"""
                        try:
                            # 确保起点和终点有朝向信息
                            start_with_heading = self._ensure_heading(start, goal)
                            goal_with_heading = self._ensure_heading(goal, start)
                            
                            # 获取步长，优先使用传入参数
                            step_size = kwargs.get('step_size', self.rs_planner.step_size)
                            
                            # 调用RS曲线生成
                            path = self.rs_planner.get_path(start_with_heading, goal_with_heading, step_size)
                            
                            if path and len(path) >= 2:
                                print(f"      RS曲线包装器成功生成路径: {len(path)}点")
                                return path
                            else:
                                print(f"      RS曲线包装器生成路径失败")
                                return None
                        
                        except Exception as e:
                            print(f"      RS曲线包装器异常: {e}")
                            return None
                    
                    def _ensure_heading(self, point, reference_point):
                        """确保点包含朝向信息"""
                        if len(point) >= 3:
                            return point
                        
                        dx = reference_point[0] - point[0]
                        dy = reference_point[1] - point[1]
                        heading = math.atan2(dy, dx)
                        return (point[0], point[1], heading)
                    
                    def get_path(self, start, goal, step_size=None):
                        """原始RS曲线接口"""
                        if step_size is None:
                            step_size = self.rs_planner.step_size
                        return self.rs_planner.get_path(start, goal, step_size)
                
                # 使用包装器
                self.planners['rs_curves'] = RSPlannerWrapper(base_rs_planner, self.env)
                
                # 向后兼容：RRT键名指向同一个规划器
                self.planners['rrt'] = self.planners['rs_curves']
                
                print("✅ 多阶段感知RS曲线规划器初始化完成（含RRT兼容性）")
            except Exception as e:
                print(f"⚠️ RS曲线规划器初始化失败: {e}")
        else:
            print("⚠️ RS曲线规划器不可用")
    
    def update_network_topology_awareness(self):
        """更新网络拓扑感知"""
        current_time = time.time()
        
        if current_time - self.last_topology_check < self.topology_check_interval:
            return
        
        if (self.backbone_network and 
            hasattr(self.backbone_network, 'get_improved_consolidation_info')):
            
            consolidation_info = self.backbone_network.get_improved_consolidation_info()
            
            # 检测网络拓扑类型
            if consolidation_info.get('is_consolidated', False):
                if consolidation_info.get('hierarchy_info'):
                    new_topology = NetworkTopologyType.HIERARCHICAL
                else:
                    new_topology = NetworkTopologyType.CONSOLIDATED
            else:
                new_topology = NetworkTopologyType.ORIGINAL
            
            # 更新配置
            if new_topology != self.current_network_topology:
                self.current_network_topology = new_topology
                self.config_manager.update_network_topology(new_topology, consolidation_info)
                self.stats['network_topology_adaptations'] += 1
                print(f"🔄 网络拓扑感知更新: {new_topology.value}")
        
        self.last_topology_check = current_time
    
    def plan_path(self, vehicle_id: str, start: tuple, goal: tuple,
                use_backbone: bool = True, check_conflicts: bool = True,
                planner_type: str = "auto", context: str = "normal",
                return_object: bool = False, **kwargs) -> Any:
        """
        集成优化版路径规划接口
        """
        # 更新网络拓扑感知
        self.update_network_topology_awareness()
        
        # 获取任务上下文
        task_context = self._extract_task_context(kwargs)
        network_context = self._extract_network_context(kwargs)
        
        # ===== 骨干网络优先逻辑（增强版） =====
        if use_backbone and self.backbone_network and context != 'backbone':
            print(f"[集成优化规划器] 尝试网络感知骨干路径: {vehicle_id}")
            
            # 增强的目标信息提取
            target_type = kwargs.get('target_type')
            target_id = kwargs.get('target_id')
            
            # 从任务上下文推断目标
            if not target_type and task_context:
                target_type = task_context.get('target_type')
                target_id = task_context.get('target_id')
            
            if target_type and target_id is not None:
                try:
                    backbone_result = self.backbone_network.get_path_from_position_to_target(
                        start, target_type, target_id, vehicle_id
                    )
                    
                    if backbone_result:
                        print(f"  ✅ 网络感知骨干路径成功!")
                        
                        # 更新统计
                        if self.config_manager.network_aware_config.consolidation_aware:
                            self.stats['consolidation_aware_plans'] += 1
                        if task_context.get('multi_stage', False):
                            self.stats['multi_stage_optimized_plans'] += 1
                        
                        # 处理返回结果
                        if isinstance(backbone_result, tuple) and len(backbone_result) >= 2:
                            path, structure = backbone_result
                            result = self._create_enhanced_result_object(
                                path, 'backbone_network', len(path), 
                                network_context, task_context
                            )
                            result.structure = structure
                            return result if return_object else path
                        else:
                            return backbone_result
                            
                except Exception as e:
                    print(f"  网络感知骨干路径尝试失败: {e}")
        
        # ===== 集成优化的渐进式回退 =====
        result = self.plan_path_with_integrated_fallback(
            vehicle_id, start, goal, context, network_context, task_context, **kwargs
        )
        
        # 根据return_object参数返回不同格式
        if return_object and result:
            return result
        elif result and hasattr(result, 'path'):
            return result.path
        else:
            return result
    
    def plan_path_with_integrated_fallback(self, vehicle_id: str, start: tuple, goal: tuple,
                                         context: str = 'normal', 
                                         network_context: Dict = None,
                                         task_context: Dict = None, **kwargs) -> Any:
        """
        集成优化的渐进式回退路径规划
        """
        import time
        
        self.stats['total_requests'] += 1
        planning_start = time.time()
        
        # 获取网络感知的回退序列
        fallback_sequence = self.config_manager.get_network_aware_fallback_sequence(
            context, network_context
        )
        
        print(f"  [集成回退] 使用{len(fallback_sequence)}个策略，网络拓扑: {self.current_network_topology.value}")
        
        for i, strategy in enumerate(fallback_sequence, 1):
            strategy_name = strategy['name']
            planner_type = strategy['planner']
            config_level = strategy.get('config')
            max_time = strategy['max_time']
            
            print(f"  [{i}/{len(fallback_sequence)}] 尝试策略: {strategy_name}")
            
            # 获取集成优化配置
            if config_level:
                if planner_type == 'hybrid_astar':
                    strategy_config = self.config_manager.get_astar_config(config_level, network_context)
                elif planner_type == 'rs_curves':
                    strategy_config = self.config_manager.get_rs_curve_config(config_level, task_context)
                else:
                    strategy_config = {}
            else:
                strategy_config = {}
            
            # 应用上下文优化
            if context != 'normal':
                context_config = self.config_manager.get_context_optimized_config(
                    context, planner_type, network_context, task_context
                )
                strategy_config.update(context_config)
            
            # 执行规划
            strategy_start = time.time()
            result = self._execute_integrated_strategy(
                vehicle_id, start, goal, planner_type, 
                strategy_config, max_time, context, network_context, task_context
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
                print(f"    ✅ 集成策略成功! 耗时: {strategy_time:.2f}s, 总耗时: {total_time:.2f}s")
                
                return result
            else:
                print(f"    ❌ 策略失败, 耗时: {strategy_time:.2f}s")
        
        print("  ❌ 所有集成回退策略均失败")
        return None
    
    def _execute_integrated_strategy(self, vehicle_id: str, start: tuple, goal: tuple,
                                   planner_type: str, config: dict, max_time: float,
                                   context: str, network_context: Dict = None,
                                   task_context: Dict = None):
        """执行集成优化的规划策略"""
        try:
            if planner_type == 'hybrid_astar' and 'hybrid_astar' in self.planners:
                return self._plan_with_integrated_astar(
                    vehicle_id, start, goal, config, max_time, network_context
                )
            
            elif (planner_type in ['rs_curves', 'rrt'] and 'rs_curves' in self.planners):
                # 向后兼容：RRT调用重定向到RS曲线
                if planner_type == 'rrt':
                    print(f"      RRT调用重定向到RS曲线规划器")
                return self._plan_with_integrated_rs_curves(
                    vehicle_id, start, goal, config, max_time, task_context
                )
            
            elif planner_type == 'direct':
                return self._plan_direct_path_enhanced(start, goal, network_context)
            
            return None
        
        except Exception as e:
            print(f"      集成策略执行异常: {e}")
            return None
    
    def _plan_with_integrated_astar(self, vehicle_id: str, start: tuple, goal: tuple, 
                                   config: dict, max_time: float, network_context: Dict = None):
        """使用集成优化的混合A*规划"""
        planner = self.planners['hybrid_astar']
        
        # 动态更新配置
        if config:
            if hasattr(planner, 'config'):
                planner.config.update({
                    'max_iterations': config.get('max_iterations', 20000),
                    'timeout': min(max_time, config.get('timeout', 18.0)),
                    'quality_threshold': config.get('quality_threshold', 0.6),
                    'rs_fitting_radius': config.get('rs_fitting_radius', 22.0)
                })
        
        # 网络感知的规划参数调整
        planning_params = {
            'agent_id': vehicle_id,
            'max_iterations': config.get('max_iterations', 20000),
            'quality_threshold': config.get('quality_threshold', 0.6)
        }
        
        # 整理网络特殊处理
        if (network_context and network_context.get('is_consolidated', False) and
            config.get('consolidation_bonus', 0) > 0):
            planning_params['quality_threshold'] *= 0.95  # 稍微降低质量要求
            self.stats['consolidation_aware_plans'] += 1
        
        # 空间冲突感知
        if (network_context and network_context.get('spatial_conflicts_detected', False) and
            config.get('spatial_awareness', False)):
            planning_params['precision_mode'] = True
            self.stats['spatial_conflict_aware_plans'] += 1
        
        path = planner.plan_path(start, goal, **planning_params)
        
        if path:
            return self._create_enhanced_result_object(
                path, 'integrated_hybrid_astar', len(path), network_context
            )
        
        return None
    
    def _plan_with_integrated_rs_curves(self, vehicle_id: str, start: tuple, goal: tuple,
                                       config: dict, max_time: float, task_context: Dict = None):
        """使用集成优化的RS曲线规划"""
        rs_planner = self.planners['rs_curves']
        
        # 确保起点和终点包含朝向信息
        start_with_heading = self._ensure_heading_for_rs(start, goal)
        goal_with_heading = self._ensure_heading_for_rs(goal, start)
        
        # 任务阶段感知的规划参数
        step_size = config.get('step_size', 0.8)
        max_attempts = config.get('max_curve_attempts', 5)
        quality_threshold = config.get('quality_threshold', 0.6)
        
        # 多阶段任务特殊处理
        if (task_context and task_context.get('multi_stage', False) and
            config.get('stage_aware_curves', False)):
            current_stage = task_context.get('current_stage', 'transport')
            
            # 根据阶段调整参数
            if current_stage in ['loading', 'unloading']:
                step_size *= 0.8  # 更精细的步长
                max_attempts += 2  # 更多尝试次数
            elif current_stage == 'parking':
                step_size *= 1.2  # 稍大的步长，更快速
            
            self.stats['multi_stage_optimized_plans'] += 1
        
        # 空间冲突感知调整
        if config.get('spatial_conflict_avoidance', False):
            step_size *= 0.9  # 更密集的采样
            max_attempts += 1
            self.stats['spatial_conflict_aware_plans'] += 1
        
        # 尝试多次生成RS曲线路径
        best_path = None
        best_quality = 0.0
        
        for attempt in range(max_attempts):
            try:
                # 调整转弯半径以获得不同的曲线
                adjusted_radius = config['turning_radius'] * (0.8 + 0.4 * attempt / max_attempts)
                rs_planner.turning_radius = adjusted_radius
                
                path = rs_planner.get_path(start_with_heading, goal_with_heading, step_size)
                
                if path and len(path) >= 2:
                    # 检查路径质量
                    path_quality = self._evaluate_rs_path_quality(path, config)
                    
                    if path_quality >= quality_threshold and path_quality > best_quality:
                        best_path = path
                        best_quality = path_quality
                        
                        # 如果质量足够好，提前返回
                        if path_quality > 0.8:
                            break
            
            except Exception as e:
                print(f"      RS曲线尝试 {attempt+1} 失败: {e}")
                continue
        
        if best_path:
            self.stats['rs_curve_plans'] += 1
            return self._create_enhanced_result_object(
                best_path, 'integrated_rs_curves', len(best_path), task_context=task_context
            )
        
        return None
    
    def _ensure_heading_for_rs(self, point: tuple, reference_point: tuple) -> tuple:
        """为RS曲线确保朝向信息"""
        if len(point) >= 3:
            return point
        
        # 计算朝向reference_point的角度
        dx = reference_point[0] - point[0]
        dy = reference_point[1] - point[1]
        heading = math.atan2(dy, dx)
        
        return (point[0], point[1], heading)
    
    def _evaluate_rs_path_quality(self, path: List[Tuple], config: Dict) -> float:
        """评估RS曲线路径质量"""
        if not path or len(path) < 2:
            return 0.0
        
        # 计算路径长度
        path_length = sum(
            math.sqrt((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)
            for i in range(len(path) - 1)
        )
        
        # 直线距离
        direct_distance = math.sqrt(
            (path[-1][0] - path[0][0])**2 + (path[-1][1] - path[0][1])**2
        )
        
        # 长度效率
        length_efficiency = direct_distance / (path_length + 0.1) if path_length > 0 else 0
        
        # 平滑度（曲率变化）
        smoothness = self._calculate_rs_path_smoothness(path)
        
        # 复杂度检查
        complexity_factor = 1.0
        if path_length > direct_distance * config.get('curve_complexity_limit', 3.0):
            complexity_factor = 0.5  # 过于复杂的路径降低质量
        
        # 整理网络加成
        consolidation_bonus = 0.0
        if config.get('consolidation_aware', False):
            consolidation_bonus = 0.1
        
        # 综合质量分数
        quality = (length_efficiency * 0.4 + 
                  smoothness * 0.4 + 
                  config.get('smoothness_preference', 0.8) * 0.2) * complexity_factor + consolidation_bonus
        
        return min(1.0, max(0.0, quality))
    
    def _calculate_rs_path_smoothness(self, path: List[Tuple]) -> float:
        """计算RS曲线路径平滑度"""
        if len(path) < 3:
            return 1.0
        
        total_angle_change = 0
        for i in range(1, len(path) - 1):
            # 计算转向角变化
            v1 = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
            v2 = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            
            len1 = math.sqrt(v1[0]**2 + v1[1]**2)
            len2 = math.sqrt(v2[0]**2 + v2[1]**2)
            
            if len1 > 1e-6 and len2 > 1e-6:
                cos_angle = (v1[0]*v2[0] + v1[1]*v2[1]) / (len1 * len2)
                cos_angle = max(-1, min(1, cos_angle))
                angle_change = math.acos(cos_angle)
                total_angle_change += angle_change
        
        avg_angle_change = total_angle_change / max(1, len(path) - 2)
        # RS曲线天然比较平滑，使用更宽松的评估
        return math.exp(-avg_angle_change * 0.8)
    
    def _plan_direct_path_enhanced(self, start: tuple, goal: tuple, network_context: Dict = None):
        """增强的直线路径回退"""
        import math
        
        # 简单直线路径，但考虑网络上下文
        distance = math.sqrt((goal[0] - start[0])**2 + (goal[1] - start[1])**2)
        
        # 根据网络拓扑调整步数
        base_steps = max(3, int(distance / 2.0))
        if network_context and network_context.get('is_consolidated', False):
            steps = int(base_steps * 1.1)  # 整理网络稍微密集一些
        else:
            steps = base_steps
        
        path = []
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * (goal[0] - start[0])
            y = start[1] + t * (goal[1] - start[1])
            theta = start[2] if len(start) > 2 else 0
            path.append((x, y, theta))
        
        return self._create_enhanced_result_object(
            path, 'enhanced_direct', len(path), network_context
        )
    
    def _create_enhanced_result_object(self, path: list, planner_used: str, path_length: int,
                                     network_context: Dict = None, task_context: Dict = None):
        """创建增强的结果对象"""
        class IntegratedPlanningResult:
            def __init__(self, path, planner_used, path_length, network_context, task_context):
                self.path = path
                self.planner_used = planner_used
                self.quality_score = self._calculate_integrated_quality_score(planner_used, network_context)
                self.planning_time = 0.0
                self.structure = {
                    'type': planner_used,
                    'total_length': path_length,
                    'planner_used': planner_used,
                    'backbone_utilization': 0.0,
                    'network_topology': network_context.get('topology_type') if network_context else 'original',
                    'consolidation_aware': network_context.get('is_consolidated', False) if network_context else False,
                    'multi_stage_optimized': task_context.get('multi_stage', False) if task_context else False,
                    'spatial_conflict_aware': network_context.get('spatial_conflicts_detected', False) if network_context else False,
                    'rs_curve_used': 'rs_curves' in planner_used  # 新增RS曲线标识
                }
                
                # 任务阶段信息
                if task_context:
                    self.structure.update({
                        'current_stage': task_context.get('current_stage'),
                        'stage_transition': task_context.get('stage_transition', False)
                    })
            
            def _calculate_integrated_quality_score(self, planner_used, network_context):
                """计算集成质量分数"""
                base_scores = {
                    'integrated_hybrid_astar': 0.85,
                    'integrated_rs_curves': 0.80,  # RS曲线基础分数
                    'enhanced_direct': 0.5,
                    'backbone_network': 0.9
                }
                
                base_score = base_scores.get(planner_used, 0.7)
                
                # 网络感知加成
                if network_context and network_context.get('is_consolidated', False):
                    base_score += 0.05  # 整理网络加成
                
                if network_context and network_context.get('hierarchy_level') == 'trunk':
                    base_score += 0.03  # 主干路径加成
                
                # RS曲线特殊加成
                if 'rs_curves' in planner_used:
                    if network_context and network_context.get('spatial_conflicts_detected', False):
                        base_score += 0.08  # RS曲线处理空间冲突有优势
                
                return min(1.0, base_score)
        
        return IntegratedPlanningResult(path, planner_used, path_length, network_context, task_context)
    
    def _extract_task_context(self, kwargs: Dict) -> Dict:
        """提取任务上下文"""
        context = {
            'multi_stage': False,
            'current_stage': kwargs.get('context', 'transport'),
            'stage_transition': False
        }
        
        # 检测多阶段任务
        if ('target_type' in kwargs and kwargs.get('target_type') in ['loading', 'unloading'] or
            'current_stage' in kwargs):
            context['multi_stage'] = True
            context['current_stage'] = kwargs.get('current_stage', kwargs.get('context', 'transport'))
        
        # 检测阶段转换
        if kwargs.get('next_stage'):
            context['stage_transition'] = True
            context['next_stage'] = kwargs['next_stage']
        
        # 目标信息
        context['target_type'] = kwargs.get('target_type')
        context['target_id'] = kwargs.get('target_id')
        
        return context
    
    def _extract_network_context(self, kwargs: Dict) -> Dict:
        """提取网络上下文"""
        context = {
            'topology_type': self.current_network_topology.value,
            'is_consolidated': self.current_network_topology != NetworkTopologyType.ORIGINAL,
            'spatial_conflicts_detected': False,
            'alternative_paths_available': 0
        }
        
        # 从骨干网络获取更详细信息
        if self.backbone_network and hasattr(self.backbone_network, 'get_improved_consolidation_info'):
            consolidation_info = self.backbone_network.get_improved_consolidation_info()
            context.update({
                'consolidation_info': consolidation_info,
                'hierarchy_level': 'trunk'  # 默认值，实际应从路径获取
            })
        
        # 从交通管理器获取冲突信息
        if self.traffic_manager and hasattr(self.traffic_manager, 'get_system_status'):
            traffic_status = self.traffic_manager.get_system_status()
            context.update({
                'spatial_conflicts_detected': traffic_status.get('spatial_conflicts_detected', False),
                'active_conflicts': traffic_status.get('active_conflicts', 0)
            })
        
        return context
    
    def get_performance_statistics(self) -> dict:
        """获取性能统计 - 增强版"""
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
        
        # 集成优化统计
        stats.update({
            'network_topology_adaptations': self.stats['network_topology_adaptations'],
            'consolidation_aware_plans': self.stats['consolidation_aware_plans'],
            'spatial_conflict_aware_plans': self.stats['spatial_conflict_aware_plans'],
            'multi_stage_optimized_plans': self.stats['multi_stage_optimized_plans'],
            'hierarchy_aware_plans': self.stats['hierarchy_aware_plans'],
            'rs_curve_plans': self.stats['rs_curve_plans'],  # RS曲线统计
            'config_adaptations': self.stats['config_adaptations'],
            'current_network_topology': self.current_network_topology.value
        })
        
        return stats
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
        
        # 立即更新网络拓扑感知
        self.update_network_topology_awareness()
        
        for planner in self.planners.values():
            if hasattr(planner, 'set_backbone_network'):
                planner.set_backbone_network(backbone_network)
    
    def set_traffic_manager(self, traffic_manager):
        """设置交通管理器"""
        self.traffic_manager = traffic_manager
        print("✅ 交通管理器已连接到规划器")
    
    def adapt_to_conflict_feedback(self, conflict_info: Dict):
        """根据冲突反馈适应配置"""
        self.config_manager.adapt_to_conflict_detection_results(conflict_info)
        self.stats['config_adaptations'] += 1
    
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
        
        self.stats['cache_hits'] = 0
        print("集成优化规划器缓存已清理")
    
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
        print("集成优化路径规划器已关闭")
    def get_professional_road_aware_config(self, road_class: str = 'secondary',
                                        engineering_context: Dict = None) -> Dict[str, Any]:
        """获取专业道路感知的配置"""
        base_config = self.get_astar_config('standard_consolidated')
        
        # 应用道路等级调整
        if road_class in self.road_class_adjustments:
            adjustments = self.road_class_adjustments[road_class]
            
            # 质量阈值调整
            quality_boost = adjustments.get('quality_threshold_boost', 0.0)
            base_config['quality_threshold'] = base_config.get('quality_threshold', 0.6) + quality_boost
            
            # 安全边距调整
            safety_multiplier = adjustments.get('safety_margin_multiplier', 1.0)
            base_config['turning_radius'] = base_config.get('turning_radius', 8.0) * safety_multiplier
            
            # 超时调整
            timeout_extension = adjustments.get('timeout_extension', 1.0)
            base_config['timeout'] = base_config.get('timeout', 18.0) * timeout_extension
            
            # 专业道路标记
            base_config['road_class_aware'] = True
            base_config['target_road_class'] = road_class
            base_config['engineering_optimized'] = True
        
        # 工程上下文调整
        if engineering_context:
            if engineering_context.get('high_safety_required', False):
                base_config['quality_threshold'] *= 1.1
                base_config['turning_radius'] *= 1.2
            
            if engineering_context.get('cost_sensitive', False):
                base_config['max_iterations'] = int(base_config.get('max_iterations', 20000) * 0.8)
                base_config['timeout'] *= 0.9
        
        return base_config

    # ==================== 便捷创建函数 ====================

    def create_integrated_planning_system(env, backbone_network=None, traffic_manager=None):
        """创建集成规划系统"""
        config_manager = IntegratedPlannerConfig()
        planner = IntegratedPathPlannerWithOptimizedConfig(env, backbone_network, traffic_manager)
        
        return {
            'config_manager': config_manager,
            'planner': planner
        }

    def apply_network_optimization_preset(config_manager: IntegratedPlannerConfig, preset: str = 'balanced'):
        """应用网络优化预设"""
        presets = {
            'performance': {
                'max_planning_time': 20.0,
                'quality_threshold': 0.5,
                'alternative_path_exploration_depth': 2,
                'safety_margin_multiplier': 1.1
            },
            'balanced': {
                'max_planning_time': 25.0,
                'quality_threshold': 0.55,
                'alternative_path_exploration_depth': 3,
                'safety_margin_multiplier': 1.2
            },
            'quality': {
                'max_planning_time': 30.0,
                'quality_threshold': 0.65,
                'alternative_path_exploration_depth': 4,
                'safety_margin_multiplier': 1.3
            }
        }
        
        if preset in presets:
            preset_config = presets[preset]
            config_manager.base_config.update(preset_config)
            config_manager.network_aware_config.alternative_path_exploration_depth = preset_config['alternative_path_exploration_depth']
            config_manager.conflict_aware_config.safety_margin_multiplier = preset_config['safety_margin_multiplier']
            print(f"✅ 已应用网络优化预设: {preset}")
        else:
            print(f"❌ 未知预设: {preset}")


# 兼容性别名
EnhancedPathPlannerWithConfig = IntegratedPathPlannerWithOptimizedConfig
OptimizedPlannerConfig = IntegratedPlannerConfig