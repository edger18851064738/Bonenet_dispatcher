"""
Clothoid-Cubic.py - 车辆动力学感知的曲线拟合模块（修正版）
重要原则：
1. 只生成连接关键节点的平滑曲线，不创建新节点
2. 强制障碍物避让，确保路径安全
"""

import math
import numpy as np
from typing import List, Tuple, Dict, Optional, Any
from dataclasses import dataclass
from enum import Enum
import scipy.interpolate as spi
from scipy.optimize import minimize_scalar, minimize
import warnings

warnings.filterwarnings('ignore', category=RuntimeWarning)

class PathType(Enum):
    """路径类型"""
    CURVE_ONLY = "curve_only"      # 仅曲线，无节点
    WAYPOINTS = "waypoints"         # 路径点（用于显示）

@dataclass
class CurveSegment:
    """曲线段"""
    start_node_id: str              # 起始关键节点ID
    end_node_id: str                # 终止关键节点ID
    curve_points: List[Tuple]       # 曲线上的采样点（仅用于显示和碰撞检测）
    curve_type: str                 # 曲线类型
    is_collision_free: bool         # 是否无碰撞

class ClothoidCubicFitter:
    """Clothoid-Cubic曲线拟合器 - 修正版"""
    
    def __init__(self, vehicle_params: Optional[Dict] = None, env=None):
        """
        初始化拟合器
        
        Args:
            vehicle_params: 车辆参数字典
            env: 环境对象（必需，用于障碍物检测）
        """
        # 车辆参数
        self.vehicle_length = vehicle_params.get('length', 6.0) if vehicle_params else 6.0
        self.vehicle_width = vehicle_params.get('width', 3.0) if vehicle_params else 3.0
        self.turning_radius = vehicle_params.get('turning_radius', 8.0) if vehicle_params else 8.0
        self.max_curvature = 1.0 / self.turning_radius
        
        self.env = env
        if not env:
            print("⚠️ 警告：未提供环境对象，无法进行障碍物检测")
        
        # 拟合配置
        self.config = {
            'sample_resolution': 0.5,        # 采样分辨率(米) - 仅用于碰撞检测
            'collision_check_step': 1.0,     # 碰撞检测步长
            'safety_margin': 1.5,            # 安全边距
            'max_deviation': 15.0,           # 最大允许偏离直线距离
            'obstacle_avoidance_weight': 0.8,# 避障权重
            'smoothness_weight': 0.2,        # 平滑性权重
        }
        
        # 道路等级配置
        self.road_configs = {
            'primary': {
                'curve_tension': 0.3,        # 曲线张力（越小越平滑）
                'safety_margin_factor': 1.5,  # 安全边距系数
                'prefer_straight': True,
            },
            'secondary': {
                'curve_tension': 0.5,
                'safety_margin_factor': 1.2,
                'prefer_straight': False,
            },
            'service': {
                'curve_tension': 0.7,
                'safety_margin_factor': 1.0,
                'prefer_straight': False,
            }
        }
        
        print("✅ Clothoid-Cubic曲线拟合器初始化完成（修正版）")
    
    def fit_path_between_nodes(self, key_nodes: List[Tuple], 
                               key_node_ids: List[str] = None,
                               road_class: str = 'secondary',
                               **kwargs) -> List[CurveSegment]:
        """
        在关键节点之间拟合曲线（不创建新节点）
        
        Args:
            key_nodes: 关键节点位置列表 [(x, y, theta), ...]
            key_node_ids: 关键节点ID列表（可选）
            road_class: 道路等级
            
        Returns:
            曲线段列表，每段连接两个相邻的关键节点
        """
        if len(key_nodes) < 2:
            print("❌ 关键节点数量不足")
            return []
        
        # 如果没有提供节点ID，自动生成
        if not key_node_ids:
            key_node_ids = [f"node_{i}" for i in range(len(key_nodes))]
        
        # 获取道路配置
        road_config = self.road_configs.get(road_class, self.road_configs['secondary'])
        
        # 拟合每一段
        curve_segments = []
        for i in range(len(key_nodes) - 1):
            segment = self._fit_segment_with_obstacle_avoidance(
                start_pos=key_nodes[i],
                end_pos=key_nodes[i + 1],
                start_id=key_node_ids[i],
                end_id=key_node_ids[i + 1],
                road_config=road_config
            )
            
            if segment:
                curve_segments.append(segment)
            else:
                # 如果拟合失败，创建直线段作为备选
                print(f"⚠️ 段 {key_node_ids[i]}-{key_node_ids[i+1]} 拟合失败，使用直线")
                fallback_segment = self._create_straight_segment(
                    key_nodes[i], key_nodes[i+1], 
                    key_node_ids[i], key_node_ids[i+1]
                )
                curve_segments.append(fallback_segment)
        
        return curve_segments
    
    def _fit_segment_with_obstacle_avoidance(self, start_pos: Tuple, end_pos: Tuple,
                                            start_id: str, end_id: str,
                                            road_config: Dict) -> Optional[CurveSegment]:
        """拟合单个段，确保避开障碍物"""
        distance = math.sqrt((end_pos[0] - start_pos[0])**2 + 
                           (end_pos[1] - start_pos[1])**2)
        
        # 首先尝试直接曲线连接
        initial_curve = self._generate_initial_curve(start_pos, end_pos, road_config)
        
        # 检查碰撞
        if self._is_curve_collision_free(initial_curve):
            return CurveSegment(
                start_node_id=start_id,
                end_node_id=end_id,
                curve_points=initial_curve,
                curve_type="direct_curve",
                is_collision_free=True
            )
        
        # 如果有碰撞，尝试避障曲线
        print(f"  检测到碰撞，尝试避障路径: {start_id} -> {end_id}")
        
        # 方法1：中点偏移法
        offset_curve = self._try_offset_curve(start_pos, end_pos, road_config)
        if offset_curve and self._is_curve_collision_free(offset_curve):
            return CurveSegment(
                start_node_id=start_id,
                end_node_id=end_id,
                curve_points=offset_curve,
                curve_type="offset_curve",
                is_collision_free=True
            )
        
        # 方法2：多控制点贝塞尔曲线
        bezier_curve = self._try_bezier_avoidance(start_pos, end_pos, road_config)
        if bezier_curve and self._is_curve_collision_free(bezier_curve):
            return CurveSegment(
                start_node_id=start_id,
                end_node_id=end_id,
                curve_points=bezier_curve,
                curve_type="bezier_avoidance",
                is_collision_free=True
            )
        
        # 方法3：分段避障
        segmented_curve = self._try_segmented_avoidance(start_pos, end_pos, road_config)
        if segmented_curve and self._is_curve_collision_free(segmented_curve):
            return CurveSegment(
                start_node_id=start_id,
                end_node_id=end_id,
                curve_points=segmented_curve,
                curve_type="segmented_avoidance",
                is_collision_free=True
            )
        
        # 所有方法都失败
        print(f"  ❌ 无法找到无碰撞路径: {start_id} -> {end_id}")
        return None
    
    def _generate_initial_curve(self, start: Tuple, end: Tuple, 
                               road_config: Dict) -> List[Tuple]:
        """生成初始曲线（三次样条）"""
        # 使用Hermite插值生成平滑曲线
        distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        
        # 估算切线
        start_tangent = self._estimate_tangent(start, end, True)
        end_tangent = self._estimate_tangent(start, end, False)
        
        # 调整张力
        tension = road_config['curve_tension'] * distance
        
        # 生成Hermite曲线点（仅用于碰撞检测和显示）
        num_samples = max(5, int(distance / self.config['sample_resolution']))
        curve_points = []
        
        for i in range(num_samples):
            t = i / (num_samples - 1)
            
            # Hermite基函数
            h00 = 2*t**3 - 3*t**2 + 1
            h10 = t**3 - 2*t**2 + t
            h01 = -2*t**3 + 3*t**2
            h11 = t**3 - t**2
            
            # 计算位置
            x = (h00 * start[0] + h10 * tension * start_tangent[0] + 
                 h01 * end[0] + h11 * tension * end_tangent[0])
            y = (h00 * start[1] + h10 * tension * start_tangent[1] + 
                 h01 * end[1] + h11 * tension * end_tangent[1])
            
            # 计算朝向
            if i < num_samples - 1:
                t_next = (i + 1) / (num_samples - 1)
                dx = ((6*t_next**2 - 6*t_next) * start[0] + 
                      (3*t_next**2 - 4*t_next + 1) * tension * start_tangent[0] +
                      (-6*t_next**2 + 6*t_next) * end[0] + 
                      (3*t_next**2 - 2*t_next) * tension * end_tangent[0])
                dy = ((6*t_next**2 - 6*t_next) * start[1] + 
                      (3*t_next**2 - 4*t_next + 1) * tension * start_tangent[1] +
                      (-6*t_next**2 + 6*t_next) * end[1] + 
                      (3*t_next**2 - 2*t_next) * tension * end_tangent[1])
                theta = math.atan2(dy, dx)
            else:
                theta = end[2] if len(end) > 2 else math.atan2(end[1] - start[1], end[0] - start[0])
            
            curve_points.append((x, y, theta))
        
        return curve_points
    
    def _is_curve_collision_free(self, curve_points: List[Tuple]) -> bool:
        """检查曲线是否无碰撞"""
        if not self.env or not hasattr(self.env, 'grid'):
            # 无环境信息，假设无碰撞
            return True
        
        # 安全边距
        safety_margin = self.config['safety_margin']
        half_width = self.vehicle_width / 2 + safety_margin
        half_length = self.vehicle_length / 2 + safety_margin
        
        # 检查曲线上的采样点
        check_step = max(1, int(len(curve_points) / 20))  # 最多检查20个点
        
        for i in range(0, len(curve_points), check_step):
            point = curve_points[i]
            x, y = point[0], point[1]
            theta = point[2] if len(point) > 2 else 0
            
            # 检查车辆占据的矩形区域
            if not self._check_rectangle_collision_free(x, y, theta, half_length, half_width):
                return False
        
        # 检查最后一个点
        if len(curve_points) > 1:
            last_point = curve_points[-1]
            if not self._check_rectangle_collision_free(
                last_point[0], last_point[1], 
                last_point[2] if len(last_point) > 2 else 0,
                half_length, half_width
            ):
                return False
        
        return True
    
    def _check_rectangle_collision_free(self, cx: float, cy: float, theta: float,
                                      half_length: float, half_width: float) -> bool:
        """检查矩形区域是否无碰撞"""
        # 矩形的四个角
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        
        corners = [
            (cx + half_length * cos_theta - half_width * sin_theta,
             cy + half_length * sin_theta + half_width * cos_theta),
            (cx + half_length * cos_theta + half_width * sin_theta,
             cy + half_length * sin_theta - half_width * cos_theta),
            (cx - half_length * cos_theta + half_width * sin_theta,
             cy - half_length * sin_theta - half_width * cos_theta),
            (cx - half_length * cos_theta - half_width * sin_theta,
             cy - half_length * sin_theta + half_width * cos_theta)
        ]
        
        # 检查矩形边界和内部
        min_x = int(min(c[0] for c in corners))
        max_x = int(max(c[0] for c in corners)) + 1
        min_y = int(min(c[1] for c in corners))
        max_y = int(max(c[1] for c in corners)) + 1
        
        # 确保在地图范围内
        min_x = max(0, min_x)
        max_x = min(self.env.width, max_x)
        min_y = max(0, min_y)
        max_y = min(self.env.height, max_y)
        
        # 检查网格
        for grid_x in range(min_x, max_x):
            for grid_y in range(min_y, max_y):
                if self.env.grid[grid_x, grid_y] == 1:  # 障碍物
                    # 检查该网格点是否在旋转的矩形内
                    if self._point_in_rotated_rectangle(
                        grid_x, grid_y, cx, cy, theta, half_length, half_width
                    ):
                        return False
        
        return True
    
    def _point_in_rotated_rectangle(self, px: float, py: float, 
                                   cx: float, cy: float, theta: float,
                                   half_length: float, half_width: float) -> bool:
        """检查点是否在旋转的矩形内"""
        # 将点转换到矩形的局部坐标系
        dx = px - cx
        dy = py - cy
        
        cos_theta = math.cos(-theta)
        sin_theta = math.sin(-theta)
        
        local_x = dx * cos_theta - dy * sin_theta
        local_y = dx * sin_theta + dy * cos_theta
        
        return (abs(local_x) <= half_length and abs(local_y) <= half_width)
    
    def _try_offset_curve(self, start: Tuple, end: Tuple, 
                         road_config: Dict) -> Optional[List[Tuple]]:
        """尝试偏移曲线避障"""
        # 计算垂直于直线的偏移方向
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 1e-6:
            return None
        
        # 归一化垂直向量
        perp_x = -dy / distance
        perp_y = dx / distance
        
        # 尝试两个方向的偏移
        for sign in [1, -1]:
            for offset_factor in [0.3, 0.5, 0.7]:
                offset = sign * offset_factor * distance * 0.3
                
                # 创建控制点
                mid_x = (start[0] + end[0]) / 2 + perp_x * offset
                mid_y = (start[1] + end[1]) / 2 + perp_y * offset
                
                # 生成二次贝塞尔曲线
                curve = self._generate_quadratic_bezier(
                    start, (mid_x, mid_y), end, road_config
                )
                
                if curve and self._is_curve_collision_free(curve):
                    return curve
        
        return None
    
    def _try_bezier_avoidance(self, start: Tuple, end: Tuple,
                             road_config: Dict) -> Optional[List[Tuple]]:
        """使用贝塞尔曲线避障"""
        # 在障碍物周围寻找可行的控制点
        control_points = self._find_feasible_control_points(start, end)
        
        if not control_points:
            return None
        
        # 尝试每个控制点
        for cp in control_points:
            curve = self._generate_cubic_bezier(start, cp[0], cp[1], end, road_config)
            if curve and self._is_curve_collision_free(curve):
                return curve
        
        return None
    
    def _try_segmented_avoidance(self, start: Tuple, end: Tuple,
                                road_config: Dict) -> Optional[List[Tuple]]:
        """分段避障"""
        # 找到中间的可行点
        mid_point = self._find_feasible_midpoint(start, end)
        
        if not mid_point:
            return None
        
        # 递归拟合两段
        curve1 = self._generate_initial_curve(start, mid_point, road_config)
        curve2 = self._generate_initial_curve(mid_point, end, road_config)
        
        if (curve1 and curve2 and 
            self._is_curve_collision_free(curve1) and 
            self._is_curve_collision_free(curve2)):
            # 合并两段（避免重复点）
            return curve1[:-1] + curve2
        
        return None
    
    def _find_feasible_control_points(self, start: Tuple, end: Tuple) -> List[Tuple]:
        """寻找可行的贝塞尔控制点"""
        control_points = []
        
        # 在起点和终点之间的区域搜索
        min_x = min(start[0], end[0]) - 10
        max_x = max(start[0], end[0]) + 10
        min_y = min(start[1], end[1]) - 10
        max_y = max(start[1], end[1]) + 10
        
        # 网格搜索
        for x in np.linspace(min_x, max_x, 5):
            for y in np.linspace(min_y, max_y, 5):
                if self._is_point_feasible(x, y):
                    # 生成两个控制点
                    cp1 = (x, y)
                    cp2_x = (x + end[0]) / 2
                    cp2_y = (y + end[1]) / 2
                    
                    if self._is_point_feasible(cp2_x, cp2_y):
                        control_points.append((cp1, (cp2_x, cp2_y)))
        
        return control_points
    
    def _find_feasible_midpoint(self, start: Tuple, end: Tuple) -> Optional[Tuple]:
        """寻找可行的中点"""
        # 尝试多个候选中点
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        
        for t in [0.5, 0.4, 0.6, 0.3, 0.7]:
            mid_x = start[0] + t * dx
            mid_y = start[1] + t * dy
            
            # 尝试不同的偏移
            for offset in [0, 5, -5, 10, -10]:
                test_x = mid_x + offset * (-dy) / math.sqrt(dx*dx + dy*dy + 1e-6)
                test_y = mid_y + offset * dx / math.sqrt(dx*dx + dy*dy + 1e-6)
                
                if self._is_point_feasible(test_x, test_y, margin=5):
                    # 计算朝向
                    theta1 = math.atan2(test_y - start[1], test_x - start[0])
                    theta2 = math.atan2(end[1] - test_y, end[0] - test_x)
                    avg_theta = (theta1 + theta2) / 2
                    
                    return (test_x, test_y, avg_theta)
        
        return None
    
    def _is_point_feasible(self, x: float, y: float, margin: float = None) -> bool:
        """检查点是否可行（无碰撞）"""
        if not self.env or not hasattr(self.env, 'grid'):
            return True
        
        if margin is None:
            margin = self.config['safety_margin']
        
        # 检查点周围的区域
        min_x = max(0, int(x - margin))
        max_x = min(self.env.width - 1, int(x + margin))
        min_y = max(0, int(y - margin))
        max_y = min(self.env.height - 1, int(y + margin))
        
        for gx in range(min_x, max_x + 1):
            for gy in range(min_y, max_y + 1):
                if self.env.grid[gx, gy] == 1:
                    dist = math.sqrt((gx - x)**2 + (gy - y)**2)
                    if dist < margin:
                        return False
        
        return True
    
    def _generate_quadratic_bezier(self, p0: Tuple, p1: Tuple, p2: Tuple,
                                  road_config: Dict) -> List[Tuple]:
        """生成二次贝塞尔曲线"""
        distance = (math.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2) +
                   math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2))
        
        num_samples = max(5, int(distance / self.config['sample_resolution']))
        curve_points = []
        
        for i in range(num_samples):
            t = i / (num_samples - 1)
            
            # 二次贝塞尔公式
            x = (1-t)**2 * p0[0] + 2*(1-t)*t * p1[0] + t**2 * p2[0]
            y = (1-t)**2 * p0[1] + 2*(1-t)*t * p1[1] + t**2 * p2[1]
            
            # 计算切线方向
            if i < num_samples - 1:
                t_next = (i + 1) / (num_samples - 1)
                x_next = (1-t_next)**2 * p0[0] + 2*(1-t_next)*t_next * p1[0] + t_next**2 * p2[0]
                y_next = (1-t_next)**2 * p0[1] + 2*(1-t_next)*t_next * p1[1] + t_next**2 * p2[1]
                theta = math.atan2(y_next - y, x_next - x)
            else:
                theta = p2[2] if len(p2) > 2 else math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            
            curve_points.append((x, y, theta))
        
        return curve_points
    
    def _generate_cubic_bezier(self, p0: Tuple, p1: Tuple, p2: Tuple, p3: Tuple,
                              road_config: Dict) -> List[Tuple]:
        """生成三次贝塞尔曲线"""
        distance = (math.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2) +
                   math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2) +
                   math.sqrt((p3[0] - p2[0])**2 + (p3[1] - p2[1])**2))
        
        num_samples = max(5, int(distance / self.config['sample_resolution']))
        curve_points = []
        
        for i in range(num_samples):
            t = i / (num_samples - 1)
            
            # 三次贝塞尔公式
            x = ((1-t)**3 * p0[0] + 3*(1-t)**2*t * p1[0] + 
                 3*(1-t)*t**2 * p2[0] + t**3 * p3[0])
            y = ((1-t)**3 * p0[1] + 3*(1-t)**2*t * p1[1] + 
                 3*(1-t)*t**2 * p2[1] + t**3 * p3[1])
            
            # 计算切线方向
            if i < num_samples - 1:
                dt = 1 / (num_samples - 1)
                dx = (-3*(1-t)**2 * p0[0] + 3*(1-t)**2 * p1[0] - 6*(1-t)*t * p1[0] + 
                      6*(1-t)*t * p2[0] - 3*t**2 * p2[0] + 3*t**2 * p3[0])
                dy = (-3*(1-t)**2 * p0[1] + 3*(1-t)**2 * p1[1] - 6*(1-t)*t * p1[1] + 
                      6*(1-t)*t * p2[1] - 3*t**2 * p2[1] + 3*t**2 * p3[1])
                theta = math.atan2(dy, dx)
            else:
                theta = p3[2] if len(p3) > 2 else math.atan2(p3[1] - p2[1], p3[0] - p2[0])
            
            curve_points.append((x, y, theta))
        
        return curve_points
    
    def _create_straight_segment(self, start: Tuple, end: Tuple,
                               start_id: str, end_id: str) -> CurveSegment:
        """创建直线段作为备选"""
        distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        num_samples = max(2, int(distance / self.config['sample_resolution']))
        
        curve_points = []
        for i in range(num_samples):
            t = i / (num_samples - 1)
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            theta = math.atan2(end[1] - start[1], end[0] - start[0])
            curve_points.append((x, y, theta))
        
        return CurveSegment(
            start_node_id=start_id,
            end_node_id=end_id,
            curve_points=curve_points,
            curve_type="straight_fallback",
            is_collision_free=self._is_curve_collision_free(curve_points)
        )
    
    def _estimate_tangent(self, start: Tuple, end: Tuple, is_start: bool) -> Tuple[float, float]:
        """估算切线方向"""
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist < 1e-6:
            return (1.0, 0.0)
        
        if is_start:
            # 起点切线：考虑初始朝向
            if len(start) > 2:
                return (math.cos(start[2]), math.sin(start[2]))
            else:
                return (dx/dist, dy/dist)
        else:
            # 终点切线：考虑最终朝向
            if len(end) > 2:
                return (math.cos(end[2]), math.sin(end[2]))
            else:
                return (dx/dist, dy/dist)
    
    def convert_to_path_format(self, curve_segments: List[CurveSegment]) -> List[Tuple]:
        """
        将曲线段转换为路径格式（用于兼容）
        注意：这只是为了显示，实际系统应该使用节点ID
        """
        complete_path = []
        
        for i, segment in enumerate(curve_segments):
            if i > 0:
                # 跳过重复的起点
                complete_path.extend(segment.curve_points[1:])
            else:
                complete_path.extend(segment.curve_points)
        
        return complete_path


# ========== 与node_clustering_professional_consolidator集成的接口 ==========

class BackbonePathFitter:
    """骨干路径拟合器 - 修正版"""
    
    def __init__(self, env=None):
        self.fitter = ClothoidCubicFitter(env=env)
        self.env = env
    
    def reconstruct_path_with_curve_fitting(self, key_node_positions: List[Tuple],
                                          key_node_ids: List[str] = None,
                                          road_class: str = 'secondary',
                                          path_quality: float = 0.7) -> Optional[List[Tuple]]:
        """
        重建路径 - 使用曲线拟合连接关键节点
        
        Returns:
            拟合后的完整路径（仅用于显示和碰撞检测）
        """
        if len(key_node_positions) < 2:
            return None
        
        # 执行拟合
        curve_segments = self.fitter.fit_path_between_nodes(
            key_node_positions, 
            key_node_ids,
            road_class
        )
        
        if not curve_segments:
            return None
        
        # 检查是否所有段都无碰撞
        all_collision_free = all(seg.is_collision_free for seg in curve_segments)
        
        if not all_collision_free:
            print(f"⚠️ 路径包含碰撞段，可能需要重新规划")
        
        # 转换为路径格式（用于显示）
        display_path = self.fitter.convert_to_path_format(curve_segments)
        
        return display_path
    
    def get_curve_segments(self, key_node_positions: List[Tuple],
                          key_node_ids: List[str] = None,
                          road_class: str = 'secondary') -> List[CurveSegment]:
        """
        获取曲线段信息（推荐使用）
        
        Returns:
            曲线段列表，每段明确标识连接的节点
        """
        return self.fitter.fit_path_between_nodes(
            key_node_positions,
            key_node_ids,
            road_class
        )


if __name__ == "__main__":
    # 测试代码
    print("Clothoid-Cubic曲线拟合模块测试（修正版）")
    
    # 创建模拟环境
    class MockEnv:
        def __init__(self):
            self.width = 100
            self.height = 100
            self.grid = np.zeros((100, 100))
            # 添加一些障碍物
            self.grid[40:60, 20:30] = 1  # 矩形障碍
            self.grid[30:35, 50:70] = 1  # 横向障碍
    
    env = MockEnv()
    
    # 测试数据
    test_nodes = [
        (10, 10, 0),
        (50, 25, math.pi/4),  # 这个点可能会与障碍物冲突
        (80, 60, math.pi/3),
        (90, 90, 0)
    ]
    
    test_node_ids = ["node_A", "node_B", "node_C", "node_D"]
    
    # 创建拟合器
    fitter = BackbonePathFitter(env)
    
    # 测试拟合
    print("\n测试曲线拟合（避障）:")
    segments = fitter.get_curve_segments(test_nodes, test_node_ids, 'secondary')
    
    for seg in segments:
        print(f"\n段 {seg.start_node_id} -> {seg.end_node_id}:")
        print(f"  类型: {seg.curve_type}")
        print(f"  采样点数: {len(seg.curve_points)}")
        print(f"  无碰撞: {seg.is_collision_free}")