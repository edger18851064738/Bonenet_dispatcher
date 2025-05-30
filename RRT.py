"""
RRT.py - 精简优化版双向RRT路径规划器
设计理念：先完成空间路径规划，再进行车辆动力学优化
"""

import math
import random
import numpy as np
import time
from typing import List, Tuple, Dict, Optional
from collections import OrderedDict
from dataclasses import dataclass

@dataclass
class RRTNode:
    """RRT节点 - 简化版"""
    x: float
    y: float
    theta: float
    parent: Optional['RRTNode'] = None
    cost: float = 0.0
    
    def distance_to(self, other: 'RRTNode') -> float:
        """计算到另一个节点的欧几里得距离"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

class VehicleDynamicsOptimizer:
    """车辆动力学优化器 - 核心创新"""
    
    def __init__(self, vehicle_length=5.0, vehicle_width=2.0, turning_radius=5.0):
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width  
        self.turning_radius = turning_radius
        self.wheel_base = vehicle_length * 0.6
        
        # 动力学约束参数
        self.max_steering_angle = math.pi / 4  # 45度最大转向
        self.max_acceleration = 2.0  # m/s²
        self.max_speed = 10.0  # m/s
        self.comfort_acceleration = 1.0  # 舒适加速度
        
        print(f"初始化车辆动力学优化器: 车长{vehicle_length}m, 转弯半径{turning_radius}m")
    
    def optimize_path_dynamics(self, raw_path: List[Tuple], target_speed=2.0) -> List[Tuple]:
        """
        对RRT原始路径进行车辆动力学优化
        输入：[(x, y, theta), ...] - RRT生成的几何路径
        输出：[(x, y, theta, v, t), ...] - 优化后的动力学轨迹
        """
        if not raw_path or len(raw_path) < 2:
            return raw_path
        
        print(f"开始车辆动力学优化: 输入路径{len(raw_path)}个点")
        
        # 阶段1: 曲率平滑 - 确保转弯半径可行
        smoothed_path = self._smooth_curvature(raw_path)
        
        # 阶段2: 轨迹细化 - 按动力学约束插值
        refined_path = self._refine_trajectory(smoothed_path)
        
        # 阶段3: 速度规划 - 基于曲率和加速度约束
        velocity_path = self._plan_velocity_profile(refined_path, target_speed)
        
        # 阶段4: 时间标定 - 计算到达时间
        timed_trajectory = self._add_time_stamps(velocity_path)
        
        print(f"动力学优化完成: 输出轨迹{len(timed_trajectory)}个点")
        return timed_trajectory
    
    def _smooth_curvature(self, path: List[Tuple]) -> List[Tuple]:
        """阶段1: 曲率平滑 - 处理尖锐转弯"""
        if len(path) < 3:
            return path
        
        smoothed = [path[0]]
        
        for i in range(1, len(path) - 1):
            prev_point = path[i-1]
            curr_point = path[i]
            next_point = path[i+1]
            
            # 计算转弯曲率
            curvature = self._calculate_curvature(prev_point, curr_point, next_point)
            min_radius = 1.0 / (curvature + 0.001)
            
            # 如果转弯半径小于车辆最小转弯半径，进行平滑
            if min_radius < self.turning_radius:
                smoothed_point = self._smooth_sharp_turn(prev_point, curr_point, next_point)
                smoothed.append(smoothed_point)
            else:
                smoothed.append(curr_point)
        
        smoothed.append(path[-1])
        return smoothed
    
    def _calculate_curvature(self, p1: Tuple, p2: Tuple, p3: Tuple) -> float:
        """计算三点间的曲率"""
        # 将点转换为向量
        v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
        
        # 计算向量长度
        len1 = np.linalg.norm(v1)
        len2 = np.linalg.norm(v2)
        
        if len1 < 0.001 or len2 < 0.001:
            return 0.0
        
        # 计算角度变化
        cos_angle = np.dot(v1, v2) / (len1 * len2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle_change = math.acos(cos_angle)
        
        # 曲率 = 角度变化 / 平均段长
        avg_length = (len1 + len2) / 2
        return angle_change / (avg_length + 0.001)
    
    def _smooth_sharp_turn(self, p1: Tuple, p2: Tuple, p3: Tuple) -> Tuple:
        """平滑尖锐转弯点"""
        # 使用加权平均进行平滑，保持朝向
        weight = 0.3  # 平滑强度
        
        x = p2[0] * (1 - weight) + (p1[0] + p3[0]) * weight / 2
        y = p2[1] * (1 - weight) + (p1[1] + p3[1]) * weight / 2
        
        # 重新计算朝向
        theta = math.atan2(p3[1] - p1[1], p3[0] - p1[0])
        
        return (x, y, theta)
    
    def _refine_trajectory(self, path: List[Tuple]) -> List[Tuple]:
        """阶段2: 轨迹细化 - 按车辆运动学插值"""
        if len(path) < 2:
            return path
        
        refined = [path[0]]
        
        for i in range(len(path) - 1):
            start_point = path[i]
            end_point = path[i + 1]
            
            # 计算段长度
            segment_length = math.sqrt(
                (end_point[0] - start_point[0])**2 + 
                (end_point[1] - start_point[1])**2
            )
            
            # 根据曲率确定插值密度
            if i < len(path) - 2:
                curvature = self._calculate_curvature(
                    path[max(0, i-1)] if i > 0 else path[i],
                    path[i], path[i+1]
                )
                # 高曲率区域需要更密集的点
                density_factor = 1 + curvature * 3
            else:
                density_factor = 1
            
            # 计算插值点数量
            target_spacing = 1.0 / density_factor  # 基础间距1米
            num_interpolations = max(1, int(segment_length / target_spacing))
            
            # 使用车辆运动学模型插值
            interpolated_points = self._kinematic_interpolation(
                start_point, end_point, num_interpolations
            )
            
            refined.extend(interpolated_points)
        
        refined.append(path[-1])
        return refined
    
    def _kinematic_interpolation(self, start: Tuple, end: Tuple, num_points: int) -> List[Tuple]:
        """使用车辆运动学模型进行插值"""
        if num_points <= 1:
            return []
        
        points = []
        
        for i in range(1, num_points):
            t = i / num_points
            
            # 计算期望的转向角
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.001:
                continue
            
            # 使用Ackermann转向几何
            target_heading = math.atan2(dy, dx)
            heading_change = target_heading - start[2]
            
            # 限制转向角
            max_heading_change = self.max_steering_angle * t
            heading_change = np.clip(heading_change, -max_heading_change, max_heading_change)
            
            # 计算新位置
            current_heading = start[2] + heading_change * t
            
            x = start[0] + t * distance * math.cos(current_heading)
            y = start[1] + t * distance * math.sin(current_heading)
            
            points.append((x, y, current_heading))
        
        return points
    
    def _plan_velocity_profile(self, path: List[Tuple], target_speed=2.0) -> List[Tuple]:
        """阶段3: 速度规划 - 基于曲率约束"""
        if len(path) < 2:
            return [(p[0], p[1], p[2], target_speed) for p in path]
        
        velocity_path = []
        
        for i, point in enumerate(path):
            # 计算当前点的曲率
            if i >= 1 and i < len(path) - 1:
                curvature = self._calculate_curvature(path[i-1], path[i], path[i+1])
            else:
                curvature = 0.0
            
            # 基于曲率限制速度 (向心力约束)
            if curvature > 0.001:
                # v = sqrt(a_lateral_max / curvature)
                max_safe_speed = math.sqrt(self.comfort_acceleration / curvature)
                safe_speed = min(target_speed, max_safe_speed, self.max_speed)
            else:
                safe_speed = min(target_speed, self.max_speed)
            
            # 考虑前后点的速度连续性
            if i > 0:
                prev_speed = velocity_path[-1][3]
                # 限制加速度
                max_speed_change = self.max_acceleration * 0.5  # 假设0.5s间隔
                safe_speed = min(safe_speed, prev_speed + max_speed_change)
                safe_speed = max(safe_speed, prev_speed - max_speed_change)
            
            velocity_path.append((point[0], point[1], point[2], safe_speed))
        
        return velocity_path
    
    def _add_time_stamps(self, velocity_path: List[Tuple]) -> List[Tuple]:
        """阶段4: 添加时间戳"""
        if len(velocity_path) < 2:
            return [(p[0], p[1], p[2], p[3], 0.0) for p in velocity_path]
        
        timed_trajectory = []
        current_time = 0.0
        
        timed_trajectory.append((
            velocity_path[0][0], velocity_path[0][1], 
            velocity_path[0][2], velocity_path[0][3], current_time
        ))
        
        for i in range(1, len(velocity_path)):
            prev_point = velocity_path[i-1]
            curr_point = velocity_path[i]
            
            # 计算距离
            distance = math.sqrt(
                (curr_point[0] - prev_point[0])**2 + 
                (curr_point[1] - prev_point[1])**2
            )
            
            # 计算平均速度
            avg_speed = (prev_point[3] + curr_point[3]) / 2
            avg_speed = max(0.1, avg_speed)  # 避免除零
            
            # 计算时间增量
            time_delta = distance / avg_speed
            current_time += time_delta
            
            timed_trajectory.append((
                curr_point[0], curr_point[1], curr_point[2], 
                curr_point[3], current_time
            ))
        
        return timed_trajectory

class SimplifiedRRTPlanner:
    """精简版RRT规划器 - 专注核心算法"""
    
    def __init__(self, env, vehicle_length=5.0, vehicle_width=2.0, 
                 turning_radius=5.0, step_size=0.8):
        self.env = env
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width
        self.turning_radius = turning_radius
        self.step_size = step_size
        
        # 算法参数
        self.max_nodes = 3000
        self.goal_bias = 0.15
        self.early_termination_distance = 2.0
        
        # 简单LRU缓存
        self.path_cache = OrderedDict()
        self.cache_limit = 50
        
        # 车辆动力学优化器
        self.dynamics_optimizer = VehicleDynamicsOptimizer(
            vehicle_length, vehicle_width, turning_radius
        )
        
        # 骨干网络（简化引用）
        self.backbone_network = None
        
        # 统计信息
        self.stats = {
            'cache_hits': 0,
            'cache_misses': 0,
            'planning_attempts': 0,
            'successful_plans': 0
        }
        
        print(f"初始化精简RRT规划器: 步长{step_size}, 最大节点{self.max_nodes}")
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络引用"""
        self.backbone_network = backbone_network
    
    def plan_path(self, start, goal, agent_id=None, max_iterations=3000, 
                  quality_threshold=0.6, enable_dynamics_optimization=True):
        """
        主要规划接口 - 两阶段设计
        阶段1: RRT空间路径规划
        阶段2: 车辆动力学优化
        """
        self.stats['planning_attempts'] += 1
        start_time = time.time()
        
        # 输入验证
        if not self._validate_inputs(start, goal):
            return None
        
        # 检查缓存
        cache_key = self._generate_cache_key(start, goal)
        cached_result = self._check_cache(cache_key)
        if cached_result:
            self.stats['cache_hits'] += 1
            return cached_result
        
        self.stats['cache_misses'] += 1
        
        print(f"开始RRT规划: {start} -> {goal}")
        
        # 阶段1: RRT几何路径规划
        raw_path = self._rrt_planning(start, goal, max_iterations)
        
        if not raw_path:
            print("RRT几何路径规划失败")
            return None
        
        print(f"RRT几何规划成功，原始路径{len(raw_path)}个点")
        
        # 阶段2: 车辆动力学优化
        if enable_dynamics_optimization:
            final_trajectory = self.dynamics_optimizer.optimize_path_dynamics(raw_path)
        else:
            # 不优化，只添加基础速度信息
            final_trajectory = [(p[0], p[1], p[2], 2.0) for p in raw_path]
        
        # 缓存结果
        if final_trajectory:
            self._add_to_cache(cache_key, final_trajectory)
            self.stats['successful_plans'] += 1
            
            planning_time = time.time() - start_time
            print(f"RRT规划完成: 最终轨迹{len(final_trajectory)}个点, 耗时{planning_time:.2f}s")
        
        return final_trajectory
    
    def _rrt_planning(self, start, goal, max_iterations):
        """RRT核心算法 - 专注空间搜索"""
        # 初始化树
        start_tree = [RRTNode(start[0], start[1], start[2] if len(start) > 2 else 0)]
        goal_tree = [RRTNode(goal[0], goal[1], goal[2] if len(goal) > 2 else 0)]
        
        for iteration in range(max_iterations):
            # 生成随机点
            if random.random() < self.goal_bias:
                random_point = goal
            else:
                random_point = self._sample_random_point()
            
            # 扩展起始树
            new_start_node = self._extend_tree(start_tree, random_point, True)
            if new_start_node:
                # 尝试连接到目标树
                connection = self._attempt_connection(new_start_node, goal_tree)
                if connection:
                    path = self._extract_path(new_start_node, connection, start_tree, goal_tree)
                    return self._post_process_path(path)
            
            # 扩展目标树
            new_goal_node = self._extend_tree(goal_tree, random_point, False)
            if new_goal_node:
                # 尝试连接到起始树
                connection = self._attempt_connection(new_goal_node, start_tree)
                if connection:
                    path = self._extract_path(connection, new_goal_node, start_tree, goal_tree)
                    return self._post_process_path(path)
            
            # 限制树的大小
            if len(start_tree) > self.max_nodes:
                start_tree = start_tree[:int(self.max_nodes * 0.8)]
            if len(goal_tree) > self.max_nodes:
                goal_tree = goal_tree[:int(self.max_nodes * 0.8)]
        
        print(f"RRT达到最大迭代次数{max_iterations}，未找到路径")
        return None
    
    def _sample_random_point(self):
        """采样随机点"""
        # 基础随机采样
        x = random.uniform(0, self.env.width)
        y = random.uniform(0, self.env.height)
        theta = random.uniform(0, 2 * math.pi)
        
        # 简单的骨干网络引导
        if self.backbone_network and random.random() < 0.2:
            return self._sample_near_backbone()
        
        return (x, y, theta)
    
    def _sample_near_backbone(self):
        """在骨干网络附近采样"""
        if not hasattr(self.backbone_network, 'backbone_paths'):
            return self._sample_random_point()
        
        # 随机选择骨干路径上的点
        paths = list(self.backbone_network.backbone_paths.values())
        if not paths:
            return self._sample_random_point()
        
        random_path = random.choice(paths)
        backbone_path = random_path.get('path', [])
        
        if not backbone_path:
            return self._sample_random_point()
        
        # 在骨干路径点附近添加噪声
        random_point = random.choice(backbone_path)
        noise_radius = 8.0
        
        x = random_point[0] + random.uniform(-noise_radius, noise_radius)
        y = random_point[1] + random.uniform(-noise_radius, noise_radius)
        theta = random.uniform(0, 2 * math.pi)
        
        return (x, y, theta)
    
    def _extend_tree(self, tree, target_point, from_start):
        """扩展树"""
        # 找到最近节点
        nearest_node = min(tree, key=lambda n: n.distance_to(
            RRTNode(target_point[0], target_point[1], target_point[2] if len(target_point) > 2 else 0)
        ))
        
        # 计算新节点位置
        direction = math.atan2(
            target_point[1] - nearest_node.y,
            target_point[0] - nearest_node.x
        )
        
        new_x = nearest_node.x + self.step_size * math.cos(direction)
        new_y = nearest_node.y + self.step_size * math.sin(direction)
        new_theta = direction  # 简化朝向计算
        
        # 验证新节点
        if self._is_valid_position(new_x, new_y):
            if self._is_path_collision_free(nearest_node, (new_x, new_y, new_theta)):
                new_cost = nearest_node.cost + self.step_size
                new_node = RRTNode(new_x, new_y, new_theta, nearest_node, new_cost)
                tree.append(new_node)
                return new_node
        
        return None
    
    def _attempt_connection(self, node, target_tree):
        """尝试连接到目标树"""
        connection_radius = 15.0
        
        for target_node in target_tree:
            distance = node.distance_to(target_node)
            if distance < connection_radius:
                if self._is_path_collision_free(node, (target_node.x, target_node.y, target_node.theta)):
                    return target_node
        
        return None
    
    def _extract_path(self, start_node, goal_node, start_tree, goal_tree):
        """提取路径"""
        path = []
        
        # 从起始节点回溯
        current = start_node
        start_path = []
        while current:
            start_path.append((current.x, current.y, current.theta))
            current = current.parent
        start_path.reverse()
        
        # 从目标节点回溯
        current = goal_node
        goal_path = []
        while current:
            goal_path.append((current.x, current.y, current.theta))
            current = current.parent
        
        # 合并路径
        path = start_path + goal_path
        return path
    
    def _post_process_path(self, path):
        """简单的路径后处理"""
        if not path or len(path) < 3:
            return path
        
        # 简单的线段优化
        optimized = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # 尝试跳过中间点
            j = min(len(path) - 1, i + 3)
            
            while j > i + 1:
                if self._is_path_collision_free_between_points(path[i], path[j]):
                    optimized.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                optimized.append(path[i + 1])
                i += 1
        
        return optimized
    
    def _is_valid_position(self, x, y):
        """检查位置是否有效"""
        if x < 0 or x >= self.env.width or y < 0 or y >= self.env.height:
            return False
        
        if hasattr(self.env, 'grid'):
            return self.env.grid[int(x), int(y)] == 0
        
        return True
    
    def _is_path_collision_free(self, from_node, to_point):
        """检查路径是否无碰撞"""
        if isinstance(from_node, RRTNode):
            start_x, start_y = from_node.x, from_node.y
        else:
            start_x, start_y = from_node[0], from_node[1]
        
        end_x, end_y = to_point[0], to_point[1]
        
        # 简单的直线碰撞检测
        distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        steps = max(5, int(distance / 0.5))
        
        for i in range(steps + 1):
            t = i / max(1, steps)
            check_x = start_x + t * (end_x - start_x)
            check_y = start_y + t * (end_y - start_y)
            
            if not self._is_valid_position(check_x, check_y):
                return False
        
        return True
    
    def _is_path_collision_free_between_points(self, point1, point2):
        """检查两点间路径是否无碰撞"""
        distance = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
        steps = max(3, int(distance))
        
        for i in range(1, steps):
            t = i / steps
            x = point1[0] + t * (point2[0] - point1[0])
            y = point1[1] + t * (point2[1] - point1[1])
            
            if not self._is_valid_position(x, y):
                return False
        
        return True
    
    def _validate_inputs(self, start, goal):
        """验证输入参数"""
        if not start or not goal:
            return False
        
        if len(start) < 2 or len(goal) < 2:
            return False
        
        for pos in [start, goal]:
            x, y = pos[0], pos[1]
            if x < 0 or x >= self.env.width or y < 0 or y >= self.env.height:
                return False
        
        return True
    
    def _generate_cache_key(self, start, goal):
        """生成缓存键"""
        start_key = (round(start[0], 1), round(start[1], 1))
        goal_key = (round(goal[0], 1), round(goal[1], 1))
        return f"{start_key}_{goal_key}"
    
    def _check_cache(self, cache_key):
        """检查缓存"""
        if cache_key in self.path_cache:
            # LRU: 移动到末尾
            self.path_cache.move_to_end(cache_key)
            return self.path_cache[cache_key]
        return None
    
    def _add_to_cache(self, cache_key, path):
        """添加到缓存"""
        if len(self.path_cache) >= self.cache_limit:
            # 移除最旧的项
            self.path_cache.popitem(last=False)
        
        self.path_cache[cache_key] = path
    
    def get_statistics(self):
        """获取统计信息"""
        total_requests = self.stats['cache_hits'] + self.stats['cache_misses']
        hit_rate = self.stats['cache_hits'] / max(1, total_requests)
        success_rate = self.stats['successful_plans'] / max(1, self.stats['planning_attempts'])
        
        return {
            'cache_hits': self.stats['cache_hits'],
            'cache_misses': self.stats['cache_misses'], 
            'cache_hit_rate': hit_rate,
            'planning_attempts': self.stats['planning_attempts'],
            'successful_plans': self.stats['successful_plans'],
            'success_rate': success_rate,
            'cached_paths': len(self.path_cache),
            'backbone_network_available': self.backbone_network is not None
        }
    
    def clear_cache(self):
        """清除缓存"""
        self.path_cache.clear()
        self.stats['cache_hits'] = 0
        self.stats['cache_misses'] = 0

# 向后兼容性
OptimizedRRTPlanner = SimplifiedRRTPlanner
RRTPlanner = SimplifiedRRTPlanner

def test_vehicle_dynamics_optimization():
    """测试车辆动力学优化"""
    print("=== 测试车辆动力学优化 ===")
    
    # 创建测试路径（模拟RRT输出）
    raw_path = [
        (0, 0, 0),
        (10, 5, 0.5),
        (20, 8, 0.8),
        (30, 15, 1.2),
        (40, 20, 1.5),
        (50, 22, 1.6),
        (60, 25, 0.5),
        (70, 30, 0)
    ]
    
    # 创建优化器
    optimizer = VehicleDynamicsOptimizer(
        vehicle_length=6.0,
        vehicle_width=3.0,
        turning_radius=8.0
    )
    
    # 执行优化
    optimized_trajectory = optimizer.optimize_path_dynamics(raw_path, target_speed=3.0)
    
    print(f"原始路径: {len(raw_path)} 个点")
    print(f"优化轨迹: {len(optimized_trajectory)} 个点")
    
    # 显示前几个点的对比
    print("\n前5个点对比:")
    print("原始: (x, y, theta)")
    for i, point in enumerate(raw_path[:5]):
        print(f"  {i}: {point}")
    
    print("优化: (x, y, theta, v, t)")
    for i, point in enumerate(optimized_trajectory[:5]):
        print(f"  {i}: ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}, {point[3]:.2f}, {point[4]:.2f})")
    
    return True

if __name__ == "__main__":
    test_vehicle_dynamics_optimization()