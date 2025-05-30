"""
astar.py - 混合A*路径规划器 (移植优化版)
基于成熟的混合A*算法，适配露天矿调度系统
"""

import numpy as np
import math
import heapq
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from typing import List, Tuple, Dict, Optional, Any
from collections import defaultdict, deque

class HybridAStarNode:
    """混合A*搜索节点 - 基于原始项目优化"""
    id_counter = 0
    
    def __init__(self, x, y, theta, cost=0, parent=None, direction=0):
        HybridAStarNode.id_counter += 1
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost  # g值 - 从起点到当前节点的代价
        self.h = 0.0      # h值 - 启发式估计值
        self.f = 0.0      # f值 - 总估计代价
        self.parent = parent  # 父节点
        self.steer = 0.0  # 转向角
        self.direction = direction  # 0表示前进，1表示倒退
        self.id = HybridAStarNode.id_counter
        self.parent_id = 0 if parent is None else parent.id
        
    def __lt__(self, other):
        """优先队列比较"""
        if abs(self.f - other.f) < 1e-6:
            return self.h < other.h
        return self.f < other.f
    
    def __eq__(self, other):
        """相等比较"""
        if other is None:
            return False
        return (abs(self.x - other.x) < 0.5 and 
                abs(self.y - other.y) < 0.5 and 
                abs(self.theta - other.theta) < 0.2 and
                self.direction == other.direction)
    
    def __hash__(self):
        """哈希函数"""
        return hash((round(self.x, 1), round(self.y, 1), 
                    round(self.theta, 1), self.direction))

class SimpleReedShepp:
    """简化的Reed-Shepp曲线实现"""
    
    def __init__(self, turning_radius):
        self.turning_radius = turning_radius
    
    def get_path(self, start, goal, step_size=0.5):
        """生成RS路径"""
        try:
            # 简化的RS路径：使用圆弧+直线组合
            x1, y1, theta1 = start
            x2, y2, theta2 = goal
            
            # 计算直线距离
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            # 如果距离很近，直接连线
            if distance < 3.0:
                steps = max(3, int(distance / step_size))
                path = []
                for i in range(steps + 1):
                    t = i / steps
                    x = x1 + t * (x2 - x1)
                    y = y1 + t * (y2 - y1)
                    theta = theta1 + t * (theta2 - theta1)
                    path.append((x, y, theta))
                return path
            
            # 使用简单的路径连接
            return self._simple_connection(start, goal, step_size)
        
        except Exception:
            return None
    
    def _simple_connection(self, start, goal, step_size):
        """简单的路径连接"""
        x1, y1, theta1 = start
        x2, y2, theta2 = goal
        
        # 中间点
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2
        
        # 生成路径
        path = []
        
        # 从起点到中间点
        steps1 = max(3, int(math.sqrt((mid_x - x1)**2 + (mid_y - y1)**2) / step_size))
        for i in range(steps1):
            t = i / steps1
            x = x1 + t * (mid_x - x1)
            y = y1 + t * (mid_y - y1)
            theta = theta1 + t * (theta2 - theta1) * 0.5
            path.append((x, y, theta))
        
        # 从中间点到终点
        steps2 = max(3, int(math.sqrt((x2 - mid_x)**2 + (y2 - mid_y)**2) / step_size))
        for i in range(steps2):
            t = i / steps2
            x = mid_x + t * (x2 - mid_x)
            y = mid_y + t * (y2 - mid_y)
            theta = theta1 + (0.5 + t * 0.5) * (theta2 - theta1)
            path.append((x, y, theta))
        
        path.append(goal)
        return path

class HybridAStarPlanner:
    """混合A*路径规划器 - 移植优化版"""
    
    def __init__(self, env, vehicle_length=6.0, vehicle_width=3.0, 
                 turning_radius=8.0, step_size=2.0, angle_resolution=30):
        """
        初始化混合A*规划器
        
        Args:
            env: 环境对象
            vehicle_length: 车辆长度
            vehicle_width: 车辆宽度
            turning_radius: 最小转弯半径
            step_size: 步长
            angle_resolution: 角度分辨率（度）
        """
        self.env = env
        self.vehicle_params = {
            'length': vehicle_length,
            'width': vehicle_width,
            'turning_radius': turning_radius,
            'wheel_base': vehicle_length * 0.6
        }
        self.step_size = step_size
        
        # 离散化参数 - 基于原始代码优化
        self.xy_grid_resolution = 2.0  # 适应大地图的网格分辨率
        self.theta_grid_resolution = math.radians(angle_resolution)  # 角度分辨率
        
        # 搜索数据结构
        self.open_list = []      # 优先队列
        self.open_dict = {}      # 开集字典
        self.close_dict = {}     # 闭集字典
        self.h_cost_map = {}     # 启发式地图
        self.bound_set = set()   # 障碍物边界集合
        
        # Reed-Shepp曲线
        self.rs_curves = SimpleReedShepp(turning_radius)
        
        # 算法配置参数 - 基于原始代码调整
        self.config = {
            'max_iterations': 8000,      # 最大迭代次数
            'timeout': 20.0,            # 超时时间
            'rs_fitting_radius': 25.0,  # RS曲线拟合半径
            'min_steering': -math.radians(25.0),  # 最小转向角
            'max_steering': math.radians(25.0),   # 最大转向角
            'angle_discrete_num': 5,    # 转向角离散化数量
            'back_penalty': 1.8,        # 倒车惩罚
            'steer_penalty': 0.5,       # 转向惩罚
            'direction_change_penalty': 3.0  # 变向惩罚
        }
        
        # 骨干网络集成
        self.backbone_network = None
        self.backbone_bias = 0.3
        
        # 统计信息
        self.stats = {
            'nodes_expanded': 0,
            'planning_time': 0,
            'path_length': 0,
            'h_map_calc_time': 0,
            'cache_hits': 0
        }
        
        # 路径缓存
        self.path_cache = {}
        self.cache_size_limit = 50
        
        # 调试模式
        self.debug = False
        
        print(f"初始化混合A*规划器 - 网格分辨率: {self.xy_grid_resolution}, "
              f"角度分辨率: {angle_resolution}°, RS半径: {self.config['rs_fitting_radius']}")
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
        if self.debug:
            print("混合A*规划器已连接骨干网络")
    
    def plan_path(self, start, goal, agent_id=None, max_iterations=None, 
                  quality_threshold=0.4, use_cache=True):
        """
        规划路径 - 主要接口
        
        Args:
            start: 起点 (x, y, theta)
            goal: 终点 (x, y, theta)
            agent_id: 智能体ID
            max_iterations: 最大迭代次数
            quality_threshold: 质量阈值
            use_cache: 是否使用缓存
            
        Returns:
            路径点列表或None
        """
        start_time = time.time()
        
        # 输入验证
        if not self._validate_inputs(start, goal):
            return None
        
        # 检查缓存
        if use_cache:
            cached_path = self._check_cache(start, goal)
            if cached_path:
                self.stats['cache_hits'] += 1
                return cached_path
        
        # 重置规划器状态
        self._reset_planner()
        
        if self.debug:
            print(f"开始混合A*规划: {start} -> {goal}")
        
        # 检查起点和终点有效性
        if not self._check_point_valid(start) or not self._check_point_valid(goal):
            if self.debug:
                print("起点或终点无效")
            return None
        
        # 初始化地图数据
        self._init_map_data()
        
        # 计算启发式地图
        h_start_time = time.time()
        if not self._calc_heuristic_map(goal):
            if self.debug:
                print("启发式地图计算失败")
            return None
        self.stats['h_map_calc_time'] = time.time() - h_start_time
        
        # 执行A*搜索
        final_path = self._astar_search(start, goal, max_iterations)
        
        if final_path:
            # 路径后处理
            processed_path = self._post_process_path(final_path)
            
            # 质量评估
            quality = self._evaluate_path_quality(processed_path, start, goal)
            
            if quality >= quality_threshold:
                # 缓存结果
                if use_cache:
                    self._add_to_cache(start, goal, processed_path)
                
                # 更新统计
                self.stats['planning_time'] = time.time() - start_time
                self.stats['path_length'] = len(processed_path)
                
                if self.debug:
                    print(f"规划成功! 路径长度: {len(processed_path)}, "
                          f"质量: {quality:.2f}, 耗时: {self.stats['planning_time']:.2f}s")
                
                return processed_path
        
        if self.debug:
            print("混合A*规划失败")
        return None
    
    def _reset_planner(self):
        """重置规划器状态"""
        self.open_list = []
        self.open_dict = {}
        self.close_dict = {}
        self.h_cost_map = {}
        self.bound_set = set()
        HybridAStarNode.id_counter = 0
    
    def _init_map_data(self):
        """初始化地图数据"""
        self._generate_bound_set()
    
    def _generate_bound_set(self):
        """生成障碍物边界集合"""
        self.bound_set = set()
        
        if hasattr(self.env, 'grid'):
            for x in range(self.env.width):
                for y in range(self.env.height):
                    if self.env.grid[x, y] == 1:  # 障碍物
                        x_grid = int(x / self.xy_grid_resolution)
                        y_grid = int(y / self.xy_grid_resolution)
                        self.bound_set.add(f"{x_grid},{y_grid}")
    
    def _calc_heuristic_map(self, goal):
        """
        计算启发式地图 - 使用Dijkstra算法
        参考原始代码的启发式计算方法
        """
        if self.debug:
            print("计算启发式地图...")
        
        start_time = time.time()
        
        # 目标点网格坐标
        goal_x_grid = int(goal[0] / self.xy_grid_resolution)
        goal_y_grid = int(goal[1] / self.xy_grid_resolution)
        
        # 初始化Dijkstra
        open_2d = [(0, goal_x_grid, goal_y_grid)]
        self.h_cost_map = {}
        
        # 执行Dijkstra搜索
        while open_2d:
            # 超时检查
            if time.time() - start_time > 15.0:
                if self.debug:
                    print("启发式地图计算超时")
                return len(self.h_cost_map) > 100  # 至少有一些启发式值
            
            current_cost, x, y = heapq.heappop(open_2d)
            current_key = f"{x},{y}"
            
            # 如果已访问，跳过
            if current_key in self.h_cost_map:
                continue
            
            # 添加到启发式地图
            self.h_cost_map[current_key] = current_cost
            
            # 扩展8个方向的邻居
            for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                nx, ny = x + dx, y + dy
                next_key = f"{nx},{ny}"
                
                # 检查边界
                if next_key in self.bound_set or self._is_out_of_bounds(nx, ny):
                    continue
                
                # 如果已访问，跳过
                if next_key in self.h_cost_map:
                    continue
                
                # 计算移动代价
                move_cost = math.sqrt(dx*dx + dy*dy) * self.xy_grid_resolution
                new_cost = current_cost + move_cost
                
                # 添加到开集
                heapq.heappush(open_2d, (new_cost, nx, ny))
        
        if self.debug:
            calc_time = time.time() - start_time
            print(f"启发式地图计算完成: {len(self.h_cost_map)}个点, 耗时{calc_time:.2f}s")
        
        return len(self.h_cost_map) > 0
    
    def _is_out_of_bounds(self, x_grid, y_grid):
        """检查网格坐标是否越界"""
        x_real = x_grid * self.xy_grid_resolution
        y_real = y_grid * self.xy_grid_resolution
        
        return (x_real < 0 or x_real >= self.env.width or 
                y_real < 0 or y_real >= self.env.height)
    
    def _astar_search(self, start, goal, max_iterations):
        """
        A*搜索算法 - 基于原始代码的搜索策略
        """
        # 设置迭代限制
        if max_iterations is None:
            max_iterations = self.config['max_iterations']
        
        # 初始化开闭集
        self._init_search(start)
        
        # 搜索变量
        iterations = 0
        best_node = None
        best_distance = float('inf')
        
        # 主搜索循环
        while self.open_list and iterations < max_iterations:
            # 获取f值最小的节点
            current_node = self._get_min_cost_node()
            if current_node is None:
                break
            
            # 移动到闭集
            grid_key = self._get_grid_key(current_node)
            self.close_dict[grid_key] = current_node
            
            # 更新最佳节点
            distance_to_goal = math.sqrt(
                (current_node.x - goal[0])**2 + (current_node.y - goal[1])**2
            )
            
            if distance_to_goal < best_distance:
                best_distance = distance_to_goal
                best_node = current_node
            
            # 检查是否可以用RS曲线连接到目标
            if distance_to_goal < self.config['rs_fitting_radius']:
                rs_path = self._try_rs_connection(current_node, goal)
                if rs_path:
                    if self.debug:
                        print(f"RS曲线连接成功，迭代次数: {iterations}")
                    
                    # 合并A*路径和RS路径
                    astar_path = self._trace_path(current_node)
                    complete_path = astar_path[:-1] + rs_path  # 避免重复点
                    
                    self.stats['nodes_expanded'] = iterations
                    return complete_path
            
            # 扩展当前节点
            self._expand_node(current_node, goal)
            
            iterations += 1
            
            if self.debug and iterations % 500 == 0:
                print(f"迭代 {iterations}, 开集大小: {len(self.open_list)}, "
                      f"最佳距离: {best_distance:.1f}")
        
        self.stats['nodes_expanded'] = iterations
        
        # 如果无法到达目标，返回最接近的路径
        if best_node and best_distance < 30.0:
            if self.debug:
                print(f"返回最接近路径，距离目标: {best_distance:.1f}")
            return self._trace_path(best_node)
        
        return None
    
    def _init_search(self, start):
        """初始化搜索"""
        # 创建初始节点（前进和后退两个方向）
        for direction in [0, 1]:  # 0:前进, 1:后退
            node = HybridAStarNode(start[0], start[1], start[2], 0, None, direction)
            self._calc_h_value(node)
            node.f = node.cost + node.h
            
            grid_key = self._get_grid_key(node)
            heapq.heappush(self.open_list, node)
            self.open_dict[grid_key] = node
    
    def _get_min_cost_node(self):
        """获取代价最小的节点"""
        while self.open_list:
            node = heapq.heappop(self.open_list)
            grid_key = self._get_grid_key(node)
            
            # 从开集字典中移除
            if grid_key in self.open_dict:
                del self.open_dict[grid_key]
                return node
        
        return None
    
    def _expand_node(self, node, goal):
        """扩展节点 - 生成后继节点"""
        # 计算转向角步长
        delta_steering = ((self.config['max_steering'] - self.config['min_steering']) / 
                         (self.config['angle_discrete_num'] - 1))
        
        # 遍历所有转向角
        for i in range(self.config['angle_discrete_num']):
            steering = self.config['min_steering'] + i * delta_steering
            
            # 对每个转向角尝试前进和后退
            for direction in [0, 1]:  # 0:前进, 1:后退
                # 使用车辆动力学计算下一状态
                next_node = self._vehicle_dynamics(node, direction, steering)
                
                # 检查状态有效性
                if not self._is_state_valid(next_node):
                    continue
                
                # 获取网格键
                grid_key = self._get_grid_key(next_node)
                
                # 跳过闭集中的节点
                if grid_key in self.close_dict:
                    continue
                
                # 检查开集
                if grid_key in self.open_dict:
                    existing_node = self.open_dict[grid_key]
                    if next_node.cost < existing_node.cost:
                        # 更新现有节点
                        existing_node.cost = next_node.cost
                        existing_node.parent = node
                        existing_node.parent_id = node.id
                        existing_node.steer = steering
                        existing_node.f = existing_node.cost + existing_node.h
                        
                        # 重新组织堆
                        self.open_list = [n for n in self.open_list if n.id != existing_node.id]
                        heapq.heappush(self.open_list, existing_node)
                else:
                    # 添加新节点
                    self._calc_h_value(next_node)
                    next_node.f = next_node.cost + next_node.h
                    next_node.steer = steering
                    
                    heapq.heappush(self.open_list, next_node)
                    self.open_dict[grid_key] = next_node
    
    def _vehicle_dynamics(self, node, direction, steering):
        """
        车辆动力学模型 - 基于原始代码的车辆模型
        """
        # 方向系数
        direction_factor = 1.0 if direction == 0 else -1.0
        
        # 创建新节点
        next_node = HybridAStarNode(0, 0, 0, 0, node, direction)
        
        # 计算新状态
        if abs(steering) < 1e-6:  # 直线运动
            next_node.x = node.x + direction_factor * self.step_size * math.cos(node.theta)
            next_node.y = node.y + direction_factor * self.step_size * math.sin(node.theta)
            next_node.theta = node.theta
        else:
            # 使用自行车模型计算圆弧运动
            wheel_base = self.vehicle_params['wheel_base']
            beta = self.step_size / wheel_base * math.tan(steering)
            
            if abs(beta) > 1e-6:
                R = self.step_size / beta
                
                next_node.x = (node.x + direction_factor * R * 
                              (math.sin(node.theta + beta) - math.sin(node.theta)))
                next_node.y = (node.y + direction_factor * R * 
                              (math.cos(node.theta) - math.cos(node.theta + beta)))
                next_node.theta = self._normalize_angle(node.theta + beta)
            else:
                # beta很小，近似直线
                next_node.x = node.x + direction_factor * self.step_size * math.cos(node.theta)
                next_node.y = node.y + direction_factor * self.step_size * math.sin(node.theta)
                next_node.theta = node.theta
        
        # 计算代价
        self._calc_cost(node, next_node, steering)
        
        return next_node
    
    def _calc_cost(self, parent_node, node, steering):
        """
        计算节点代价 - 基于原始代码的代价函数
        """
        # 基础距离代价
        distance_cost = self.step_size
        
        # 后退惩罚
        if node.direction == 1:  # 后退
            distance_cost *= self.config['back_penalty']
        
        # 转向惩罚
        steer_cost = abs(steering) * self.config['steer_penalty']
        
        # 方向改变惩罚
        direction_change_cost = 0
        if parent_node.direction != node.direction:
            direction_change_cost = self.config['direction_change_penalty']
        
        # 角度变化代价
        angle_diff = abs(self._normalize_angle(node.theta - parent_node.theta))
        angle_cost = angle_diff * 0.5
        
        # 总代价
        node.cost = (parent_node.cost + distance_cost + steer_cost + 
                    direction_change_cost + angle_cost)
    
    def _calc_h_value(self, node):
        """计算启发式值"""
        # 获取网格坐标
        x_grid = int(node.x / self.xy_grid_resolution)
        y_grid = int(node.y / self.xy_grid_resolution)
        grid_key = f"{x_grid},{y_grid}"
        
        # 从启发式地图获取值
        if grid_key in self.h_cost_map:
            node.h = self.h_cost_map[grid_key]
            
            # 添加骨干网络偏置
            if self.backbone_network:
                backbone_bonus = self._calc_backbone_bonus(node)
                node.h -= backbone_bonus
        else:
            # 使用默认启发式
            node.h = 0.0
    
    def _calc_backbone_bonus(self, node):
        """计算骨干网络奖励"""
        if not hasattr(self.backbone_network, 'backbone_paths'):
            return 0
        
        min_distance = float('inf')
        
        # 检查到骨干路径的最小距离
        for path_data in self.backbone_network.backbone_paths.values():
            backbone_path = path_data.get('path', [])
            
            for bp in backbone_path[::5]:  # 采样检查
                distance = math.sqrt((node.x - bp[0])**2 + (node.y - bp[1])**2)
                min_distance = min(min_distance, distance)
        
        # 如果接近骨干路径，给予奖励
        if min_distance < 15:
            return (15 - min_distance) * self.backbone_bias
        
        return 0
    
    def _try_rs_connection(self, node, goal):
        """尝试Reed-Shepp曲线连接"""
        try:
            # 使用简化的RS曲线
            start_state = (node.x, node.y, node.theta)
            rs_path = self.rs_curves.get_path(start_state, goal, step_size=1.0)
            
            if rs_path and len(rs_path) > 1:
                # 检查RS路径是否无碰撞
                if self._check_rs_path_collision_free(rs_path):
                    return rs_path
        
        except Exception as e:
            if self.debug:
                print(f"RS曲线连接失败: {e}")
        
        return None
    
    def _check_rs_path_collision_free(self, rs_path):
        """检查RS路径是否无碰撞"""
        for point in rs_path:
            if not self._check_point_valid(point):
                return False
        
        # 检查相邻点间的连线
        for i in range(len(rs_path) - 1):
            if not self._check_line_collision_free(rs_path[i], rs_path[i + 1]):
                return False
        
        return True
    
    def _check_line_collision_free(self, p1, p2):
        """检查两点间连线是否无碰撞"""
        steps = max(3, int(math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)))
        
        for i in range(1, steps):
            t = i / steps
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            theta = p1[2] + t * (p2[2] - p1[2])
            
            if not self._check_point_valid((x, y, theta)):
                return False
        
        return True
    
    def _trace_path(self, final_node):
        """回溯路径"""
        path = []
        all_nodes = {}
        
        # 收集所有节点
        for node in self.open_dict.values():
            all_nodes[node.id] = node
        for node in self.close_dict.values():
            all_nodes[node.id] = node
        
        # 回溯
        current = final_node
        while current:
            path.append((current.x, current.y, current.theta))
            
            if current.parent_id == 0:
                break
            
            current = all_nodes.get(current.parent_id)
        
        path.reverse()
        return path
    
    def _post_process_path(self, path):
        """路径后处理"""
        if not path or len(path) < 3:
            return path
        
        # 简单的路径平滑
        smoothed = self._simple_smooth(path)
        
        # 密度调整
        return self._adjust_path_density(smoothed)
    
    def _simple_smooth(self, path):
        """简单路径平滑"""
        if len(path) < 3:
            return path
        
        smoothed = [path[0]]
        
        i = 0
        while i < len(path) - 1:
            # 尝试跳过中间点
            j = min(len(path) - 1, i + 4)
            
            while j > i + 1:
                if self._check_line_collision_free(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                smoothed.append(path[i + 1])
                i += 1
        
        return smoothed
    
    def _adjust_path_density(self, path):
        """调整路径密度"""
        if len(path) < 2:
            return path
        
        target_spacing = 1.5  # 目标间距
        dense_path = [path[0]]
        
        for i in range(len(path) - 1):
            current = path[i]
            next_point = path[i + 1]
            
            distance = math.sqrt(
                (next_point[0] - current[0])**2 + 
                (next_point[1] - current[1])**2
            )
            
            if distance > target_spacing:
                # 需要插入中间点
                num_inserts = int(distance / target_spacing)
                
                for j in range(1, num_inserts + 1):
                    t = j / (num_inserts + 1)
                    x = current[0] + t * (next_point[0] - current[0])
                    y = current[1] + t * (next_point[1] - current[1])
                    theta = current[2] + t * (next_point[2] - current[2])
                    dense_path.append((x, y, theta))
            
            dense_path.append(next_point)
        
        return dense_path
    
    def _get_grid_key(self, node):
        """获取节点网格键"""
        x_grid = int(node.x / self.xy_grid_resolution)
        y_grid = int(node.y / self.xy_grid_resolution)
        theta_grid = int(node.theta / self.theta_grid_resolution)
        
        return f"{x_grid},{y_grid},{theta_grid},{node.direction}"
    
    def _is_state_valid(self, node):
        """检查状态是否有效"""
        # 边界检查
        if (node.x < 2 or node.x >= self.env.width - 2 or
            node.y < 2 or node.y >= self.env.height - 2):
            return False
        
        # 碰撞检测
        return self._check_point_valid((node.x, node.y, node.theta))
    
    def _check_point_valid(self, point):
        """检查点是否有效（简化碰撞检测）"""
        x, y, theta = point
        
        # 检查车辆中心和几个关键点
        check_points = [
            (x, y),
            (x + 2 * math.cos(theta), y + 2 * math.sin(theta)),  # 前方
            (x - 2 * math.cos(theta), y - 2 * math.sin(theta)),  # 后方
        ]
        
        for px, py in check_points:
            if not self._is_point_free(px, py):
                return False
        
        return True
    
    def _is_point_free(self, x, y):
        """检查点是否无障碍"""
        ix, iy = int(x), int(y)
        
        if (ix < 0 or ix >= self.env.width or 
            iy < 0 or iy >= self.env.height):
            return False
        
        if hasattr(self.env, 'grid'):
            return self.env.grid[ix, iy] == 0
        
        return True
    
    def _normalize_angle(self, angle):
        """角度归一化到[0, 2π]"""
        while angle < 0:
            angle += 2 * math.pi
        while angle >= 2 * math.pi:
            angle -= 2 * math.pi
        return angle
    
    def _evaluate_path_quality(self, path, start, goal):
        """评估路径质量"""
        if not path or len(path) < 2:
            return 0.0
        
        # 长度效率
        path_length = sum(
            math.sqrt((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)
            for i in range(len(path) - 1)
        )
        direct_distance = math.sqrt((goal[0] - start[0])**2 + (goal[1] - start[1])**2)
        
        length_efficiency = direct_distance / (path_length + 0.1) if path_length > 0 else 0
        
        # 平滑度
        smoothness = self._calc_smoothness(path)
        
        # 综合质量
        return 0.6 * length_efficiency + 0.4 * smoothness
    
    def _calc_smoothness(self, path):
        """计算路径平滑度"""
        if len(path) < 3:
            return 1.0
        
        total_curvature = 0
        
        for i in range(1, len(path) - 1):
            # 计算曲率
            v1 = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
            v2 = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            
            len1 = math.sqrt(v1[0]**2 + v1[1]**2)
            len2 = math.sqrt(v2[0]**2 + v2[1]**2)
            
            if len1 > 1e-6 and len2 > 1e-6:
                cos_angle = (v1[0]*v2[0] + v1[1]*v2[1]) / (len1 * len2)
                cos_angle = max(-1, min(1, cos_angle))
                angle = math.acos(cos_angle)
                total_curvature += angle
        
        avg_curvature = total_curvature / max(1, len(path) - 2)
        return math.exp(-avg_curvature * 2)
    
    def _validate_inputs(self, start, goal):
        """验证输入"""
        if not start or not goal or len(start) < 2 or len(goal) < 2:
            return False
        
        for point in [start, goal]:
            if (point[0] < 0 or point[0] >= self.env.width or
                point[1] < 0 or point[1] >= self.env.height):
                return False
        
        return True
    
    def _check_cache(self, start, goal):
        """检查缓存"""
        cache_key = self._generate_cache_key(start, goal)
        return self.path_cache.get(cache_key)
    
    def _add_to_cache(self, start, goal, path):
        """添加到缓存"""
        if len(self.path_cache) >= self.cache_size_limit:
            # 移除最旧的项
            oldest_key = next(iter(self.path_cache))
            del self.path_cache[oldest_key]
        
        cache_key = self._generate_cache_key(start, goal)
        self.path_cache[cache_key] = path
    
    def _generate_cache_key(self, start, goal):
        """生成缓存键"""
        start_str = f"{start[0]:.1f},{start[1]:.1f},{start[2]:.1f}" if len(start) > 2 else f"{start[0]:.1f},{start[1]:.1f},0"
        goal_str = f"{goal[0]:.1f},{goal[1]:.1f},{goal[2]:.1f}" if len(goal) > 2 else f"{goal[0]:.1f},{goal[1]:.1f},0"
        return f"{start_str}_{goal_str}"
    
    def get_statistics(self):
        """获取统计信息"""
        stats = self.stats.copy()
        stats['cache_size'] = len(self.path_cache)
        stats['h_map_size'] = len(self.h_cost_map)
        return stats
    
    def clear_cache(self):
        """清除缓存"""
        self.path_cache.clear()
        self.stats['cache_hits'] = 0
    
    def visualize_path(self, path, start, goal, save_path=None):
        """可视化路径"""
        if not path:
            print("无路径可视化")
            return
        
        plt.figure(figsize=(12, 10))
        
        # 绘制环境
        if hasattr(self.env, 'grid'):
            plt.imshow(self.env.grid.T, cmap='binary', origin='lower', alpha=0.7)
        
        # 绘制路径
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        plt.plot(path_x, path_y, 'b-', linewidth=2, label='混合A*路径')
        
        # 绘制起点和终点
        plt.plot(start[0], start[1], 'go', markersize=10, label='起点')
        plt.plot(goal[0], goal[1], 'ro', markersize=10, label='终点')
        
        # 绘制车辆方向
        for i in range(0, len(path), max(1, len(path)//8)):
            x, y, theta = path[i]
            dx = 3 * math.cos(theta)
            dy = 3 * math.sin(theta)
            plt.arrow(x, y, dx, dy, head_width=1, head_length=1, fc='red', ec='red')
        
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('混合A*路径规划结果（移植优化版）')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
        
        plt.show()

# 测试函数
def test_optimized_hybrid_astar():
    """测试优化的混合A*规划器"""
    print("=== 测试优化的混合A*规划器 ===")
    
    # 创建测试环境
    class TestEnv:
        def __init__(self, width=150, height=150):
            self.width = width
            self.height = height
            self.grid = np.zeros((width, height), dtype=int)
            
            # 添加一些障碍物
            self.grid[40:70, 50:80] = 1
            self.grid[90:110, 30:60] = 1
            self.grid[30:60, 100:130] = 1
    
    # 创建环境和规划器
    env = TestEnv(150, 150)
    planner = HybridAStarPlanner(
        env,
        vehicle_length=6.0,
        vehicle_width=3.0,
        turning_radius=8.0,
        step_size=2.0,
        angle_resolution=30
    )
    
    # 测试用例
    test_cases = [
        {
            'name': '短距离测试',
            'start': (10, 10, 0),
            'goal': (30, 30, math.pi/2)
        },
        {
            'name': '中距离测试',
            'start': (20, 20, 0),
            'goal': (120, 120, 0)
        },
        {
            'name': '复杂环境测试',
            'start': (10, 130, 0),
            'goal': (130, 10, math.pi)
        }
    ]
    
    success_count = 0
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n--- 测试 {i}: {test_case['name']} ---")
        
        start_time = time.time()
        path = planner.plan_path(test_case['start'], test_case['goal'])
        planning_time = time.time() - start_time
        
        if path:
            success_count += 1
            print(f"✅ 规划成功!")
            print(f"   路径长度: {len(path)} 个点")
            print(f"   规划时间: {planning_time:.2f} 秒")
            
            stats = planner.get_statistics()
            print(f"   扩展节点: {stats['nodes_expanded']}")
            print(f"   启发式地图: {stats['h_map_size']} 个点")
        else:
            print(f"❌ 规划失败")
    
    print(f"\n=== 测试完成 ===")
    print(f"成功率: {success_count}/{len(test_cases)} ({success_count/len(test_cases)*100:.1f}%)")
    
    return success_count == len(test_cases)

if __name__ == "__main__":
    test_optimized_hybrid_astar()