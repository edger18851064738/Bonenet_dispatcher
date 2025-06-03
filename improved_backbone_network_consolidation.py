"""
improved_backbone_network_consolidation.py - 改进的骨干路径网络整理优化系统
专注于处理混合A*算法收敛产生的重复节点和相似路径，避免过度整理
"""

import math
import time
import threading
from collections import defaultdict, deque
from typing import Dict, List, Tuple, Optional, Any, Set
from dataclasses import dataclass, field
import numpy as np

@dataclass
class ConsolidatedNode:
    """整理后的节点"""
    node_id: str
    position: Tuple[float, float, float]
    cluster_id: int
    merged_count: int = 1
    original_nodes: List = field(default_factory=list)
    usage_frequency: int = 0
    is_junction: bool = False
    connected_paths: List[str] = field(default_factory=list)

@dataclass
class PathSegment:
    """路径段"""
    segment_id: str
    nodes: List[Tuple[float, float, float]]
    source_paths: List[str] = field(default_factory=list)
    usage_count: int = 0
    length: float = 0.0
    direction_vector: Tuple[float, float] = (0.0, 0.0)
    




@dataclass
class ConsolidatedPath:
    """整理后的路径 - 保持原有BiDirectionalPath接口兼容"""
    path_id: str
    path_type: str  # "merged", "original", "optimized"
    nodes: List[Tuple[float, float, float]]
    
    # 原有 BiDirectionalPath 属性保持兼容
    point_a: Dict
    point_b: Dict
    forward_path: List[Tuple]
    reverse_path: List[Tuple]
    length: float
    quality: float
    planner_used: str
    created_time: float
    usage_count: int = 0
    current_load: int = 0
    max_capacity: int = 5
    
    # 新增：质量历史追踪（兼容BiDirectionalPath）
    quality_history: List[float] = field(default_factory=list)
    last_quality_update: float = 0.0
    
    # 整理相关信息
    original_paths: List[str] = field(default_factory=list)
    consolidation_info: Dict = field(default_factory=dict)
    merge_reason: str = ""
    
    def __post_init__(self):
        """初始化后处理 - 确保完全兼容性"""
        # 初始化质量历史
        if not self.quality_history:
            self.quality_history = [self.quality]
        
        # 设置质量更新时间
        if self.last_quality_update == 0.0:
            self.last_quality_update = self.created_time
    
    # ==================== 兼容性方法（与BiDirectionalPath完全一致） ====================
    
    def get_average_quality(self) -> float:
        """获取平均质量 - 兼容 BiDirectionalPath 接口"""
        if not self.quality_history:
            return self.quality
        
        # 如果是合并路径，考虑原始路径的质量范围
        if (self.path_type == "merged" and 
            self.consolidation_info and 
            'quality_range' in self.consolidation_info):
            
            quality_range = self.consolidation_info['quality_range']
            historical_avg = sum(self.quality_history) / len(self.quality_history)
            
            # 结合历史平均和原始质量范围
            range_avg = (quality_range[0] + quality_range[1]) / 2
            return (historical_avg + range_avg) / 2
        
        # 返回历史平均值
        return sum(self.quality_history) / len(self.quality_history)
    
    def update_quality_history(self, new_quality: float):
        """更新质量历史 - 兼容 BiDirectionalPath 接口"""
        import time
        
        self.quality_history.append(new_quality)
        self.last_quality_update = time.time()
        
        # 更新当前质量
        self.quality = new_quality
        
        # 限制历史长度
        if len(self.quality_history) > 20:
            self.quality_history = self.quality_history[-10:]
    
    def get_path(self, from_point_type: str, from_point_id: int,
                to_point_type: str, to_point_id: int) -> Optional[List[Tuple]]:
        """获取指定方向的路径 - 兼容方法"""
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
        """增加使用计数 - 兼容方法"""
        self.usage_count += 1
    
    def add_vehicle(self, vehicle_id: str):
        """添加车辆到路径 - 兼容方法"""
        self.current_load += 1
    
    def remove_vehicle(self, vehicle_id: str):
        """从路径移除车辆 - 兼容方法"""
        self.current_load = max(0, self.current_load - 1)
    
    def get_load_factor(self) -> float:
        """获取负载因子 - 兼容方法"""
        if self.max_capacity <= 0:
            return 0.0
        return self.current_load / self.max_capacity
    
    # ==================== 增强方法（整理路径特有） ====================
    
    def get_consolidation_quality_score(self) -> float:
        """获取整理质量分数"""
        base_quality = self.get_average_quality()
        
        # 合并路径的质量加成
        if self.path_type == "merged":
            merge_count = self.consolidation_info.get('merged_count', 1)
            if merge_count > 1:
                # 合并的路径越多，说明相似度越高，质量加成
                merge_bonus = min(0.1, (merge_count - 1) * 0.02)
                base_quality += merge_bonus
        
        # 使用频率加成
        if self.usage_count > 0:
            usage_bonus = min(0.05, self.usage_count * 0.01)
            base_quality += usage_bonus
        
        return min(1.0, base_quality)
    
    def get_original_paths_info(self) -> Dict:
        """获取原始路径信息"""
        return {
            'original_paths': self.original_paths.copy(),
            'merge_reason': self.merge_reason,
            'consolidation_info': self.consolidation_info.copy(),
            'path_type': self.path_type
        }
    
    def is_consolidated_path(self) -> bool:
        """检查是否是整理后的路径"""
        return self.path_type in ["merged", "optimized"]
    
    def get_hierarchy_level(self) -> str:
        """获取层次级别"""
        return self.consolidation_info.get('hierarchy_level', 'trunk')
    
    def supports_vehicle_type(self, vehicle_type: str) -> bool:
        """检查是否支持特定车辆类型"""
        # 整理后的路径通常有更好的通用性
        if self.path_type == "merged":
            return True
        
        # 根据路径质量和宽度判断
        return self.get_average_quality() > 0.5
    
    def get_detailed_info(self) -> Dict:
        """获取详细路径信息"""
        return {
            'path_id': self.path_id,
            'path_type': self.path_type,
            'quality': self.quality,
            'average_quality': self.get_average_quality(),
            'consolidation_quality': self.get_consolidation_quality_score(),
            'length': self.length,
            'usage_count': self.usage_count,
            'current_load': self.current_load,
            'load_factor': self.get_load_factor(),
            'max_capacity': self.max_capacity,
            'is_consolidated': self.is_consolidated_path(),
            'hierarchy_level': self.get_hierarchy_level(),
            'original_paths_count': len(self.original_paths),
            'merge_reason': self.merge_reason,
            'created_time': self.created_time,
            'last_quality_update': self.last_quality_update,
            'quality_history_length': len(self.quality_history)
        }


class ConservativeNodeClusterManager:
    """保守的节点聚类管理器 - 只合并真正重叠的节点"""
    
    def __init__(self, overlap_threshold=0.8):  # 0.8米以内认为重叠
        self.overlap_threshold = overlap_threshold
        self.node_clusters = {}
        self.position_map = {}  # {rounded_pos: cluster_id}
        
    def cluster_overlapping_nodes(self, bidirectional_paths: Dict) -> Dict[int, ConsolidatedNode]:
        """保守地聚类重叠节点"""
        print(f"\n🔧 [保守节点聚类] 合并阈值: {self.overlap_threshold}m")
        
        # 1. 收集所有节点位置，保留路径信息
        all_node_positions = []
        node_path_mapping = defaultdict(list)  # {position_key: [path_ids]}
        
        for path_id, path_data in bidirectional_paths.items():
            for i, node in enumerate(path_data.forward_path):
                # 使用更精确的位置键
                pos_key = (round(node[0], 1), round(node[1], 1))
                node_info = {
                    'position': node,
                    'path_id': path_id,
                    'index': i,
                    'pos_key': pos_key
                }
                all_node_positions.append(node_info)
                node_path_mapping[pos_key].append(path_id)
        
        print(f"   收集到 {len(all_node_positions)} 个节点")
        
        # 2. 识别真正重叠的节点组
        overlapping_groups = self._find_overlapping_groups(all_node_positions)
        print(f"   发现 {len(overlapping_groups)} 个重叠组")
        
        # 3. 创建合并后的节点
        consolidated_nodes = {}
        cluster_id = 0
        
        for group in overlapping_groups:
            if len(group) > 1:  # 只处理真正有重叠的组
                representative_node = self._create_merged_node(cluster_id, group)
                consolidated_nodes[cluster_id] = representative_node
                cluster_id += 1
        
        # 4. 为未重叠的节点创建单独的簇
        processed_positions = set()
        for group in overlapping_groups:
            for node_info in group:
                processed_positions.add(node_info['pos_key'])
        
        for node_info in all_node_positions:
            if node_info['pos_key'] not in processed_positions:
                individual_node = self._create_individual_node(cluster_id, node_info)
                consolidated_nodes[cluster_id] = individual_node
                cluster_id += 1
        
        original_count = len(all_node_positions)
        merged_count = len(consolidated_nodes)
        reduction_rate = (original_count - merged_count) / original_count * 100 if original_count > 0 else 0
        
        print(f"   ✅ 保守聚类完成: {original_count} -> {merged_count} 节点")
        print(f"   合并率: {reduction_rate:.1f}% (只合并真正重叠的节点)")
        
        self.node_clusters = consolidated_nodes
        return consolidated_nodes
    
    def _find_overlapping_groups(self, all_node_positions):
        """查找重叠的节点组"""
        groups = []
        processed = set()
        
        for i, node1 in enumerate(all_node_positions):
            if i in processed:
                continue
            
            current_group = [node1]
            processed.add(i)
            
            # 查找与当前节点重叠的所有节点
            for j, node2 in enumerate(all_node_positions[i+1:], i+1):
                if j in processed:
                    continue
                
                distance = math.sqrt(
                    (node1['position'][0] - node2['position'][0])**2 +
                    (node1['position'][1] - node2['position'][1])**2
                )
                
                if distance <= self.overlap_threshold:
                    current_group.append(node2)
                    processed.add(j)
            
            groups.append(current_group)
        
        return groups
    
    def _create_merged_node(self, cluster_id, node_group):
        """创建合并节点"""
        # 计算质心
        total_x = sum(node['position'][0] for node in node_group)
        total_y = sum(node['position'][1] for node in node_group)
        total_theta = sum(node['position'][2] for node in node_group)
        
        count = len(node_group)
        center_pos = (total_x/count, total_y/count, total_theta/count)
        
        # 收集连接的路径
        connected_paths = list(set(node['path_id'] for node in node_group))
        
        return ConsolidatedNode(
            node_id=f"merged_node_{cluster_id}",
            position=center_pos,
            cluster_id=cluster_id,
            merged_count=count,
            original_nodes=node_group,
            usage_frequency=len(connected_paths),
            is_junction=len(connected_paths) > 1,
            connected_paths=connected_paths
        )
    
    def _create_individual_node(self, cluster_id, node_info):
        """创建个体节点"""
        return ConsolidatedNode(
            node_id=f"node_{cluster_id}",
            position=node_info['position'],
            cluster_id=cluster_id,
            merged_count=1,
            original_nodes=[node_info],
            usage_frequency=1,
            connected_paths=[node_info['path_id']]
        )

class SmartPathSimilarityAnalyzer:
    """智能路径相似性分析器 - 更严格的相似性判断"""
    
    def __init__(self, min_similarity=0.92, min_overlap_ratio=0.85):
        self.min_similarity = min_similarity  # 提高相似度阈值
        self.min_overlap_ratio = min_overlap_ratio  # 路径重叠比例阈值
        self.min_common_length = 15.0  # 最小公共段长度(米)
        
    def find_similar_paths(self, bidirectional_paths: Dict) -> Dict[str, List]:
        """查找相似路径组"""
        print(f"\n🔧 [智能路径分析] 相似度阈值: {self.min_similarity:.2f}")
        
        path_list = list(bidirectional_paths.values())
        similar_groups = []
        processed_paths = set()
        
        for i, path1 in enumerate(path_list):
            if path1.path_id in processed_paths:
                continue
            
            current_group = [path1]
            processed_paths.add(path1.path_id)
            
            # 查找与当前路径相似的路径
            for j, path2 in enumerate(path_list[i+1:], i+1):
                if path2.path_id in processed_paths:
                    continue
                
                # 检查是否连接相同的端点
                if not self._connects_same_endpoints(path1, path2):
                    continue
                
                similarity = self._calculate_path_similarity(path1.forward_path, path2.forward_path)
                
                if similarity >= self.min_similarity:
                    current_group.append(path2)
                    processed_paths.add(path2.path_id)
            
            # 只保留确实相似的组（多于1条路径）
            if len(current_group) > 1:
                similar_groups.append(current_group)
        
        print(f"   发现 {len(similar_groups)} 个相似路径组")
        for i, group in enumerate(similar_groups):
            avg_length = sum(self._calculate_path_length(p.forward_path) for p in group) / len(group)
            print(f"     组{i+1}: {len(group)}条路径, 平均长度{avg_length:.1f}m")
        
        return {f"similar_group_{i}": group for i, group in enumerate(similar_groups)}
    
    def _connects_same_endpoints(self, path1, path2) -> bool:
        """检查两条路径是否连接相同的端点"""
        return (
            (path1.point_a['type'] == path2.point_a['type'] and 
             path1.point_a['id'] == path2.point_a['id'] and
             path1.point_b['type'] == path2.point_b['type'] and 
             path1.point_b['id'] == path2.point_b['id']) or
            (path1.point_a['type'] == path2.point_b['type'] and 
             path1.point_a['id'] == path2.point_b['id'] and
             path1.point_b['type'] == path2.point_a['type'] and 
             path1.point_b['id'] == path2.point_a['id'])
        )
    
    def _calculate_path_similarity(self, path1: List[Tuple], path2: List[Tuple]) -> float:
        """计算路径相似度 - 改进算法"""
        if not path1 or not path2:
            return 0.0
        
        # 长度相似性检查
        len1, len2 = len(path1), len(path2)
        length_ratio = min(len1, len2) / max(len1, len2)
        if length_ratio < 0.7:  # 长度差异太大
            return 0.0
        
        # 几何相似性检查
        geometric_similarity = self._calculate_geometric_similarity(path1, path2)
        
        # 端点距离检查
        start_dist = math.sqrt((path1[0][0] - path2[0][0])**2 + (path1[0][1] - path2[0][1])**2)
        end_dist = math.sqrt((path1[-1][0] - path2[-1][0])**2 + (path1[-1][1] - path2[-1][1])**2)
        
        if start_dist > 3.0 or end_dist > 3.0:  # 端点距离超过3米
            return 0.0
        
        # 方向一致性检查
        direction_similarity = self._calculate_direction_similarity(path1, path2)
        
        # 综合相似度
        overall_similarity = (geometric_similarity * 0.5 + 
                            direction_similarity * 0.3 + 
                            length_ratio * 0.2)
        
        return overall_similarity
    
    def _calculate_geometric_similarity(self, path1: List[Tuple], path2: List[Tuple]) -> float:
        """计算几何相似性"""
        # 使用动态时间规整(DTW)思想，允许路径长度不同
        min_len = min(len(path1), len(path2))
        max_len = max(len(path1), len(path2))
        
        # 重采样到相同长度
        resampled_path1 = self._resample_path(path1, min_len)
        resampled_path2 = self._resample_path(path2, min_len)
        
        # 计算点到点的距离
        total_distance = 0.0
        for p1, p2 in zip(resampled_path1, resampled_path2):
            distance = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            total_distance += distance
        
        avg_distance = total_distance / min_len
        
        # 转换为相似度分数（距离越小相似度越高）
        max_tolerance = 2.0  # 2米最大容忍距离
        similarity = max(0.0, 1.0 - avg_distance / max_tolerance)
        
        return similarity
    
    def _calculate_direction_similarity(self, path1: List[Tuple], path2: List[Tuple]) -> float:
        """计算方向相似性"""
        if len(path1) < 2 or len(path2) < 2:
            return 0.0
        
        # 计算整体方向向量
        dir1 = (path1[-1][0] - path1[0][0], path1[-1][1] - path1[0][1])
        dir2 = (path2[-1][0] - path2[0][0], path2[-1][1] - path2[0][1])
        
        # 归一化
        len1 = math.sqrt(dir1[0]**2 + dir1[1]**2)
        len2 = math.sqrt(dir2[0]**2 + dir2[1]**2)
        
        if len1 == 0 or len2 == 0:
            return 0.0
        
        norm_dir1 = (dir1[0]/len1, dir1[1]/len1)
        norm_dir2 = (dir2[0]/len2, dir2[1]/len2)
        
        # 计算余弦相似度
        dot_product = norm_dir1[0] * norm_dir2[0] + norm_dir1[1] * norm_dir2[1]
        similarity = (dot_product + 1) / 2  # 转换到0-1范围
        
        return similarity
    
    def _resample_path(self, path: List[Tuple], target_length: int) -> List[Tuple]:
        """重采样路径到目标长度"""
        if len(path) == target_length:
            return path
        
        if len(path) < 2:
            return path
        
        # 计算累积距离
        cumulative_distances = [0.0]
        for i in range(1, len(path)):
            dist = math.sqrt((path[i][0] - path[i-1][0])**2 + (path[i][1] - path[i-1][1])**2)
            cumulative_distances.append(cumulative_distances[-1] + dist)
        
        total_length = cumulative_distances[-1]
        if total_length == 0:
            return path
        
        # 重采样
        resampled = []
        for i in range(target_length):
            target_dist = (i / (target_length - 1)) * total_length
            
            # 找到对应的路径段
            for j in range(len(cumulative_distances) - 1):
                if cumulative_distances[j] <= target_dist <= cumulative_distances[j + 1]:
                    # 线性插值
                    ratio = ((target_dist - cumulative_distances[j]) / 
                           (cumulative_distances[j + 1] - cumulative_distances[j]))
                    
                    x = path[j][0] + ratio * (path[j + 1][0] - path[j][0])
                    y = path[j][1] + ratio * (path[j + 1][1] - path[j][1])
                    theta = path[j][2] + ratio * (path[j + 1][2] - path[j][2])
                    
                    resampled.append((x, y, theta))
                    break
        
        return resampled if resampled else path
    
    def _calculate_path_length(self, path: List[Tuple]) -> float:
        """计算路径长度"""
        if len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            length += math.sqrt(dx*dx + dy*dy)
        
        return length

class SmartPathMerger:
    """智能路径合并器"""
    
    def __init__(self):
        pass 
        
    def merge_similar_paths(self, similar_groups: Dict, original_paths: Dict) -> Dict[str, ConsolidatedPath]:
        """合并相似路径"""
        print(f"\n🔧 [智能路径合并] 处理 {len(similar_groups)} 个相似组")
        
        consolidated_paths = {}
        
        # 处理相似路径组
        for group_name, similar_paths in similar_groups.items():
            if len(similar_paths) <= 1:
                continue
            
            # 选择最佳代表路径
            representative_path = self._select_representative_path(similar_paths)
            
            # 创建合并后的路径
            merged_path = self._create_merged_path(representative_path, similar_paths, group_name)
            consolidated_paths[merged_path.path_id] = merged_path
            
            print(f"   合并组 {group_name}: {len(similar_paths)}条路径 -> 1条路径")
            print(f"     代表路径: {representative_path.path_id} (质量: {representative_path.quality:.2f})")
        
        # 添加未合并的原始路径
        merged_path_ids = set()
        for group in similar_groups.values():
            merged_path_ids.update(path.path_id for path in group)
        
        for path_id, path_data in original_paths.items():
            if path_id not in merged_path_ids:
                # 转换为ConsolidatedPath格式
                preserved_path = ConsolidatedPath(
                    path_id=path_id,
                    path_type="original",
                    nodes=path_data.forward_path,
                    point_a=path_data.point_a,
                    point_b=path_data.point_b,
                    forward_path=path_data.forward_path,
                    reverse_path=path_data.reverse_path,
                    length=path_data.length,
                    quality=path_data.quality,
                    planner_used=path_data.planner_used,
                    created_time=path_data.created_time,
                    usage_count=path_data.usage_count,
                    current_load=path_data.current_load,
                    max_capacity=path_data.max_capacity,
                    original_paths=[path_id],
                    merge_reason="no_similar_paths_found"
                )
                consolidated_paths[path_id] = preserved_path
        
        print(f"   ✅ 路径合并完成: 保留 {len(consolidated_paths)} 条路径")
        
        return consolidated_paths
    
    def _select_representative_path(self, similar_paths: List):
        """选择代表路径"""
        # 综合评分：质量 + 使用次数 + 路径长度优化
        best_path = None
        best_score = -1
        
        for path in similar_paths:
            # 质量分数 (0-1)
            quality_score = path.quality
            
            # 使用频率分数 (归一化)
            max_usage = max(p.usage_count for p in similar_paths) or 1
            usage_score = path.usage_count / max_usage
            
            # 路径长度分数 (较短的路径得分更高)
            path_lengths = [len(p.forward_path) for p in similar_paths]
            min_length = min(path_lengths)
            length_score = min_length / len(path.forward_path)
            
            # 综合评分
            total_score = quality_score * 0.5 + usage_score * 0.2 + length_score * 0.3
            
            if total_score > best_score:
                best_score = total_score
                best_path = path
        
        return best_path
    
    def _create_merged_path(self, representative_path, similar_paths: List, group_name: str) -> ConsolidatedPath:
        """创建合并后的路径 - 修复版"""
        import time
        
        # 计算合并后的统计信息
        total_usage = sum(p.usage_count for p in similar_paths)
        avg_quality = sum(p.quality for p in similar_paths) / len(similar_paths)
        original_path_ids = [p.path_id for p in similar_paths]
        
        # 选择最佳路径作为几何形状
        best_geometry_path = max(similar_paths, key=lambda p: p.quality)
        
        # 计算质量历史
        quality_history = [avg_quality]
        for path in similar_paths:
            if hasattr(path, 'quality_history') and path.quality_history:
                quality_history.extend(path.quality_history)
        
        # 去重并限制长度
        quality_history = list(set(quality_history))
        if len(quality_history) > 10:
            quality_history = quality_history[-10:]
        
        return ConsolidatedPath(
            path_id=f"merged_{representative_path.point_a['type'][0].upper()}{representative_path.point_a['id']}_to_{representative_path.point_b['type'][0].upper()}{representative_path.point_b['id']}",
            path_type="merged",
            nodes=best_geometry_path.forward_path,
            point_a=representative_path.point_a,
            point_b=representative_path.point_b,
            forward_path=best_geometry_path.forward_path,
            reverse_path=best_geometry_path.reverse_path,
            length=representative_path.length,
            quality=avg_quality,
            planner_used=representative_path.planner_used,
            created_time=time.time(),
            usage_count=total_usage,
            current_load=0,  # 重置负载
            max_capacity=max(p.max_capacity for p in similar_paths),
            
            # 质量历史
            quality_history=quality_history,
            last_quality_update=time.time(),
            
            # 整理信息
            original_paths=original_path_ids,
            consolidation_info={
                'merged_count': len(similar_paths),
                'quality_range': (min(p.quality for p in similar_paths), 
                                max(p.quality for p in similar_paths)),
                'representative_path': representative_path.path_id,
                'merge_reason': 'similar_paths_consolidated',
                'hierarchy_level': 'trunk'  # 默认层次级别
            },
            merge_reason=f"Merged {len(similar_paths)} similar paths from {group_name}"
        )

class ImprovedBackboneNetworkConsolidator:
    """改进的骨干网络整理器"""
    
    def __init__(self, config: Dict = None):
        self.config = {
            'node_overlap_threshold': 0.8,       # 节点重叠阈值(米) - 更保守
            'path_similarity_threshold': 0.92,   # 路径相似度阈值 - 更严格
            'min_overlap_ratio': 0.85,           # 最小重叠比例
            'preserve_connectivity': True,       # 保持连通性
            'enable_quality_validation': True,   # 启用质量验证
            'max_merge_ratio': 0.5,             # 最大合并比例(防止过度合并)
        }
        
        if config:
            self.config.update(config)
        
        self.node_manager = ConservativeNodeClusterManager(
            overlap_threshold=self.config['node_overlap_threshold']
        )
        self.path_analyzer = SmartPathSimilarityAnalyzer(
            min_similarity=self.config['path_similarity_threshold'],
            min_overlap_ratio=self.config['min_overlap_ratio']
        )
        self.path_merger = SmartPathMerger()
        
        self.consolidation_results = {
            'original_paths': {},
            'consolidated_paths': {},
            'consolidation_stats': {},
            'validation_report': {}
        }
        
        print(f"改进骨干网络整理器初始化完成")
        print(f"  节点重叠阈值: {self.config['node_overlap_threshold']}m")
        print(f"  路径相似度阈值: {self.config['path_similarity_threshold']:.2f}")
    
    def consolidate_backbone_network(self, original_backbone_network) -> Dict:
        """改进的骨干网络整理"""
        print(f"\n🚀 开始改进的骨干网络整理...")
        consolidation_start = time.time()
        
        # 1. 获取原始数据
        original_paths = original_backbone_network.bidirectional_paths.copy()
        self.consolidation_results['original_paths'] = original_paths
        
        original_count = len(original_paths)
        print(f"原始网络: {original_count} 条双向路径")
        
        # 2. 验证是否需要整理
        if original_count < 3:
            print("路径数量太少，无需整理")
            self.consolidation_results['consolidated_paths'] = {
                path_id: self._convert_to_consolidated_path(path_data, "preserved")
                for path_id, path_data in original_paths.items()
            }
            return self.consolidation_results
        
        # 3. 保守的节点聚类（主要用于统计）
        consolidated_nodes = self.node_manager.cluster_overlapping_nodes(original_paths)
        
        # 4. 智能路径相似性分析
        similar_groups = self.path_analyzer.find_similar_paths(original_paths)
        
        # 5. 检查合并比例，防止过度整理
        potential_merged_count = len(similar_groups)
        merge_ratio = potential_merged_count / original_count
        
        if merge_ratio > self.config['max_merge_ratio']:
            print(f"⚠️ 预期合并比例过高 ({merge_ratio:.1%})，降低整理强度")
            # 只保留最明显的相似组
            similar_groups = self._filter_most_similar_groups(similar_groups, original_count)
        
        # 6. 智能路径合并
        consolidated_paths = self.path_merger.merge_similar_paths(similar_groups, original_paths)
        
        # 7. 连通性验证
        if self.config['preserve_connectivity']:
            connectivity_check = self._validate_connectivity(original_paths, consolidated_paths)
            if not connectivity_check['is_valid']:
                print("❌ 连通性验证失败，恢复原始路径")
                consolidated_paths = {
                    path_id: self._convert_to_consolidated_path(path_data, "connectivity_preserved")
                    for path_id, path_data in original_paths.items()
                }
        
        # 8. 质量验证
        if self.config['enable_quality_validation']:
            quality_report = self._validate_path_quality(original_paths, consolidated_paths)
            if quality_report['quality_degradation'] > 0.2:  # 质量下降超过20%
                print("⚠️ 质量下降过多，部分恢复原始路径")
                consolidated_paths = self._recover_high_quality_paths(original_paths, consolidated_paths, quality_report)
        
        # 9. 生成统计信息
        consolidation_time = time.time() - consolidation_start
        stats = self._generate_improved_stats(original_paths, consolidated_paths, consolidated_nodes, consolidation_time)
        
        self.consolidation_results.update({
            'consolidated_paths': consolidated_paths,
            'consolidation_stats': stats
        })
        
        # 显示结果
        final_count = len(consolidated_paths)
        reduction_ratio = (original_count - final_count) / original_count
        
        print(f"\n🎉 改进骨干网络整理完成!")
        print(f"   耗时: {consolidation_time:.2f}s")
        print(f"   路径数量: {original_count} -> {final_count}")
        print(f"   整理率: {reduction_ratio:.1%} (保守整理)")
        print(f"   合并的路径组: {len(similar_groups)}")
        print(f"   保留的原始路径: {final_count - len(similar_groups)}")
        
        return self.consolidation_results
    
    def _filter_most_similar_groups(self, similar_groups: Dict, original_count: int) -> Dict:
        """过滤出最相似的路径组"""
        # 按相似度排序，只保留最明显的相似组
        max_groups = max(1, int(original_count * 0.3))  # 最多保留30%的组
        
        # 简化实现：保留路径数量最多的组
        sorted_groups = sorted(similar_groups.items(), 
                             key=lambda x: len(x[1]), reverse=True)
        
        return dict(sorted_groups[:max_groups])
    
    def _validate_connectivity(self, original_paths: Dict, consolidated_paths: Dict) -> Dict:
        """验证连通性"""
        # 检查所有原始连接是否仍然存在
        original_connections = set()
        for path_data in original_paths.values():
            point_a = (path_data.point_a['type'], path_data.point_a['id'])
            point_b = (path_data.point_b['type'], path_data.point_b['id'])
            original_connections.add((point_a, point_b))
            original_connections.add((point_b, point_a))  # 双向
        
        consolidated_connections = set()
        for path_data in consolidated_paths.values():
            point_a = (path_data.point_a['type'], path_data.point_a['id'])
            point_b = (path_data.point_b['type'], path_data.point_b['id'])
            consolidated_connections.add((point_a, point_b))
            consolidated_connections.add((point_b, point_a))  # 双向
        
        missing_connections = original_connections - consolidated_connections
        
        return {
            'is_valid': len(missing_connections) == 0,
            'missing_connections': missing_connections,
            'connectivity_preserved_ratio': len(consolidated_connections) / len(original_connections)
        }
    
    def _validate_path_quality(self, original_paths: Dict, consolidated_paths: Dict) -> Dict:
        """验证路径质量"""
        original_avg_quality = sum(p.quality for p in original_paths.values()) / len(original_paths)
        consolidated_avg_quality = sum(p.quality for p in consolidated_paths.values()) / len(consolidated_paths)
        
        quality_degradation = (original_avg_quality - consolidated_avg_quality) / original_avg_quality
        
        return {
            'original_avg_quality': original_avg_quality,
            'consolidated_avg_quality': consolidated_avg_quality,
            'quality_degradation': quality_degradation,
            'quality_acceptable': quality_degradation <= 0.1  # 10%质量损失可接受
        }
    
    def _recover_high_quality_paths(self, original_paths: Dict, consolidated_paths: Dict, quality_report: Dict):
        """恢复高质量路径"""
        # 找出质量较高但被合并的原始路径，将其恢复
        high_quality_threshold = quality_report['original_avg_quality'] * 0.9
        
        for path_id, path_data in original_paths.items():
            if path_data.quality >= high_quality_threshold:
                # 检查是否被合并了
                path_still_exists = any(path_id in cp.original_paths for cp in consolidated_paths.values())
                
                if path_still_exists and path_data.quality > quality_report['consolidated_avg_quality']:
                    # 恢复这条高质量路径
                    consolidated_paths[path_id] = self._convert_to_consolidated_path(path_data, "quality_recovered")
        
        return consolidated_paths
    
    def _convert_to_consolidated_path(self, original_path, reason: str) -> ConsolidatedPath:
        """将原始路径转换为整理后路径格式"""
        return ConsolidatedPath(
            path_id=original_path.path_id,
            path_type="original",
            nodes=original_path.forward_path,
            point_a=original_path.point_a,
            point_b=original_path.point_b,
            forward_path=original_path.forward_path,
            reverse_path=original_path.reverse_path,
            length=original_path.length,
            quality=original_path.quality,
            planner_used=original_path.planner_used,
            created_time=original_path.created_time,
            usage_count=original_path.usage_count,
            current_load=original_path.current_load,
            max_capacity=original_path.max_capacity,
            original_paths=[original_path.path_id],
            merge_reason=reason
        )
    
    def _generate_improved_stats(self, original_paths: Dict, consolidated_paths: Dict, 
                               consolidated_nodes: Dict, consolidation_time: float) -> Dict:
        """生成改进的统计信息"""
        original_count = len(original_paths)
        final_count = len(consolidated_paths)
        
        # 分类统计
        merged_paths = [p for p in consolidated_paths.values() if p.path_type == "merged"]
        preserved_paths = [p for p in consolidated_paths.values() if p.path_type == "original"]
        
        # 节点统计
        original_node_count = sum(len(p.forward_path) for p in original_paths.values())
        final_node_count = sum(len(p.nodes) for p in consolidated_paths.values())
        
        # 质量统计
        original_avg_quality = sum(p.quality for p in original_paths.values()) / len(original_paths)
        final_avg_quality = sum(p.quality for p in consolidated_paths.values()) / len(consolidated_paths)
        
        return {
            'summary': {
                'original_path_count': original_count,
                'final_path_count': final_count,
                'paths_merged': len(merged_paths),
                'paths_preserved': len(preserved_paths),
                'reduction_ratio': (original_count - final_count) / original_count,
                'consolidation_time': consolidation_time
            },
            'node_analysis': {
                'original_node_count': original_node_count,
                'final_node_count': final_node_count,
                'node_clusters_found': len(consolidated_nodes),
                'node_reduction_ratio': (original_node_count - final_node_count) / original_node_count
            },
            'quality_analysis': {
                'original_avg_quality': original_avg_quality,
                'final_avg_quality': final_avg_quality,
                'quality_change': (final_avg_quality - original_avg_quality) / original_avg_quality,
                'quality_preserved': abs((final_avg_quality - original_avg_quality) / original_avg_quality) < 0.1
            },
            'merge_details': {
                'total_similar_groups_found': len([p for p in consolidated_paths.values() if len(p.original_paths) > 1]),
                'avg_paths_per_group': sum(len(p.original_paths) for p in merged_paths) / len(merged_paths) if merged_paths else 0,
                'max_paths_in_group': max(len(p.original_paths) for p in merged_paths) if merged_paths else 0
            }
        }
    
    def apply_consolidation_to_backbone_network(self, original_backbone_network):
        """将改进的整理结果应用到原始骨干网络对象"""
        if not self.consolidation_results.get('consolidated_paths'):
            print("❌ 没有整理结果，请先执行整理")
            return False
        
        try:
            # 备份原始数据
            original_backbone_network._original_bidirectional_paths = original_backbone_network.bidirectional_paths.copy()
            
            # 替换为整理后的路径
            original_backbone_network.bidirectional_paths = self.consolidation_results['consolidated_paths'].copy()
            
            # 添加整理信息
            original_backbone_network.consolidation_info = {
                'is_consolidated': True,
                'consolidation_type': 'improved_conservative',
                'consolidation_time': time.time(),
                'consolidation_stats': self.consolidation_results['consolidation_stats']
            }
            
            # 重建连接索引
            original_backbone_network._build_connection_index()
            
            print("✅ 改进整理结果已应用到骨干网络")
            return True
            
        except Exception as e:
            print(f"❌ 应用整理结果失败: {e}")
            return False
    
    def get_consolidation_report(self) -> Dict:
        """获取详细的整理报告"""
        if not self.consolidation_results.get('consolidation_stats'):
            return {"error": "没有整理数据"}
        
        stats = self.consolidation_results['consolidation_stats']
        
        return {
            'type': 'improved_conservative_consolidation',
            'summary': stats['summary'],
            'quality_impact': {
                'quality_preserved': stats['quality_analysis']['quality_preserved'],
                'quality_change_percentage': f"{stats['quality_analysis']['quality_change']:.1%}",
                'final_avg_quality': f"{stats['quality_analysis']['final_avg_quality']:.2f}"
            },
            'efficiency_gains': {
                'path_reduction': f"{stats['summary']['reduction_ratio']:.1%}",
                'node_reduction': f"{stats['node_analysis']['node_reduction_ratio']:.1%}",
                'merge_efficiency': f"{stats['merge_details']['avg_paths_per_group']:.1f} paths/group"
            },
            'recommendations': self._generate_improved_recommendations(stats)
        }
    
    def _generate_improved_recommendations(self, stats: Dict) -> List[str]:
        """生成改进的建议"""
        recommendations = []
        
        reduction_ratio = stats['summary']['reduction_ratio']
        quality_change = stats['quality_analysis']['quality_change']
        
        if reduction_ratio < 0.1:
            recommendations.append("整理效果有限，路径间差异较大，这是正常的")
        elif reduction_ratio > 0.4:
            recommendations.append("整理效果显著，成功识别并合并了相似路径")
        
        if quality_change < -0.1:
            recommendations.append("质量有所下降，建议检查合并策略")
        elif quality_change > 0.05:
            recommendations.append("质量有所提升，合并选择了更优路径")
        else:
            recommendations.append("质量保持稳定，整理策略合理")
        
        if stats['merge_details']['max_paths_in_group'] > 5:
            recommendations.append("发现了高度相似的路径组，说明算法收敛性良好")
        
        return recommendations

# ==================== 便捷使用接口 ====================

def improved_consolidate_backbone_network(backbone_network, 
                                        node_threshold=0.8, 
                                        path_similarity=0.92,
                                        apply_immediately=True):
    """
    改进的骨干网络整理便捷接口
    
    Args:
        backbone_network: 骨干网络对象
        node_threshold: 节点重叠阈值(米)，默认0.8米
        path_similarity: 路径相似度阈值，默认0.92
        apply_immediately: 是否立即应用
    
    Returns:
        整理器对象
    """
    config = {
        'node_overlap_threshold': node_threshold,
        'path_similarity_threshold': path_similarity,
        'preserve_connectivity': True,
        'enable_quality_validation': True
    }
    
    consolidator = ImprovedBackboneNetworkConsolidator(config)
    results = consolidator.consolidate_backbone_network(backbone_network)
    
    if apply_immediately:
        success = consolidator.apply_consolidation_to_backbone_network(backbone_network)
        if success:
            print("✅ 改进整理完成并已应用")
        else:
            print("❌ 整理完成但应用失败")
    
    return consolidator

# ==================== 使用示例 ====================

def demo_improved_consolidation():
    """改进整理功能演示"""
    print("改进的骨干网络整理演示")
    
    # 使用改进的整理接口
    # consolidator = improved_consolidate_backbone_network(
    #     backbone_network,
    #     node_threshold=0.8,      # 只合并0.8米内的重叠节点
    #     path_similarity=0.92,    # 只合并92%以上相似的路径
    #     apply_immediately=True
    # )
    
    # 获取详细报告
    # report = consolidator.get_consolidation_report()
    # print("改进整理报告:", report)
    
    pass

if __name__ == "__main__":
    demo_improved_consolidation()