"""
node_clustering_professional_consolidator.py - 基于节点聚类的专业道路网络整合器
优化版本：
1. 保护路径端点（装载点、卸载点、停车点）不被聚类
2. 实现多轮聚类策略
3. 增强节点权重计算
"""

import math
import time
import numpy as np
from collections import defaultdict, OrderedDict
from typing import Dict, List, Tuple, Optional, Any, Set
from dataclasses import dataclass, field
from enum import Enum

# 导入eastar.py中的算法用于路径重建
try:
    from eastar import HybridAStarPlanner, MiningOptimizedReedShepp
    EASTAR_AVAILABLE = True
    print("✅ 成功导入eastar.py用于路径重建")
except ImportError as e:
    EASTAR_AVAILABLE = False
    print(f"⚠️ 无法导入eastar.py: {e}")
try:
    from ClothoidCubic import BackbonePathFitter
    CURVE_FITTING_AVAILABLE = True
except ImportError:
    CURVE_FITTING_AVAILABLE = False
    print("⚠️ Clothoid-Cubic曲线拟合模块不可用")
class RoadClass(Enum):
    """道路等级"""
    PRIMARY = "primary"        # 主干道
    SECONDARY = "secondary"    # 次干道  
    SERVICE = "service"        # 作业道

class NodeType(Enum):
    """节点类型"""
    ENDPOINT = "endpoint"      # 端点（不可聚类）
    WAYPOINT = "waypoint"      # 路径点（可聚类）
    KEY_NODE = "key_node"      # 关键节点

@dataclass
class KeyNode:
    """关键节点 - 整合后的节点"""
    node_id: str
    position: Tuple[float, float, float]
    cluster_center: Tuple[float, float, float]
    
    # 节点属性
    node_type: NodeType = NodeType.KEY_NODE
    is_endpoint: bool = False
    endpoint_info: Dict = field(default_factory=dict)  # 端点信息
    
    # 节点属性继承
    original_nodes: List[Tuple] = field(default_factory=list)  # 原始节点列表
    path_memberships: Set[str] = field(default_factory=set)   # 所属路径ID集合
    node_importance: float = 1.0  # 节点重要性（基于所属路径数量）
    
    # 工程属性
    road_class: RoadClass = RoadClass.SECONDARY
    traffic_capacity: int = 80
    safety_rating: float = 1.0
    
    # 连接信息
    connected_nodes: Set[str] = field(default_factory=set)    # 连接的其他关键节点
    backbone_segments: List[str] = field(default_factory=list) # 骨干路径段
    
    def add_original_node(self, node: Tuple, path_id: str, is_endpoint: bool = False):
        """添加原始节点信息"""
        if node not in self.original_nodes:
            self.original_nodes.append(node)
        self.path_memberships.add(path_id)
        
        if is_endpoint:
            self.is_endpoint = True
            self.node_type = NodeType.ENDPOINT
        
        # 更新节点重要性
        self.node_importance = len(self.path_memberships)
        if self.is_endpoint:
            self.node_importance *= 2  # 端点重要性加倍
        
        # 根据路径数量确定道路等级
        if self.is_endpoint or len(self.path_memberships) >= 4:
            self.road_class = RoadClass.PRIMARY
            self.traffic_capacity = 120
        elif len(self.path_memberships) >= 2:
            self.road_class = RoadClass.SECONDARY
            self.traffic_capacity = 80
        else:
            self.road_class = RoadClass.SERVICE
            self.traffic_capacity = 40
    
    def get_average_position(self) -> Tuple[float, float, float]:
        """获取所有原始节点的平均位置"""
        if self.is_endpoint:
            # 端点保持原位置不变
            return self.position
        
        if not self.original_nodes:
            return self.position
        
        sum_x = sum(node[0] for node in self.original_nodes)
        sum_y = sum(node[1] for node in self.original_nodes)
        sum_z = sum(node[2] if len(node) > 2 else 0 for node in self.original_nodes)
        count = len(self.original_nodes)
        
        return (sum_x / count, sum_y / count, sum_z / count)

@dataclass
class ConsolidatedBackbonePath:
    """整合后的骨干路径"""
    path_id: str
    original_path_id: str
    key_nodes: List[str]  # 关键节点ID序列
    
    # 路径属性
    path_length: float = 0.0
    road_class: RoadClass = RoadClass.SECONDARY
    quality_score: float = 0.7
    
    # 原始路径信息保留
    original_endpoints: Tuple = None
    original_quality: float = 0.7
    endpoint_nodes: Dict = field(default_factory=dict)  # 端点节点信息
    
    # 重建路径
    reconstructed_path: List[Tuple] = field(default_factory=list)
    reconstruction_success: bool = False
    
    def calculate_path_properties(self, key_nodes_dict: Dict[str, KeyNode]):
        """计算路径属性"""
        if len(self.key_nodes) < 2:
            return
        
        # 计算总长度
        total_length = 0.0
        for i in range(len(self.key_nodes) - 1):
            node1 = key_nodes_dict.get(self.key_nodes[i])
            node2 = key_nodes_dict.get(self.key_nodes[i + 1])
            
            if node1 and node2:
                pos1 = node1.position
                pos2 = node2.position
                distance = math.sqrt(
                    (pos2[0] - pos1[0])**2 + 
                    (pos2[1] - pos1[1])**2
                )
                total_length += distance
        
        self.path_length = total_length
        
        # 确定道路等级（基于关键节点的最高等级）
        max_importance = 0
        for node_id in self.key_nodes:
            node = key_nodes_dict.get(node_id)
            if node and node.node_importance > max_importance:
                max_importance = node.node_importance
                self.road_class = node.road_class

class NodeClusteringConsolidator:
    """基于节点聚类的专业道路网络整合器"""
    
    def __init__(self, env, config: Dict = None):
        self.env = env
        
        # 整合配置
        self.config = {
            # 多轮聚类配置
            'multi_round_clustering': True,
            'clustering_rounds': [
                {'radius': 5.0, 'name': '第一轮'},
                {'radius': 5.0, 'name': '第二轮'},
                {'radius': 3.0, 'name': '第三轮'}
            ],
            
            # 端点保护
            'protect_endpoints': True,
            'endpoint_buffer_radius': 10.0,  # 端点周围的保护半径
            
            # 聚类参数
            'min_cluster_size': 2,             # 最小聚类大小
            'importance_threshold': 1.5,       # 重要性阈值
            
            # 路径重建参数
            'enable_path_reconstruction': True,
            'reconstruction_quality_threshold': 0.4,
            'max_reconstruction_attempts': 3,
            
            # 质量控制
            'preserve_original_on_failure': True,
            'min_path_quality': 0.3,
            
            # 优化参数
            'enable_node_optimization': True,
            'safety_margin': 3.0,
        }
        
        if config:
            self.config.update(config)
        
        # 核心数据结构
        self.original_paths = {}           # 原始路径数据
        self.key_nodes = {}               # 关键节点字典 {node_id: KeyNode}
        self.consolidated_paths = {}      # 整合后的路径
        self.node_clusters = []           # 节点聚类结果
        
        # 端点信息
        self.endpoint_nodes = {}          # 端点节点 {node_id: endpoint_info}
        self.protected_positions = set()  # 受保护的位置
        
        # 路径重建器
        self.path_reconstructor = None
        if EASTAR_AVAILABLE:
            try:
                self.path_reconstructor = HybridAStarPlanner(
                    env=self.env,
                    vehicle_length=6.0,
                    vehicle_width=3.0,
                    turning_radius=8.0,
                    step_size=2.0,
                    angle_resolution=30
                )
                print("✅ 路径重建器初始化成功")
            except Exception as e:
                print(f"⚠️ 路径重建器初始化失败: {e}")
                self.path_reconstructor = None
        
        # 统计信息
        self.consolidation_stats = {
            'original_nodes_count': 0,
            'endpoint_nodes_count': 0,
            'clusterable_nodes_count': 0,
            'key_nodes_count': 0,
            'node_reduction_ratio': 0.0,
            'clustering_time': 0.0,
            'reconstruction_time': 0.0,
            'paths_reconstructed': 0,
            'reconstruction_success_rate': 0.0
        }
        
        print(f"🔧 基于节点聚类的专业道路整合器初始化完成")
        print(f"  多轮聚类: {'✅' if self.config['multi_round_clustering'] else '❌'}")
        print(f"  端点保护: {'✅' if self.config['protect_endpoints'] else '❌'}")
    
    def consolidate_backbone_network_professional(self, backbone_network):
        """执行基于节点聚类的专业整合"""
        print(f"\n🔧 开始基于节点聚类的专业道路网络整合...")
        start_time = time.time()
        
        try:
            # 阶段1: 提取和分析原始路径
            print(f"\n📊 阶段1: 提取和分析原始路径")
            if not self._extract_original_paths(backbone_network):
                print(f"❌ 原始路径提取失败")
                return False
            
            # 阶段2: 识别和保护端点
            print(f"\n🔒 阶段2: 识别和保护端点")
            self._identify_and_protect_endpoints()
            
            # 阶段3: 多轮节点聚类
            print(f"\n🎯 阶段3: 执行多轮节点聚类")
            clustering_start = time.time()
            if not self._perform_multi_round_clustering():
                print(f"❌ 节点聚类失败")
                return False
            self.consolidation_stats['clustering_time'] = time.time() - clustering_start
            
            # 阶段4: 生成关键节点
            print(f"\n⭐ 阶段4: 生成关键节点")
            if not self._generate_key_nodes():
                print(f"❌ 关键节点生成失败")
                return False
            
            # 阶段5: 重建骨干路径
            print(f"\n🛤️ 阶段5: 重建骨干路径")
            reconstruction_start = time.time()
            if not self._reconstruct_backbone_paths():
                print(f"❌ 路径重建失败")
                return False
            self.consolidation_stats['reconstruction_time'] = time.time() - reconstruction_start
            
            # 阶段6: 应用整合结果
            print(f"\n✅ 阶段6: 应用整合结果")
            if not self._apply_consolidation_to_backbone(backbone_network):
                print(f"❌ 整合结果应用失败")
                return False
            
            total_time = time.time() - start_time
            self._generate_consolidation_report(total_time)
            
            print(f"🎉 基于节点聚类的专业道路网络整合完成!")
            return True
            
        except Exception as e:
            print(f"❌ 专业整合失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _extract_original_paths(self, backbone_network) -> bool:
        """提取原始路径数据"""
        print(f"   提取原始骨干路径...")
        
        if not hasattr(backbone_network, 'bidirectional_paths'):
            print(f"   ❌ 骨干网络缺少bidirectional_paths属性")
            return False
        
        self.original_paths = {}
        original_nodes_count = 0
        
        for path_id, path_data in backbone_network.bidirectional_paths.items():
            if not hasattr(path_data, 'forward_path') or not path_data.forward_path:
                print(f"   ⚠️ 路径 {path_id} 缺少forward_path")
                continue
            
            # 提取路径节点
            forward_path = path_data.forward_path
            processed_nodes = []
            
            for node in forward_path:
                if len(node) >= 3:
                    processed_nodes.append((float(node[0]), float(node[1]), float(node[2])))
                elif len(node) == 2:
                    processed_nodes.append((float(node[0]), float(node[1]), 0.0))
                else:
                    continue
            
            if len(processed_nodes) < 2:
                print(f"   ⚠️ 路径 {path_id} 有效节点太少")
                continue
            
            # 存储原始路径信息
            self.original_paths[path_id] = {
                'path_id': path_id,
                'nodes': processed_nodes,
                'quality': getattr(path_data, 'quality', 0.7),
                'path_data': path_data,
                'endpoints': (processed_nodes[0], processed_nodes[-1])
            }
            
            original_nodes_count += len(processed_nodes)
            
        self.consolidation_stats['original_nodes_count'] = original_nodes_count
        
        print(f"   ✅ 提取完成: {len(self.original_paths)} 条路径, {original_nodes_count} 个节点")
        return len(self.original_paths) > 0
    
    def _identify_and_protect_endpoints(self):
        """识别和保护端点"""
        print(f"   识别路径端点...")
        
        endpoint_count = 0
        
        # 收集所有端点
        for path_id, path_info in self.original_paths.items():
            nodes = path_info['nodes']
            
            if len(nodes) < 2:
                continue
            
            # 起点和终点
            start_point = nodes[0]
            end_point = nodes[-1]
            
            # 创建端点ID
            start_id = f"endpoint_start_{path_id}"
            end_id = f"endpoint_end_{path_id}"
            
            # 记录端点信息
            self.endpoint_nodes[start_id] = {
                'position': start_point,
                'type': 'start',
                'path_id': path_id,
                'paths': {path_id}
            }
            
            self.endpoint_nodes[end_id] = {
                'position': end_point,
                'type': 'end',
                'path_id': path_id,
                'paths': {path_id}
            }
            
            # 添加到保护位置集合
            self.protected_positions.add(start_point)
            self.protected_positions.add(end_point)
            
            endpoint_count += 2
        
        self.consolidation_stats['endpoint_nodes_count'] = endpoint_count
        print(f"   ✅ 识别了 {endpoint_count} 个端点并设置保护")
    
    def _perform_multi_round_clustering(self) -> bool:
        """执行多轮节点聚类"""
        print(f"   开始多轮节点聚类分析...")
        
        # 收集所有可聚类节点（排除端点）
        clusterable_nodes = []
        
        for path_id, path_info in self.original_paths.items():
            nodes = path_info['nodes']
            
            # 跳过起点和终点，只处理中间节点
            for i, node in enumerate(nodes):
                if i == 0 or i == len(nodes) - 1:
                    # 端点，跳过
                    continue
                
                # 检查是否在保护范围内
                if self._is_near_protected_position(node):
                    continue
                
                clusterable_nodes.append({
                    'position': node,
                    'path_id': path_id,
                    'node_index': i,
                    'paths': {path_id}
                })
        
        self.consolidation_stats['clusterable_nodes_count'] = len(clusterable_nodes)
        print(f"   可聚类节点数: {len(clusterable_nodes)}")
        
        if len(clusterable_nodes) == 0:
            print(f"   ⚠️ 没有可聚类的节点")
            self.node_clusters = []
            return True
        
        # 执行多轮聚类
        current_nodes = clusterable_nodes
        
        for round_idx, round_config in enumerate(self.config['clustering_rounds']):
            radius = round_config['radius']
            round_name = round_config['name']
            
            print(f"\n   === {round_name} (半径: {radius}m) ===")
            print(f"   输入节点数: {len(current_nodes)}")
            
            # 执行本轮聚类
            round_clusters = self._perform_single_round_clustering(current_nodes, radius)
            
            print(f"   生成聚类数: {len(round_clusters)}")
            
            # 如果是最后一轮，保存结果
            if round_idx == len(self.config['clustering_rounds']) - 1:
                self.node_clusters = round_clusters
            else:
                # 将聚类转换为下一轮的输入
                current_nodes = self._convert_clusters_to_nodes(round_clusters)
                print(f"   输出节点数: {len(current_nodes)}")
        
        # 统计结果
        if self.node_clusters:
            cluster_sizes = [len(cluster['nodes']) for cluster in self.node_clusters]
            print(f"\n   ✅ 多轮聚类完成: {len(self.node_clusters)} 个最终聚类")
            print(f"   聚类大小分布: 最小{min(cluster_sizes)}, 最大{max(cluster_sizes)}, "
                  f"平均{sum(cluster_sizes)/len(cluster_sizes):.1f}")
        
        return True
    
    def _is_near_protected_position(self, position: Tuple, radius: float = None) -> bool:
        """检查位置是否在保护区域内"""
        if radius is None:
            radius = self.config['endpoint_buffer_radius']
        
        for protected_pos in self.protected_positions:
            distance = self._calculate_distance(position, protected_pos)
            if distance < radius:
                return True
        
        return False
    
    def _perform_single_round_clustering(self, nodes: List[Dict], radius: float) -> List[Dict]:
        """执行单轮聚类"""
        clusters = []
        visited = set()
        
        for i, node in enumerate(nodes):
            if i in visited:
                continue
            
            # 创建新聚类
            cluster = {
                'center': node['position'],
                'nodes': [node],
                'paths': node['paths'].copy()
            }
            visited.add(i)
            
            # 查找聚类半径内的其他节点
            for j, other_node in enumerate(nodes):
                if j in visited:
                    continue
                
                distance = self._calculate_distance(node['position'], other_node['position'])
                
                if distance <= radius:
                    cluster['nodes'].append(other_node)
                    cluster['paths'].update(other_node['paths'])
                    visited.add(j)
            
            # 只保留满足最小大小要求的聚类
            if (len(cluster['nodes']) >= self.config['min_cluster_size'] or
                len(cluster['paths']) >= 2):  # 多路径交汇点总是保留
                clusters.append(cluster)
        
        return clusters
    
    def _convert_clusters_to_nodes(self, clusters: List[Dict]) -> List[Dict]:
        """将聚类转换为节点，用于下一轮聚类"""
        nodes = []
        
        for cluster in clusters:
            # 计算聚类中心
            center = self._calculate_weighted_cluster_center(cluster['nodes'])
            
            # 合并路径信息
            all_paths = set()
            for node in cluster['nodes']:
                all_paths.update(node['paths'])
            
            # 创建代表节点
            representative_node = {
                'position': center,
                'paths': all_paths,
                'original_cluster': cluster
            }
            
            nodes.append(representative_node)
        
        return nodes
    
    def _calculate_weighted_cluster_center(self, nodes: List[Dict]) -> Tuple[float, float, float]:
        """计算加权聚类中心"""
        if not nodes:
            return (0.0, 0.0, 0.0)
        
        sum_x = 0.0
        sum_y = 0.0
        sum_z = 0.0
        total_weight = 0
        
        for node in nodes:
            pos = node['position']
            weight = len(node['paths'])  # 使用路径数量作为权重
            
            sum_x += pos[0] * weight
            sum_y += pos[1] * weight
            sum_z += (pos[2] if len(pos) > 2 else 0) * weight
            total_weight += weight
        
        if total_weight > 0:
            return (sum_x / total_weight, sum_y / total_weight, sum_z / total_weight)
        else:
            # 如果没有权重，使用简单平均
            sum_x = sum(node['position'][0] for node in nodes)
            sum_y = sum(node['position'][1] for node in nodes)
            sum_z = sum(node['position'][2] if len(node['position']) > 2 else 0 for node in nodes)
            count = len(nodes)
            return (sum_x / count, sum_y / count, sum_z / count)
    
    def _generate_key_nodes(self) -> bool:
        """生成关键节点"""
        print(f"   生成关键节点...")
        
        self.key_nodes = {}
        node_id_counter = 0
        
        # 1. 首先添加所有端点作为关键节点
        for endpoint_id, endpoint_info in self.endpoint_nodes.items():
            key_node = KeyNode(
                node_id=endpoint_id,
                position=endpoint_info['position'],
                cluster_center=endpoint_info['position'],
                node_type=NodeType.ENDPOINT,
                is_endpoint=True
            )
            
            key_node.endpoint_info = endpoint_info
            key_node.path_memberships = endpoint_info['paths']
            key_node.node_importance = 10.0  # 端点最高重要性
            key_node.road_class = RoadClass.PRIMARY
            key_node.traffic_capacity = 150
            
            self.key_nodes[endpoint_id] = key_node
        
        print(f"   添加了 {len(self.endpoint_nodes)} 个端点作为关键节点")
        
        # 2. 处理聚类生成的关键节点
        for cluster in self.node_clusters:
            # 计算聚类中心
            cluster_center = self._calculate_weighted_cluster_center(cluster['nodes'])
            
            # 创建关键节点
            key_node_id = f"key_node_{node_id_counter}"
            key_node = KeyNode(
                node_id=key_node_id,
                position=cluster_center,
                cluster_center=cluster_center,
                node_type=NodeType.KEY_NODE
            )
            
            # 添加原始节点信息
            for node in cluster['nodes']:
                original_pos = node['position']
                for path_id in node['paths']:
                    key_node.add_original_node(original_pos, path_id, is_endpoint=False)
            
            # 优化节点位置（可选）
            if self.config['enable_node_optimization']:
                optimized_position = self._optimize_node_position(key_node, cluster)
                key_node.position = optimized_position
            
            self.key_nodes[key_node_id] = key_node
            node_id_counter += 1
        
        self.consolidation_stats['key_nodes_count'] = len(self.key_nodes)
        
        # 计算节点减少比例
        original_count = self.consolidation_stats['original_nodes_count']
        key_count = len(self.key_nodes)
        
        if original_count > 0:
            self.consolidation_stats['node_reduction_ratio'] = (
                1.0 - key_count / original_count
            )
        
        print(f"   ✅ 关键节点生成完成: {len(self.key_nodes)} 个")
        print(f"   节点减少: {original_count} -> {key_count} "
              f"({self.consolidation_stats['node_reduction_ratio']:.1%})")
        
        return len(self.key_nodes) > 0
    
    def _reconstruct_backbone_paths(self) -> bool:
        """重建骨干路径"""
        print(f"   重建骨干路径...")
        
        self.consolidated_paths = {}
        reconstruction_success_count = 0
        
        for path_id, path_info in self.original_paths.items():
            print(f"     重建路径: {path_id}")
            
            # 找到路径对应的关键节点序列
            key_node_sequence = self._find_key_node_sequence_for_path(path_id)
            
            if not key_node_sequence or len(key_node_sequence) < 2:
                print(f"       ⚠️ 无法找到有效的关键节点序列")
                # 保留原路径
                self._preserve_original_path_as_consolidated(path_id, path_info)
                continue
            
            # 创建整合路径对象
            consolidated_path = ConsolidatedBackbonePath(
                path_id=f"consolidated_{path_id}",
                original_path_id=path_id,
                key_nodes=key_node_sequence,
                original_endpoints=path_info['endpoints'],
                original_quality=path_info['quality']
            )
            
            # 记录端点节点
            consolidated_path.endpoint_nodes = {
                'start': key_node_sequence[0],
                'end': key_node_sequence[-1]
            }
            
            # 计算路径属性
            consolidated_path.calculate_path_properties(self.key_nodes)
            
            # 重建路径几何
            if self.config['enable_path_reconstruction']:
                success = self._reconstruct_path_geometry(consolidated_path)
                if success:
                    reconstruction_success_count += 1
                    print(f"       ✅ 路径重建成功")
                else:
                    print(f"       ⚠️ 路径重建失败，使用关键节点直连")
                    self._create_direct_connection_path(consolidated_path)
            else:
                # 直接使用关键节点连接
                self._create_direct_connection_path(consolidated_path)
                consolidated_path.reconstruction_success = True
                reconstruction_success_count += 1
            
            self.consolidated_paths[consolidated_path.path_id] = consolidated_path
        
        # 更新统计
        self.consolidation_stats['paths_reconstructed'] = reconstruction_success_count
        if len(self.original_paths) > 0:
            self.consolidation_stats['reconstruction_success_rate'] = (
                reconstruction_success_count / len(self.original_paths)
            )
        
        print(f"   ✅ 路径重建完成: {reconstruction_success_count}/{len(self.original_paths)} 成功")
        
        return len(self.consolidated_paths) > 0
    
    def _find_key_node_sequence_for_path(self, path_id: str) -> List[str]:
        """为路径找到对应的关键节点序列"""
        path_info = self.original_paths[path_id]
        path_nodes = path_info['nodes']
        
        # 起点和终点的关键节点ID
        start_key_node_id = None
        end_key_node_id = None
        
        # 查找起点和终点对应的关键节点
        for key_node_id, key_node in self.key_nodes.items():
            if key_node.is_endpoint:
                if self._calculate_distance(path_nodes[0], key_node.position) < 0.1:
                    start_key_node_id = key_node_id
                elif self._calculate_distance(path_nodes[-1], key_node.position) < 0.1:
                    end_key_node_id = key_node_id
        
        if not start_key_node_id or not end_key_node_id:
            print(f"       ⚠️ 无法找到端点对应的关键节点")
            return []
        
        # 构建关键节点序列
        key_node_sequence = [start_key_node_id]
        
        # 查找中间的关键节点
        middle_key_nodes = []
        
        for key_node_id, key_node in self.key_nodes.items():
            if (key_node_id != start_key_node_id and 
                key_node_id != end_key_node_id and
                path_id in key_node.path_memberships):
                
                # 找到节点在原路径中的位置
                min_distance = float('inf')
                best_index = -1
                
                for i, path_node in enumerate(path_nodes[1:-1], 1):
                    distance = self._calculate_distance(path_node, key_node.position)
                    if distance < min_distance:
                        min_distance = distance
                        best_index = i
                
                if best_index > 0:
                    middle_key_nodes.append((best_index, key_node_id))
        
        # 按照在原路径中的位置排序
        middle_key_nodes.sort(key=lambda x: x[0])
        
        # 添加中间节点到序列
        for _, key_node_id in middle_key_nodes:
            key_node_sequence.append(key_node_id)
        
        # 添加终点
        key_node_sequence.append(end_key_node_id)
        
        return key_node_sequence
    
    def _reconstruct_path_geometry(self, consolidated_path: ConsolidatedBackbonePath) -> bool:
        """重建路径几何 - 使用曲线拟合"""
        if not CURVE_FITTING_AVAILABLE:
            # 回退到原方法
            return self._reconstruct_path_geometry_original(consolidated_path)
        

        key_node_positions = []
        for key_node_id in consolidated_path.key_nodes:
            key_node = self.key_nodes[key_node_id]
            key_node_positions.append(key_node.position)
        
        if len(key_node_positions) < 2:
            return False
        
        # 使用曲线拟合器
        path_fitter = BackbonePathFitter(self.env)
        
        # 执行拟合
        fitted_path = path_fitter.reconstruct_path_with_curve_fitting(
            key_node_positions,
            road_class=consolidated_path.road_class.value,
            path_quality=consolidated_path.original_quality
        )
        
        if fitted_path and len(fitted_path) >= 2:
            consolidated_path.reconstructed_path = fitted_path
            consolidated_path.reconstruction_success = True
            consolidated_path.quality_score = self._evaluate_reconstructed_path_quality(fitted_path)
            
            print(f"       ✅ 曲线拟合成功: {len(fitted_path)}个点")
            return True

    
    def _create_direct_connection_path(self, consolidated_path: ConsolidatedBackbonePath):
        """创建关键节点的直接连接路径"""
        direct_path = []
        
        for key_node_id in consolidated_path.key_nodes:
            key_node = self.key_nodes[key_node_id]
            direct_path.append(key_node.position)
        
        # 密化路径以提供更好的插值
        densified_path = self._densify_path(direct_path, target_spacing=3.0)
        
        consolidated_path.reconstructed_path = densified_path
        consolidated_path.reconstruction_success = True
        consolidated_path.quality_score = 0.6  # 直接连接的默认质量
    
    def _preserve_original_path_as_consolidated(self, path_id: str, path_info: Dict):
        """将原路径保留为整合路径"""
        consolidated_path = ConsolidatedBackbonePath(
            path_id=f"preserved_{path_id}",
            original_path_id=path_id,
            key_nodes=[],  # 无关键节点
            original_endpoints=path_info['endpoints'],
            original_quality=path_info['quality']
        )
        
        # 直接使用原路径
        consolidated_path.reconstructed_path = path_info['nodes']
        consolidated_path.reconstruction_success = True
        consolidated_path.quality_score = path_info['quality']
        consolidated_path.path_length = self._calculate_path_length(path_info['nodes'])
        
        self.consolidated_paths[consolidated_path.path_id] = consolidated_path
    
    def _apply_consolidation_to_backbone(self, backbone_network) -> bool:
        """应用整合结果到骨干网络"""
        print(f"   应用整合结果到骨干网络...")
        
        # 创建新的骨干路径字典
        new_bidirectional_paths = {}
        
        for consolidated_path in self.consolidated_paths.values():
            if not consolidated_path.reconstructed_path:
                continue
            
            # 创建新的骨干路径对象
            new_path_object = self._create_consolidated_backbone_path_object(consolidated_path)
            
            if new_path_object:
                new_bidirectional_paths[consolidated_path.path_id] = new_path_object
                print(f"     ✅ 整合路径: {consolidated_path.path_id} ({len(consolidated_path.reconstructed_path)} 节点)")
        
        # 更新骨干网络
        backbone_network.bidirectional_paths = new_bidirectional_paths
        
        # 添加整合信息到骨干网络
        backbone_network.consolidation_info = {
            'consolidation_applied': True,
            'consolidation_type': 'node_clustering_professional',
            'key_nodes': self.key_nodes,
            'consolidation_stats': self.consolidation_stats,
            'original_paths_count': len(self.original_paths),
            'consolidated_paths_count': len(new_bidirectional_paths),
            'multi_round_clustering': self.config['multi_round_clustering'],
            'endpoint_protection': self.config['protect_endpoints']
        }
        
        # 重建连接索引
        if hasattr(backbone_network, '_build_connection_index'):
            backbone_network._build_connection_index()
        
        print(f"   ✅ 整合结果已应用: {len(new_bidirectional_paths)} 条整合路径")
        return True
    
    def _create_consolidated_backbone_path_object(self, consolidated_path: ConsolidatedBackbonePath):
        """创建整合后的骨干路径对象"""
        
        class ConsolidatedProfessionalBackbonePath:
            def __init__(self, consolidated_path, key_nodes_dict):
                self.path_id = consolidated_path.path_id
                self.original_path_id = consolidated_path.original_path_id
                
                # 路径数据
                self.forward_path = consolidated_path.reconstructed_path
                self.reverse_path = list(reversed(consolidated_path.reconstructed_path))
                
                # 属性
                self.length = consolidated_path.path_length
                self.quality = consolidated_path.quality_score
                self.planner_used = 'node_clustering_professional_consolidator'
                self.created_time = time.time()
                
                # 负载管理
                self.usage_count = 0
                self.current_load = 0
                self.max_capacity = 5
                
                # 整合特有属性
                self.is_professional_design = True
                self.is_consolidated = True
                self.consolidation_method = 'multi_round_node_clustering'
                self.key_nodes = consolidated_path.key_nodes
                self.road_class = consolidated_path.road_class.value
                self.design_class = 'professional_consolidated'
                self.has_protected_endpoints = True
                
                # 节点信息
                if consolidated_path.key_nodes:
                    first_key_node = key_nodes_dict.get(consolidated_path.key_nodes[0])
                    last_key_node = key_nodes_dict.get(consolidated_path.key_nodes[-1])
                    
                    if first_key_node and last_key_node:
                        self.node_importance = (first_key_node.node_importance + last_key_node.node_importance) / 2
                        self.traffic_capacity = max(first_key_node.traffic_capacity, last_key_node.traffic_capacity)
                
                # 端点信息（从原路径推断）
                if consolidated_path.original_endpoints:
                    start_pos, end_pos = consolidated_path.original_endpoints
                    self.point_a = {'type': 'loading', 'id': 0, 'position': start_pos}
                    self.point_b = {'type': 'unloading', 'id': 0, 'position': end_pos}
                else:
                    # 使用重建路径的端点
                    self.point_a = {'type': 'loading', 'id': 0, 'position': self.forward_path[0]}
                    self.point_b = {'type': 'unloading', 'id': 0, 'position': self.forward_path[-1]}
                
                # 质量追踪
                self.quality_history = [self.quality]
                self.last_quality_update = time.time()
            
            def get_path(self, from_point_type, from_point_id, to_point_type, to_point_id):
                return self.forward_path
            
            def increment_usage(self):
                self.usage_count += 1
            
            def add_vehicle(self, vehicle_id):
                self.current_load += 1
            
            def remove_vehicle(self, vehicle_id):
                self.current_load = max(0, self.current_load - 1)
            
            def get_load_factor(self):
                return self.current_load / self.max_capacity if self.max_capacity > 0 else 0
            
            def update_quality_history(self, new_quality):
                self.quality_history.append(new_quality)
                self.last_quality_update = time.time()
                if len(self.quality_history) > 10:
                    self.quality_history = self.quality_history[-5:]
            
            def get_average_quality(self):
                return sum(self.quality_history) / len(self.quality_history) if self.quality_history else self.quality
        
        return ConsolidatedProfessionalBackbonePath(consolidated_path, self.key_nodes)
    
    def _generate_consolidation_report(self, total_time: float):
        """生成整合报告"""
        print(f"\n   生成整合报告...")
        
        # 道路等级统计
        road_class_dist = {'primary': 0, 'secondary': 0, 'service': 0}
        endpoint_count = 0
        
        for key_node in self.key_nodes.values():
            road_class_dist[key_node.road_class.value] += 1
            if key_node.is_endpoint:
                endpoint_count += 1
        
        for consolidated_path in self.consolidated_paths.values():
            if consolidated_path.road_class:
                road_class_dist[consolidated_path.road_class.value] += 1
        
        # 更新统计
        self.consolidation_stats.update({
            'total_time': total_time,
            'road_class_distribution': road_class_dist,
            'average_path_quality': sum(p.quality_score for p in self.consolidated_paths.values()) / len(self.consolidated_paths) if self.consolidated_paths else 0,
            'key_nodes_importance_distribution': {
                'high': len([n for n in self.key_nodes.values() if n.node_importance >= 3]),
                'medium': len([n for n in self.key_nodes.values() if 1 < n.node_importance < 3]),
                'low': len([n for n in self.key_nodes.values() if n.node_importance <= 1])
            },
            'protected_endpoints': endpoint_count,
            'clustering_rounds': len(self.config['clustering_rounds'])
        })
        
        print(f"   ✅ 整合报告生成完成")
    
    # ==================== 辅助方法 ====================
    
    def _calculate_distance(self, pos1: Tuple, pos2: Tuple) -> float:
        """计算两点间距离"""
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)
    
    def _optimize_node_position(self, key_node: KeyNode, cluster: Dict) -> Tuple[float, float, float]:
        """优化关键节点位置"""
        # 如果是端点，不优化
        if key_node.is_endpoint:
            return key_node.position
        
        # 简单优化：避开障碍物，选择最安全的位置
        candidate_positions = [key_node.get_average_position()]
        
        # 在聚类中心周围生成候选位置
        center = key_node.cluster_center
        for angle in [0, math.pi/2, math.pi, 3*math.pi/2]:
            for radius in [2.0, 4.0]:
                candidate_x = center[0] + radius * math.cos(angle)
                candidate_y = center[1] + radius * math.sin(angle)
                candidate_positions.append((candidate_x, candidate_y, center[2]))
        
        # 选择最安全的位置
        best_position = candidate_positions[0]
        best_score = self._evaluate_position_safety(best_position)
        
        for position in candidate_positions[1:]:
            score = self._evaluate_position_safety(position)
            if score > best_score:
                best_score = score
                best_position = position
        
        return best_position
    
    def _evaluate_position_safety(self, position: Tuple[float, float, float]) -> float:
        """评估位置安全性"""
        x, y = position[0], position[1]
        
        # 检查是否在地图边界内
        if (x < self.config['safety_margin'] or x >= self.env.width - self.config['safety_margin'] or
            y < self.config['safety_margin'] or y >= self.env.height - self.config['safety_margin']):
            return 0.0
        
        # 检查周围障碍物密度
        obstacle_count = 0
        total_cells = 0
        check_radius = 5
        
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                check_x, check_y = int(x + dx), int(y + dy)
                
                if (0 <= check_x < self.env.width and 0 <= check_y < self.env.height):
                    total_cells += 1
                    if hasattr(self.env, 'grid') and self.env.grid[check_x, check_y] == 1:
                        obstacle_count += 1
        
        if total_cells > 0:
            obstacle_density = obstacle_count / total_cells
            return 1.0 - obstacle_density
        
        return 0.5
    
    def _create_line_segment(self, start_pos: Tuple, end_pos: Tuple) -> List[Tuple]:
        """创建两点间的直线段"""
        distance = self._calculate_distance(start_pos, end_pos)
        steps = max(3, int(distance / 2.0))
        
        segment = []
        for i in range(steps + 1):
            t = i / steps
            x = start_pos[0] + t * (end_pos[0] - start_pos[0])
            y = start_pos[1] + t * (end_pos[1] - start_pos[1])
            z = start_pos[2] + t * (end_pos[2] - start_pos[2]) if len(start_pos) > 2 else 0
            segment.append((x, y, z))
        
        return segment
    
    def _densify_path(self, path: List[Tuple], target_spacing: float = 3.0) -> List[Tuple]:
        """密化路径"""
        if len(path) < 2:
            return path
        
        dense_path = [path[0]]
        
        for i in range(len(path) - 1):
            current = path[i]
            next_point = path[i + 1]
            
            distance = self._calculate_distance(current, next_point)
            
            if distance > target_spacing:
                num_inserts = int(distance / target_spacing)
                
                for j in range(1, num_inserts + 1):
                    t = j / (num_inserts + 1)
                    x = current[0] + t * (next_point[0] - current[0])
                    y = current[1] + t * (next_point[1] - current[1])
                    z = current[2] + t * (next_point[2] - current[2]) if len(current) > 2 else 0
                    dense_path.append((x, y, z))
            
            dense_path.append(next_point)
        
        return dense_path
    
    def _calculate_path_length(self, path: List[Tuple]) -> float:
        """计算路径长度"""
        if len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            length += self._calculate_distance(path[i], path[i + 1])
        
        return length
    
    def _evaluate_reconstructed_path_quality(self, path: List[Tuple]) -> float:
        """评估重建路径质量"""
        if len(path) < 2:
            return 0.0
        
        # 基础质量评估
        quality_score = 0.6  # 基础分数
        
        # 路径平滑度
        if len(path) >= 3:
            smoothness = self._calculate_path_smoothness(path)
            quality_score += smoothness * 0.2
        
        # 路径可行性（简单碰撞检测）
        collision_free_ratio = self._check_path_collision_free_ratio(path)
        quality_score += collision_free_ratio * 0.2
        
        return min(1.0, quality_score)
    
    def _calculate_path_smoothness(self, path: List[Tuple]) -> float:
        """计算路径平滑度"""
        if len(path) < 3:
            return 1.0
        
        total_angle_change = 0
        for i in range(1, len(path) - 1):
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
        return math.exp(-avg_angle_change * 2.0)
    
    def _check_path_collision_free_ratio(self, path: List[Tuple]) -> float:
        """检查路径无碰撞比例"""
        if not hasattr(self.env, 'grid'):
            return 1.0
        
        collision_free_count = 0
        total_checked = 0
        
        for point in path[::3]:  # 采样检查
            x, y = int(point[0]), int(point[1])
            total_checked += 1
            
            if (0 <= x < self.env.width and 0 <= y < self.env.height and
                self.env.grid[x, y] == 0):
                collision_free_count += 1
        
        return collision_free_count / max(1, total_checked)
    
    # ==================== 公共接口方法 ====================
    
    def get_consolidation_stats(self) -> Dict:
        """获取整合统计信息"""
        return self.consolidation_stats.copy()
    
    def get_key_nodes_info(self) -> Dict:
        """获取关键节点信息"""
        nodes_info = {}
        for node_id, key_node in self.key_nodes.items():
            nodes_info[node_id] = {
                'position': key_node.position,
                'importance': key_node.node_importance,
                'road_class': key_node.road_class.value,
                'path_memberships': list(key_node.path_memberships),
                'traffic_capacity': key_node.traffic_capacity,
                'original_nodes_count': len(key_node.original_nodes),
                'is_endpoint': key_node.is_endpoint,
                'node_type': key_node.node_type.value
            }
        return nodes_info
    
    def get_consolidated_paths_info(self) -> Dict:
        """获取整合路径信息"""
        paths_info = {}
        for path_id, consolidated_path in self.consolidated_paths.items():
            paths_info[path_id] = {
                'original_path_id': consolidated_path.original_path_id,
                'key_nodes': consolidated_path.key_nodes,
                'path_length': consolidated_path.path_length,
                'road_class': consolidated_path.road_class.value,
                'quality_score': consolidated_path.quality_score,
                'reconstruction_success': consolidated_path.reconstruction_success,
                'node_count': len(consolidated_path.reconstructed_path),
                'has_protected_endpoints': True
            }
        return paths_info

# 便捷创建函数
def create_node_clustering_consolidator(env, config=None):
    """创建基于节点聚类的专业道路整合器"""
    default_config = {
        'multi_round_clustering': True,
        'clustering_rounds': [
            {'radius': 5.0, 'name': '第一轮'},
            {'radius': 5.0, 'name': '第二轮'},
            {'radius': 3.0, 'name': '第三轮'}
        ],
        'protect_endpoints': True,
        'endpoint_buffer_radius': 10.0,
        'enable_path_reconstruction': True,
        'enable_node_optimization': True,
        'preserve_original_on_failure': True
    }
    
    if config:
        default_config.update(config)
    
    return NodeClusteringConsolidator(env, default_config)

def apply_node_clustering_consolidation(backbone_network, env, consolidation_mode='balanced'):
    """应用基于节点聚类的专业整合到骨干网络"""
    mode_configs = {
        'aggressive': {
            'clustering_rounds': [
                {'radius': 8.0, 'name': '第一轮'},
                {'radius': 6.0, 'name': '第二轮'},
                {'radius': 4.0, 'name': '第三轮'}
            ],
            'enable_path_reconstruction': True,
            'reconstruction_quality_threshold': 0.3,
        },
        'balanced': {
            'clustering_rounds': [
                {'radius': 5.0, 'name': '第一轮'},
                {'radius': 5.0, 'name': '第二轮'},
                {'radius': 3.0, 'name': '第三轮'}
            ],
            'enable_path_reconstruction': True,
            'reconstruction_quality_threshold': 0.4,
        },
        'conservative': {
            'clustering_rounds': [
                {'radius': 3.0, 'name': '第一轮'},
                {'radius': 3.0, 'name': '第二轮'},
                {'radius': 2.0, 'name': '第三轮'}
            ],
            'enable_path_reconstruction': True,
            'reconstruction_quality_threshold': 0.5,
        }
    }
    
    config = mode_configs.get(consolidation_mode, mode_configs['balanced'])
    config['protect_endpoints'] = True  # 始终保护端点
    
    consolidator = NodeClusteringConsolidator(env, config)
    success = consolidator.consolidate_backbone_network_professional(backbone_network)
    
    if success:
        print(f"✅ 基于节点聚类的专业道路整合成功 (模式: {consolidation_mode})")
        stats = consolidator.get_consolidation_stats()
        print(f"   节点减少: {stats['node_reduction_ratio']:.1%}")
        print(f"   端点保护: {stats['protected_endpoints']} 个")
        print(f"   重建成功率: {stats['reconstruction_success_rate']:.1%}")
        return consolidator
    else:
        print(f"❌ 基于节点聚类的专业道路整合失败")
        return None

# 替换原有的整合器
OptimizedProfessionalMiningRoadConsolidator = NodeClusteringConsolidator