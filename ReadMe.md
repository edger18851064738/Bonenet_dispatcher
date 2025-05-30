# 露天矿骨干网络多车协同调度系统

## 项目概述

本项目是一个基于骨干网络的露天矿多车协同调度系统，通过智能路径规划、冲突检测与解决、负载均衡等技术，实现露天矿场环境下多车辆的高效协同作业。

## 系统架构

### 核心组件

```
┌─────────────────────────────────────────────────────────────┐
│                    GUI界面层 (gui.py)                        │
├─────────────────────────────────────────────────────────────┤
│  冲突控制组件  │  批量任务管理  │  实时可视化  │  调试工具    │
└─────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────┐
│                    调度管理层                                │
├─────────────────────────────────────────────────────────────┤
│  车辆调度器           │  交通管理器         │  冲突检测器    │
│  (vehicle_scheduler)  │  (traffic_manager)  │ (conflict_control) │
└─────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────┐
│                    路径规划层                                │
├─────────────────────────────────────────────────────────────┤
│  骨干网络管理         │  路径规划器         │  接口管理      │
│  (backbone_network)   │  (path_planner)     │                │
└─────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────┐
│                    环境基础层                                │
├─────────────────────────────────────────────────────────────┤
│  环境管理 (environment.py) - 地图、车辆、障碍物、安全检测   │
└─────────────────────────────────────────────────────────────┘
```

## 主要特性

### 🚗 智能车辆调度
- **多阶段任务支持**：装载→运输→卸载的完整任务流程
- **均衡分配策略**：避免任务集中，提高整体效率
- **优先级管理**：支持5级任务优先级
- **实时状态监控**：车辆位置、任务进度、性能指标

### 🛤️ 骨干网络路径管理
- **双向路径网络**：连接装载点、卸载点、停车区
- **负载均衡**：动态监控路径负载，智能分流
- **路径稳定性**：减少频繁切换，提高系统稳定性
- **质量追踪**：实时评估路径质量，优化选择

### 🚨 冲突检测与解决
- **多层次冲突检测**：
  - 空间安全包络检测
  - 预测性冲突检测
  - 时间重叠检测
  - 全路径冲突分析
- **智能解决策略**：
  - 先到先行 (FCFS)
  - 优先级抢占
  - 时间调整
  - 路径切换
  - 协商式解决

### 🎯 高级路径规划
- **多算法支持**：混合A*、RRT、直线规划
- **渐进式回退**：策略失败时自动降级
- **上下文优化**：根据场景选择最佳参数
- **骨干网络优先**：优先使用预建的高质量路径

### 🛡️ 安全管理
- **车辆安全参数**：长度、宽度、转弯半径、安全边距
- **安全矩形检测**：精确的碰撞检测算法
- **动态安全边距**：根据载重、路况动态调整
- **紧急停车机制**：危险情况下的紧急处理

## 安装与使用

### 环境要求
```
Python >= 3.8
PyQt5 >= 5.15
NumPy >= 1.20
其他依赖见requirements.txt
```

### 快速开始

1. **启动系统**
```bash
python gui.py
```

2. **加载地图**
- 点击"浏览文件"选择地图JSON文件
- 点击"加载环境"初始化环境

3. **生成骨干网络**
- 设置质量阈值（建议0.6-0.8）
- 点击"生成骨干网络"

4. **创建和分配任务**
- 使用"批量任务"功能批量创建任务
- 选择分配策略（推荐"随机均衡"）
- 点击"分配所有任务"

5. **开始仿真**
- 点击"开始"启动仿真
- 调整仿真速度
- 观察车辆运行和冲突解决

## 配置参数

### 骨干网络配置
```python
config = {
    'primary_quality_threshold': 0.7,      # 主要质量阈值
    'interface_spacing': 8,                # 接口节点间距
    'max_capacity': 5,                     # 路径最大容量
    'load_balancing_weight': 0.3,          # 负载均衡权重
}
```

### 冲突检测配置
```python
config = {
    'base_safety_margin': 10.0,           # 基础安全边距
    'prediction_horizon': 300.0,          # 预测时间窗口
    'spatial_check_enabled': True,        # 启用空间检测
    'conflict_detection_interval': 2.0,   # 检测间隔
}
```

### 车辆调度配置
```python
config = {
    'default_assignment_strategy': 'random_balanced',  # 分配策略
    'max_tasks_per_vehicle': 3,                       # 单车最大任务数
    'task_timeout': 3600.0,                           # 任务超时时间
    'safety_time_margin': 5.0,                        # 安全时间边距
}
```

## API参考

### 核心类说明

#### OptimizedOpenPitMineEnv
环境管理器，负责地图、车辆、障碍物管理
```python
env = OptimizedOpenPitMineEnv()
env.load_from_file("map.json")
env.add_vehicle("truck_1", position=(10, 10, 0))
```

#### OptimizedBackboneNetwork
骨干网络管理器，处理路径生成和管理
```python
backbone = OptimizedBackboneNetwork(env)
backbone.generate_backbone_network(quality_threshold=0.7)
path_result = backbone.get_path_from_position_to_target(
    current_pos, "unloading", target_id, vehicle_id
)
```

#### EnhancedBackboneVehicleScheduler
车辆调度器，处理任务创建和分配
```python
scheduler = EnhancedBackboneVehicleScheduler(env, backbone, planner, detector, manager)
task_id = scheduler.create_transport_task(start_loc, end_loc, priority)
success = scheduler.assign_task(task_id, vehicle_id)
```

#### EnhancedBackboneConflictDetector
冲突检测器，检测和报告各类冲突
```python
detector = EnhancedBackboneConflictDetector(backbone)
conflicts = detector.detect_backbone_conflicts()
```

#### EnhancedBackboneTrafficManager
交通管理器，执行冲突解决策略
```python
manager = EnhancedBackboneTrafficManager(env, backbone, detector)
results = manager.process_conflicts(conflicts)
```

## 系统特色功能

### 🔧 调试工具
- **冲突系统调试**：实时监控冲突状态和解决过程
- **任务分配调试**：车辆状态、任务队列、分配策略分析
- **路径占用调试**：骨干路径占用情况和时序分析

### 📊 性能监控
- **实时统计**：成功率、平均时间、资源利用率
- **历史分析**：冲突解决历史、性能趋势
- **效率评估**：车辆效率、路径质量、系统稳定性

### 🎨 可视化界面
- **专业配色**：现代化暗色主题
- **实时更新**：车辆位置、路径、冲突状态
- **交互操作**：缩放、拖拽、状态查看
- **多面板设计**：控制、监控、调试功能分离

## 测试场景

### 基础场景
- 单车点对点运输
- 多车无冲突协同
- 简单装载卸载任务

### 复杂场景
- 高密度多车协同
- 复杂路网环境
- 动态任务调度
- 冲突密集场景

### 压力测试
- 50+车辆同时运行
- 持续任务生成
- 极限负载条件
- 故障恢复测试

## 开发指南

### 扩展新的冲突解决策略
```python
class CustomResolutionStrategy:
    def resolve_conflict(self, conflict):
        # 实现自定义解决逻辑
        pass

# 注册到交通管理器
traffic_manager.register_strategy("custom", CustomResolutionStrategy())
```

### 添加新的分配策略
```python
def custom_assignment_strategy(task):
    # 实现自定义分配逻辑
    return selected_vehicle_id

# 注册到调度器
scheduler.assignment_strategies["custom"] = custom_assignment_strategy
```

### 自定义车辆类型
```python
custom_safety_params = {
    'length': 8.0,
    'width': 3.5,
    'safety_margin': 2.0,
    'turning_radius': 10.0
}
env.add_vehicle("heavy_truck", position, safety_params=custom_safety_params)
```

## 故障排除

### 常见问题

**Q: 骨干网络生成失败**
A: 检查装载点和卸载点是否正确设置，调整质量阈值

**Q: 车辆无法移动**
A: 检查车辆安全参数和环境碰撞检测设置

**Q: 任务分配失败**
A: 检查车辆状态、任务队列容量和分配策略配置

**Q: 冲突无法解决**
A: 启用调试模式，查看冲突详情和解决尝试历史

### 日志分析
系统提供详细的调试输出，关键信息包括：
- `✅` 成功操作
- `❌` 失败操作  
- `⚠️` 警告信息
- `🔧` 系统配置
- `🚨` 冲突事件
- `📊` 统计信息

## 性能优化建议

1. **合理设置参数**：根据实际场景调整安全边距、质量阈值
2. **均衡负载**：使用随机均衡或轮询分配策略
3. **监控资源**：定期检查内存使用和CPU负载
4. **优化路径**：适当增加骨干路径数量，提高网络连通性
5. **调试工具**：定期使用调试功能检查系统健康状态

## 贡献指南

欢迎贡献代码、报告问题或提出改进建议。请遵循以下规范：

1. 代码风格：遵循PEP 8规范
2. 测试：新功能需要包含相应测试
3. 文档：更新相关文档和注释
4. 提交：使用清晰的提交信息

## 许可证

[指定许可证类型]

## 联系方式

- 项目维护者：[姓名]
- 邮箱：[邮箱地址]
- 问题反馈：[GitHub Issues链接]

---

**最后更新：2025年**