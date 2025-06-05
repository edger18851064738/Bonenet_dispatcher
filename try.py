#!/usr/bin/env python3
"""
enhanced_integrated_gui.py - 修复版：集成冲突控制系统的专业GUI
修复问题：
1. 任务归属验证错误
2. 车辆位置更新逻辑
3. 冲突检测失效
4. 多车共享任务问题
"""

import sys
import os
import math
import time
import json
from typing import Dict, List, Tuple, Optional
import random
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QPropertyAnimation, QEasingCurve, QPointF, pyqtProperty
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QComboBox, QSpinBox, QDoubleSpinBox,
    QProgressBar, QTextEdit, QFileDialog, QMessageBox, QSplitter,
    QGroupBox, QGridLayout, QTableWidget, QTableWidgetItem,
    QGraphicsScene, QGraphicsView, QGraphicsEllipseItem, QDockWidget,
    QGraphicsRectItem, QGraphicsPathItem, QTabWidget, QFrame,
    QSlider, QCheckBox, QLCDNumber, QScrollArea, QTreeWidget, QTreeWidgetItem,
    QListWidget, QGraphicsItemGroup, QGraphicsPolygonItem, QGraphicsLineItem,
    QGraphicsTextItem, QAction, QToolBar, QMenuBar, QMenu, QStatusBar, QDial
)
from PyQt5.QtGui import (
    QPen, QBrush, QColor, QPainter, QPainterPath, QFont, QPixmap, QIcon,
    QLinearGradient, QRadialGradient, QPolygonF, QTransform
)

# 导入系统组件
try:
    from optimized_backbone_network import OptimizedBackboneNetwork
    from optimized_planner_config import EnhancedPathPlannerWithConfig
    from conflict_control import EnhancedBackboneConflictDetector
    from traffic_manager import EnhancedBackboneTrafficManager
    from vehicle_scheduler import (
        EnhancedBackboneVehicleScheduler, 
        TaskPriority, 
        TaskStatus, 
        VehicleStatus
    )
    ENHANCED_COMPONENTS_AVAILABLE = True
    print("✅ 增强冲突控制组件加载成功")
except ImportError as e:
    print(f"⚠️ 增强组件不可用: {e}")
    ENHANCED_COMPONENTS_AVAILABLE = False
    sys.exit(1)

from environment import OptimizedOpenPitMineEnv

# 专业配色方案
PROFESSIONAL_COLORS = {
    'background': QColor(45, 47, 57),
    'surface': QColor(55, 58, 71),
    'primary': QColor(66, 135, 245),
    'secondary': QColor(156, 163, 175),
    'success': QColor(16, 185, 129),
    'warning': QColor(245, 158, 11),
    'error': QColor(239, 68, 68),
    'text': QColor(229, 231, 235),
    'text_muted': QColor(156, 163, 175),
    'border': QColor(75, 85, 99)
}

# 车辆状态配色
VEHICLE_STATUS_COLORS = {
    'idle': QColor(156, 163, 175),
    'loading': QColor(16, 185, 129),
    'unloading': QColor(245, 158, 11),
    'moving': QColor(66, 135, 245),
    'waiting': QColor(168, 85, 247),
    'planning': QColor(244, 63, 94),
    'maintenance': QColor(239, 68, 68),
    'conflict_resolving': QColor(255, 99, 71)
}

class ConflictControlWidget(QWidget):
    """冲突控制组件"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.conflict_detector = None
        self.traffic_manager = None
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 标题
        title = QLabel("冲突控制系统")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: rgb(229, 231, 235);
                padding: 8px;
                background-color: rgb(239, 68, 68);
                border-radius: 4px;
            }
        """)
        layout.addWidget(title)
        
        # 冲突状态
        status_group = QGroupBox("冲突状态")
        status_layout = QVBoxLayout()
        
        self.active_conflicts_label = QLabel("活跃冲突: 0")
        self.resolved_conflicts_label = QLabel("已解决: 0")
        self.success_rate_label = QLabel("解决率: 0%")
        
        status_layout.addWidget(self.active_conflicts_label)
        status_layout.addWidget(self.resolved_conflicts_label)
        status_layout.addWidget(self.success_rate_label)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 解决策略统计
        strategy_group = QGroupBox("解决策略")
        strategy_layout = QVBoxLayout()
        
        self.first_come_count_label = QLabel("先到先行: 0")
        self.priority_count_label = QLabel("优先级抢占: 0")
        self.temporal_count_label = QLabel("时间调整: 0")
        self.backbone_switch_label = QLabel("路径切换: 0")
        
        strategy_layout.addWidget(self.first_come_count_label)
        strategy_layout.addWidget(self.priority_count_label)
        strategy_layout.addWidget(self.temporal_count_label)
        strategy_layout.addWidget(self.backbone_switch_label)
        
        strategy_group.setLayout(strategy_layout)
        layout.addWidget(strategy_group)
        
        # 控制按钮
        control_layout = QHBoxLayout()
        
        self.detect_conflicts_btn = QPushButton("检测冲突")
        self.resolve_all_btn = QPushButton("解决全部")
        self.emergency_clear_btn = QPushButton("紧急清除")
        
        self.emergency_clear_btn.setStyleSheet("""
            QPushButton {
                background-color: rgb(239, 68, 68);
                color: white;
            }
        """)
        
        control_layout.addWidget(self.detect_conflicts_btn)
        control_layout.addWidget(self.resolve_all_btn)
        control_layout.addWidget(self.emergency_clear_btn)
        
        layout.addLayout(control_layout)
        layout.addStretch()
    
    def set_components(self, conflict_detector, traffic_manager):
        """设置组件引用"""
        self.conflict_detector = conflict_detector
        self.traffic_manager = traffic_manager
    
    def update_display(self):
        """更新显示"""
        if not self.conflict_detector or not self.traffic_manager:
            return
        
        try:
            # 获取冲突状态
            active_conflicts = self.conflict_detector.get_active_conflicts()
            system_status = self.traffic_manager.get_system_status()
            
            self.active_conflicts_label.setText(f"活跃冲突: {len(active_conflicts)}")
            self.resolved_conflicts_label.setText(f"已解决: {system_status.get('conflicts_resolved', 0)}")
            
            success_rate = system_status.get('resolution_success_rate', 0) * 100
            self.success_rate_label.setText(f"解决率: {success_rate:.1f}%")
            
            # 策略统计
            perf_metrics = system_status.get('performance_metrics', {})
            self.first_come_count_label.setText(f"先到先行: {perf_metrics.get('first_come_resolutions', 0)}")
            self.priority_count_label.setText(f"优先级抢占: {perf_metrics.get('priority_resolutions', 0)}")
            self.temporal_count_label.setText(f"时间调整: {perf_metrics.get('temporal_adjustments', 0)}")
            self.backbone_switch_label.setText(f"路径切换: {perf_metrics.get('backbone_switches', 0)}")
            
        except Exception as e:
            print(f"更新冲突控制显示失败: {e}")

class BatchTaskWidget(QWidget):
    """批量任务控制组件"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.vehicle_scheduler = None
        self.env = None
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 标题
        title = QLabel("批量任务分配")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: rgb(229, 231, 235);
                padding: 8px;
                background-color: rgb(16, 185, 129);
                border-radius: 4px;
            }
        """)
        layout.addWidget(title)
        
        # 任务配置
        config_group = QGroupBox("任务配置")
        config_layout = QGridLayout()
        
        config_layout.addWidget(QLabel("车辆数量:"), 0, 0)
        self.vehicle_count_spin = QSpinBox()
        self.vehicle_count_spin.setRange(1, 50)
        self.vehicle_count_spin.setValue(5)
        config_layout.addWidget(self.vehicle_count_spin, 0, 1)
        
        config_layout.addWidget(QLabel("任务优先级:"), 1, 0)
        self.priority_combo = QComboBox()
        self.priority_combo.addItems(["低", "普通", "高", "紧急", "关键"])
        self.priority_combo.setCurrentIndex(1)
        config_layout.addWidget(self.priority_combo, 1, 1)
        
        config_layout.addWidget(QLabel("任务模式:"), 2, 0)
        self.task_mode_combo = QComboBox()
        self.task_mode_combo.addItems([
            "装载→卸载→停车",
            "装载→卸载",
            "随机循环",
            "只装载",
            "只卸载"
        ])
        config_layout.addWidget(self.task_mode_combo, 2, 1)
        
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)
        
        # 批量操作按钮
        batch_layout = QVBoxLayout()
        
        self.create_batch_btn = QPushButton("创建批量任务")
        self.assign_all_btn = QPushButton("分配所有任务")
        self.clear_tasks_btn = QPushButton("清除所有任务")
        
        batch_layout.addWidget(self.create_batch_btn)
        batch_layout.addWidget(self.assign_all_btn)
        batch_layout.addWidget(self.clear_tasks_btn)
        
        layout.addLayout(batch_layout)
        
        # 任务统计
        stats_group = QGroupBox("任务统计")
        stats_layout = QVBoxLayout()
        
        self.created_tasks_label = QLabel("已创建: 0")
        self.assigned_tasks_label = QLabel("已分配: 0")
        self.completed_tasks_label = QLabel("已完成: 0")
        self.failed_tasks_label = QLabel("失败: 0")
        
        stats_layout.addWidget(self.created_tasks_label)
        stats_layout.addWidget(self.assigned_tasks_label)
        stats_layout.addWidget(self.completed_tasks_label)
        stats_layout.addWidget(self.failed_tasks_label)
        
        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)
        
        layout.addStretch()
    
    def set_components(self, vehicle_scheduler, env):
        """设置组件引用"""
        self.vehicle_scheduler = vehicle_scheduler
        self.env = env
    
    def update_display(self):
        """更新显示"""
        if not self.vehicle_scheduler:
            return
        
        try:
            stats = self.vehicle_scheduler.get_comprehensive_stats()
            scheduler_stats = stats.get('scheduler_stats', {})
            task_dist = stats.get('task_distribution', {})
            
            self.created_tasks_label.setText(f"已创建: {scheduler_stats.get('total_tasks_created', 0)}")
            self.assigned_tasks_label.setText(f"已分配: {scheduler_stats.get('total_tasks_assigned', 0)}")
            self.completed_tasks_label.setText(f"已完成: {scheduler_stats.get('total_tasks_completed', 0)}")
            self.failed_tasks_label.setText(f"失败: {scheduler_stats.get('total_tasks_failed', 0)}")
            
        except Exception as e:
            print(f"更新批量任务显示失败: {e}")

class EnhancedVehicleGraphicsItem(QGraphicsItemGroup):
    """增强的车辆图形项"""
    
    def __init__(self, vehicle_id, vehicle_data, parent=None):
        super().__init__(parent)
        self.vehicle_id = vehicle_id
        self.vehicle_data = vehicle_data
        self.position = vehicle_data.get('position', (0, 0, 0))
        
        # 创建车辆组件
        self.vehicle_body = QGraphicsPolygonItem(self)
        self.status_indicator = QGraphicsEllipseItem(self)
        self.load_indicator = QGraphicsRectItem(self)
        self.direction_line = QGraphicsLineItem(self)
        self.conflict_indicator = QGraphicsEllipseItem(self)  # 冲突指示器
        
        # 标签
        self.vehicle_label = QGraphicsTextItem(str(vehicle_id), self)
        self.vehicle_label.setDefaultTextColor(PROFESSIONAL_COLORS['text'])
        self.vehicle_label.setFont(QFont("Arial", 2, QFont.Bold))
        
        self.status_text = QGraphicsTextItem("", self)
        self.status_text.setDefaultTextColor(PROFESSIONAL_COLORS['text'])
        self.status_text.setFont(QFont("Arial", 1))
        
        self.setZValue(15)
        self.update_appearance()
        self.update_position()
    
    def update_appearance(self):
        """更新车辆外观（修改版 - 考虑passing_status）"""
        status = self.vehicle_data.get('status', 'idle')
        passing_status = self.vehicle_data.get('passing_status', 0)
        
        # 如果车辆被停车，使用特殊颜色
        if passing_status == 1:
            color = QColor(255, 69, 0)  # 橙红色表示停车
            print(f"车辆 {self.vehicle_id} 显示为停车状态")
        else:
            color = VEHICLE_STATUS_COLORS.get(status, VEHICLE_STATUS_COLORS['idle'])
        
        # 车辆主体
        self.vehicle_body.setBrush(QBrush(color))
        self.vehicle_body.setPen(QPen(color.darker(150), 1))
        
        # 状态指示器
        self.status_indicator.setBrush(QBrush(color.lighter(130)))
        self.status_indicator.setPen(QPen(color.darker(150), 1))
        
        # 冲突指示器
        has_conflict = self.vehicle_data.get('has_conflict', False)
        if has_conflict:
            self.conflict_indicator.setBrush(QBrush(QColor(239, 68, 68, 200)))
            self.conflict_indicator.setPen(QPen(QColor(239, 68, 68), 2))
        else:
            self.conflict_indicator.setBrush(QBrush(Qt.NoBrush))
            self.conflict_indicator.setPen(QPen(Qt.NoPen))
        
        # 状态文本 - 显示passing_status
        if passing_status == 1:
            status_text = "STOPPED"
        else:
            status_text = status.upper()
        
        self.status_text.setPlainText(status_text)
    
    def update_position(self):
        """更新车辆位置"""
        if not self.position or len(self.position) < 3:
            return
        
        x, y, theta = self.position
        
        # 车辆形状
        length, width = 6.0, 3.0
        half_length, half_width = length/2, width/2
        
        # 创建卡车形状
        truck_points = [
            QPointF(half_length, half_width * 0.8),
            QPointF(half_length * 0.8, half_width),
            QPointF(-half_length * 0.8, half_width),
            QPointF(-half_length, half_width * 0.6),
            QPointF(-half_length, -half_width * 0.6),
            QPointF(-half_length * 0.8, -half_width),
            QPointF(half_length * 0.8, -half_width),
            QPointF(half_length, -half_width * 0.8)
        ]
        
        # 应用旋转和平移
        transform = QTransform()
        transform.translate(x, y)
        transform.rotate(math.degrees(theta))
        
        polygon = QPolygonF()
        for point in truck_points:
            polygon.append(transform.map(point))
        
        self.vehicle_body.setPolygon(polygon)
        
        # 更新其他组件位置
        self.vehicle_label.setPos(x - 8, y - 12)
        self.status_text.setPos(x - 6, y + 8)
        self.status_indicator.setRect(x - 1, y - 1, 2, 2)
        self.conflict_indicator.setRect(x - 2, y - 2, 4, 4)
        
        # 负载指示器
        current_load = self.vehicle_data.get('current_load', 0)
        max_load = self.vehicle_data.get('max_load', 100)
        load_ratio = current_load / max_load if max_load > 0 else 0
        self.load_indicator.setRect(x - half_length, y - width/2 - 3, length * load_ratio, 2)
        
        # 方向指示线
        line_length = 4
        end_x = x + line_length * math.cos(theta)
        end_y = y + line_length * math.sin(theta)
        self.direction_line.setLine(x, y, end_x, end_y)
    
    def update_data(self, vehicle_data):
        """更新车辆数据"""
        self.vehicle_data = vehicle_data
        new_position = vehicle_data.get('position', self.position)
        
        self.position = new_position
        self.update_position()
        self.update_appearance()

class EnhancedMineGUI(QMainWindow):
    """增强的露天矿调度系统GUI - 修复版"""
    
    def __init__(self):
        super().__init__()
        
        # 系统组件
        self.env = None
        self.backbone_network = None
        self.path_planner = None
        self.conflict_detector = None
        self.traffic_manager = None
        self.vehicle_scheduler = None
        
        # 状态
        self.is_simulating = False
        self.simulation_time = 0
        self.simulation_speed = 1.0
        self.map_file_path = None
        
        # 任务ID计数器
        self.task_counter = 0
        
        # 调试模式
        self.debug_mode = True
        
        # 初始化界面
        self.init_ui()
        
        # 定时器
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)
        
        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self.simulation_step)
        
        self.stats_timer = QTimer(self)
        self.stats_timer.timeout.connect(self.update_statistics)
        self.stats_timer.start(2000)
        
        print("🔧 初始化修复版GUI（集成调试工具）")
    
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("露天矿多车协同调度系统 - 修复调试版")
        self.setGeometry(100, 100, 1600, 1000)
        
        # 设置样式
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {PROFESSIONAL_COLORS['background'].name()};
                color: {PROFESSIONAL_COLORS['text'].name()};
            }}
            QGroupBox {{
                font-weight: bold;
                border: 1px solid {PROFESSIONAL_COLORS['border'].name()};
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 6px;
                color: {PROFESSIONAL_COLORS['text'].name()};
            }}
            QPushButton {{
                background-color: {PROFESSIONAL_COLORS['primary'].name()};
                color: white;
                border: none;
                padding: 6px 12px;
                border-radius: 4px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {PROFESSIONAL_COLORS['primary'].darker(110).name()};
            }}
        """)
        
        # 中央组件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(8, 8, 8, 8)
        
        # 左侧控制面板
        self.control_panel = self.create_control_panel()
        self.control_panel.setMaximumWidth(300)
        main_layout.addWidget(self.control_panel)
        
        # 中央视图
        self.graphics_view = self.create_graphics_view()
        main_layout.addWidget(self.graphics_view, 1)
        
        # 右侧面板
        right_widget = QTabWidget()
        right_widget.setMaximumWidth(350)
        
        # 冲突控制标签页
        self.conflict_widget = ConflictControlWidget()
        right_widget.addTab(self.conflict_widget, "冲突控制")
        
        # 批量任务标签页
        self.batch_task_widget = BatchTaskWidget()
        right_widget.addTab(self.batch_task_widget, "批量任务")
        
        main_layout.addWidget(right_widget)
        
        # 连接信号（在组件创建之后）
        self.connect_signals()
        
        # 创建菜单和状态栏
        self.create_menu_bar()
        self.create_status_bar()
    
    def connect_signals(self):
        """连接所有信号"""
        # 批量任务按钮连接
        self.batch_task_widget.create_batch_btn.clicked.connect(self.create_batch_tasks)
        self.batch_task_widget.assign_all_btn.clicked.connect(self.assign_all_batch_tasks)
        self.batch_task_widget.clear_tasks_btn.clicked.connect(self.clear_all_tasks)
        
        # 冲突控制按钮连接
        self.conflict_widget.detect_conflicts_btn.clicked.connect(self.detect_conflicts)
        self.conflict_widget.resolve_all_btn.clicked.connect(self.resolve_all_conflicts)
        self.conflict_widget.emergency_clear_btn.clicked.connect(self.emergency_clear_conflicts)
    
    def create_control_panel(self):
        """创建控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setSpacing(12)
        
        # 环境管理
        env_group = QGroupBox("环境管理")
        env_layout = QVBoxLayout()
        
        file_layout = QHBoxLayout()
        self.file_label = QLabel("未选择文件")
        self.file_label.setStyleSheet("""
            QLabel {
                background-color: rgb(45, 47, 57);
                padding: 8px;
                border: 1px solid rgb(75, 85, 99);
                border-radius: 4px;
                color: rgb(156, 163, 175);
            }
        """)
        
        self.browse_btn = QPushButton("浏览文件")
        file_layout.addWidget(self.file_label, 1)
        file_layout.addWidget(self.browse_btn)
        
        env_layout.addLayout(file_layout)
        
        control_layout = QHBoxLayout()
        self.load_btn = QPushButton("加载环境")
        self.save_btn = QPushButton("保存环境")
        control_layout.addWidget(self.load_btn)
        control_layout.addWidget(self.save_btn)
        
        env_layout.addLayout(control_layout)
        env_group.setLayout(env_layout)
        layout.addWidget(env_group)
        
        # 骨干网络
        backbone_group = QGroupBox("骨干网络")
        backbone_layout = QVBoxLayout()
        
        param_layout = QGridLayout()
        param_layout.addWidget(QLabel("质量阈值:"), 0, 0)
        self.quality_spin = QDoubleSpinBox()
        self.quality_spin.setRange(0.1, 1.0)
        self.quality_spin.setSingleStep(0.1)
        self.quality_spin.setValue(0.6)
        param_layout.addWidget(self.quality_spin, 0, 1)
        
        backbone_layout.addLayout(param_layout)
        
        self.generate_btn = QPushButton("生成骨干网络")
        backbone_layout.addWidget(self.generate_btn)
        
        self.backbone_stats_label = QLabel("路径: 0 条")
        self.backbone_stats_label.setStyleSheet("color: rgb(16, 185, 129); font-weight: bold;")
        backbone_layout.addWidget(self.backbone_stats_label)
        
        backbone_group.setLayout(backbone_layout)
        layout.addWidget(backbone_group)
        
        # 任务管理
        task_group = QGroupBox("任务管理")
        task_layout = QVBoxLayout()
        
        priority_layout = QHBoxLayout()
        priority_layout.addWidget(QLabel("优先级:"))
        self.priority_combo = QComboBox()
        self.priority_combo.addItems(["低", "普通", "高", "紧急", "关键"])
        self.priority_combo.setCurrentIndex(1)
        priority_layout.addWidget(self.priority_combo)
        task_layout.addLayout(priority_layout)
        
        assign_layout = QHBoxLayout()
        self.assign_single_btn = QPushButton("分配单个")
        self.assign_all_btn = QPushButton("批量分配")
        assign_layout.addWidget(self.assign_single_btn)
        assign_layout.addWidget(self.assign_all_btn)
        
        task_layout.addLayout(assign_layout)
        task_group.setLayout(task_layout)
        layout.addWidget(task_group)
        
        # 仿真控制
        sim_group = QGroupBox("仿真控制")
        sim_layout = QVBoxLayout()
        
        control_layout = QHBoxLayout()
        self.start_btn = QPushButton("开始")
        self.pause_btn = QPushButton("暂停")
        self.reset_btn = QPushButton("重置")
        
        control_layout.addWidget(self.start_btn)
        control_layout.addWidget(self.pause_btn)
        control_layout.addWidget(self.reset_btn)
        
        sim_layout.addLayout(control_layout)
        
        # 速度控制
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("速度:"))
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(50)
        self.speed_label = QLabel("1.0x")
        
        speed_layout.addWidget(self.speed_slider, 1)
        speed_layout.addWidget(self.speed_label)
        
        sim_layout.addLayout(speed_layout)
        
        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(True)
        sim_layout.addWidget(self.progress_bar)
        
        sim_group.setLayout(sim_layout)
        layout.addWidget(sim_group)
        
        # 调试控制组（新增）
        debug_group = QGroupBox("调试控制")
        debug_layout = QVBoxLayout()
        
        self.debug_conflicts_btn = QPushButton("调试冲突系统")
        self.debug_tasks_btn = QPushButton("调试任务分配")
        self.debug_paths_btn = QPushButton("调试路径占用")
        
        debug_layout.addWidget(self.debug_conflicts_btn)
        debug_layout.addWidget(self.debug_tasks_btn)
        debug_layout.addWidget(self.debug_paths_btn)
        
        debug_group.setLayout(debug_layout)
        layout.addWidget(debug_group)
        
        layout.addStretch()
        
        # 连接信号
        self.browse_btn.clicked.connect(self.browse_file)
        self.load_btn.clicked.connect(self.load_environment)
        self.save_btn.clicked.connect(self.save_environment)
        self.generate_btn.clicked.connect(self.generate_backbone_network)
        
        self.assign_single_btn.clicked.connect(self.assign_single_vehicle)
        self.assign_all_btn.clicked.connect(self.assign_all_vehicles)
        
        self.start_btn.clicked.connect(self.start_simulation)
        self.pause_btn.clicked.connect(self.pause_simulation)
        self.reset_btn.clicked.connect(self.reset_simulation)
        
        self.speed_slider.valueChanged.connect(self.update_simulation_speed)
        
        # 调试按钮连接
        self.debug_conflicts_btn.clicked.connect(self.debug_conflict_system)
        self.debug_tasks_btn.clicked.connect(self.debug_task_assignment)
        self.debug_paths_btn.clicked.connect(self.debug_path_occupations)
        
        return panel
    
    def create_graphics_view(self):
        """创建图形视图"""
        from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene
        
        view = QGraphicsView()
        scene = QGraphicsScene()
        view.setScene(scene)
        
        view.setDragMode(QGraphicsView.RubberBandDrag)
        view.setRenderHint(QPainter.Antialiasing)
        
        view.setStyleSheet(f"""
            QGraphicsView {{
                background-color: {PROFESSIONAL_COLORS['background'].name()};
                border: 1px solid {PROFESSIONAL_COLORS['border'].name()};
            }}
        """)
        
        return view
    
    def create_menu_bar(self):
        """创建菜单栏"""
        menubar = self.menuBar()
        
        # 文件菜单
        file_menu = menubar.addMenu('文件')
        
        open_action = file_menu.addAction('打开地图')
        open_action.setShortcut('Ctrl+O')
        open_action.triggered.connect(self.browse_file)
        
        save_action = file_menu.addAction('保存环境')
        save_action.setShortcut('Ctrl+S')
        save_action.triggered.connect(self.save_environment)
        
        # 冲突菜单
        conflict_menu = menubar.addMenu('冲突控制')
        
        detect_action = conflict_menu.addAction('检测冲突')
        detect_action.setShortcut('F9')
        detect_action.triggered.connect(self.detect_conflicts)
        
        resolve_action = conflict_menu.addAction('解决冲突')
        resolve_action.setShortcut('F10')
        resolve_action.triggered.connect(self.resolve_all_conflicts)
        
        emergency_action = conflict_menu.addAction('紧急清除')
        emergency_action.setShortcut('Ctrl+F10')
        emergency_action.triggered.connect(self.emergency_clear_conflicts)
        
        # 调试菜单（新增）
        debug_menu = menubar.addMenu('调试')
        
        debug_conflict_action = debug_menu.addAction('调试冲突系统')
        debug_conflict_action.setShortcut('F12')
        debug_conflict_action.triggered.connect(self.debug_conflict_system)
        
        debug_task_action = debug_menu.addAction('调试任务分配')
        debug_task_action.setShortcut('Ctrl+F12')
        debug_task_action.triggered.connect(self.debug_task_assignment)
        
        debug_path_action = debug_menu.addAction('调试路径占用')
        debug_path_action.setShortcut('Shift+F12')
        debug_path_action.triggered.connect(self.debug_path_occupations)
    
    def create_status_bar(self):
        """创建状态栏"""
        self.status_bar = self.statusBar()
        
        self.status_label = QLabel("系统就绪")
        self.status_bar.addWidget(self.status_label)
        
        self.status_bar.addPermanentWidget(QLabel(" | "))
        
        self.vehicle_count_label = QLabel("车辆: 0")
        self.status_bar.addPermanentWidget(self.vehicle_count_label)
        
        self.status_bar.addPermanentWidget(QLabel(" | "))
        
        self.conflicts_label = QLabel("冲突: 0")
        self.status_bar.addPermanentWidget(self.conflicts_label)
        
        self.status_bar.addPermanentWidget(QLabel(" | "))
        
        self.sim_time_label = QLabel("时间: 00:00")
        self.status_bar.addPermanentWidget(self.sim_time_label)
    
    # ==================== 系统组件创建 ====================
    
    def create_system_components(self):
        """创建增强系统组件"""
        try:
            print("开始创建增强系统组件...")
            
            # 创建路径规划器
            self.path_planner = EnhancedPathPlannerWithConfig(self.env)
            print("✅ 路径规划器创建成功")
            
            # 创建骨干网络
            self.backbone_network = OptimizedBackboneNetwork(self.env)
            self.backbone_network.set_path_planner(self.path_planner)
            print("✅ 骨干网络创建成功")
            
            # 创建冲突检测器
            self.conflict_detector = EnhancedBackboneConflictDetector(self.backbone_network)
            print("✅ 冲突检测器创建成功")
            
            # 创建交通管理器
            self.traffic_manager = EnhancedBackboneTrafficManager(
                self.env, self.backbone_network, self.conflict_detector
            )
            print("✅ 交通管理器创建成功")
            
            # 创建车辆调度器
            self.vehicle_scheduler = EnhancedBackboneVehicleScheduler(
                self.env, self.backbone_network, self.path_planner, 
                self.conflict_detector, self.traffic_manager
            )
            print("✅ 车辆调度器创建成功")
            
            # 设置组件间引用
            self.path_planner.set_backbone_network(self.backbone_network)
            
            # 初始化车辆状态
            self.vehicle_scheduler.initialize_vehicles()
            
            # 设置GUI组件引用
            self.conflict_widget.set_components(self.conflict_detector, self.traffic_manager)
            self.batch_task_widget.set_components(self.vehicle_scheduler, self.env)
            
            print("🎉 增强系统组件创建完成")
            return True
            
        except Exception as e:
            print(f"❌ 系统组件创建失败: {e}")
            return False
    
    # ==================== 调试工具方法（新增） ====================
    def debug_task_stages(self, task_id=None):
        """调试任务阶段状态"""
        print("\n" + "="*50)
        print("📋 任务阶段调试")
        print("="*50)
        
        if not self.vehicle_scheduler:
            print("❌ 调度器未初始化")
            return
        
        if task_id:
            # 调试特定任务
            if task_id not in self.vehicle_scheduler.tasks:
                print(f"任务 {task_id} 不存在")
                return
            
            task = self.vehicle_scheduler.tasks[task_id]
            print(f"\n🔍 任务详情: {task_id}")
            print(f"任务类型: {task.task_type}")
            print(f"归属车辆: {task.vehicle_id}")
            print(f"当前阶段: {task.current_stage_index}/{len(task.stages)}")
            print(f"路径进度: {getattr(task, 'path_progress', 'N/A')}")
            print(f"状态: {task.status.value}")
            
            for i, stage in enumerate(task.stages):
                marker = "👉" if i == task.current_stage_index else "  "
                status = "✅" if stage.completed else "⏳"
                print(f"{marker} 阶段{i}: {stage.stage.value} @ {stage.target_type}_{stage.target_id} {status}")
                print(f"     位置: ({stage.location[0]:.1f}, {stage.location[1]:.1f})")
        else:
            # 调试所有活跃任务
            active_tasks = []
            for task_id, task in self.vehicle_scheduler.tasks.items():
                if task.status in [TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS]:
                    active_tasks.append((task_id, task))
            
            print(f"📊 活跃任务数量: {len(active_tasks)}")
            
            for task_id, task in active_tasks:
                print(f"\n📝 任务 {task_id}:")
                print(f"   类型: {task.task_type}")
                print(f"   车辆: {task.vehicle_id}")
                print(f"   阶段: {task.current_stage_index}/{len(task.stages)}")
                print(f"   进度: {getattr(task, 'path_progress', 'N/A')}")
                
                current_stage = task.get_current_stage()
                if current_stage:
                    print(f"   当前: {current_stage.stage.value} @ {current_stage.target_type}_{current_stage.target_id}")
        
        print("="*50)
    def debug_conflict_system(self):
        """调试冲突检测系统 - 修复增强版"""
        print("\n" + "="*60)
        print("🔍 冲突检测系统调试 - 修复版")
        print("="*60)
        
        if not self.vehicle_scheduler or not self.conflict_detector:
            print("❌ 系统组件未初始化")
            return
        
        # 1. 检查车辆任务分配
        print("\n📋 车辆任务分配状态:")
        task_assignments = {}
        duplicate_assignments = []
        
        for vehicle_id, vehicle_state in self.vehicle_scheduler.vehicle_states.items():
            task_id = vehicle_state.current_task_id
            if task_id:
                if task_id not in task_assignments:
                    task_assignments[task_id] = []
                task_assignments[task_id].append(vehicle_id)
                
                task = self.vehicle_scheduler.tasks.get(task_id)
                if task:
                    ownership_status = "✅" if task.vehicle_id == vehicle_id else "❌"
                    print(f"  车辆 {vehicle_id}: 任务 {task_id} (归属: {task.vehicle_id}) {ownership_status}")
                    if task.vehicle_id != vehicle_id:
                        print(f"    ⚠️ 归属不匹配!")
        
        # 2. 检查重复分配
        print("\n🚨 重复分配检查:")
        for task_id, vehicles in task_assignments.items():
            if len(vehicles) > 1:
                print(f"  ❌ 任务 {task_id} 被分配给多个车辆: {vehicles}")
                duplicate_assignments.append(task_id)
            else:
                print(f"  ✅ 任务 {task_id} 正常分配给: {vehicles[0]}")
        
        # 3. 检查骨干路径占用
        print("\n🛤️ 骨干路径占用:")
        total_occupations = 0
        conflicting_segments = []
        
        for segment_id, occupations in self.conflict_detector.segment_occupations.items():
            total_occupations += len(occupations)
            if len(occupations) > 1:
                print(f"  ⚠️ 路径段 {segment_id}: {len(occupations)} 个占用")
                conflicting_segments.append(segment_id)
                for occ in occupations:
                    print(f"    车辆 {occ.vehicle_id}: {occ.planned_entry_time:.1f}-{occ.planned_exit_time:.1f}s")
                    
                    # 检查时间重叠
                    for other_occ in occupations:
                        if other_occ.vehicle_id != occ.vehicle_id and occ.overlaps_with(other_occ):
                            overlap = occ.get_overlap_duration(other_occ)
                            print(f"      ❌ 与车辆 {other_occ.vehicle_id} 重叠 {overlap:.1f}s")
            else:
                print(f"  ✅ 路径段 {segment_id}: 1 个占用 (车辆 {occupations[0].vehicle_id})")
        
        print(f"\n📊 总占用记录: {total_occupations}")
        print(f"📊 冲突段数: {len(conflicting_segments)}")
        
        # 4. 强制冲突检测
        print("\n🔍 强制冲突检测:")
        conflicts = self.conflict_detector.detect_backbone_conflicts()
        print(f"  检测结果: {len(conflicts)} 个冲突")
        
        for conflict in conflicts:
            print(f"    冲突 {conflict.conflict_id}:")
            print(f"      车辆: {conflict.conflicting_vehicles}")
            print(f"      路径段: {conflict.segment_id}")
            print(f"      严重程度: {conflict.severity.value}")
            print(f"      优先级顺序: {conflict.priority_order}")
        
        # 5. 车辆位置检查
        print("\n📍 车辆位置:")
        vehicle_positions = {}
        for vehicle_id in self.env.vehicles:
            if vehicle_id in self.vehicle_scheduler.vehicle_states:
                pos = self.vehicle_scheduler.vehicle_states[vehicle_id].current_position
                vehicle_positions[vehicle_id] = pos
                print(f"  车辆 {vehicle_id}: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.3f})")
        
        # 检查位置重叠
        print("\n🔄 位置重叠检查:")
        for vid1, pos1 in vehicle_positions.items():
            for vid2, pos2 in vehicle_positions.items():
                if vid1 < vid2:  # 避免重复检查
                    distance = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
                    if distance < 5.0:  # 距离小于5米认为可能重叠
                        print(f"  ⚠️ 车辆 {vid1} 和 {vid2} 距离过近: {distance:.1f}m")
        
        # 总结
        print(f"\n" + "="*60)
        print("📊 调试总结:")
        print(f"  重复任务分配: {len(duplicate_assignments)} 个")
        print(f"  冲突路径段: {len(conflicting_segments)} 个")
        print(f"  检测到冲突: {len(conflicts)} 个")
        print("="*60)
        
        # 更新状态栏
        self.conflicts_label.setText(f"冲突: {len(conflicts)}")
    
    def debug_task_assignment(self):
        """调试任务分配"""
        print("\n" + "="*50)
        print("📋 任务分配系统调试")
        print("="*50)
        
        if not self.vehicle_scheduler:
            print("❌ 调度器未初始化")
            return
        
        # 获取调试信息
        debug_info = self.vehicle_scheduler.debug_vehicle_assignment_status()
        
        print(f"\n🚗 车辆总览:")
        print(f"  总车辆数: {debug_info['assignment_summary']['total_vehicles']}")
        print(f"  空闲车辆: {debug_info['assignment_summary']['idle_vehicles']}")
        print(f"  忙碌车辆: {debug_info['assignment_summary']['busy_vehicles']}")
        print(f"  可分配车辆: {debug_info['assignment_summary']['available_for_assignment']}")
        
        print(f"\n📝 车辆详情:")
        for vehicle_id, vinfo in debug_info['vehicles'].items():
            status_icon = "✅" if vinfo['available_for_assignment'] else "❌"
            print(f"  {status_icon} 车辆 {vehicle_id}:")
            print(f"      状态: {vinfo['status']}")
            print(f"      位置: ({vinfo['position'][0]:.1f}, {vinfo['position'][1]:.1f})")
            print(f"      任务队列: {vinfo['task_queue_size']}")
            print(f"      当前任务: {vinfo['current_task']}")
            print(f"      完成任务: {vinfo['total_tasks_completed']}")
            print(f"      冲突次数: {vinfo['conflict_involvements']}")
        
        # 分配策略信息
        strategy_info = self.vehicle_scheduler.get_assignment_strategy_info()
        print(f"\n⚙️ 分配策略:")
        print(f"  当前策略: {strategy_info['current_strategy']}")
        print(f"  轮询索引: {strategy_info['last_assigned_vehicle_index']}")
        
        # 任务状态统计
        stats = self.vehicle_scheduler.get_comprehensive_stats()
        task_dist = stats.get('task_distribution', {})
        
        print(f"\n📊 任务统计:")
        for status, count in task_dist.items():
            print(f"  {status}: {count}")
        
        print("="*50)
    
    def debug_path_occupations(self):
        """调试路径占用"""
        print("\n" + "="*50)
        print("🛤️ 路径占用调试")
        print("="*50)
        
        if not self.conflict_detector:
            print("❌ 冲突检测器未初始化")
            return
        
        print(f"\n📈 占用统计:")
        total_segments = len(self.conflict_detector.segment_occupations)
        total_occupations = sum(len(occs) for occs in self.conflict_detector.segment_occupations.values())
        
        print(f"  路径段总数: {total_segments}")
        print(f"  占用记录总数: {total_occupations}")
        
        print(f"\n📋 详细占用:")
        for segment_id, occupations in self.conflict_detector.segment_occupations.items():
            print(f"  路径段 {segment_id}: {len(occupations)} 个占用")
            
            for i, occ in enumerate(occupations):
                print(f"    [{i+1}] 车辆 {occ.vehicle_id}:")
                print(f"        时间: {occ.planned_entry_time:.1f} - {occ.planned_exit_time:.1f}s")
                print(f"        优先级: {occ.vehicle_priority}")
                print(f"        请求时间: {occ.request_time:.1f}")
                
                # 检查与其他占用的重叠
                for j, other_occ in enumerate(occupations):
                    if i != j and occ.overlaps_with(other_occ):
                        overlap = occ.get_overlap_duration(other_occ)
                        print(f"        ⚠️ 与 [{j+1}] 重叠 {overlap:.1f}s")
        
        # 车辆占用索引
        print(f"\n🚗 车辆占用索引:")
        for vehicle_id, segment_ids in self.conflict_detector.vehicle_occupations.items():
            print(f"  车辆 {vehicle_id}: {len(segment_ids)} 个路径段")
            for seg_id in segment_ids:
                print(f"    - {seg_id}")
        
        print("="*50)
    
    # ==================== 主要功能方法 ====================
    
    def browse_file(self):
        """浏览文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "打开地图文件", "", "JSON文件 (*.json);;所有文件 (*)"
        )
        
        if file_path:
            self.map_file_path = file_path
            filename = os.path.basename(file_path)
            self.file_label.setText(filename)
            self.file_label.setStyleSheet("""
                QLabel {
                    background-color: rgb(16, 185, 129);
                    color: white;
                    padding: 8px;
                    border: 1px solid rgb(75, 85, 99);
                    border-radius: 4px;
                }
            """)
    
    def load_environment(self):
        """加载环境"""
        if not self.map_file_path:
            QMessageBox.warning(self, "警告", "请先选择地图文件")
            return
        
        try:
            self.status_label.setText("正在加载环境...")
            
            # 创建环境
            self.env = OptimizedOpenPitMineEnv()
            if not self.env.load_from_file(self.map_file_path):
                raise Exception("环境加载失败")
            
            # 设置到视图
            self.setup_graphics_view()
            
            # 创建系统组件
            if not self.create_system_components():
                raise Exception("系统组件创建失败")
            
            self.status_label.setText("环境加载成功")
            self.enable_controls(True)
            
            # 更新车辆计数
            self.vehicle_count_label.setText(f"车辆: {len(self.env.vehicles)}")
            
        except Exception as e:
            self.status_label.setText("加载失败")
            QMessageBox.critical(self, "错误", f"加载环境失败:\n{str(e)}")
    
    def setup_graphics_view(self):
        """设置图形视图"""
        scene = self.graphics_view.scene()
        scene.clear()
        
        if not self.env:
            return
        
        # 设置场景范围
        scene.setSceneRect(0, 0, self.env.width, self.env.height)
        
        # 绘制背景
        background = QGraphicsRectItem(0, 0, self.env.width, self.env.height)
        background.setBrush(QBrush(PROFESSIONAL_COLORS['background']))
        background.setPen(QPen(Qt.NoPen))
        background.setZValue(-100)
        scene.addItem(background)
        
        # 绘制障碍物
        for x, y in self.env.obstacle_points:
            rect = QGraphicsRectItem(x, y, 1, 1)
            rect.setBrush(QBrush(PROFESSIONAL_COLORS['surface']))
            rect.setPen(QPen(PROFESSIONAL_COLORS['border'], 0.1))
            rect.setZValue(-50)
            scene.addItem(rect)
        
        # 绘制特殊点
        self.draw_special_points()
        
        # 绘制车辆
        self.draw_vehicles()
        
        # 适应视图
        self.graphics_view.fitInView(scene.sceneRect(), Qt.KeepAspectRatio)
    
    def draw_special_points(self):
        """绘制特殊点"""
        scene = self.graphics_view.scene()
        
        # 装载点
        for i, point in enumerate(self.env.loading_points):
            x, y = point[0], point[1]
            area = QGraphicsEllipseItem(x-3, y-3, 6, 6)
            area.setBrush(QBrush(QColor(16, 185, 129, 100)))
            area.setPen(QPen(PROFESSIONAL_COLORS['success'], 2))
            area.setZValue(-20)
            scene.addItem(area)
            
            text = QGraphicsTextItem(f"L{i+1}")
            text.setPos(x-8, y-20)
            text.setDefaultTextColor(PROFESSIONAL_COLORS['success'])
            text.setFont(QFont("Arial", 2, QFont.Bold))
            scene.addItem(text)
        
        # 卸载点
        for i, point in enumerate(self.env.unloading_points):
            x, y = point[0], point[1]
            area = QGraphicsRectItem(x-3, y-3, 6, 6)
            area.setBrush(QBrush(QColor(245, 158, 11, 100)))
            area.setPen(QPen(PROFESSIONAL_COLORS['warning'], 2))
            area.setZValue(-20)
            scene.addItem(area)
            
            text = QGraphicsTextItem(f"U{i+1}")
            text.setPos(x-8, y-20)
            text.setDefaultTextColor(PROFESSIONAL_COLORS['warning'])
            text.setFont(QFont("Arial", 2, QFont.Bold))
            scene.addItem(text)
    
    def draw_vehicles(self):
        """绘制车辆"""
        if not self.env:
            return
        
        scene = self.graphics_view.scene()
        
        # 清除现有车辆和路径
        for item in scene.items():
            if isinstance(item, EnhancedVehicleGraphicsItem):
                scene.removeItem(item)
            elif hasattr(item, 'item_type') and item.item_type == 'vehicle_path':
                scene.removeItem(item)
        
        # 添加车辆
        for vehicle_id, vehicle_info in self.env.vehicles.items():
            # 创建增强车辆数据
            vehicle_data = self.create_enhanced_vehicle_data(vehicle_id, vehicle_info)
            
            vehicle_item = EnhancedVehicleGraphicsItem(vehicle_id, vehicle_data)
            scene.addItem(vehicle_item)
            
            # 绘制车辆路径
            self.draw_vehicle_path(vehicle_id)
    
    def draw_vehicle_path(self, vehicle_id):
        """绘制车辆路径"""
        if not self.vehicle_scheduler:
            return
        
        vehicle_state = self.vehicle_scheduler.vehicle_states.get(vehicle_id)
        if not vehicle_state or not vehicle_state.current_task_id:
            return
        
        task = self.vehicle_scheduler.tasks.get(vehicle_state.current_task_id)
        if not task or not task.complete_path:
            return
        
        scene = self.graphics_view.scene()
        
        # 创建路径
        if len(task.complete_path) >= 2:
            painter_path = QPainterPath()
            painter_path.moveTo(task.complete_path[0][0], task.complete_path[0][1])
            
            for point in task.complete_path[1:]:
                painter_path.lineTo(point[0], point[1])
            
            path_item = QGraphicsPathItem(painter_path)
            
            # 根据任务状态设置颜色
            if task.status.value == 'assigned':
                color = QColor(66, 135, 245, 150)  # 蓝色 - 已分配
            elif task.status.value == 'in_progress':
                color = QColor(16, 185, 129, 150)  # 绿色 - 执行中
            else:
                color = QColor(156, 163, 175, 100)  # 灰色 - 其他
            
            pen = QPen(color, 2)
            pen.setStyle(Qt.DashLine)
            path_item.setPen(pen)
            path_item.setZValue(5)
            path_item.item_type = 'vehicle_path'  # 标记类型
            
            scene.addItem(path_item)
    
    def create_enhanced_vehicle_data(self, vehicle_id, vehicle_info):
        """创建增强车辆数据（修改版 - 包含passing_status）"""
        enhanced_data = {}
        
        if hasattr(vehicle_info, '__dict__'):
            enhanced_data['vehicle_id'] = getattr(vehicle_info, 'vehicle_id', vehicle_id)
            enhanced_data['position'] = getattr(vehicle_info, 'position', (0, 0, 0))
            enhanced_data['status'] = getattr(vehicle_info, 'status', 'idle')
            enhanced_data['current_load'] = getattr(vehicle_info, 'current_load', 0)
            enhanced_data['max_load'] = getattr(vehicle_info, 'max_load', 100)
            
            # === 新增：passing_status ===
            enhanced_data['passing_status'] = getattr(vehicle_info, 'passing_status', 0)
            
        elif isinstance(vehicle_info, dict):
            enhanced_data = vehicle_info.copy()
            enhanced_data['passing_status'] = vehicle_info.get('passing_status', 0)
        
        # 添加冲突信息
        enhanced_data['has_conflict'] = False
        if self.conflict_detector:
            try:
                conflicts = self.conflict_detector.get_vehicle_conflicts(vehicle_id)
                enhanced_data['has_conflict'] = len(conflicts) > 0
            except:
                pass
        
        return enhanced_data
    
    def generate_backbone_network(self):
        """生成骨干网络"""
        if not self.env or not self.backbone_network:
            QMessageBox.warning(self, "警告", "请先加载环境")
            return
        
        try:
            self.status_label.setText("正在生成骨干网络...")
            
            quality_threshold = self.quality_spin.value()
            
            success = self.backbone_network.generate_backbone_network(
                quality_threshold=quality_threshold
            )
            
            if success:
                # 更新可视化
                self.draw_backbone_network()
                
                # 获取网络状态
                network_status = self.backbone_network.get_network_status()
                path_count = network_status['bidirectional_paths']
                
                self.backbone_stats_label.setText(f"路径: {path_count} 条")
                self.status_label.setText("骨干网络生成成功")
                
                QMessageBox.information(self, "成功", 
                    f"骨干网络生成成功\n双向路径: {path_count} 条")
            else:
                self.status_label.setText("生成失败")
                QMessageBox.critical(self, "错误", "骨干网络生成失败")
                
        except Exception as e:
            self.status_label.setText("生成异常")
            QMessageBox.critical(self, "错误", f"生成骨干网络失败:\n{str(e)}")
    
    def draw_backbone_network(self):
        """绘制骨干网络和节点"""
        if not self.backbone_network:
            return
        
        scene = self.graphics_view.scene()
        
        # 绘制双向路径
        for path_id, path_data in self.backbone_network.bidirectional_paths.items():
            if not path_data.forward_path or len(path_data.forward_path) < 2:
                continue
            
            # 获取路径负载
            load_factor = path_data.get_load_factor()
            
            # 根据负载设置颜色
            if load_factor > 0.8:
                color = QColor(239, 68, 68)
                width = 1.0
            elif load_factor > 0.5:
                color = QColor(245, 158, 11)
                width = 1.0
            else:
                color = QColor(66, 135, 245)
                width = 1.0
            
            # 创建路径
            painter_path = QPainterPath()
            painter_path.moveTo(path_data.forward_path[0][0], path_data.forward_path[0][1])
            
            for point in path_data.forward_path[1:]:
                painter_path.lineTo(point[0], point[1])
            
            path_item = QGraphicsPathItem(painter_path)
            pen = QPen(color, width)
            pen.setCapStyle(Qt.RoundCap)
            path_item.setPen(pen)
            path_item.setZValue(-25)
            
            scene.addItem(path_item)
        
        # 绘制关键节点（替换原来的接口节点部分）
        if hasattr(self.backbone_network, 'consolidation_info') and self.backbone_network.consolidation_info:
            key_nodes = self.backbone_network.consolidation_info.get('key_nodes', {})
            
            for i, (node_id, key_node) in enumerate(key_nodes.items()):
                # 获取位置
                if hasattr(key_node, 'position'):
                    x, y = key_node.position[0], key_node.position[1]
                elif isinstance(key_node, dict) and 'position' in key_node:
                    x, y = key_node['position'][0], key_node['position'][1]
                else:
                    continue
                
                # 使用路径颜色（可以选择一个默认颜色）
                color = QColor(66, 135, 245)  # 使用蓝色作为默认
                
                # 创建节点（小圆圈，半径0.5）
                node_circle = QGraphicsEllipseItem(x-0.5, y-0.5, 1.0, 1.0)
                node_circle.setBrush(QBrush(QColor(255, 255, 255, 200)))  # 白色半透明
                node_circle.setPen(QPen(color, 1))
                node_circle.setZValue(-20)  # 在路径之上
                
                scene.addItem(node_circle)
                
                # 可选：添加节点编号文本（很小）
                if i % 2 == 0:  # 每隔一个节点显示编号
                    text_item = QGraphicsTextItem(str(i))
                    text_item.setPos(x + 1, y + 1)
                    text_item.setDefaultTextColor(color)
                    text_item.setFont(QFont("Arial", 1))
                    text_item.setZValue(-15)
                    scene.addItem(text_item)
    
    # ==================== 任务管理 ====================
    
    def assign_single_vehicle(self):
        """分配单个车辆任务"""
        if not self.vehicle_scheduler or not self.env:
            return
        
        # 获取优先级
        priority_map = {0: 1, 1: 2, 2: 3, 3: 4, 4: 5}
        priority_index = self.priority_combo.currentIndex()
        priority_value = priority_map[priority_index]
        
        try:
            priority = TaskPriority(priority_value)
            
            # 选择随机位置
            if self.env.loading_points and self.env.unloading_points:
                import random
                start_location = random.choice(self.env.loading_points)
                end_location = random.choice(self.env.unloading_points)
                
                # 创建任务
                task_id = self.vehicle_scheduler.create_transport_task(
                    start_location=start_location,
                    end_location=end_location,
                    priority=priority
                )
                
                # 分配任务
                success = self.vehicle_scheduler.assign_task(task_id)
                
                if success:
                    self.status_label.setText(f"任务分配成功: {task_id}")
                else:
                    QMessageBox.information(self, "提示", "没有找到合适的车辆")
            else:
                QMessageBox.warning(self, "警告", "没有可用的装载点或卸载点")
                
        except Exception as e:
            QMessageBox.critical(self, "错误", f"任务分配失败: {str(e)}")
    
    def assign_all_vehicles(self):
        """批量分配任务"""
        if not self.vehicle_scheduler or not self.env:
            return
        
        priority_map = {0: 1, 1: 2, 2: 3, 3: 4, 4: 5}
        priority_index = self.priority_combo.currentIndex()
        priority_value = priority_map[priority_index]
        
        try:
            priority = TaskPriority(priority_value)
            assigned_count = 0
            
            if self.env.loading_points and self.env.unloading_points:
                import random
                
                for vehicle_id in self.env.vehicles.keys():
                    start_location = random.choice(self.env.loading_points)
                    end_location = random.choice(self.env.unloading_points)
                    
                    # 创建任务
                    task_id = self.vehicle_scheduler.create_transport_task(
                        start_location=start_location,
                        end_location=end_location,
                        priority=priority,
                        vehicle_id=vehicle_id
                    )
                    
                    # 分配任务
                    success = self.vehicle_scheduler.assign_task(task_id, vehicle_id)
                    
                    if success:
                        assigned_count += 1
                
                self.status_label.setText(f"批量分配完成: {assigned_count}个任务")
                
                QMessageBox.information(self, "批量分配成功", 
                    f"已为 {assigned_count} 个车辆分配任务")
            else:
                QMessageBox.warning(self, "警告", "没有可用的装载点或卸载点")
                
        except Exception as e:
            QMessageBox.critical(self, "错误", f"批量任务分配失败: {str(e)}")
    
    def create_batch_tasks(self):
        """创建批量任务（支持随机循环）"""
        if not self.vehicle_scheduler or not self.env:
            QMessageBox.warning(self, "警告", "请先加载环境")
            return
        
        try:
            vehicle_count = self.batch_task_widget.vehicle_count_spin.value()
            priority_index = self.batch_task_widget.priority_combo.currentIndex()
            task_mode = self.batch_task_widget.task_mode_combo.currentText()
            
            priority_map = {0: 1, 1: 2, 2: 3, 3: 4, 4: 5}
            from vehicle_scheduler import TaskPriority
            priority = TaskPriority(priority_map[priority_index])
            
            created_count = 0
            created_task_ids = []
            
            available_vehicles = list(self.env.vehicles.keys())[:vehicle_count]
            
            print(f"\n🎯 创建批量任务: {task_mode}")
            print(f"   车辆数量: {len(available_vehicles)}")
            print(f"   优先级: {priority.name}")
            
            for vehicle_id in available_vehicles:
                task_id = None
                
                if task_mode == "装载→卸载→停车":
                    task_id = self._create_load_unload_park_task(vehicle_id, priority)
                    
                elif task_mode == "装载→卸载":
                    task_id = self._create_load_unload_task(vehicle_id, priority)
                    
                elif task_mode == "随机循环":
                    # 处理随机循环任务
                    task_id = self._create_random_cycle_task(vehicle_id, priority)
                    
                elif task_mode == "只装载":
                    task_id = self._create_loading_only_task(vehicle_id, priority)
                    
                elif task_mode == "只卸载":
                    task_id = self._create_unloading_only_task(vehicle_id, priority)
                
                if task_id:
                    created_count += 1
                    created_task_ids.append(task_id)
                    print(f"✅ 创建任务 {task_id} 给车辆 {vehicle_id} ({task_mode})")
            
            print(f"\n📊 批量任务创建总结:")
            print(f"   成功创建: {created_count} 个任务")
            print(f"   任务模式: {task_mode}")
            print(f"   任务ID列表: {created_task_ids}")
            
            QMessageBox.information(self, "批量任务创建", 
                f"成功创建 {created_count} 个 {task_mode} 任务\n"
                f"任务ID: {', '.join(created_task_ids[:3])}{'...' if len(created_task_ids) > 3 else ''}")
            
        except Exception as e:
            print(f"❌ 批量任务创建失败: {e}")
            QMessageBox.critical(self, "错误", f"批量任务创建失败: {str(e)}")



    def _create_random_cycle_task(self, vehicle_id: str, priority) -> Optional[str]:
        """创建随机循环任务"""
        try:
            print(f"   🔄 创建随机循环任务给车辆 {vehicle_id}")
            
            # 获取车辆当前位置作为起点
            vehicle_info = self.env.vehicles.get(vehicle_id)
            if not vehicle_info:
                print(f"   ❌ 找不到车辆信息: {vehicle_id}")
                return None
            
            if hasattr(vehicle_info, 'position'):
                start_position = vehicle_info.position
            else:
                start_position = vehicle_info.get('position', (0, 0, 0))
            
            # 确保位置是3D坐标
            if len(start_position) < 3:
                start_position = (*start_position, 0.0)
            
            print(f"     起点位置: ({start_position[0]:.1f}, {start_position[1]:.1f})")
            
            # 随机选择装载点
            if not self.env.loading_points:
                print(f"   ❌ 没有可用的装载点")
                return None
            
            import random
            loading_point = random.choice(self.env.loading_points)
            loading_location = self._ensure_3d_point(loading_point)
            print(f"     随机装载点: ({loading_location[0]:.1f}, {loading_location[1]:.1f})")
            
            # 随机选择卸载点
            if not self.env.unloading_points:
                print(f"   ❌ 没有可用的卸载点")
                return None
            
            unloading_point = random.choice(self.env.unloading_points)
            unloading_location = self._ensure_3d_point(unloading_point)
            print(f"     随机卸载点: ({unloading_location[0]:.1f}, {unloading_location[1]:.1f})")
            
            # 随机选择停车点
            parking_areas = getattr(self.env, 'parking_areas', [])
            if not parking_areas:
                # 如果没有专门的停车区，使用装载点附近作为停车点
                parking_location = self._generate_parking_near_loading(loading_location)
                print(f"     生成停车点: ({parking_location[0]:.1f}, {parking_location[1]:.1f})")
            else:
                parking_point = random.choice(parking_areas)
                parking_location = self._ensure_3d_point(parking_point)
                print(f"     随机停车点: ({parking_location[0]:.1f}, {parking_location[1]:.1f})")
            
            # 使用调度器创建4阶段循环任务
            if hasattr(self.vehicle_scheduler, 'create_random_cycle_task'):
                task_id = self.vehicle_scheduler.create_random_cycle_task(
                    vehicle_id=vehicle_id,
                    start_location=start_position,
                    loading_location=loading_location,
                    unloading_location=unloading_location,
                    parking_location=parking_location,
                    priority=priority,
                    enable_cycle=True  # 启用循环
                )
            else:
                # 如果调度器没有随机循环方法，使用传统方法创建多阶段任务
                print(f"   ⚠️ 调度器不支持随机循环，使用传统多阶段任务")
                task_id = self.vehicle_scheduler.create_transport_task_integrated(
                    start_location=loading_location,
                    end_location=parking_location,
                    priority=priority,
                    vehicle_id=vehicle_id
                )
            
            if task_id:
                print(f"     ✅ 随机循环任务创建成功: {task_id}")
                return task_id
            else:
                print(f"     ❌ 随机循环任务创建失败")
                return None
                
        except Exception as e:
            print(f"   ❌ 创建随机循环任务异常: {e}")
            return None

    def _create_load_unload_park_task(self, vehicle_id: str, priority) -> Optional[str]:
        """创建装载→卸载→停车任务"""
        try:
            if self.env.loading_points and self.env.unloading_points:
                import random
                start_location = random.choice(self.env.loading_points)
                end_location = random.choice(self.env.unloading_points)
                
                task_id = self.vehicle_scheduler.create_transport_task_integrated(
                    start_location=start_location,
                    end_location=end_location,
                    priority=priority,
                    vehicle_id=vehicle_id
                )
                return task_id
        except Exception as e:
            print(f"   ❌ 创建装载→卸载→停车任务失败: {e}")
        return None

    def _create_load_unload_task(self, vehicle_id: str, priority) -> Optional[str]:
        """创建装载→卸载任务"""
        try:
            if self.env.loading_points and self.env.unloading_points:
                import random
                start_location = random.choice(self.env.loading_points)
                end_location = random.choice(self.env.unloading_points)
                
                # 使用现有的任务创建方法
                task_id = self.vehicle_scheduler.create_transport_task(
                    start_location=start_location,
                    end_location=end_location,
                    priority=priority,
                    vehicle_id=vehicle_id
                )
                return task_id
        except Exception as e:
            print(f"   ❌ 创建装载→卸载任务失败: {e}")
        return None

    def _create_loading_only_task(self, vehicle_id: str, priority) -> Optional[str]:
        """创建只装载任务"""
        try:
            if self.env.loading_points:
                import random
                # 获取车辆当前位置
                vehicle_info = self.env.vehicles.get(vehicle_id)
                if vehicle_info:
                    if hasattr(vehicle_info, 'position'):
                        start_pos = vehicle_info.position
                    else:
                        start_pos = vehicle_info.get('position', (0, 0, 0))
                else:
                    start_pos = (0, 0, 0)
                
                loading_location = random.choice(self.env.loading_points)
                
                # 使用现有方法创建简单任务
                task_id = self.vehicle_scheduler.create_transport_task(
                    start_location=start_pos,
                    end_location=loading_location,
                    priority=priority,
                    vehicle_id=vehicle_id
                )
                return task_id
        except Exception as e:
            print(f"   ❌ 创建只装载任务失败: {e}")
        return None

    def _create_unloading_only_task(self, vehicle_id: str, priority) -> Optional[str]:
        """创建只卸载任务"""
        try:
            if self.env.unloading_points:
                import random
                # 获取车辆当前位置
                vehicle_info = self.env.vehicles.get(vehicle_id)
                if vehicle_info:
                    if hasattr(vehicle_info, 'position'):
                        start_pos = vehicle_info.position
                    else:
                        start_pos = vehicle_info.get('position', (0, 0, 0))
                else:
                    start_pos = (0, 0, 0)
                
                unloading_location = random.choice(self.env.unloading_points)
                
                # 使用现有方法创建简单任务
                task_id = self.vehicle_scheduler.create_transport_task(
                    start_location=start_pos,
                    end_location=unloading_location,
                    priority=priority,
                    vehicle_id=vehicle_id
                )
                return task_id
        except Exception as e:
            print(f"   ❌ 创建只卸载任务失败: {e}")
        return None

    def _ensure_3d_point(self, point) -> Tuple[float, float, float]:
        """确保点坐标为3D"""
        if not point:
            return (0.0, 0.0, 0.0)
        elif len(point) >= 3:
            return (float(point[0]), float(point[1]), float(point[2]))
        elif len(point) == 2:
            return (float(point[0]), float(point[1]), 0.0)
        else:
            return (0.0, 0.0, 0.0)

    def _generate_parking_near_loading(self, loading_location: Tuple) -> Tuple[float, float, float]:
        """在装载点附近生成停车位置"""
        import random
        x, y, z = loading_location
        # 在装载点附近10-20米范围内生成随机停车位
        offset_x = random.uniform(-20, 20)
        offset_y = random.uniform(-20, 20)
        return (x + offset_x, y + offset_y, z)
    
    def assign_all_batch_tasks(self):
        """分配所有批量任务"""
        if not self.vehicle_scheduler:
            return
        
        try:
            # 获取所有待分配任务
            assigned_count = 0
            while self.vehicle_scheduler.task_queue:
                task_id = self.vehicle_scheduler.task_queue.popleft()
                if self.vehicle_scheduler.assign_task(task_id):
                    assigned_count += 1
                else:
                    # 重新排队
                    self.vehicle_scheduler.task_queue.append(task_id)
                    break
            
            QMessageBox.information(self, "批量分配完成", 
                f"成功分配 {assigned_count} 个任务")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"批量分配失败: {str(e)}")
    
    def clear_all_tasks(self):
        """清除所有任务"""
        if not self.vehicle_scheduler:
            return
        
        try:
            # 清除任务队列
            self.vehicle_scheduler.task_queue.clear()
            
            # 清除车辆任务
            for vehicle_state in self.vehicle_scheduler.vehicle_states.values():
                vehicle_state.current_task_id = None
                vehicle_state.task_queue.clear()
                vehicle_state.current_status = VehicleStatus.IDLE
            
            QMessageBox.information(self, "清除完成", "所有任务已清除")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"清除任务失败: {str(e)}")
    
    # ==================== 冲突控制 ====================
    
    def detect_conflicts(self):
        """检测冲突"""
        if not self.conflict_detector:
            return
        
        try:
            conflicts = self.conflict_detector.detect_backbone_conflicts()
            
            if conflicts:
                QMessageBox.information(self, "冲突检测", 
                    f"检测到 {len(conflicts)} 个冲突")
            else:
                QMessageBox.information(self, "冲突检测", "未检测到冲突")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"冲突检测失败: {str(e)}")
    
    def resolve_all_conflicts(self):
        """解决所有冲突"""
        if not self.traffic_manager:
            return
        
        try:
            conflicts = self.conflict_detector.get_active_conflicts()
            
            if not conflicts:
                QMessageBox.information(self, "提示", "当前没有活跃冲突")
                return
            
            self.status_label.setText("正在解决冲突...")
            
            # 处理冲突
            results = self.traffic_manager.process_conflicts(conflicts)
            
            success_count = sum(1 for result in results.values() 
                              if result.value == 'success')
            
            QMessageBox.information(self, "冲突解决完成", 
                f"尝试解决 {len(conflicts)} 个冲突\n"
                f"成功解决 {success_count} 个")
            
            self.status_label.setText("冲突解决完成")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"解决冲突失败: {str(e)}")
    
    def emergency_clear_conflicts(self):
        """紧急清除冲突"""
        if not self.traffic_manager:
            return
        
        reply = QMessageBox.question(
            self, '确认紧急操作',
            '这将让所有车辆紧急停车并清除所有冲突\n确定要执行吗？',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            try:
                self.traffic_manager.emergency_clear_all_conflicts()
                QMessageBox.information(self, "紧急操作完成", "所有冲突已清除")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"紧急清除失败: {str(e)}")
    
    # ==================== 仿真控制 ====================
    
    def start_simulation(self):
        """开始仿真"""
        if not self.env:
            return
        
        self.is_simulating = True
        self.start_btn.setEnabled(False)
        self.pause_btn.setEnabled(True)
        
        # 启动定时器
        interval = max(50, int(100 / self.simulation_speed))
        self.sim_timer.start(interval)
        
        self.status_label.setText("仿真运行中...")
    
    def pause_simulation(self):
        """暂停仿真"""
        self.is_simulating = False
        self.start_btn.setEnabled(True)
        self.pause_btn.setEnabled(False)
        
        self.sim_timer.stop()
        self.status_label.setText("仿真已暂停")
    
    def reset_simulation(self):
        """重置仿真"""
        if self.is_simulating:
            self.pause_simulation()
        
        if self.env:
            self.env.reset()
        
        if self.vehicle_scheduler:
            self.vehicle_scheduler.initialize_vehicles()
        
        self.simulation_time = 0
        self.progress_bar.setValue(0)
        
        self.status_label.setText("仿真已重置")
    
    def simulation_step(self):
        """仿真步骤（修复版）"""
        if not self.is_simulating or not self.env:
            return
        
        time_step = 0.5 * self.simulation_speed
        self.simulation_time += time_step
        
        # 更新环境
        self.env.current_time = self.simulation_time
        
        # 更新车辆位置（修复版）
        self.update_vehicle_positions_fixed(time_step)
        
        # 更新调度器
        if self.vehicle_scheduler:
            try:
                self.vehicle_scheduler.update(time_step)
            except Exception as e:
                print(f"调度器更新错误: {e}")
        
        # 更新交通管理器
        if self.traffic_manager:
            try:
                self.traffic_manager.update(time_step)
            except Exception as e:
                print(f"交通管理器更新错误: {e}")
        
        # 更新进度条
        max_time = 1800  # 30分钟
        progress = min(100, int(self.simulation_time * 100 / max_time))
        self.progress_bar.setValue(progress)
        
        # 更新时间显示
        minutes = int(self.simulation_time // 60)
        seconds = int(self.simulation_time % 60)
        self.sim_time_label.setText(f"时间: {minutes:02d}:{seconds:02d}")
        
        if progress >= 100:
            self.pause_simulation()
            QMessageBox.information(self, "完成", "仿真已完成！")
    
    def update_vehicle_positions_fixed(self, time_step):
        """更新车辆位置，让车辆沿路径移动（修复版）"""
        if not self.vehicle_scheduler:
            return
        
        for vehicle_id, vehicle_state in self.vehicle_scheduler.vehicle_states.items():
            if not vehicle_state.current_task_id:
                continue
            
            task = self.vehicle_scheduler.tasks.get(vehicle_state.current_task_id)
            if not task or not task.complete_path or len(task.complete_path) < 2:
                continue
            
            # 修复1: 验证任务归属权
            if task.vehicle_id != vehicle_id:
                if self.debug_mode:
                    print(f"⚠️ 任务归属错误: {task.task_id} 属于车辆 {task.vehicle_id}，但被车辆 {vehicle_id} 引用")
                vehicle_state.current_task_id = None
                continue
            
            # 修复2: 检查任务状态 - 使用正确的枚举引用
            if task.status == TaskStatus.COMPLETED:
                # 任务已完成，清理车辆状态
                vehicle_state.current_task_id = None
                vehicle_state.current_status = VehicleStatus.IDLE
                continue
            
            # 修复3: 检查车辆状态 - 紧急停车时不移动
            if vehicle_state.current_status == VehicleStatus.WAITING:
                # 车辆处于等待状态（紧急停车），跳过移动
                if self.debug_mode:
                    print(f"🛑 车辆 {vehicle_id} 紧急停车中，暂停移动")
                continue
            
            # 修复4: 检查任务状态 - 暂停任务时不移动
            if task.status == TaskStatus.SUSPENDED:
                # 任务被暂停，车辆不移动
                if self.debug_mode:
                    print(f"⏸️ 车辆 {vehicle_id} 任务已暂停，停止移动")
                continue
            
            # 计算车辆应该移动的距离
            speed = vehicle_state.max_speed
            distance_to_move = speed * time_step
            
            # 获取当前进度
            if not hasattr(task, 'path_progress'):
                task.path_progress = 0.0
                task.current_path_index = 0
            
            # 沿路径移动
            new_position = self.move_along_path(
                task.complete_path, 
                task.path_progress, 
                distance_to_move
            )
            
            if new_position:
                # 更新任务进度
                task.path_progress = new_position['progress']
                task.current_path_index = new_position['index']
                
                # 更新车辆位置
                vehicle_state.update_position(new_position['position'], time_step)
                
                # 更新环境中的车辆位置
                if vehicle_id in self.env.vehicles:
                    env_vehicle = self.env.vehicles[vehicle_id]
                    if hasattr(env_vehicle, 'position'):
                        env_vehicle.position = new_position['position']
                    elif hasattr(env_vehicle, '__setitem__'):
                        env_vehicle['position'] = new_position['position']
                
                # 修复3: 使用安全的任务完成处理
                if task.path_progress >= 1.0:
                    self._handle_task_completion_safely(vehicle_id, task)

    
    def _handle_task_completion_safely(self, vehicle_id: str, task):
        """安全地处理任务完成（修复版）"""
        print(f"\n🔄 处理任务完成: 车辆 {vehicle_id}, 任务 {task.task_id}")
        
        # 双重检查任务状态和归属
        if (task.status != TaskStatus.COMPLETED and 
            task.vehicle_id == vehicle_id and
            task.path_progress >= 1.0):
            
            print(f"   进度: {task.path_progress:.2f}")
            print(f"   当前阶段: {task.current_stage_index}/{len(task.stages)}")
            
            # 检查是否是多阶段任务
            if hasattr(task, 'advance_to_next_stage') and not task.is_complete():
                print(f"   推进到下一阶段...")
                
                # 推进到下一阶段
                success = self.vehicle_scheduler.advance_task_stage(task.task_id)
                if success:
                    print(f"   ✅ 阶段推进成功")
                else:
                    print(f"   ❌ 阶段推进失败")
            else:
                print(f"   任务完全完成")
                # 任务真正完成
                task.status = TaskStatus.COMPLETED
                task.completion_time = time.time()
                
                # 更新车辆状态
                vehicle_state = self.vehicle_scheduler.vehicle_states.get(vehicle_id)
                if vehicle_state:
                    vehicle_state.current_task_id = None
                    vehicle_state.current_status = VehicleStatus.IDLE
                    vehicle_state.total_tasks_completed += 1
                
                # 释放骨干网络资源
                if self.backbone_network:
                    self.backbone_network.release_vehicle_from_path(vehicle_id)
                
                # 更新统计
                self.vehicle_scheduler.stats['total_tasks_completed'] += 1
                
                print(f"   🎉 车辆 {vehicle_id} 完成任务 {task.task_id}")
    def move_along_path(self, path, current_progress, distance_to_move):
        """沿路径移动指定距离"""
        if not path or len(path) < 2:
            return None
        
        # 计算总路径长度
        total_length = 0
        segment_lengths = []
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            length = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            segment_lengths.append(length)
            total_length += length
        
        if total_length == 0:
            return None
        
        # 当前位置（基于进度）
        current_distance = current_progress * total_length
        new_distance = min(total_length, current_distance + distance_to_move)
        new_progress = new_distance / total_length
        
        # 找到新位置在哪个路径段
        accumulated_length = 0
        for i, segment_length in enumerate(segment_lengths):
            if accumulated_length + segment_length >= new_distance:
                # 在第i段中
                segment_progress = (new_distance - accumulated_length) / segment_length
                
                # 插值计算位置
                p1, p2 = path[i], path[i + 1]
                x = p1[0] + segment_progress * (p2[0] - p1[0])
                y = p1[1] + segment_progress * (p2[1] - p1[1])
                
                # 计算朝向
                if len(p1) > 2:
                    theta = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
                else:
                    theta = 0.0
                
                return {
                    'position': (x, y, theta),
                    'progress': new_progress,
                    'index': i
                }
            
            accumulated_length += segment_length
        
        # 到达终点
        final_point = path[-1]
        return {
            'position': final_point,
            'progress': 1.0,
            'index': len(path) - 1
        }
    
    def update_simulation_speed(self, value):
        """更新仿真速度"""
        self.simulation_speed = value / 50.0
        self.speed_label.setText(f"{self.simulation_speed:.1f}x")
        
        # 更新定时器间隔
        if self.is_simulating:
            interval = max(50, int(100 / self.simulation_speed))
            self.sim_timer.start(interval)
    
    # ==================== 显示更新 ====================
    
    def update_display(self):
        """更新显示"""
        if not self.env:
            return
        
        # 更新车辆显示
        self.update_vehicles_display()
    
    def update_vehicles_display(self):
        """更新车辆显示"""
        scene = self.graphics_view.scene()
        
        # 清除旧的车辆路径
        for item in scene.items():
            if hasattr(item, 'item_type') and item.item_type == 'vehicle_path':
                scene.removeItem(item)
        
        # 找到现有车辆项
        vehicle_items = {}
        for item in scene.items():
            if isinstance(item, EnhancedVehicleGraphicsItem):
                vehicle_items[item.vehicle_id] = item
        
        # 更新或添加车辆
        for vehicle_id, vehicle_info in self.env.vehicles.items():
            vehicle_data = self.create_enhanced_vehicle_data(vehicle_id, vehicle_info)
            
            if vehicle_id in vehicle_items:
                vehicle_items[vehicle_id].update_data(vehicle_data)
            else:
                vehicle_item = EnhancedVehicleGraphicsItem(vehicle_id, vehicle_data)
                scene.addItem(vehicle_item)
            
            # 重新绘制车辆路径
            self.draw_vehicle_path(vehicle_id)
    
    def update_statistics(self):
        """更新统计信息"""
        # 更新冲突控制显示
        self.conflict_widget.update_display()
        
        # 更新批量任务显示
        self.batch_task_widget.update_display()
        
        # 更新状态栏
        if self.conflict_detector:
            try:
                conflicts = self.conflict_detector.get_active_conflicts()
                self.conflicts_label.setText(f"冲突: {len(conflicts)}")
            except:
                pass
    
    def enable_controls(self, enabled):
        """启用/禁用控件"""
        self.start_btn.setEnabled(enabled)
        self.reset_btn.setEnabled(enabled)
        self.generate_btn.setEnabled(enabled)
        self.assign_single_btn.setEnabled(enabled)
        self.assign_all_btn.setEnabled(enabled)
        self.save_btn.setEnabled(enabled)
    
    def save_environment(self):
        """保存环境"""
        if not self.env:
            QMessageBox.warning(self, "警告", "没有可保存的环境")
            return
        
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存环境", 
            f"enhanced_mine_env_{time.strftime('%Y%m%d_%H%M%S')}.json",
            "JSON文件 (*.json);;所有文件 (*)"
        )
        
        if file_path:
            try:
                if self.env.save_to_file(file_path):
                    self.status_label.setText("环境保存成功")
                    QMessageBox.information(self, "成功", "环境保存成功")
                else:
                    raise Exception("保存失败")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"保存失败:\n{str(e)}")
    
    def closeEvent(self, event):
        """关闭事件"""
        if self.is_simulating:
            reply = QMessageBox.question(
                self, '确认退出',
                '仿真正在运行，确定要退出吗？',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            
            if reply == QMessageBox.No:
                event.ignore()
                return
        
        # 停止所有定时器
        self.update_timer.stop()
        self.sim_timer.stop()
        self.stats_timer.stop()
        
        # 关闭系统组件
        try:
            if self.vehicle_scheduler:
                self.vehicle_scheduler.shutdown()
            if self.traffic_manager:
                self.traffic_manager.shutdown()
        except Exception as e:
            print(f"组件关闭错误: {e}")
        
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("露天矿多车协同调度系统 - 修复调试版")
    app.setApplicationVersion("2.1.0")
    
    # 设置全局样式
    app.setStyleSheet(f"""
        QApplication {{
            font-family: "Microsoft YaHei", "SimHei", Arial, sans-serif;
            font-size: 9pt;
        }}
        QMainWindow {{
            background-color: {PROFESSIONAL_COLORS['background'].name()};
        }}
        QToolTip {{
            background-color: {PROFESSIONAL_COLORS['surface'].name()};
            color: {PROFESSIONAL_COLORS['text'].name()};
            border: 1px solid {PROFESSIONAL_COLORS['border'].name()};
            padding: 4px;
            border-radius: 4px;
        }}
    """)
    
    try:
        main_window = EnhancedMineGUI()
        main_window.show()
        
        print("🚀 修复版露天矿调度系统启动成功")
        print("✨ 修复功能: 任务归属验证、冲突检测、调试工具")
        print("📞 调试快捷键: F12-冲突系统, Ctrl+F12-任务分配, Shift+F12-路径占用")
        
        sys.exit(app.exec_())
        
    except Exception as e:
        print(f"❌ 应用程序启动失败: {e}")
        sys.exit(1)