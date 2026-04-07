# FAR Planner Navigation System

这是一个基于ROS的自主导航系统，包含地形分析、全局规划和局部规划模块。

## 1. 环境依赖 (Prerequisites)

- Ubuntu 20.04 (推荐)
- ROS Noetic
- PCL (Point Cloud Library)

## 2. 编译 (Build)

请确保你的工作空间名为 `far_planner` 并位于主目录下，或者根据实际路径调整以下命令。

```bash
cd far_planner
catkin_make
```

## 3. 运行指南 (Usage)

本仓库提供两种启动方式：

- 推荐：统一启动（task5）
- 兼容：分模块启动（传统方式）

### 3.1 推荐：统一启动 (task5)

统一启动会同时拉起：

- `terrain_analysis`
- `local_planner`
- `far_planner`
- person_tracker 兼容话题桥接

```bash
cd far_planner
source devel/setup.bash
roslaunch far_planner task5_farplanner.launch
```

可选参数示例：

```bash
roslaunch far_planner task5_farplanner.launch far_config:=default odom_topic:=/Odometry scan_cloud_topic:=/cloud_registered terrain_topic:=/terrain_map
```

默认话题约定：

- 里程计输入：`/Odometry`
- 点云输入：`/cloud_registered`
- 地形图：`/terrain_map`

person_tracker 兼容桥接（默认开启）：

- `/Odometry -> /local_odom`（relay）
- `/move_base_simple/goal (PoseStamped) -> /way_point (PointStamped)`

如不需要兼容桥接，可关闭：

```bash
roslaunch far_planner task5_farplanner.launch enable_person_tracker_topic_bridge:=false
```

### 3.2 兼容：分模块启动 (传统方式)

请在不同终端窗口分别运行以下模块。

### 3.2.1 地形分析 (Terrain Analysis)

该模块负责分析环境地形，提供可通行区域信息。

```bash
cd far_planner
source devel/setup.bash
roslaunch terrain_analysis terrain_analysis.launch
```

### 3.2.2 宏观大脑 (FAR Planner)

全局规划器，负责生成宏观路径。

```bash
cd far_planner
source devel/setup.bash
roslaunch far_planner far_planner.launch
```

### 3.2.3 微观司机 (Local Planner)

局部规划器，负责具体的运动控制和避障。

```bash
cd far_planner
source devel/setup.bash
roslaunch local_planner local_planner.launch
```

## 4. 可视化交互 (Visualization & Interaction)

现在，用户可以通过按下 RVIZ 中的 'Goalpoint' 按钮，然后点击一个点来设置目标。车辆将导航到目标并沿途构建可视性图（青色）。可视性图覆盖的区域变为自由空间。在自由空间导航时，规划器使用已构建的可视性图；在未知空间导航时，规划器尝试发现通往目标的路径。

- **重置可视性图 (Reset Visibility Graph)**: 按下 'Reset Visibility Graph' 按钮，规划器将重新初始化可视性图。
- **规划尝试 (Planning Attemptable)**: 取消选中 'Planning Attemptable' 复选框，规划器将首先尝试在自由空间中寻找路径（显示为绿色）。如果不存在这样的路径，规划器将同时考虑未知空间（路径显示为蓝色）。
- **更新可视性图 (Update Visibility Graph)**: 取消选中 'Update Visibility Graph' 复选框，规划器将停止更新可视性图。
- **读取/保存 (Read/Save)**: 要从文件读取/保存可视性图，请按 'Read'/'Save' 按钮。室内环境的可视性图文件示例位于 `src/far_planner/data/indoor.vgh`。
- **智能操纵杆 (Smart Joystick)**: 在导航过程中的任何时候，用户都可以通过点击控制面板中的黑色方框来手动驾驶车辆。此时系统将切换到智能操纵杆模式——车辆将尝试跟随虚拟操纵杆指令，同时自动避免碰撞。要恢复 FAR Planner 导航，请按 'Resume Navigation to Goal' 按钮，或使用 'Goalpoint' 按钮设置新目标。
