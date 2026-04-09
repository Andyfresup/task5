# FAST-LIO 工作区（Task5 / MID360）

本工作区用于 Task5 的定位与点云输出链路，当前基线已经统一为：

- ROS1（catkin）
- `livox_ros_driver2`（ROS1 launch）
- MID360 启动文件：`launch_ROS1/msg_MID360.launch`
- FAST-LIO 主启动：`fast_lio/task5_fastlio.launch`

## 当前默认行为

`task5_fastlio.launch` 会同时完成三件事：

1. 启动 `livox_ros_driver2`（ROS1）并输出 Livox 数据。
2. 启动 FAST-LIO（`mapping_mid360.launch`）。
3. 启动 Task5 兼容层（`task5_compat_bridge.launch`）。

兼容层默认桥接：

- `/cloud_registered -> /jh_cloud`
- `/Odometry -> /odom`
- `/cloud_registered -> /scan`（由 `pointcloud_to_laserscan` 生成）

兼容层默认 TF 策略：

- 开启：`map -> odom_fusion`
- 开启：`body -> base_link`
- 关闭：`base_link -> base_link_fusion`
- 开启：`livox_frame -> base_link_fusion`

说明：默认采用 Livox 外参链路，避免 `base_link_fusion` 出现双父节点冲突。

## 依赖与前置条件

- Ubuntu 20.04
- ROS Noetic
- 已安装 Livox SDK（`liblivox_sdk_static.a` 或 `liblivox_sdk_shared.so`）
- 建议安装：`ros-noetic-tf2-sensor-msgs`

安装常见依赖示例：

```bash
sudo apt update
sudo apt install -y build-essential cmake git libpcl-dev libeigen3-dev ros-noetic-tf2-sensor-msgs
```

## 编译

```bash
cd fastlio_ws
catkin_make
source devel/setup.bash
```

## 启动方式

推荐使用工作区脚本（实机）：

```bash
cd fastlio_ws
bash run_task5_fastlio_real.sh
```

或直接 roslaunch：

```bash
cd fastlio_ws
source devel/setup.bash
roslaunch fast_lio task5_fastlio.launch
```

常见参数示例：

```bash
roslaunch fast_lio task5_fastlio.launch \
	livox_bd_list:=<MID360序列号> \
	livox_frame:=livox_frame \
	fastlio_rviz:=true
```

## 运行时可覆盖环境变量（脚本方式）

`run_task5_fastlio_real.sh` 支持以下开关（仅列出常用）：

- `ENABLE_TASK5_COMPAT_BRIDGE`（默认 `true`）
- `ENABLE_CLOUD_TO_JH_CLOUD`（默认 `true`）
- `ENABLE_ODOMETRY_TO_ODOM`（默认 `true`）
- `ENABLE_CLOUD_TO_SCAN`（默认 `true`）
- `ENABLE_TF_BASE_LINK_TO_BASE_LINK_FUSION`（默认 `false`）
- `ENABLE_TF_LIVOX_TO_BASE_LINK_FUSION`（默认 `true`）
- `LIVOX_TO_BASE_LINK_FUSION_X/Y/Z`、`LIVOX_TO_BASE_LINK_FUSION_ROLL/PITCH/YAW`

示例：切换为 `base_link -> base_link_fusion` 链路（并关闭 Livox 外参链）

```bash
cd fastlio_ws
ENABLE_TF_BASE_LINK_TO_BASE_LINK_FUSION=true \
ENABLE_TF_LIVOX_TO_BASE_LINK_FUSION=false \
bash run_task5_fastlio_real.sh
```

## 快速自检

```bash
source /opt/ros/noetic/setup.bash
rostopic hz /cloud_registered
rostopic hz /jh_cloud
rostopic hz /scan
rostopic hz /Odometry
rostopic hz /odom
```

TF 检查：

```bash
rosrun tf view_frames
```

## 常见问题

- 编译进到 ROS2/ament 分支：检查 `src/livox_ros_driver2/CMakeLists.txt` 是否保持 ROS1 默认分支。
- 报 Livox SDK 库找不到：确认 SDK 已安装，且库文件可被 CMake 搜索到。
- `/scan` 不发布：确认兼容桥接开启且 `pointcloud_to_laserscan` 包可被 `rospack find` 找到。

## 可迁移性说明

为减少跨仓迁移依赖，本工作区已内置 `src/pointcloud_to_laserscan`，无需依赖外部工作区同名包。
