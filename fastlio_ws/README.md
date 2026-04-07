# FAST-LIO 映射工作区（部署说明）

本工作区基于 ROS（catkin），包含 FAST-LIO 算法和相关驱动（如 Livox / Velodyne）。当前已按官方 Livox ROS Driver 仓库结构整理，可直接编译并启动 `mapping_velodyne.launch`。

## 先决条件

- 操作系统：Ubuntu 18.04 / 20.04（与目标 ROS 发行版匹配）
- ROS：Melodic（对应 Ubuntu 18.04）或 Noetic（对应 Ubuntu 20.04）
- 依赖库：PCL, Eigen, Boost 等（通常随 ROS 与系统包安装）

建议先安装常见依赖：

```bash
sudo apt update
sudo apt install build-essential cmake git libpcl-dev libeigen3-dev
```

## 获取代码（如果从零开始）

将仓库克隆到工作目录并切换到工作区根：

```bash
git clone <your-repo-url> fastlio_ws
cd fastlio_ws
```

## 编译（以 catkin_make 为例）

在工作区根运行：

```bash
cd fastlio_ws
catkin_make
```

如果你手动替换过 `src/livox_ros_driver`，请确认包根目录同时包含 `package.xml` 和 `CMakeLists.txt`；官方仓库的标准结构是 `ws/src/livox_ros_driver/livox_ros_driver/` 作为包目录，而不是再多嵌套一层。

编译完成后在每个新终端加载环境：

```bash
cd fastlio_ws
source devel/setup.bash
```

## 推荐启动（task5 集成）

在加载环境后，优先使用 `task5_fastlio.launch`（已适配当前三仓联调方案）：

```bash
cd fastlio_ws
bash run_task5_fastlio_real.sh
```

等价手动命令（兼容旧流程）：

```bash
cd fastlio_ws
source devel/setup.bash
roslaunch fast_lio task5_fastlio.launch
```

默认行为：

- 启动 Livox 驱动并发布 `/livox/lidar`、`/livox/imu`
- 启动 FAST-LIO（`mapping_avia.launch`）
- 输出核心结果：`map -> body`、`/Odometry`、`/cloud_registered`

可选参数示例：

```bash
LIVOX_BD_LIST=<你的雷达序列号> LIVOX_FRAME=livox_frame FASTLIO_RVIZ=true bash run_task5_fastlio_real.sh
```

说明：

- `livox_bd_list`：Livox 设备序列号列表
- `livox_frame`：Livox 发布的 frame 名称（默认 `livox_frame`）
- `fastlio_rviz`：是否启用 FAST-LIO 自带 Rviz

## 传统启动（Velodyne 场景）

在加载环境后，通过以下命令启动建图（你提供的命令）：

```bash
cd fastlio_ws
source devel/setup.bash
roslaunch fast_lio mapping_velodyne.launch
```

该 `launch` 文件通常会启动 FAST-LIO 的核心节点、参数加载与 Rviz 可视化。请确保硬件或 rosbag 发布的主题名称（常见：`/velodyne_points`, `/imu/data`）与 `launch` 文件中的配置一致。

## 使用 rosbag 回放（无真实传感器时测试）

1. 在一终端启动 `mapping_velodyne.launch`。
2. 在另一终端播放数据包：

```bash
rosbag play your_bag_file.bag --clock
```

注意：许多节点依赖 ROS 时间（`/use_sim_time`），回放时建议使用 `--clock` 并确保参数 `use_sim_time` 在 launch 或参数文件中被设置为 `true`（如需要）。

## 常见配置点

- `topics`：确认点云话题（Velodyne 常用 `/velodyne_points`）和 IMU 话题名称一致。
- `frame_id`：确认坐标系（`map`、`odom`、`base_link`）设置一致，避免 TF 错误。
- `param` 文件：可在 `src/FAST_LIO/config/` 下查看 `velodyne.yaml`、`imu.yaml` 等并根据传感器调参。

## 故障排查

- 节点未启动或找不到包：确认已在工作区根执行 `source devel/setup.bash`。
- 没有点云输入：使用 `rostopic echo /velodyne_points` 或 `rosrun rqt_graph Rqt_graph` 检查话题与连接。
- TF 错误：使用 `rosrun tf tf_monitor` 或 `rosrun rqt_tf_tree rqt_tf_tree` 检查坐标变换。

## 额外建议

- 在不同机器或 ROS 版本上部署时，先确认依赖版本并在干净环境中编译。
- 若使用 Livox 设备，请参考 `src/livox_ros_driver` 目录下的说明并使用相应的 launch 文件（如果存在）。

## 联系与许可

如需更多帮助，请在仓库中打开 Issue 或联系维护者。

---

（本文档为部署快速参考，必要时可扩展为“安装与调试手册”）
