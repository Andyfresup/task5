# base_4drive 仿真机器人

这是一个基于 ROS (Robot Operating System) 和 Gazebo 的四轮驱动移动机器人仿真项目。

## 1. 环境依赖

* **OS**: Ubuntu (推荐 18.04 或 20.04)
* **ROS**: Melodic 或 Noetic
* **Simulator**: Gazebo

## 2. 下载与编译

如果你是第一次使用此仓库，请按照以下步骤进行下载和编译：

```bash
# 1. 克隆仓库到本地
git clone https://github.com/CuiLikun/base_4drive.git

# 2. 进入项目工作空间
cd base_4drive

# 3. 安装依赖 (可选，确保安装了所有依赖包)
# rosdep install --from-paths src --ignore-src -r -y

# 4. 编译项目
catkin_make
```

> **注意**: 如果编译过程中报错提示缺少包，请先安装对应的 ROS 依赖包。

## 3. 运行仿真

编译成功后，使用以下命令启动仿真环境：

```bash
# 1. 设置环境变量 (每次打开新终端都需要执行)
source devel/setup.bash

# 2. 启动 Gazebo 仿真模型
roslaunch smartcar_description smartcar_gazebo.launch
```

## 4. 实机运行

```bash
# 1. 设置环境变量 (每次打开新终端都需要执行)
source devel/setup.bash

# 2. 启动 底盘 launch
roslaunch smartcar_description task5_base.launch
```

## 5. 功能包说明

* **src/smartcar_description**: 包含机器人的 URDF 模型文件和 Gazebo 启动文件。
* **src/smartcar_control**: (如有) 包含机器人的控制配置。
* **src/serui**: 串口通信或用户界面相关功能。
