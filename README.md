# RoboCup26 Task5 部署与运行说明（中文）

本仓库根目录聚合了 Task5 相关代码与依赖工程，当前推荐的运行入口是：

- `task5_person_tracker/run_task5_person_follow_voice.sh`

该入口会启动人物检测、跟随目标发布、点云栅格桥接、速度仲裁等模块，并支持：

- 顾客点单语义识别（默认走局域网 Ollama）
- 吧台前实时 YOLO 检测
- 检测标签与点单别名模糊匹配（复用点单语义后端）

## 1. 目录概览

根目录包含多个子工程，Task5 运行主要依赖：

- `task5_person_tracker`：任务主流程（检测、跟随、语义、状态机）
- `26-WrightEagle.AI-YOLO-Perception`：实时吧台物体检测（默认调用 `realsenseinfer.py`）
- `26-WrightEagle.AI-Speech`：TTS/ASR 模块（脚本中自动按路径启用）
- `far_planner`、`fastlio_ws`、`base_4drive`：导航/定位相关工程（按你的系统启动）

## 2. 环境要求

建议环境：

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+

在仓库根目录创建并激活虚拟环境：

```bash
cd robocup26
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

在虚拟环境中安装 Task5 根依赖：

```bash
cd robocup26
pip install -r requirements.txt
```

如果 `pyaudio` 安装失败，请先安装系统依赖后重试：

```bash
sudo apt update
sudo apt install -y portaudio19-dev
```

## 3. 局域网 Ollama 配置（默认语义后端）

当前脚本默认：

- `FOOD_SEMANTIC_BACKEND=ollama`
- 吧台模糊匹配 `TABLE_FOOD_FUZZY_BACKEND=reuse_order_backend`（复用同一语义通道）

启动前建议显式设置：

```bash
export FOOD_SEMANTIC_OLLAMA_URL="http://<局域网语义服务器IP>:11434"
export FOOD_SEMANTIC_OLLAMA_MODEL="llama3.2:3b"
export FOOD_SEMANTIC_TIMEOUT="8.0"
```

如果你需要完整的远端 Ubuntu20.04 + 40 系 GPU 部署流程，请参考：

- `task5_person_tracker/docs/OLLAMA_LAN_DEPLOY_GUIDE_ZH.md`

## 4. 启动前置条件

运行 Task5 主脚本前，请确保你的底层系统已提供或即将提供以下关键话题：

- `/cloud_registered`：点云输入（用于生成占据栅格）
- `/cmd_vel_nav`：导航器速度输出（由仲裁器合成为 `/cmd_vel`）

脚本内部会启动：

- `pointcloud_to_occupancy_grid.py`（发布 `/person_following/occupancy_grid`）
- `person_goal_publisher.py`
- `cmd_vel_arbiter.py`
- `person_detection_with_voice.py`

## 5. 一键启动

```bash
cd robocup26
source .venv/bin/activate
bash run_task5_all.sh
```

其中根目录脚本 `run_task5_all.sh` 会统一调用 `task5_person_tracker/run_task5_person_follow_voice.sh`，启动当前 Task5 所需脚本集合。

## 6. 当前默认行为说明

1. 锁定顾客（手势/语音触发）。
2. 跟随并在稳定近距离停下，询问点单。
3. ASR 文本走语义解析，提取食品与数量并存入顾客文件夹。
4. 语音确认后返回锚点区域。
5. 靠近吧台后进行长边前停靠。
6. 启动实时 YOLO 检测（默认 `python3 realsenseinfer.py`）。
7. 用“别名精确 + 词法匹配 + 语义模糊匹配”比对吧台食物与点单内容。
8. 若缺失，播报：`The customer wants ...`。

## 7. 常用可调参数（环境变量）

在启动脚本前可覆盖：

- `FOOD_SEMANTIC_OLLAMA_URL`
- `FOOD_SEMANTIC_OLLAMA_MODEL`
- `FOOD_SEMANTIC_TIMEOUT`
- `YOLO_PERCEPTION_DIR`（默认相对路径 `../26-WrightEagle.AI-YOLO-Perception`）
- `TABLE_FOOD_CHECK_DELAY`
- `TABLE_FOOD_DETECT_TIMEOUT`
- `TABLE_FOOD_LEXICAL_THRESHOLD`

示例：

```bash
export FOOD_SEMANTIC_OLLAMA_URL="http://192.168.1.88:11434"
export FOOD_SEMANTIC_OLLAMA_MODEL="llama3.2:3b"
export TABLE_FOOD_CHECK_DELAY="1.2"
bash task5_person_tracker/run_task5_person_follow_voice.sh
```

## 8. 快速排障

- 语义不工作：检查 `FOOD_SEMANTIC_OLLAMA_URL` 连通（`curl http://<ip>:11434/api/tags`）。
- 检测未触发：确认 RealSense 与 `realsenseinfer.py` 可运行。
- 不发布跟随目标：确认 `/person/base_link_3d_position` 持续更新。
- 不转向/不移动：检查 `/cmd_vel_nav` 与仲裁输出 `/cmd_vel`。
- 地图碰撞检查异常：确认 `/cloud_registered` 正常并生成 `/person_following/occupancy_grid`。

---

如果你希望把该根 README 同步为团队标准文档，可以继续补充：

- 导航底层（FAST-LIO/FAR）独立启动指令
- 比赛现场网络拓扑图
- 模型与参数版本基线
