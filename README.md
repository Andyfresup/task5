# RoboCup26 Task5 部署与运行说明（中文）

本仓库根目录聚合了 Task5 相关代码与依赖工程，当前推荐的运行入口是：

- `run_task5_all.sh`

该入口脚本会先做预检查并统一注入运行参数，功能包括：

- 支持 `--check` 仅执行预检查（不启动主流程）
- 默认依次启动 `base_4drive`、`fastlio_ws`、`far_planner` 的实机脚本，再启动 `task5_person_tracker`
- 支持 `--person-only`、`--no-base`、`--no-fastlio`、`--no-far` 进行组件裁剪
- 支持 `--` 后透传参数到 `task5_person_tracker/run_task5_person_follow_voice.sh`（当前为预留透传位）
- 检查主启动脚本、YOLO 目录、语音目录、`python3` 可用性
- 支持 `STARTUP_DELAY`（秒）控制底层模块分步拉起间隔，默认 `2`
- 底层三模块在后台运行，主脚本退出时会自动清理这些后台进程
- 若存在 `.venv` 则自动激活（仅供 `task5_person_tracker` Python 依赖），并注入相对路径默认变量（`YOLO_PERCEPTION_DIR`、`SPEECH_MODULE_FILE`、`SPEECH_ASR_FILE`）
- 将额外参数透传给 `task5_person_tracker/run_task5_person_follow_voice.sh` 并启动 Task5 主流程

说明：三个独立实机脚本 `fastlio_ws/run_task5_fastlio_real.sh`、`far_planner/run_task5_farplanner_real.sh`、`base_4drive/run_task5_base_real.sh` 不包含虚拟环境激活逻辑，仅依赖各自 ROS 工作区的 `devel/setup.bash`。

## 1. 目录概览

根目录包含多个子工程，Task5 运行主要依赖：

- `task5_person_tracker`：任务主流程（检测、跟随、语义、状态机）
- `26-WrightEagle.AI-YOLO-Perception`：实时吧台物体检测（默认调用 `realsenseinfer.py`）
- `26-WrightEagle.AI-Speech`：TTS/ASR 模块（脚本中自动按路径启用）
- `far_planner`、`fastlio_ws`、`base_4drive`：导航/定位相关工程（按你的系统启动）

根目录其他工程（当前 `run_task5_all.sh` 默认不直接拉起）：

- `26-WrightEagle.AI-MHRC-planning`：任务规划/推理相关工程（独立测试与演示）

根目录关键文件：

- `run_task5_all.sh`：Task5 全局入口脚本
- `requirements.txt`：根级 Python 依赖清单

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

`run_task5_all.sh` 的默认行为是先启动底层三套实机脚本，再启动 Task5 主流程：

- `base_4drive/run_task5_base_real.sh`
- `fastlio_ws/run_task5_fastlio_real.sh`
- `far_planner/run_task5_farplanner_real.sh`
- `task5_person_tracker/run_task5_person_follow_voice.sh`

因此在默认模式下，以下关键话题通常由前置模块提供：

- `/cloud_registered`：点云输入（用于生成占据栅格）
- `/cmd_vel_nav`：导航器速度输出（由仲裁器合成为 `/cmd_vel`）

如果你使用 `--person-only`、`--no-fastlio` 或 `--no-far` 裁剪启动组件，则需要外部系统自行提供这些话题。

其中 `task5_person_tracker/run_task5_person_follow_voice.sh` 内部会启动：

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

其中根目录脚本 `run_task5_all.sh` 默认启动顺序为：

1. `base_4drive/run_task5_base_real.sh`
2. `fastlio_ws/run_task5_fastlio_real.sh`
3. `far_planner/run_task5_farplanner_real.sh`
4. `task5_person_tracker/run_task5_person_follow_voice.sh`

常用模式：

```bash
# 仅预检查
bash run_task5_all.sh --check

# 仅启动 person_tracker（不启动底层三模块）
bash run_task5_all.sh --person-only

# 跳过 FAR，仅启动 base + fastlio + person_tracker
bash run_task5_all.sh --no-far

# 调整底层模块启动间隔（秒）
STARTUP_DELAY=3 bash run_task5_all.sh

# 参数透传占位（当前 person_tracker 启动脚本未消费位置参数）
bash run_task5_all.sh -- --example-arg
```

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
bash run_task5_all.sh
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
