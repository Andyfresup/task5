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

## 3. 机器人相机连接部署（RealSense）

本项目视觉链路依赖 Intel RealSense（彩色 + 深度）。部署到机器人时，建议按以下顺序完成检查。

### 3.1 安装驱动与 Python 依赖

```bash
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev

cd robocup26
source .venv/bin/activate
pip install pyrealsense2
```

快速验证 Python 侧依赖：

```bash
cd robocup26
source .venv/bin/activate
python3 -c "import pyrealsense2 as rs; print('pyrealsense2 ok')"
```

### 3.2 硬件连接检查（上电后先做）

1. 使用稳定 USB 3.0 线缆与接口，优先直连主机高带宽口。
2. 确认系统识别到设备：

```bash
lsusb | grep -i intel
rs-enumerate-devices
```

3. 用官方工具确认彩色/深度流正常：

```bash
realsense-viewer
```

如果这里没有稳定画面，不建议继续跑 Task5 主流程。

### 3.3 权限与设备访问

若出现权限不足（无法打开相机设备），可检查当前用户组并重登：

```bash
id
groups
```

确保具备视频设备访问权限（常见为 `video` 或 `plugdev` 组）。

### 3.4 本仓库内的相机连通性验证

先验证 YOLO + RealSense 检测链路：

```bash
cd robocup26/26-WrightEagle.AI-YOLO-Perception
python3 realsenseinfer.py
```

正常时会打印深度尺度，并持续输出检测结果到 `detections.json`。

再验证 Task5 人体坐标发布：

```bash
cd robocup26/task5_person_tracker
bash run_task5_person_follow_voice.sh
```

另开终端查看：

```bash
rostopic hz /person/base_link_3d_position
rostopic echo -n 1 /person/base_link_3d_position
```

### 3.5 多相机场景（重要）

当前仓库脚本默认不绑定相机序列号，系统会使用可用设备中的默认设备。

- 单相机部署：通常可直接运行。
- 多相机部署：建议在 RealSense 初始化处增加序列号绑定（`config.enable_device(<serial>)`），避免启动时连到错误设备。

序列号可通过 `rs-enumerate-devices` 获取。

### 3.6 坐标对齐建议

当前视觉节点将检测结果发布为 `base_link` 坐标（`/person/base_link_3d_position`），并使用 `cam_to_base_x/y/z` 做平移补偿。

- 若相机安装位不在机器人几何中心，请标定并配置补偿量。
- 若出现“人物在地图中漂移或偏移”，优先检查相机安装外参与 TF 一致性。

## 4. 机器人音频设备连接部署（麦克风与扬声器）

本项目语音链路分为两类：

- 呼叫词监听（`person_detection_with_voice.py`）：默认使用系统默认输入设备。
- 点单回复识别（`vad-whisper.py`）：默认按麦克风名 `Newmine` 查找，找不到则回退默认输入设备。
- TTS 播报（`synthesizer.py`）：默认播放到系统默认输出设备。

因此部署到机器人时，建议在启动任务前先固定系统默认输入/输出设备。

### 4.1 安装音频依赖

```bash
sudo apt update
sudo apt install -y portaudio19-dev alsa-utils pulseaudio

cd robocup26
source .venv/bin/activate
pip install pyaudio sounddevice faster-whisper resampy silero-vad
```

### 4.2 枚举并确认设备

先看系统识别到的录音/播放设备：

```bash
arecord -l
aplay -l
```

再看 Python 侧设备名称（用于匹配麦克风名）：

```bash
python3 - <<'PY'
import sounddevice as sd
for i, d in enumerate(sd.query_devices()):
	print(i, d['name'], 'in', d['max_input_channels'], 'out', d['max_output_channels'])
PY
```

### 4.3 设置默认输入/输出设备（推荐）

如果系统使用 PulseAudio：

```bash
pactl list short sources
pactl list short sinks
pactl set-default-source <source_name>
pactl set-default-sink <sink_name>
```

设置后建议重新登录一次，避免服务侧仍使用旧默认设备。

### 4.4 与当前脚本参数的对应关系

- `run_task5_person_follow_voice.sh` 支持 `SPEECH_MODULE_FILE`、`SPEECH_ASR_FILE` 切换语音模块文件。
- `person_goal_publisher.py` 的点单回复 ASR 参数 `pause_reply_mic_name` 默认是 `Newmine`。
- 目前根启动脚本没有单独暴露麦克风/扬声器设备编号参数，主要依赖系统默认设备。

如果现场麦克风名称不是 `Newmine`，需要在启动参数中显式传入或按现场设备名修改默认值。

### 4.5 部署前自检（建议每次上机执行）

1. 录音输入自检：

```bash
cd robocup26/26-WrightEagle.AI-Speech/src/asr
python3 vad-whisper.py
```

2. 扬声器输出自检：

```bash
cd robocup26/26-WrightEagle.AI-Speech/src/tts
python3 synthesizer.py
```

3. 任务链路自检（启动后检查语音话题和日志）：

```bash
cd robocup26
source .venv/bin/activate
bash run_task5_all.sh --check
```

### 4.6 常见问题

- 能识别到设备但无声：默认 sink 不是机器人扬声器。
- 能播报但收不到语音：默认 source 不是机器人麦克风。
- 麦克风偶发消失：USB 供电或线材不稳定，建议固定端口并避免热插拔。
- 首次部署权限问题：确认用户具备音频设备访问权限（常见 `audio` 组）。

## 5. 局域网 Ollama 配置（默认语义后端）

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

## 6. 启动前置条件

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

## 7. 一键启动

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

## 8. 当前默认行为说明

1. 锁定顾客（手势/语音触发）。
2. 跟随并在稳定近距离停下，询问点单。
3. ASR 文本走语义解析，提取食品与数量并存入顾客文件夹。
4. 语音确认后返回锚点区域。
5. 靠近吧台后进行长边前停靠。
6. 启动实时 YOLO 检测（默认 `python3 realsenseinfer.py`）。
7. 用“别名精确 + 词法匹配 + 语义模糊匹配”比对吧台食物与点单内容。
8. 若缺失，播报：`The customer wants ...`。

## 9. 常用可调参数（环境变量）

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

## 10. 快速排障

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
