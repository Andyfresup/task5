# 局域网 Ollama 部署与 Task5 模糊语义匹配对接指南（Ubuntu 20.04 + RTX 40 系）

本文档说明如何在局域网中另一台 Ubuntu 20.04 电脑上部署 Ollama（使用 RTX 40 系 GPU），并与本仓库中的点单语义识别与吧台检测标签模糊匹配接口对接。

## 1. 架构说明

建议将系统拆成两台机器：

- 机器人主控机（运行本仓库）
- 语义服务器（Ubuntu 20.04 + RTX 40 系，运行 Ollama）

调用链路如下：

1. 机器人端识别顾客语音文本，调用 Ollama 做点单语义提取。
2. 机器人到吧台后运行实时 YOLO 检测（`realsenseinfer.py`）。
3. 检测到的标签先做别名精确/词法匹配，失败后再走语义通道模糊匹配。
4. 模糊匹配默认复用点单语义后端（`reuse_order_backend`），即同一局域网 Ollama 通道。

## 2. 语义服务器（Ubuntu 20.04）部署步骤

### 2.1 安装 NVIDIA 驱动（建议 550 或 535）

```bash
sudo apt update
ubuntu-drivers devices
sudo apt install -y nvidia-driver-550
# 若 550 不可用可改为 535
sudo reboot
```

重启后验证：

```bash
nvidia-smi
```

要求：能看到 40 系显卡信息。

### 2.2 安装 Ollama

```bash
curl -fsSL https://ollama.com/install.sh | sh
```

检查服务状态：

```bash
systemctl status ollama --no-pager
```

### 2.3 配置 Ollama 监听局域网地址

编辑 systemd 覆盖配置：

```bash
sudo systemctl edit ollama
```

写入以下内容：

```ini
[Service]
Environment="OLLAMA_HOST=0.0.0.0:11434"
Environment="OLLAMA_ORIGINS=*"
Environment="OLLAMA_KEEP_ALIVE=5m"
```

重载并重启：

```bash
sudo systemctl daemon-reload
sudo systemctl restart ollama
sudo systemctl status ollama --no-pager
```

### 2.4 防火墙放通（仅限局域网）

假设局域网网段为 `192.168.1.0/24`：

```bash
sudo ufw allow from 192.168.1.0/24 to any port 11434 proto tcp
sudo ufw status
```

### 2.5 拉取模型并测试

推荐先用轻量模型：

```bash
ollama pull llama3.2:3b
```

本机测试：

```bash
curl http://127.0.0.1:11434/api/generate \
  -H "Content-Type: application/json" \
  -d '{
    "model":"llama3.2:3b",
    "prompt":"Return strict JSON: {\"name\":\"burger\"}",
    "stream":false
  }'
```

## 3. 从机器人主控机验证局域网连通

假设语义服务器 IP 为 `192.168.1.88`：

```bash
curl http://192.168.1.88:11434/api/tags
```

若返回模型列表，表示网络连通正常。

## 4. 与本仓库对接（点单语义 + 模糊语义匹配）

本仓库已内置如下默认行为：

- 语义后端默认：`FOOD_SEMANTIC_BACKEND=ollama`
- 吧台模糊匹配后端默认：`TABLE_FOOD_FUZZY_BACKEND=reuse_order_backend`
- YOLO 感知仓库默认相对路径：`../26-WrightEagle.AI-YOLO-Perception`
- 实时检测命令固定：`python3 realsenseinfer.py`

你通常只需设置局域网 Ollama 地址与模型：

```bash
cd /home/andy/robocup26/task5_person_tracker

export FOOD_SEMANTIC_OLLAMA_URL="http://192.168.1.88:11434"
export FOOD_SEMANTIC_OLLAMA_MODEL="llama3.2:3b"
export FOOD_SEMANTIC_TIMEOUT="8.0"

bash run_task5_person_follow_voice.sh
```

## 5. 两类语义请求的接口约定

程序会通过 `POST /api/generate` 请求 Ollama，`stream=false`。

### 5.1 顾客点单语义提取

目标输出格式（严格 JSON）：

```json
{"items":[{"name":"coke","qty":2},{"name":"burger","qty":1}]}
```

### 5.2 吧台检测标签模糊匹配

目标输出格式（严格 JSON）：

```json
{"name":"burger"}
```

无匹配时：

```json
{"name":""}
```

## 6. 40 系 GPU 性能与稳定性建议

- 首选 `llama3.2:3b` 起步，稳定后再尝试更大模型。
- 通过 `nvidia-smi -l 1` 观察推理期间显存和 GPU 利用率。
- 若请求超时，优先调大 `FOOD_SEMANTIC_TIMEOUT`（例如 12~20 秒）。
- 语义服务器建议避免并发跑其他重负载任务。

## 7. 常见问题排查

### 7.1 机器人端报连接失败

检查：

1. `FOOD_SEMANTIC_OLLAMA_URL` 是否写对 IP 与端口。
2. 服务器上 `systemctl status ollama` 是否正常。
3. 防火墙是否放通 11434。

### 7.2 返回不是 JSON

处理建议：

1. 换更稳定的 instruct 模型。
2. 保持默认 prompt，不要在外部命令再二次包装自然语言。
3. 必要时自建 Ollama 模型模板（Modelfile）强化 JSON 输出约束。

### 7.3 没有使用到 GPU

检查：

1. `nvidia-smi` 是否可用。
2. 驱动是否正确安装。
3. 推理时 `nvidia-smi` 是否出现 `ollama` 进程。

## 8. 快速验收清单

- 语义服务器 `nvidia-smi` 正常。
- `curl http://<server_ip>:11434/api/tags` 正常返回。
- 机器人端启动脚本后，点单语义能解析出 JSON。
- 到吧台后，检测标签与点单别名可完成模糊匹配。
- 缺失食物时触发播报：`The customer wants ...`。
