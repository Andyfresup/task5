# MHRC 架构与 Task5 语义接入实战指南

本文面向当前仓库，目标是把 26-WrightEagle.AI-MHRC-planning 中现成的 LLM 方案，稳定接入 task5_person_tracker 的语义识别和语义匹配链路。

## 1. 先给结论

- MHRC 子工程已经有可用的 LLM 客户端和规划链路，不是空壳。
- 直接复用到 Task5 的最佳路径是：先走 command 桥接（低侵入），再考虑在 Task5 代码内新增 mhrc 后端（中侵入）。
- 不建议第一步就深改 Task5 主状态机，容易把导航/感知链路也拖进回归风险。

---

## 2. MHRC 的详细架构与功能

### 2.1 架构分层

MHRC 子工程本质是一个「感知-记忆-规划-执行」的模块化单机框架：

- Observation：采集输入（当前以占位实现为主）
- Memory：记录对话、任务、反馈（轻量内存实现）
- Planning：LLM 规划（核心已完整）
- Execution：动作执行（当前主打 Mock 执行）

### 2.2 核心代码入口

- 启动入口：src/main.py
- 控制器：src/robot_controller.py
- 规划器：src/modules/planning/planner.py
- LLM 客户端：src/modules/planning/llm_client.py
- Prompt 模板：src/modules/planning/prompts.py
- 输出 Schema：src/modules/planning/schemas.py
- 执行接口：src/modules/execution/robot_interface.py
- Mock 执行：src/modules/execution/mock_robot.py
- 记忆管理：src/modules/memory/memory_manager.py
- 观测接口：src/modules/observation/observer_interface.py

### 2.3 运行流程（代码层）

控制器主循环是：

1. 接收用户输入
2. 调用 LLMClient.get_decision()
3. 解析为 RobotDecision（Pydantic 校验）
4. 逐步执行单个动作
5. 写回对话历史

这条链路已经具备：

- OpenAI 兼容接口调用
- 云端/本地 Ollama 配置切换
- JSON 输出提取与失败重试

### 2.4 LLM 能力边界

MHRC 默认输出契约是 RobotDecision：

```json
{
  "thought": "...",
  "reply": "...",
  "action": {"type": "navigate|search|pick|place|speak|wait", "...": "..."}
}
```

这与 Task5 的 food JSON 契约不同，因此「直接拿 get_decision() 回包给 Task5」不可行；应优先使用 LLMClient.chat() 自定义 prompt 和输出格式。

---

## 3. Task5 现有语义链路（你要接入的位置）

Task5 的语义能力主要集中在 person_goal_publisher.py：

- 点单语义提取：
  - 构建 prompt
  - 调后端
  - 解析 food JSON
- 吧台标签语义匹配：
  - 构建 fuzzy prompt
  - 复用同语义后端
  - 解析 canonical food 名称

现有后端类型：

- command
- ollama
- transformers
- auto（按顺序回退）

这意味着 MHRC 最容易切入的点就是 command 后端。

---

## 4. 最实用接法（推荐）

## 4.1 接法选择

推荐先做 A，再视效果做 B：

- A 方案（推荐）：command 桥接脚本（低风险）
- B 方案：Task5 内新增 mhrc 语义后端（中风险）

### 4.2 A 方案：command 桥接脚本

思路：

- 不改 Task5 主体流程
- 新增一个脚本 mhrc_semantic_bridge.py
- Task5 通过 food_semantic_command 调它
- 脚本内部调用 MHRC 的 LLMClient.chat()
- 输出严格 JSON 给 Task5 现有解析器

#### 4.2.1 目录建议

建议放在：

- task5_person_tracker/person_following/mhrc_semantic_bridge.py

#### 4.2.2 脚本功能建议

支持两个 mode：

- order：点单提取，输出
  - {"items":[{"name":"burger","qty":2}]}
- fuzzy：标签匹配，输出
  - {"name":"coke"}

输入建议：

- --mode order|fuzzy
- --text "原始文本"

#### 4.2.3 最小可用桥接代码（示意）

```python
#!/usr/bin/env python3
import argparse
import json
import os
import sys

# 把 MHRC src 加进路径
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
MHRC_SRC = os.path.join(ROOT, "26-WrightEagle.AI-MHRC-planning", "src")
sys.path.insert(0, MHRC_SRC)

from modules.planning.llm_client import LLMClient

ORDER_PROMPT = (
    "You are a food order extractor. Return strict JSON only. "
    "Schema: {\"items\":[{\"name\":\"food_name\",\"qty\":1}]}. "
    "qty must be integer >=1."
)

FUZZY_PROMPT = (
    "You are a food label matcher. Return strict JSON only. "
    "Schema: {\"name\":\"canonical_or_empty\"}."
)


def extract_json_blob(text: str) -> str:
    raw = (text or "").strip()
    if raw.startswith("```"):
        raw = raw.split("\n", 1)[-1]
        if raw.endswith("```"):
            raw = raw[:-3].strip()
    s1, e1 = raw.find("{"), raw.rfind("}")
    if s1 >= 0 and e1 > s1:
        cand = raw[s1:e1+1]
        json.loads(cand)
        return cand
    json.loads(raw)
    return raw


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["order", "fuzzy"], required=True)
    parser.add_argument("--text", required=True)
    args = parser.parse_args()

    prompt = ORDER_PROMPT if args.mode == "order" else FUZZY_PROMPT

    client = LLMClient()
    out = client.chat([
        {"role": "system", "content": prompt},
        {"role": "user", "content": args.text},
    ])

    blob = extract_json_blob(out)
    print(blob)


if __name__ == "__main__":
    main()
```

#### 4.2.4 Task5 参数接入

示例（点单）：

```bash
export FOOD_SEMANTIC_BACKEND=command
export FOOD_SEMANTIC_COMMAND='python3 person_following/mhrc_semantic_bridge.py --mode order --text "{text}"'
export FOOD_SEMANTIC_COMMAND_USE_SHELL=true
```

示例（吧台 fuzzy）：

```bash
export TABLE_FOOD_FUZZY_BACKEND=reuse_order_backend
```

这样 fuzzy 会复用同一语义通道。

### 4.3 B 方案：Task5 新增 mhrc 后端

在 person_goal_publisher.py 的 backend 分派里新增 mhrc：

- _extract_food_items_semantic()
- _run_table_food_fuzzy_semantic_backend()

优点：

- 调用链更干净
- 无需 shell command

缺点：

- 侵入主代码
- 需要补依赖和更多回归测试

---

## 5. 依赖与配置注意事项

### 5.1 依赖

Task5 当前 requirements 没有显式包含 openai/httpx。

如果桥接脚本复用 MHRC LLMClient，请确保运行环境安装：

- openai
- httpx
- pydantic

可直接复用 26-WrightEagle.AI-MHRC-planning/requirements.txt 的 LLM 依赖。

### 5.2 配置源

MHRC LLMClient 默认从 Config.get_llm_config() 读取配置，优先级建议：

1. config_local.py（推荐）
2. config.py 默认值（不推荐在生产中写死 key）

### 5.3 延迟与超时

Task5 语义链路有 timeout 约束，建议：

- 云端模型：先测平均耗时
- 本地 Ollama：优先小模型稳定响应
- 保证桥接脚本输出一定是可解析 JSON

---

## 6. 验证清单（建议按顺序）

1. 单独测 MHRC LLMClient

```bash
cd 26-WrightEagle.AI-MHRC-planning/src
python main.py --test
```

2. 单独测桥接脚本

```bash
cd task5_person_tracker
python3 person_following/mhrc_semantic_bridge.py --mode order --text "I want two cokes and one burger"
python3 person_following/mhrc_semantic_bridge.py --mode fuzzy --text "coca cola can"
```

3. 接入 Task5 后端参数

```bash
cd task5_person_tracker
export FOOD_SEMANTIC_BACKEND=command
export FOOD_SEMANTIC_COMMAND='python3 person_following/mhrc_semantic_bridge.py --mode order --text "{text}"'
export FOOD_SEMANTIC_COMMAND_USE_SHELL=true
bash run_task5_person_follow_voice.sh
```

4. 观察日志

- 点单提取是否稳定输出 items JSON
- table food missing check 是否能正常命中 canonical 名称

---

## 7. 风险与规避

- 风险：模型输出不是 JSON
  - 规避：桥接脚本做 extract_json_blob + 兜底
- 风险：语义耗时过长
  - 规避：小模型 + timeout + 失败回退关键词匹配
- 风险：食品别名不统一
  - 规避：统一 canonical 名称表，桥接输出前做 canonicalize
- 风险：配置分散
  - 规避：在启动脚本集中导出语义相关环境变量

---

## 8. 实施建议（最短路径）

- 第一步：只做 A 方案（桥接脚本）
- 第二步：验证 3 天稳定后再考虑 B 方案内嵌后端
- 第三步：把桥接脚本和参数示例写入根 README 与 task5 README

如果目标是尽快上线比赛环境，A 方案是当前性价比最高的接法。