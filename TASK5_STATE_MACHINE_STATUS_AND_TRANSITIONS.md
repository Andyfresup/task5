# Task5 状态机状态与转换条件总览

本文档把当前代码中的状态机状态与转换条件集中整理，便于联调和后续改造。

适用代码基线：

- `task5_person_tracker/person_following/person_goal_publisher.py`
- `MHRC_TASK5_OPTIONAL_MODE_PLAN.md`（模式约束补充）

## 1. 服务主状态：active_customer_state

初始化：`IDLE`

- 初始化位置：`person_goal_publisher.py:567`
- 状态写入入口：`_set_serving_customer_state(...)`，`person_goal_publisher.py:635`

当前代码中出现的主状态：

1. `IDLE`
2. `LOCKED`
3. `TRACKING`
4. `PAUSED_ORDERING`
5. `ORDERED`
6. `RETURNING`
7. `TABLE_APPROACH`
8. `AT_TABLE_FRONT`

### 1.1 主状态转换条件

1. `IDLE -> LOCKED`
- 触发：收到有效 `active_customer_folder` 话题，且目录存在。
- 代码：`active_customer_folder_callback`，`person_goal_publisher.py:622-633`。

2. `LOCKED/TRACKING -> PAUSED_ORDERING`
- 触发：机器人到达跟随目标并稳定保持（距离阈值 + 保持时长阈值）。
- 条件核心：
  - `dist_to_goal <= goal_reach_distance`
  - `held >= goal_reach_hold_time`
- 代码：`person_goal_publisher.py:3665-3688`。
- 状态写入：`person_goal_publisher.py:2523`。

3. `PAUSED_ORDERING -> TRACKING`
- 触发：顾客再次移动，满足任一恢复条件。
- 条件核心：
  - 全局位移 `moved >= person_reacquire_distance`
  - 或近场位移/朝向变化超过阈值（前向、侧向、朝向任一满足）
- 代码：`person_goal_publisher.py:3621-3650`。

4. `PAUSED_ORDERING/TRACKING -> ORDERED`
- 触发：语音回复被解析出有效点单并成功写入（在 PAUSED_ORDERING 或 TRACKING 下均可触发）。
- 条件核心：
  - `food_summary` 非空
  - `stored_to_file == True` 且 `payload` 有效
- 代码：`_process_pause_reply_text`，`person_goal_publisher.py:3206-3230`。
- 状态写入：`person_goal_publisher.py:3225`。

5. `ORDERED -> RETURNING`
- 触发：订单确认后触发回锚点成功。
- 条件核心：
  - `return_to_anchor_on_order_confirm == True`
  - 成功加载 `return_anchor.json`
- 代码：`_trigger_return_to_anchor`，`person_goal_publisher.py:2602-2621`。

6. `RETURNING -> TABLE_APPROACH`
- 触发：返回中接近锚点且完成吧台前目标规划。
- 条件核心：
  - `return_navigation_state == GO_TO_ANCHOR`
  - `dist_to_anchor <= return_table_trigger_distance`
  - `plan_table_front_goal(...)` 成功
- 代码：`_run_return_navigation_cycle`，`person_goal_publisher.py:2643-2668`。

7. `TABLE_APPROACH -> AT_TABLE_FRONT`
- 触发：机器人到达吧台前目标点。
- 条件核心：`dist_to_table <= return_table_arrive_distance`
- 代码：`person_goal_publisher.py:2682-2689`。

8. `AT_TABLE_FRONT -> (无显式自动回退到 IDLE)`
- 当前实现中，进入该状态后执行缺失食物检测/播报。
- 代码：`person_goal_publisher.py:2692-2998`。

### 1.2 额外状态改写入口（非业务闭环）

1. 任意主状态可被调试注入改写
- 触发：debug state override 话题携带 `active_customer_state`。
- 代码：`debug_state_override_callback`，`person_goal_publisher.py:663-711`。

2. 任意主状态可被新的 active customer 话题再次置为 `LOCKED`
- 代码：`person_goal_publisher.py:622-633`。

## 2. 返回导航子状态：return_navigation_state

初始化：`IDLE`

- 初始化位置：`person_goal_publisher.py:557`

当前代码中出现的返回子状态：

1. `IDLE`
2. `GO_TO_ANCHOR`
3. `TABLE_APPROACH`
4. `AT_TABLE_FRONT`

### 2.1 子状态转换条件

1. `IDLE -> GO_TO_ANCHOR`
- 触发：`_trigger_return_to_anchor()` 成功启动。
- 代码：`person_goal_publisher.py:2615`。

2. `GO_TO_ANCHOR -> TABLE_APPROACH`
- 触发：接近锚点并成功规划吧台前目标。
- 代码：`person_goal_publisher.py:2643-2665`。

3. `TABLE_APPROACH -> AT_TABLE_FRONT`
- 触发：到达吧台前目标。
- 代码：`person_goal_publisher.py:2682-2686`。

4. `AT_TABLE_FRONT -> (无显式自动回退到 IDLE)`
- 当前实现中未看到自动复位逻辑。

## 3. 与主循环运行关系

主循环中持续调用：

1. `_run_return_navigation_cycle()`
2. `_run_gaze_tracking_cycle()`

入口：`run()`，`person_goal_publisher.py` 末尾。

另外，在 `person_callback` 中若 `return_to_anchor_active == True`，会提前返回并停止注视跟踪，避免跟随逻辑与回锚流程冲突（`person_goal_publisher.py:3698-3700`）。

## 4. 计划中的模式状态约束（文档层）

来源：`MHRC_TASK5_OPTIONAL_MODE_PLAN.md`

当前计划定义的模式状态：

1. `legacy_fsm`（默认）
2. `mhrc_coexist`（非默认）

在 `mhrc_coexist` 中：

1. 仅允许 MHRC 在两个白名单窗口工作：
- Window A：稳定到达顾客面前后的点单交互窗口。
- Window B：稳定到达吧台前后的缺失食物请求决策窗口。

2. 除上述窗口外，一律禁用 MHRC。

3. 新增动作 `save_order`，用于接入 Task5 现有点单 JSON 存储链路。

4. 新增“订单 JSON 已生成检测阶段”：
- 检测到顾客文件夹下订单 JSON 生成后，才触发回吧台状态序列；
- 回吧台过程中保持 MHRC 禁用。

## 5. 快速检查清单（联调使用）

1. 收到 active customer 话题后是否进入 `LOCKED`。
2. 达到目标并稳定保持后是否进入 `PAUSED_ORDERING`。
3. 点单识别成功后是否进入 `ORDERED` 并触发回锚点。
4. 回锚点流程是否按 `GO_TO_ANCHOR -> TABLE_APPROACH -> AT_TABLE_FRONT` 演进。
5. `AT_TABLE_FRONT` 后是否执行缺失食物检测与播报。
6. 是否存在需要补充的 `AT_TABLE_FRONT` 结束态回退规则（当前代码未显式定义）。
