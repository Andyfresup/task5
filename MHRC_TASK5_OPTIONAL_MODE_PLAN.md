# MHRC Optional Coexistence Mode Plan (Default Legacy FSM)

This plan defines how MHRC coexists with the original Task5 state machine. The original Task5 state machine remains the default mode.

## 1. Design Goal

- Keep Task5 legacy state machine as the default behavior.
- Add one optional non-default mode where MHRC is enabled only in explicitly allowed windows.
- Forbid MHRC in all other states.
- Connect MHRC with existing Task5 order storage by using MHRC action `save_order`.

## 2. Mode Strategy

### 2.1 Mode Variable

- `TASK5_ORDERING_MODE` (default: `legacy_fsm`)

### 2.2 Mode Values

- `legacy_fsm`:
  - Default mode.
  - Existing Task5 state machine fully owns dialog, order handling, and downstream flow.
- `mhrc_coexist`:
  - Optional non-default mode.
  - Window A is triggered when entering `PAUSED_ORDERING` and injects Window A prompt into MHRC.
  - In this mode, MHRC action whitelist is `navigate|search|pick|place|speak|wait|save_order`.
  - MHRC is globally disabled before Window A trigger.
  - MHRC is globally disabled after entering `RETURNING` until Window B (`AT_TABLE_FRONT`) is reached.
  - Window B is bound to `AT_TABLE_FRONT`; after Window B, MHRC remains allowed.

## 3. Mandatory Constraint Update

Removed requirement:

- Do not inject first-contact robot initial coordinates into prompt.

This plan no longer uses initial pose prompt injection.

## 4. MHRC Windows, Gate, and Priority

### 4.1 Activation Interval Rule (must follow)

- Before Window A trigger: MHRC forbidden.
- Window A trigger point: state enters `PAUSED_ORDERING`.
- Window A trigger has higher priority than generic state checks.
- After Window A trigger and before entering `RETURNING`: MHRC allowed.
- From entering `RETURNING` onward: MHRC forbidden until Window B.
- Window B state is `AT_TABLE_FRONT`.
- After Window B trigger: MHRC allowed.

Gating is based only on interval boundaries above.

### 4.2 Window A: Entering PAUSED_ORDERING (MHRC triggered)

Trigger condition:

- State transition into `PAUSED_ORDERING`.
- Customer identifier `No.` comes from the customer folder name in Task5.

#### 4.2.1 Prompt Injection Protocol (MUST)

At Window A trigger, inject the following sentence into MHRC as a user-input payload:

`Customer {No.} waved and called me to serve and I have arrived at the position of customer {No.}. Now I need to ask him for his order politely`

After ask-order speech is completed and ASR receives order text, inject:

`Customer {No.} said: {customer's order}`

Protocol constraints:

1. Injection MUST be text-only user input to planner call, not appended to system prompt.
2. Injection MUST preserve exact semantic content above.
3. `{No.}` MUST be resolved from active customer folder and validated before injection.
4. Empty or invalid `{customer's order}` MUST NOT trigger `save_order`.

#### 4.2.2 JSON Output Constraints for Window A (MUST)

For both Window A injections, MHRC output MUST satisfy:

1. Exactly one JSON object.
2. No non-JSON prefix/suffix text.
3. Exactly one action per turn.
4. Action type MUST be in `navigate|search|pick|place|speak|wait|save_order`.
5. Unknown top-level/action fields are rejected (fail-closed target behavior).

On parse/validation failure:

1. Use bounded repair retries.
2. If still invalid, fallback to deterministic Task5 legacy ask-order flow in the same cycle.

Expected MHRC behavior:

- First injection: usually `speak` for polite ask-order.
- Second injection: choose `speak|wait|save_order` based on order confidence and completeness.

### 4.3 Pre-RETURNING Allowed Interval (Window A -> before RETURNING)

Definition:

- Interval starts immediately after Window A trigger.
- Interval ends when state first enters `RETURNING`.

Rule:

- MHRC is allowed in this interval regardless of TRACKING state checks.
- Do not add additional TRACKING-only enable/disable conditions.

### 4.4 Window B: AT_TABLE_FRONT (missing-food request decision)

Trigger condition:

- State is `AT_TABLE_FRONT`.

Prompt must include:

- Detected item list on bar.
- Customer order JSON content.

Expected MHRC behavior:

- Decide whether to trigger `speak` to ask bartender for missing ordered foods.

Post-Window B rule:

- After Window B, MHRC stays allowed.

## 5. MHRC Forbidden States

Except the allowed intervals/windows above, MHRC is disabled in all states, including but not limited to:

- Any state before Window A trigger.
- `RETURNING` and `TABLE_APPROACH`.

This is a hard prohibition rule.

## 6. MHRC Action: save_order

Add `save_order` to MHRC action set in coexist mode.

### 6.1 Intent

- Bridge MHRC decision output to Task5 existing order processing and storage mechanism.
- Do not introduce a new parallel order file format.
- Keep Task5 customer folder JSON as source of truth.

### 6.2 Strict Contract (MUST)

`save_order` MUST follow RobotDecision JSON shape (no side schema):

```json
{
  "thought": "optional string",
  "reply": "optional string",
  "action": {
    "type": "save_order",
    "customer_no": "string",
    "raw_order_text": "string",
    "normalized_items": ["string"]
  }
}
```

Rules:

1. `customer_no`, `raw_order_text`, `normalized_items` are required for `save_order`.
2. `normalized_items` MUST be a non-empty string array after normalization.
3. LLM MUST NOT output filesystem paths or direct write directives.
4. Unknown fields SHOULD be rejected by schema (fail-closed target behavior).
5. `customer_no` MUST equal the basename of current active customer folder (No. tag).
6. On `customer_no` and active folder-name mismatch, `save_order` MUST be rejected.

### 6.3 Execution Bridge Rules (MUST)

1. `save_order` dispatch MUST call existing Task5 order normalization/storage path.
2. Storage MUST be scoped to active customer folder only.
3. Storage path selection MUST be derived from current No. tag folder basename, not free text path.
4. On success, return structured action result with `success=true`.
5. On failure, return stable `error_code` in `result.data.error_code`.

Recommended `error_code` set for `save_order`:

- `invalid_customer_no`
- `empty_order_items`
- `order_normalization_failed`
- `order_storage_failed`
- `busy_service_workflow`

### 6.4 Replan/Fallback Policy (MUST)

1. Hard validation failures (`invalid_customer_no`, `empty_order_items`) SHOULD NOT trigger replan.
2. Transient failures (`order_storage_failed`) MAY trigger bounded replan.
3. If retries exhaust, fallback to deterministic legacy order handling in same cycle.
4. `save_order` failure MUST NOT break Task5 state progression.

### 6.5 Test Gate for save_order (MUST)

Minimum checks:

1. Accept valid `save_order` payload and write expected customer JSON.
2. Reject unknown/extra fields if fail-closed mode is enabled.
3. Reject empty `normalized_items`.
4. Verify `error_code` propagation to feedback/replan decision logic.
5. Verify fallback to legacy path on parse/validation timeout.

## 7. State Machine Addition: Order-JSON-Ready Check Stage

Add a dedicated stage after ordering interaction:

- Check whether customer `{No.}` folder contains generated order-food JSON.
- If JSON exists, trigger return-to-bar state sequence (`RETURNING` then `TABLE_APPROACH`).
- During `RETURNING` and `TABLE_APPROACH`, MHRC is disabled.
- At `AT_TABLE_FRONT` (Window B), re-enable MHRC.
- After Window B, keep MHRC allowed.

This stage is mandatory to connect MHRC dialog output with existing Task5 storage truth.

## 8. Default-Preserving Safety Rules

- `legacy_fsm` remains default in startup scripts and competition profile.
- `mhrc_coexist` must be explicitly enabled.
- No MHRC direct low-level navigation control bypassing Task5 gating.
- Any MHRC failure in allowed windows must degrade to legacy deterministic behavior in the same cycle.
- Task5 state files and generated order JSON remain source of truth.

## 9. Staged Implementation Plan

### Phase 0: Feature Flag and Interval Gate Skeleton

- Add `TASK5_ORDERING_MODE` parsing.
- Keep default `legacy_fsm`.
- Add interval gate variables:
  - `window_a_triggered`
  - `entered_returning`
  - `window_b_triggered`
- Remove TRACKING-specific enable/disable branching.

### Phase 1: Window A Trigger on PAUSED_ORDERING

- On transition into `PAUSED_ORDERING`, trigger Window A.
- Inject Window A sentence into MHRC and execute MHRC ask-order response.
- Append `Customer {No.} said: {customer's order}` after ASR.
- Enable `speak|wait|save_order` handling.

### Phase 2: Order-JSON-Ready Stage + RETURNING Cutoff

- Add order JSON existence check stage.
- If JSON generated, enter `RETURNING` sequence.
- Disable MHRC for `RETURNING` and `TABLE_APPROACH`.

### Phase 3: Window B on AT_TABLE_FRONT + Post-Window-B Allow

- Bind Window B trigger to `AT_TABLE_FRONT`.
- Inject bar item list + order JSON into MHRC.
- Let LLM decide bartender request speech.
- Keep MHRC allowed after Window B.

## 10. Acceptance Criteria

- Default behavior unchanged when `TASK5_ORDERING_MODE` is unset.
- Window A is triggered on entering `PAUSED_ORDERING` and injects required prompt to MHRC.
- Before Window A trigger, MHRC invocation count is zero.
- From Window A trigger until entering `RETURNING`, MHRC is allowed.
- `RETURNING`/`TABLE_APPROACH` interval has MHRC invocation count zero.
- Window B is bound to `AT_TABLE_FRONT`.
- After Window B, MHRC remains allowed.
- `save_order` uses RobotDecision JSON shape and passes schema validation.
- `save_order` successfully links to existing Task5 order JSON generation.
- Window A/B prompt injections produce single-object strict JSON decisions only.
- On JSON parse/validation failure, bounded retries then same-cycle legacy fallback are verified.
- `save_order` failures expose stable `error_code` and follow replan gating policy.

## 11. Suggested Implementation Touchpoints

- `task5_person_tracker/person_following/person_goal_publisher.py`
  - add mode gate and interval-based MHRC enable/disable logic
  - trigger Window A when entering `PAUSED_ORDERING` and inject required prompt into MHRC
  - append customer-order prompt line after ASR text arrival
  - remove TRACKING-only gating branch
  - add order-json-ready check stage
  - disable MHRC in `RETURNING` and `TABLE_APPROACH`
  - trigger Window B at `AT_TABLE_FRONT` and keep post-Window-B MHRC enabled
  - route `save_order` action to existing order storage path
- `26-WrightEagle.AI-MHRC-planning/src/modules/planning/schemas.py`
  - add `save_order` action schema
- `26-WrightEagle.AI-MHRC-planning/src/modules/planning/llm_client.py`
  - tighten JSON extraction/repair behavior for single-object strict parse
- `26-WrightEagle.AI-MHRC-planning/src/robot_controller.py`
  - support `save_order` action dispatch bridge hook
- `26-WrightEagle.AI-MHRC-planning/src/modules/planning/prompts.py`
  - enforce constrained output and required Window A/B prompt snippets
- `26-WrightEagle.AI-MHRC-planning/src/modules/execution/feedback.py`
  - classify `save_order` error codes into replan/no-replan sets
- `26-WrightEagle.AI-MHRC-planning/tests/`
  - add malformed-output and `save_order` contract regression tests
- `run_task5_all.sh`
  - keep `legacy_fsm` as default and allow optional `mhrc_coexist` override

## 12. Example Startup

Default (unchanged):

```bash
bash run_task5_all.sh
```

Optional coexist mode:

```bash
export TASK5_ORDERING_MODE=mhrc_coexist
bash run_task5_all.sh --test --person-only
```

Notes:

- This document defines the target behavior contract for implementation.
- Until code wiring is complete, mode values are planning-level controls.
