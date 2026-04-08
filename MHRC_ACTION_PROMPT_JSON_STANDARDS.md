# MHRC Action, Prompt, and Strict JSON Standards (v1)

Status: Draft for implementation
Date: 2026-04-08
Scope: 26-WrightEagle.AI-MHRC-planning

## 1) Purpose

This document defines a reusable engineering standard for:

1. strict JSON command generation from LLM output,
2. safe extension of new robot actions,
3. prompt authoring rules for deterministic behavior,
4. execution feedback and replanning boundaries.

The goal is to make action outputs machine-reliable and fail-closed.

## 2) Current End-to-End Pipeline (Reference)

Current code path:

1. Prompt selection in `src/modules/planning/prompts.py` via `get_system_prompt(...)`.
2. Decision generation in `src/modules/planning/llm_client.py` via `get_decision(...)`.
3. JSON extraction in `src/modules/planning/llm_client.py` via `_extract_json(...)`.
4. Action typed parsing in `src/modules/planning/schemas.py` via `parse_action(...)`.
5. Action execution dispatch in `src/robot_controller.py` via `_execute_action(...)`.
6. Robot adapter execution in `src/modules/execution/task5_ros_adapter.py`.
7. Failure routing and replanning in `src/modules/execution/feedback.py` and `src/modules/planning/planner.py`.

This standard preserves that architecture and hardens each stage.

## 3) Normative Language

The key words MUST, MUST NOT, SHOULD, SHOULD NOT, and MAY are normative.

## 4) Strict JSON Output Standard (JSON-HARDENING-v1)

### 4.1 Top-level Contract

The model output MUST be exactly one JSON object.

Canonical target shape:

```json
{
  "thought": "optional internal reasoning string",
  "reply": "optional user-facing string",
  "action": {
    "type": "navigate|search|pick|place|speak|wait",
    "...": "action-specific parameters"
  }
}
```

Rules:

1. Output MUST contain valid JSON, parseable by `json.loads`.
2. Output MUST NOT include prose before or after JSON.
3. Output MUST NOT contain multiple JSON objects.
4. Output SHOULD NOT rely on markdown fences.
5. `action.type` MUST be in the allowed action set.

### 4.2 Validation Rules

1. Schema validation MUST happen immediately after parse.
2. Unknown top-level fields SHOULD be rejected (fail-closed).
3. Unknown action fields SHOULD be rejected (fail-closed).
4. If parse or validation fails, execution MUST NOT start.
5. Parse/validation failure MUST trigger controlled retry or safe fallback.

### 4.3 Retry and Repair Rules

1. Retry prompt MUST include only:
   - concise parse/validation error,
   - prior raw output,
   - instruction to regenerate full JSON in the same schema.
2. Repair logic MUST NOT guess missing semantic fields.
3. Max retries MUST be bounded.
4. Final failure MUST return controlled error to controller, not crash.

## 5) Prompt Authoring Standard (PROMPT-SPEC-v1)

### 5.1 Single Source of Truth

1. Action whitelist MUST be defined consistently across prompt variants.
2. Parameter names in prompts MUST exactly match schema field names.
3. Prompt examples MUST use only valid schema combinations.

### 5.2 Behavioral Constraints in Prompt

Every system prompt MUST state:

1. one step only per turn,
2. use only known action types,
3. strict JSON output requirement,
4. no extra text outside JSON,
5. on uncertainty, prefer `speak` or `wait` over unsafe action.

### 5.3 Replan Prompt Requirements

Replan input MUST include:

1. original decision JSON,
2. action execution feedback JSON,
3. instruction not to repeat hard-failed actions,
4. instruction to emit exactly one safe next action.

### 5.4 Determinism Profile

For command-generation mode, defaults SHOULD be:

1. temperature: 0.0 to 0.2,
2. bounded max tokens,
3. stable model selection.

Rationale: lower sampling variance improves JSON compliance and action stability.

## 6) New Action Extension Standard (SOP-ACTION-v1)

### 6.1 Step 0: Action Contract Card (Required)

Before code changes, define:

1. action name and purpose,
2. required and optional parameters,
3. parameter types and constraints,
4. success payload shape,
5. error codes,
6. replanning policy: retryable or hard-fail.

### 6.2 Step 1: Schema Layer

Update `src/modules/planning/schemas.py`:

1. add new action model,
2. add it to `RobotAction` union,
3. add map entry in `parse_action(...)`,
4. add validators for all safety-critical fields.

### 6.3 Step 2: Prompt Layer

Update `src/modules/planning/prompts.py`:

1. add capability definition,
2. add one positive example,
3. add one boundary/fallback example,
4. keep wording consistent in all prompt modes.

### 6.4 Step 3: Execution Layer

Update:

1. `src/modules/execution/robot_interface.py` (new abstract method),
2. mock executor implementation,
3. real adapter implementation (Task5 ROS adapter if applicable),
4. `src/robot_controller.py` dispatch branch in `_execute_action(...)`.

### 6.5 Step 4: Feedback and Replan Policy

Update `src/modules/execution/feedback.py`:

1. classify new error codes into replan/no-replan sets,
2. avoid replanning for deterministic hard failures,
3. allow replanning for transient failures only.

### 6.6 Step 5: Tests (Mandatory)

Add or update tests for:

1. schema valid/invalid payloads,
2. unknown action rejection,
3. unknown field rejection (if fail-closed enabled),
4. controller dispatch for new action,
5. adapter result propagation (success/error_code),
6. replanning behavior by error code class.

No action is considered integrated until all above pass.

## 7) Error Code and Execution Feedback Standard

Action execution failures SHOULD produce structured payload with:

```json
{
  "success": false,
  "action": "navigate",
  "error": "human readable",
  "data": {
    "error_code": "machine_readable_code",
    "request_id": "optional correlation id",
    "...": "context fields"
  }
}
```

Rules:

1. `error_code` MUST be stable and machine-readable.
2. `error_code` MUST drive replan gating logic.
3. Adapter MUST preserve request correlation ids when available.
4. Timeout and busy-state failures MUST be distinguishable.

## 8) Recommended Acceptance Gates (DoD)

A change is Done only if all checks pass:

1. schema compile and unit tests pass,
2. malformed LLM output tests pass,
3. controller action dispatch tests pass,
4. adapter ack/timeout behavior validated,
5. replan policy tests pass,
6. prompt examples match schema exactly.

## 9) Immediate Implementation Priorities

Priority-1:

1. tighten JSON parsing strategy to reduce false extraction,
2. move command profile temperature to low-randomness settings,
3. enforce fail-closed schema behavior for unknown fields,
4. add malformed-output regression tests.

Priority-2:

1. centralize action whitelist and schema references in one source,
2. standardize retry-repair prompt template,
3. formalize error_code taxonomy in a dedicated module.

## 10) Change Log

v1 (2026-04-08): Initial standard covering action extension, prompt writing, strict JSON, and feedback-replan boundaries.
