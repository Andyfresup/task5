#!/usr/bin/env python3
"""Run Task5 navigate gate integration matrix against live ROS topics.

This tool drives /person_following/navigate_request and validates matched
/person_following/navigate_ack payloads by request_id.
"""

import argparse
import json
import os
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rospy
from std_msgs.msg import String


@dataclass
class CaseSpec:
    case_id: str
    expected_success: bool
    expected_error_code: str
    setup_mode: str = "debug_state"
    active_state: str = "IDLE"
    return_state: str = "IDLE"
    clear_customer_context: bool = True
    request_action: str = "navigate"
    request_pose: Dict[str, float] = field(default_factory=lambda: {"x": 1.0, "y": 0.2, "yaw": 0.0})
    include_pose: bool = True
    request_frame_id: str = "map"
    request_timestamp_offset: float = 0.0
    request_raw_payload: Optional[str] = None
    gate_overrides: Optional[Dict[str, object]] = None


@dataclass
class CaseResult:
    case_id: str
    status: str
    request_id: str
    observed_success: Optional[bool]
    observed_error_code: str
    note: str


class AckStore:
    def __init__(self):
        self._cond = threading.Condition()
        self._acks: Dict[str, List[dict]] = {}

    def callback(self, msg: String):
        raw = str(msg.data or "").strip()
        ack = {"raw": raw, "parse_error": ""}
        try:
            payload = json.loads(raw)
            if isinstance(payload, dict):
                ack.update(payload)
            else:
                ack["parse_error"] = "payload_not_object"
        except Exception as exc:
            ack["parse_error"] = str(exc)

        request_id = str(ack.get("request_id") or "").strip()
        if not request_id:
            request_id = "__missing_request_id__"

        with self._cond:
            self._acks.setdefault(request_id, []).append(ack)
            self._cond.notify_all()

    def wait_for_request(self, request_id: str, timeout: float) -> Optional[dict]:
        deadline = time.monotonic() + max(0.1, float(timeout))
        with self._cond:
            while True:
                queue = self._acks.get(request_id, [])
                if queue:
                    return queue.pop(0)
                remain = deadline - time.monotonic()
                if remain <= 0.0:
                    return None
                self._cond.wait(remain)


class MatrixRunner:
    def __init__(self, args):
        self.args = args
        self.ack_store = AckStore()
        self.last_state_payload: Dict[str, object] = {}

        self.request_pub = rospy.Publisher(args.request_topic, String, queue_size=20)
        self.active_folder_pub = rospy.Publisher(args.active_folder_topic, String, queue_size=2)
        self.debug_state_pub = rospy.Publisher(args.debug_state_topic, String, queue_size=5)

        rospy.Subscriber(args.ack_topic, String, self.ack_store.callback, queue_size=50)
        rospy.Subscriber(args.state_topic, String, self._state_callback, queue_size=20)

        self._wait_for_ros_graph()

    def _wait_for_ros_graph(self):
        deadline = time.monotonic() + max(2.0, self.args.graph_wait_timeout)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.request_pub.get_num_connections() >= 0:
                break
            rospy.sleep(0.05)

    def _state_callback(self, msg: String):
        raw = str(msg.data or "").strip()
        try:
            obj = json.loads(raw)
            if isinstance(obj, dict):
                self.last_state_payload = obj
        except Exception:
            self.last_state_payload = {"raw": raw}

    def _set_gate_params(self, overrides: Optional[Dict[str, object]]):
        values = {
            "mhrc_nav_state_gating_enabled": self.args.default_gate_enabled,
            "mhrc_nav_force_accept": self.args.default_force_accept,
            "mhrc_nav_allow_locked": self.args.default_allow_locked,
            "mhrc_nav_request_ttl": self.args.default_request_ttl,
        }
        if isinstance(overrides, dict):
            values.update(overrides)

        for key, value in values.items():
            full_key = f"{self.args.node_ns}/{key}"
            rospy.set_param(full_key, value)

    def _publish_debug_state(self, state: str, return_state: str, clear_customer_context: bool):
        payload = {
            "active_customer_state": state,
            "return_navigation_state": return_state,
            "clear_customer_context": bool(clear_customer_context),
            "active_customer_id": "matrix_runner",
        }
        self.debug_state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_active_folder(self):
        folder = os.path.join(self.args.customer_data_root, self.args.customer_id)
        os.makedirs(folder, exist_ok=True)
        self.active_folder_pub.publish(String(data=folder))

    def _build_payload(self, spec: CaseSpec, request_id: str) -> str:
        if spec.request_raw_payload:
            return spec.request_raw_payload.format(request_id=request_id)

        payload = {
            "request_id": request_id,
            "action": spec.request_action,
            "frame_id": spec.request_frame_id,
            "timestamp": time.time() + spec.request_timestamp_offset,
        }
        if spec.include_pose:
            payload["pose"] = spec.request_pose
        return json.dumps(payload, ensure_ascii=False)

    def _apply_state(self, spec: CaseSpec) -> Optional[str]:
        if spec.setup_mode == "active_folder":
            self._publish_active_folder()
            rospy.sleep(self.args.state_settle_sec)
            return None

        if spec.setup_mode == "debug_state":
            if not self.args.enable_debug_state:
                return "debug_state_disabled"
            self._publish_debug_state(spec.active_state, spec.return_state, spec.clear_customer_context)
            rospy.sleep(self.args.state_settle_sec)
            return None

        if spec.setup_mode == "none":
            return None

        return f"unsupported_setup_mode:{spec.setup_mode}"

    def run_case(self, spec: CaseSpec) -> CaseResult:
        self._set_gate_params(spec.gate_overrides)

        apply_err = self._apply_state(spec)
        if apply_err:
            return CaseResult(
                case_id=spec.case_id,
                status="SKIP",
                request_id="",
                observed_success=None,
                observed_error_code="",
                note=apply_err,
            )

        request_id = f"{spec.case_id}-{uuid.uuid4().hex[:8]}"
        raw_payload = self._build_payload(spec, request_id)
        self.request_pub.publish(String(data=raw_payload))

        ack = self.ack_store.wait_for_request(request_id, timeout=self.args.ack_wait_timeout)
        if ack is None:
            return CaseResult(
                case_id=spec.case_id,
                status="FAIL",
                request_id=request_id,
                observed_success=None,
                observed_error_code="ack_missing",
                note="timeout_waiting_ack",
            )

        observed_success = bool(ack.get("success", False))
        observed_error_code = str(ack.get("error_code") or "").strip()

        ok = observed_success == spec.expected_success
        if ok and spec.expected_error_code:
            ok = observed_error_code == spec.expected_error_code
        if ok and not spec.expected_error_code:
            ok = observed_error_code == ""

        note = ""
        if not ok:
            note = f"expected_success={spec.expected_success}, expected_error_code={spec.expected_error_code}"

        return CaseResult(
            case_id=spec.case_id,
            status="PASS" if ok else "FAIL",
            request_id=request_id,
            observed_success=observed_success,
            observed_error_code=observed_error_code,
            note=note,
        )


def build_case_specs(suite: str) -> List[CaseSpec]:
    gating = [
        CaseSpec("A1_IDLE_ALLOW", True, "", setup_mode="debug_state", active_state="IDLE", return_state="IDLE"),
        CaseSpec("A2_LOCKED_DENY", False, "busy_service_workflow", setup_mode="active_folder"),
        CaseSpec(
            "A3_LOCKED_ALLOW",
            True,
            "",
            setup_mode="active_folder",
            gate_overrides={"mhrc_nav_allow_locked": True},
        ),
        CaseSpec(
            "A4_TRACKING_DENY",
            False,
            "busy_service_workflow",
            setup_mode="debug_state",
            active_state="TRACKING",
            return_state="IDLE",
        ),
        CaseSpec(
            "A5_PAUSED_ORDERING_DENY",
            False,
            "busy_service_workflow",
            setup_mode="debug_state",
            active_state="PAUSED_ORDERING",
            return_state="IDLE",
        ),
        CaseSpec(
            "A6_ORDERED_DENY",
            False,
            "busy_service_workflow",
            setup_mode="debug_state",
            active_state="ORDERED",
            return_state="IDLE",
        ),
        CaseSpec(
            "A7_RETURNING_DENY",
            False,
            "busy_returning_navigation",
            setup_mode="debug_state",
            active_state="RETURNING",
            return_state="GO_TO_ANCHOR",
        ),
        CaseSpec(
            "A8_TABLE_APPROACH_DENY",
            False,
            "busy_service_workflow",
            setup_mode="debug_state",
            active_state="TABLE_APPROACH",
            return_state="IDLE",
        ),
        CaseSpec(
            "A9_AT_TABLE_FRONT_DENY",
            False,
            "busy_service_workflow",
            setup_mode="debug_state",
            active_state="AT_TABLE_FRONT",
            return_state="IDLE",
        ),
    ]

    boundary = [
        CaseSpec(
            "B1_INVALID_ACTION",
            False,
            "invalid_action",
            setup_mode="debug_state",
            active_state="IDLE",
            request_action="search",
        ),
        CaseSpec(
            "B2_INVALID_TARGET_MISSING_POSE",
            False,
            "invalid_target",
            setup_mode="debug_state",
            active_state="IDLE",
            include_pose=False,
        ),
        CaseSpec(
            "B3_INVALID_TARGET_NONFINITE",
            False,
            "invalid_target",
            setup_mode="debug_state",
            active_state="IDLE",
            request_pose={"x": float("nan"), "y": 1.0, "yaw": 0.0},
        ),
        CaseSpec(
            "B4_INVALID_FRAME",
            False,
            "invalid_frame",
            setup_mode="debug_state",
            active_state="IDLE",
            request_frame_id="odom",
            request_pose={"x": 1.0, "y": 0.2, "yaw": 0.0},
        ),
        CaseSpec(
            "B5_REQUEST_TIMEOUT",
            False,
            "request_timeout",
            setup_mode="debug_state",
            active_state="IDLE",
            request_pose={"x": 0.5, "y": 0.5, "yaw": 0.0},
            request_timestamp_offset=-180.0,
            gate_overrides={"mhrc_nav_request_ttl": 30.0},
        ),
    ]

    if suite == "gating":
        return gating
    if suite == "boundary":
        return boundary
    return gating + boundary


def parse_args():
    def parse_bool(value: str) -> bool:
        v = str(value).strip().lower()
        if v in ("1", "true", "yes", "on", "y"):
            return True
        if v in ("0", "false", "no", "off", "n"):
            return False
        raise argparse.ArgumentTypeError(f"invalid bool value: {value}")

    parser = argparse.ArgumentParser(description="Run Task5 navigate gating matrix")
    parser.add_argument("--suite", choices=("gating", "boundary", "all"), default="all")
    parser.add_argument("--request-topic", default="/person_following/navigate_request")
    parser.add_argument("--ack-topic", default="/person_following/navigate_ack")
    parser.add_argument("--state-topic", default="/person_following/serving_customer_state")
    parser.add_argument("--active-folder-topic", default="/person_following/active_customer_folder")
    parser.add_argument("--debug-state-topic", default="/person_following/debug_state_override")
    parser.add_argument("--enable-debug-state", action="store_true", default=False)
    parser.add_argument("--graph-wait-timeout", type=float, default=2.0)
    parser.add_argument("--ack-wait-timeout", type=float, default=3.0)
    parser.add_argument("--state-settle-sec", type=float, default=0.25)
    parser.add_argument("--customer-data-root", default="/home/andy/robocup26/task5_person_tracker/person_following/service_customers")
    parser.add_argument("--customer-id", default="matrix_runner_customer")
    parser.add_argument("--node-ns", default="/person_goal_publisher")
    parser.add_argument("--default-gate-enabled", type=parse_bool, default=True)
    parser.add_argument("--default-force-accept", type=parse_bool, default=False)
    parser.add_argument("--default-allow-locked", type=parse_bool, default=False)
    parser.add_argument("--default-request-ttl", type=float, default=30.0)
    parser.add_argument("--fail-on-skip", action="store_true", default=False)
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("task5_nav_gate_matrix_runner", anonymous=True)

    if not args.enable_debug_state:
        args.enable_debug_state = bool(
            rospy.get_param(f"{args.node_ns}/mhrc_nav_debug_state_override_enabled", False)
        )

    runner = MatrixRunner(args)
    specs = build_case_specs(args.suite)

    results: List[CaseResult] = []
    for spec in specs:
        result = runner.run_case(spec)
        results.append(result)
        print(
            f"[{result.status}] {result.case_id} "
            f"request_id={result.request_id or '-'} "
            f"success={result.observed_success} error_code={result.observed_error_code} "
            f"{result.note}"
        )

    passed = sum(1 for x in results if x.status == "PASS")
    failed = sum(1 for x in results if x.status == "FAIL")
    skipped = sum(1 for x in results if x.status == "SKIP")

    print("\n=== Matrix Summary ===")
    print(f"total={len(results)} passed={passed} failed={failed} skipped={skipped}")

    if args.fail_on_skip and skipped > 0:
        return 2
    if failed > 0:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
