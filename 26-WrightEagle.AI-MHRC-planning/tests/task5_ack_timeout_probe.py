#!/usr/bin/env python3
"""Probe MHRC Task5 adapter ACK behavior (timeout/success/deny) in ROS runtime."""

import argparse
import json
import os
import sys
import threading
import time

import rospy
from std_msgs.msg import String


class FakeAckResponder:
    def __init__(
        self,
        request_topic: str,
        ack_topic: str,
        delay_sec: float,
        success: bool,
        error_code: str,
        message: str,
    ):
        self.delay_sec = max(0.0, float(delay_sec))
        self.success = bool(success)
        self.error_code = str(error_code or "").strip()
        self.message = str(message or "").strip()
        self.ack_pub = rospy.Publisher(ack_topic, String, queue_size=20)
        self.req_sub = rospy.Subscriber(request_topic, String, self._on_request, queue_size=20)

    def _on_request(self, msg: String):
        raw = str(msg.data or "").strip()
        request_id = ""
        try:
            payload = json.loads(raw)
            if isinstance(payload, dict):
                request_id = str(payload.get("request_id") or "").strip()
        except Exception:
            request_id = ""

        if not request_id:
            return

        th = threading.Thread(target=self._publish_after_delay, args=(request_id,), daemon=True)
        th.start()

    def _publish_after_delay(self, request_id: str):
        if self.delay_sec > 0.0:
            time.sleep(self.delay_sec)
        payload = {
            "request_id": request_id,
            "success": self.success,
            "error_code": self.error_code,
            "message": self.message or self.error_code,
            "customer_no": "probe_customer_001",
            "active_customer_state": "IDLE",
            "return_navigation_state": "IDLE",
            "recommendation": "",
            "timestamp": time.time(),
        }
        self.ack_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def parse_args():
    parser = argparse.ArgumentParser(description="Probe Task5ROSAdapter ACK path")
    parser.add_argument("--mode", choices=("timeout", "delayed-ack", "deny-ack"), default="timeout")
    parser.add_argument("--request-topic", default="/person_following/navigate_request")
    parser.add_argument("--ack-topic", default="/person_following/navigate_ack_probe")
    parser.add_argument("--target", default="home")
    parser.add_argument("--ack-timeout", type=float, default=2.0)
    parser.add_argument("--ack-delay", type=float, default=0.0)
    parser.add_argument("--reply-error-code", default="busy_returning_navigation")
    parser.add_argument("--reply-message", default="injected_by_probe")
    parser.add_argument("--expect-success", action="store_true", default=False)
    parser.add_argument("--expected-error-code", default="")
    parser.add_argument(
        "--src-dir",
        default="/home/andy/robocup26/26-WrightEagle.AI-MHRC-planning/src",
        help="MHRC src directory containing modules/",
    )
    return parser.parse_args()


def resolve_expectations(args):
    if args.expect_success:
        return True, ""

    if args.expected_error_code:
        return False, args.expected_error_code

    if args.mode == "timeout":
        return False, "ack_timeout"

    if args.mode == "deny-ack":
        return False, str(args.reply_error_code or "").strip() or "navigate_ack_failed"

    return False, "ack_timeout"


def configure_adapter_env(args):
    os.environ["MHRC_TASK5_NAV_REQUEST_TOPIC"] = args.request_topic
    os.environ["MHRC_TASK5_NAV_ACK_TOPIC"] = args.ack_topic
    os.environ["MHRC_TASK5_ACK_TIMEOUT"] = str(args.ack_timeout)
    os.environ["MHRC_TASK5_ACK_REQUIRED"] = "true"
    os.environ["MHRC_TASK5_NAV_ACK_REQUIRED"] = "true"
    os.environ["MHRC_TASK5_NAV_DELEGATE_TO_TASK5"] = "true"


def main():
    args = parse_args()
    expect_success, expect_error_code = resolve_expectations(args)

    configure_adapter_env(args)

    if os.path.isdir(args.src_dir) and args.src_dir not in sys.path:
        sys.path.insert(0, args.src_dir)

    try:
        from modules.execution.task5_ros_adapter import Task5ROSAdapter
    except Exception as exc:
        print(f"[probe] failed to import Task5ROSAdapter: {exc}", file=sys.stderr)
        return 2

    try:
        rospy.init_node("task5_ack_timeout_probe", anonymous=True, disable_signals=True)
    except Exception:
        pass

    responder = None
    if args.mode in ("delayed-ack", "deny-ack"):
        success = args.mode == "delayed-ack"
        responder = FakeAckResponder(
            request_topic=args.request_topic,
            ack_topic=args.ack_topic,
            delay_sec=args.ack_delay,
            success=success,
            error_code="" if success else args.reply_error_code,
            message="ok" if success else args.reply_message,
        )
        rospy.sleep(0.2)

    try:
        adapter = Task5ROSAdapter(name="Task5AckProbeAdapter")
    except Exception as exc:
        print(f"[probe] adapter init failed: {exc}", file=sys.stderr)
        return 2

    success = adapter.navigate(args.target)
    result = adapter.get_last_action_result()

    observed_error_code = ""
    observed_customer_no = ""
    if isinstance(result, dict):
        data = result.get("data", {}) if isinstance(result.get("data", {}), dict) else {}
        observed_error_code = str(data.get("error_code") or "").strip()
        observed_customer_no = str(data.get("customer_no") or "").strip()

    print(json.dumps(
        {
            "mode": args.mode,
            "target": args.target,
            "success": success,
            "result": result,
            "observed_error_code": observed_error_code,
            "observed_customer_no": observed_customer_no,
            "expected_success": expect_success,
            "expected_error_code": expect_error_code,
            "responder_enabled": bool(responder is not None),
            "ack_timeout": args.ack_timeout,
            "ack_delay": args.ack_delay,
            "ack_topic": args.ack_topic,
        },
        ensure_ascii=False,
        indent=2,
    ))

    if bool(success) != bool(expect_success):
        print("[probe] unexpected success flag", file=sys.stderr)
        return 1

    if expect_error_code and observed_error_code != expect_error_code:
        print(
            f"[probe] unexpected error_code: observed={observed_error_code} expected={expect_error_code}",
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
