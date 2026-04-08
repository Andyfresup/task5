"""
Task5 ROS execution adapter.

Maps MHRC actions to Task5 ROS topics so the existing Task5 runtime can execute
navigate/search/speak/pick/place/wait with a unified RobotInterface.
"""

import importlib.util
import json
import math
import os
import socket
import threading
import time
import uuid
from typing import Dict, Optional, Tuple
from urllib.parse import urlparse

from modules.execution.robot_interface import RobotInterface, RobotState


class Task5ROSAdapter(RobotInterface):
    """Real robot adapter using ROS1 topics from Task5 stack."""

    def __init__(self, name: str = "Task5ROSAdapter"):
        super().__init__()
        self.name = name

        self.global_frame = os.environ.get("MHRC_TASK5_GLOBAL_FRAME", "map")
        self.goal_topic = os.environ.get("MHRC_TASK5_GOAL_TOPIC", "/move_base_simple/goal")
        self.search_topic = os.environ.get("MHRC_TASK5_SEARCH_TOPIC", "/person_following/search_cmd_vel")
        self.speak_topic = os.environ.get("MHRC_TASK5_SPEAK_TOPIC", "/person_following/mhrc_tts_text")
        self.pick_topic = os.environ.get("MHRC_TASK5_PICK_TOPIC", "/person_following/pick_request")
        self.place_topic = os.environ.get("MHRC_TASK5_PLACE_TOPIC", "/person_following/place_request")
        self.navigate_request_topic = os.environ.get(
            "MHRC_TASK5_NAV_REQUEST_TOPIC",
            "/person_following/navigate_request",
        )
        self.navigate_ack_topic = os.environ.get("MHRC_TASK5_NAV_ACK_TOPIC", "/person_following/navigate_ack")
        self.pick_ack_topic = os.environ.get("MHRC_TASK5_PICK_ACK_TOPIC", "/person_following/pick_ack")
        self.place_ack_topic = os.environ.get("MHRC_TASK5_PLACE_ACK_TOPIC", "/person_following/place_ack")

        self.goal_republish_count = max(1, int(os.environ.get("MHRC_TASK5_GOAL_REPUBLISH", "2")))
        self.search_spin_speed = float(os.environ.get("MHRC_TASK5_SEARCH_SPIN_SPEED", "0.35"))
        self.search_spin_duration = max(0.1, float(os.environ.get("MHRC_TASK5_SEARCH_SPIN_DURATION", "1.2")))
        self.wait_seconds = max(0.0, float(os.environ.get("MHRC_TASK5_WAIT_SECONDS", "0.5")))
        self.ack_timeout = max(0.2, float(os.environ.get("MHRC_TASK5_ACK_TIMEOUT", "6.0")))
        self.ack_required_default = self._env_flag("MHRC_TASK5_ACK_REQUIRED", False)
        self.navigate_ack_required = self._env_flag("MHRC_TASK5_NAV_ACK_REQUIRED", self.ack_required_default)
        self.pick_ack_required = self._env_flag("MHRC_TASK5_PICK_ACK_REQUIRED", self.ack_required_default)
        self.place_ack_required = self._env_flag("MHRC_TASK5_PLACE_ACK_REQUIRED", self.ack_required_default)
        self.navigate_delegate_to_task5 = self._env_flag("MHRC_TASK5_NAV_DELEGATE_TO_TASK5", True)

        self.tts_module_file = os.environ.get("MHRC_TASK5_TTS_MODULE_FILE", "").strip()
        self.tts_class_name = os.environ.get("MHRC_TASK5_TTS_CLASS", "TTS").strip() or "TTS"

        self._rospy = None
        self._PoseStamped = None
        self._Twist = None
        self._String = None

        self.goal_pub = None
        self.navigate_request_pub = None
        self.search_pub = None
        self.speak_pub = None
        self.pick_pub = None
        self.place_pub = None

        self._tts_instance = None
        self._tts_load_attempted = False
        self._ack_cond = threading.Condition()
        self._ack_seq = {"navigate": 0, "pick": 0, "place": 0}
        self._last_ack = {"navigate": None, "pick": None, "place": None}

        self.known_locations = self._load_location_map()
        self._ensure_ros_ready()

    def _env_flag(self, name: str, default: bool) -> bool:
        value = os.environ.get(name)
        if value is None:
            return bool(default)
        return str(value).strip().lower() in ("1", "true", "yes", "on", "y")

    def _load_location_map(self) -> Dict[str, Tuple[float, float, float]]:
        mapping = {
            "home": (0.0, 0.0, 0.0),
            "bar": (0.0, 0.0, 0.0),
            "counter": (0.0, 0.0, 0.0),
            "kitchen": (5.0, 2.0, 0.0),
            "table": (4.0, 1.0, 0.0),
            "desk": (1.0, 3.0, 0.0),
            "living_room": (3.0, -1.0, 0.0),
        }

        json_blob = os.environ.get("MHRC_TASK5_LOCATION_MAP_JSON", "").strip()
        if json_blob:
            try:
                data = json.loads(json_blob)
                if isinstance(data, dict):
                    for key, value in data.items():
                        pose = self._coerce_pose(value)
                        if pose is not None:
                            mapping[str(key).strip().lower()] = pose
            except Exception:
                pass

        json_file = os.environ.get("MHRC_TASK5_LOCATION_MAP_FILE", "").strip()
        if json_file and os.path.isfile(json_file):
            try:
                with open(json_file, "r", encoding="utf-8") as fp:
                    data = json.load(fp)
                if isinstance(data, dict):
                    for key, value in data.items():
                        pose = self._coerce_pose(value)
                        if pose is not None:
                            mapping[str(key).strip().lower()] = pose
            except Exception:
                pass

        return mapping

    def _coerce_pose(self, value) -> Optional[Tuple[float, float, float]]:
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            try:
                x = float(value[0])
                y = float(value[1])
                yaw = float(value[2]) if len(value) >= 3 else 0.0
                return (x, y, yaw)
            except Exception:
                return None
        return None

    def _ensure_ros_ready(self):
        if self._rospy is not None:
            return

        try:
            import rospy
            from geometry_msgs.msg import PoseStamped, Twist
            from std_msgs.msg import String
        except Exception as exc:
            raise RuntimeError("ROS dependencies unavailable for Task5ROSAdapter") from exc

        self._rospy = rospy
        self._PoseStamped = PoseStamped
        self._Twist = Twist
        self._String = String

        if not rospy.core.is_initialized():
            if not self._is_ros_master_reachable():
                raise RuntimeError("ROS master is not reachable")
            rospy.init_node("mhrc_task5_adapter", anonymous=True, disable_signals=True)

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.navigate_request_pub = rospy.Publisher(self.navigate_request_topic, String, queue_size=10)
        self.search_pub = rospy.Publisher(self.search_topic, Twist, queue_size=1)
        self.speak_pub = rospy.Publisher(self.speak_topic, String, queue_size=10)
        self.pick_pub = rospy.Publisher(self.pick_topic, String, queue_size=10)
        self.place_pub = rospy.Publisher(self.place_topic, String, queue_size=10)
        rospy.Subscriber(self.navigate_ack_topic, String, self._make_ack_callback("navigate"), queue_size=20)
        rospy.Subscriber(self.pick_ack_topic, String, self._make_ack_callback("pick"), queue_size=20)
        rospy.Subscriber(self.place_ack_topic, String, self._make_ack_callback("place"), queue_size=20)

        rospy.sleep(0.15)
        rospy.loginfo(
            "Task5ROSAdapter ready: goal=%s search=%s ack_timeout=%.1fs",
            self.goal_topic,
            self.search_topic,
            self.ack_timeout,
        )

    def _is_ros_master_reachable(self) -> bool:
        master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311").strip()
        parsed = urlparse(master_uri)
        host = parsed.hostname or "localhost"
        port = int(parsed.port or 11311)

        try:
            sock = socket.create_connection((host, port), timeout=0.35)
            sock.close()
            return True
        except Exception:
            return False

    def _resolve_target_pose(self, target) -> Optional[Tuple[float, float, float]]:
        if isinstance(target, (list, tuple)):
            return self._coerce_pose(target)

        if not isinstance(target, str):
            return None

        key = target.strip().lower().replace(" ", "_")
        if key in self.known_locations:
            return self.known_locations[key]

        compact = target.replace(",", " ")
        parts = [p for p in compact.split() if p]
        if len(parts) >= 2:
            try:
                x = float(parts[0])
                y = float(parts[1])
                yaw = float(parts[2]) if len(parts) >= 3 else 0.0
                return (x, y, yaw)
            except Exception:
                return None

        return None

    def _make_ack_callback(self, action_type: str):
        def _callback(msg):
            payload = self._parse_ack_payload(msg.data)
            payload["stamp"] = time.monotonic()
            payload["action"] = action_type
            with self._ack_cond:
                self._ack_seq[action_type] += 1
                self._last_ack[action_type] = payload
                self._ack_cond.notify_all()

        return _callback

    def _parse_ack_payload(self, raw: str) -> dict:
        text = str(raw or "").strip()
        payload = {}
        if text:
            try:
                obj = json.loads(text)
                if isinstance(obj, dict):
                    payload = obj
            except Exception:
                payload = {"status": text}

        success = payload.get("success")
        if success is None:
            success = payload.get("ok")
        if success is None:
            status = str(payload.get("status", "")).strip().lower()
            if status in ("ok", "success", "succeeded", "done", "1", "true"):
                success = True
            elif status in ("fail", "failed", "error", "timeout", "0", "false"):
                success = False
            elif text.lower() in ("ok", "success", "true", "1"):
                success = True
            elif text.lower() in ("fail", "failed", "false", "0", "timeout"):
                success = False

        return {
            "success": bool(success) if success is not None else False,
            "error": str(payload.get("error") or payload.get("message") or "").strip(),
            "error_code": str(payload.get("error_code") or "").strip(),
            "message": str(payload.get("message") or payload.get("error") or "").strip(),
            "request_id": str(payload.get("request_id") or "").strip(),
            "customer_no": str(payload.get("customer_no") or "").strip(),
            "active_customer_state": str(payload.get("active_customer_state") or "").strip(),
            "return_navigation_state": str(payload.get("return_navigation_state") or "").strip(),
            "recommendation": str(payload.get("recommendation") or "").strip(),
            "raw": text,
        }

    def _wait_for_ack(self, action_type: str, request_id: str = "", timeout: Optional[float] = None) -> dict:
        wait_timeout = max(0.2, float(timeout or self.ack_timeout))
        deadline = time.monotonic() + wait_timeout

        with self._ack_cond:
            seen_seq = int(self._ack_seq.get(action_type, 0))

            while True:
                curr_seq = int(self._ack_seq.get(action_type, 0))
                ack = self._last_ack.get(action_type)
                if curr_seq > seen_seq and isinstance(ack, dict):
                    ack_request_id = str(ack.get("request_id") or "").strip()
                    if request_id and ack_request_id and request_id != ack_request_id:
                        seen_seq = curr_seq
                    else:
                        return ack

                remain = deadline - time.monotonic()
                if remain <= 0.0:
                    break
                self._ack_cond.wait(remain)

        return {
            "success": False,
            "error": "ack_timeout",
            "error_code": "ack_timeout",
            "message": "ack_timeout",
            "request_id": request_id,
            "customer_no": "",
            "raw": "",
        }

    def _load_tts_instance(self):
        if self._tts_instance is not None:
            return self._tts_instance
        if self._tts_load_attempted:
            return None

        self._tts_load_attempted = True
        module_file = self.tts_module_file
        if not module_file or not os.path.isfile(module_file):
            return None

        try:
            spec = importlib.util.spec_from_file_location("mhrc_task5_tts", module_file)
            if spec is None or spec.loader is None:
                return None
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            tts_cls = getattr(module, self.tts_class_name, None)
            if tts_cls is None:
                return None
            self._tts_instance = tts_cls()
            return self._tts_instance
        except Exception:
            return None

    def _speak_with_module(self, content: str) -> bool:
        tts = self._load_tts_instance()
        if tts is None:
            return False

        try:
            if hasattr(tts, "say"):
                tts.say(content)
                return True
            if hasattr(tts, "speak"):
                tts.speak(content)
                return True
        except Exception:
            return False

        return False

    def navigate(self, target) -> bool:
        self.set_state(RobotState.EXECUTING)
        try:
            pose = self._resolve_target_pose(target)
            if pose is None:
                self.set_state(RobotState.ERROR)
                self.set_last_action_result(
                    "navigate",
                    False,
                    "invalid_target",
                    {"target": target, "error_code": "invalid_target"},
                )
                return False

            x, y, yaw = pose
            request_id = uuid.uuid4().hex
            msg = self._PoseStamped()
            msg.header.stamp = self._rospy.Time.now()
            msg.header.frame_id = self.global_frame
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 0.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = math.sin(yaw * 0.5)
            msg.pose.orientation.w = math.cos(yaw * 0.5)

            nav_req = {
                "action": "navigate",
                "request_id": request_id,
                "target": target,
                "pose": {"x": x, "y": y, "yaw": yaw},
                "frame_id": self.global_frame,
                "timestamp": time.time(),
            }
            self.navigate_request_pub.publish(self._String(data=json.dumps(nav_req, ensure_ascii=False)))

            need_ack = bool(self.navigate_ack_required or self.navigate_delegate_to_task5)
            ack = {}
            if need_ack:
                ack = self._wait_for_ack("navigate", request_id=request_id, timeout=self.ack_timeout)
                if not ack.get("success", False):
                    error_code = str(ack.get("error_code") or ack.get("error") or "navigate_ack_failed").strip()
                    error_msg = str(ack.get("message") or ack.get("error") or error_code).strip()
                    self.set_state(RobotState.ERROR)
                    self.set_last_action_result(
                        "navigate",
                        False,
                        error_msg,
                        {
                            "request_id": request_id,
                            "target": target,
                            "error_code": error_code,
                            "customer_no": str(ack.get("customer_no") or "").strip(),
                            "active_customer_state": ack.get("active_customer_state", ""),
                            "return_navigation_state": ack.get("return_navigation_state", ""),
                            "recommendation": ack.get("recommendation", ""),
                        },
                    )
                    return False

            if not self.navigate_delegate_to_task5:
                for _ in range(self.goal_republish_count):
                    self.goal_pub.publish(msg)
                    self._rospy.sleep(0.03)

            self.current_position = str(target)
            self.set_state(RobotState.IDLE)
            self.set_last_action_result(
                "navigate",
                True,
                "",
                {
                    "request_id": request_id,
                    "target": target,
                    "delegated": bool(self.navigate_delegate_to_task5),
                    "customer_no": str(ack.get("customer_no") or "").strip(),
                },
            )
            return True
        except Exception as exc:
            self.set_state(RobotState.ERROR)
            self.set_last_action_result(
                "navigate",
                False,
                str(exc),
                {"target": target, "error_code": "navigate_exception"},
            )
            return False

    def search(self, object_name: str) -> Optional[dict]:
        self.set_state(RobotState.EXECUTING)
        try:
            twist = self._Twist()
            twist.angular.z = self.search_spin_speed

            stop = self._Twist()
            end_at = time.monotonic() + self.search_spin_duration
            rate = self._rospy.Rate(10)
            while time.monotonic() < end_at and not self._rospy.is_shutdown():
                self.search_pub.publish(twist)
                rate.sleep()

            self.search_pub.publish(stop)
            self.set_state(RobotState.IDLE)
            result = {
                "name": object_name,
                "position": None,
                "source": "task5_search_cmd",
            }
            self.set_last_action_result("search", True, "", result)
            return result
        except Exception as exc:
            self.set_state(RobotState.ERROR)
            self.set_last_action_result("search", False, str(exc), {"object_name": object_name})
            return None

    def pick(self, object_name: str, object_id: Optional[int] = None) -> bool:
        self.set_state(RobotState.EXECUTING)
        try:
            request_id = uuid.uuid4().hex
            payload = {
                "action": "pick",
                "request_id": request_id,
                "object_name": object_name,
                "object_id": object_id,
                "timestamp": time.time(),
            }
            self.pick_pub.publish(self._String(data=json.dumps(payload, ensure_ascii=False)))

            if self.pick_ack_required:
                ack = self._wait_for_ack("pick", request_id=request_id, timeout=self.ack_timeout)
                if not ack.get("success", False):
                    error_msg = ack.get("error") or "pick_ack_failed"
                    self.set_state(RobotState.ERROR)
                    self.set_last_action_result("pick", False, error_msg, {"request_id": request_id})
                    return False

            self.holding_object = object_name
            self.set_state(RobotState.IDLE)
            self.set_last_action_result(
                "pick",
                True,
                "",
                {"request_id": request_id, "object_name": object_name, "object_id": object_id},
            )
            return True
        except Exception as exc:
            self.set_state(RobotState.ERROR)
            self.set_last_action_result("pick", False, str(exc), {"object_name": object_name})
            return False

    def place(self, location) -> bool:
        self.set_state(RobotState.EXECUTING)
        try:
            request_id = uuid.uuid4().hex
            payload = {
                "action": "place",
                "request_id": request_id,
                "location": location,
                "holding_object": self.holding_object,
                "timestamp": time.time(),
            }
            self.place_pub.publish(self._String(data=json.dumps(payload, ensure_ascii=False)))

            if self.place_ack_required:
                ack = self._wait_for_ack("place", request_id=request_id, timeout=self.ack_timeout)
                if not ack.get("success", False):
                    error_msg = ack.get("error") or "place_ack_failed"
                    self.set_state(RobotState.ERROR)
                    self.set_last_action_result("place", False, error_msg, {"request_id": request_id})
                    return False

            self.holding_object = None
            self.set_state(RobotState.IDLE)
            self.set_last_action_result("place", True, "", {"request_id": request_id, "location": location})
            return True
        except Exception as exc:
            self.set_state(RobotState.ERROR)
            self.set_last_action_result("place", False, str(exc), {"location": location})
            return False

    def speak(self, content: str) -> bool:
        self.set_state(RobotState.EXECUTING)
        try:
            if not self._speak_with_module(content):
                self.speak_pub.publish(self._String(data=str(content)))
            self.set_state(RobotState.IDLE)
            self.set_last_action_result("speak", True, "", {"content": content})
            return True
        except Exception as exc:
            self.set_state(RobotState.ERROR)
            self.set_last_action_result("speak", False, str(exc), {"content": content})
            return False

    def wait(self, reason: Optional[str] = None) -> bool:
        self.set_state(RobotState.EXECUTING)
        try:
            if self.wait_seconds > 0.0:
                self._rospy.sleep(self.wait_seconds)
            self.set_state(RobotState.IDLE)
            self.set_last_action_result("wait", True, "", {"reason": reason})
            return True
        except Exception as exc:
            self.set_state(RobotState.ERROR)
            self.set_last_action_result("wait", False, str(exc), {"reason": reason})
            return False
