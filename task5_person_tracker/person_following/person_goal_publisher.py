#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Publish person-follow navigation goals with obstacle-aware target selection.

Behavior:
- Subscribe person position from /person/base_link_3d_position.
- Transform person point into global frame.
- Sample candidate goals in front sector of person and score them.
- Filter candidates with occupancy-grid collision checks.
- Publish PoseStamped to /move_base_simple/goal.
- Built-in bridge publishes PointStamped to /way_point and /goal_point.
"""

import importlib.util
import json
import math
import os
import re
import difflib
import shlex
import subprocess
import sys
import threading
import time
import urllib.request
from collections import deque
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeoutError

import numpy as np

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PersonGoalPublisher:
    def __init__(self):
        rospy.init_node("person_goal_publisher", anonymous=False)

        # Topics and frames.
        # Defaults below are tuned for far_planner + fastlio integrated navigation.
        self.person_topic = rospy.get_param("~person_topic", "/person/base_link_3d_position")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.waypoint_topic = rospy.get_param("~waypoint_topic", "/way_point")
        self.goal_point_topic = rospy.get_param("~goal_point_topic", "/goal_point")
        self.map_topic = rospy.get_param("~map_topic", "/move_base/global_costmap/costmap")
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        # Goal generation and publish control.
        # Equivalent to:
        # _follow_distance:=1.1
        # _robot_radius:=0.34
        # _occupancy_threshold:=50
        # _switch_score_margin:=0.35
        # _switch_score_ratio:=0.15
        self.follow_distance = rospy.get_param("~follow_distance", 1.1)
        self.goal_update_distance = rospy.get_param("~goal_update_distance", 0.22)
        self.person_timeout = rospy.get_param("~person_timeout", 0.8)
        self.min_publish_interval = rospy.get_param("~min_publish_interval", 0.3)
        self.publish_waypoint_direct = rospy.get_param("~publish_waypoint_direct", True)

        # Stop goal updates after robot reaches current goal stably to reduce jitter.
        self.stop_publish_on_reach = rospy.get_param("~stop_publish_on_reach", True)
        self.goal_reach_distance = rospy.get_param("~goal_reach_distance", 0.28)
        self.goal_reach_hold_time = rospy.get_param("~goal_reach_hold_time", 1.0)
        self.person_reacquire_distance = rospy.get_param("~person_reacquire_distance", 0.45)
        self.person_reacquire_forward = rospy.get_param("~person_reacquire_forward", 0.28)
        self.person_reacquire_lateral = rospy.get_param("~person_reacquire_lateral", 0.20)
        self.person_reacquire_heading_deg = rospy.get_param("~person_reacquire_heading_deg", 14.0)

        # Gaze tracking stage after reaching a stable goal.
        self.gaze_tracking_on_pause = rospy.get_param("~gaze_tracking_on_pause", True)
        self.search_cmd_topic = rospy.get_param("~search_cmd_topic", "/person_following/search_cmd_vel")
        self.gaze_yaw_deadband_deg = rospy.get_param("~gaze_yaw_deadband_deg", 4.0)
        self.gaze_kp_angular = rospy.get_param("~gaze_kp_angular", 1.2)
        self.gaze_max_angular = rospy.get_param("~gaze_max_angular", 0.45)
        self.gaze_track_linear = rospy.get_param("~gaze_track_linear", True)
        self.gaze_target_distance = rospy.get_param("~gaze_target_distance", self.follow_distance)
        self.gaze_distance_tolerance = rospy.get_param("~gaze_distance_tolerance", 0.12)
        self.gaze_kp_linear = rospy.get_param("~gaze_kp_linear", 0.30)
        self.gaze_max_forward = rospy.get_param("~gaze_max_forward", 0.04)
        self.gaze_max_reverse = rospy.get_param("~gaze_max_reverse", 0.06)
        self.gaze_linear_heading_gate_deg = rospy.get_param("~gaze_linear_heading_gate_deg", 18.0)
        self.gaze_track_lateral = rospy.get_param("~gaze_track_lateral", True)
        self.gaze_kp_lateral = rospy.get_param("~gaze_kp_lateral", 0.22)
        self.gaze_max_lateral = rospy.get_param("~gaze_max_lateral", 0.03)
        self.gaze_lateral_deadband = rospy.get_param("~gaze_lateral_deadband", 0.05)
        self.gaze_lateral_heading_gate_deg = rospy.get_param("~gaze_lateral_heading_gate_deg", 24.0)
        self.gaze_collision_probe_distance = rospy.get_param("~gaze_collision_probe_distance", 0.55)
        self.gaze_person_timeout = rospy.get_param("~gaze_person_timeout", 0.6)

        # Run loop rate controls gaze-tracking and obstacle checks frequency.
        self.run_rate_hz = rospy.get_param("~run_rate_hz", 20.0)

        # Voice prompt when entering pause-gaze state.
        self.pause_prompt_enabled = rospy.get_param("~pause_prompt_enabled", True)
        self.pause_prompt_text = rospy.get_param("~pause_prompt_text", "What do you want")
        self.pause_prompt_topic = rospy.get_param("~pause_prompt_topic", "")
        self.pause_prompt_use_mhrc_speak = rospy.get_param("~pause_prompt_use_mhrc_speak", True)
        self.pause_prompt_mhrc_speak_topic = rospy.get_param(
            "~pause_prompt_mhrc_speak_topic",
            "/person_following/mhrc_tts_text",
        )
        self.pause_prompt_mhrc_require_subscriber = rospy.get_param(
            "~pause_prompt_mhrc_require_subscriber",
            True,
        )
        self.pause_prompt_task5_priority_higher = rospy.get_param(
            "~pause_prompt_task5_priority_higher",
            True,
        )
        self.mhrc_speak_bridge_enabled = rospy.get_param(
            "~mhrc_speak_bridge_enabled",
            True,
        )
        self.mhrc_speak_bridge_topic = rospy.get_param(
            "~mhrc_speak_bridge_topic",
            self.pause_prompt_mhrc_speak_topic,
        )
        self.pause_prompt_command = rospy.get_param("~pause_prompt_command", "")
        self.pause_prompt_use_shell = rospy.get_param("~pause_prompt_use_shell", False)
        self.pause_prompt_cooldown = rospy.get_param("~pause_prompt_cooldown", 2.0)
        default_speech_module_file = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "26-WrightEagle.AI-Speech",
                "src",
                "tts",
                "synthesizer.py",
            )
        )
        self.pause_prompt_use_speech_module = rospy.get_param("~pause_prompt_use_speech_module", False)
        self.pause_prompt_speech_module_file = rospy.get_param(
            "~pause_prompt_speech_module_file",
            default_speech_module_file,
        )
        self.pause_prompt_speech_class = rospy.get_param("~pause_prompt_speech_class", "TTS")

        # Optional one-shot ASR after pause prompt, reusing AI-Speech ASR module.
        default_asr_module_file = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "26-WrightEagle.AI-Speech",
                "src",
                "asr",
                "vad-whisper.py",
            )
        )
        self.pause_reply_listen_enabled = rospy.get_param("~pause_reply_listen_enabled", False)
        self.pause_reply_use_speech_module = rospy.get_param("~pause_reply_use_speech_module", True)
        self.pause_reply_speech_module_file = rospy.get_param(
            "~pause_reply_speech_module_file",
            default_asr_module_file,
        )
        self.pause_reply_speech_class = rospy.get_param("~pause_reply_speech_class", "SpeechRecognizer")
        self.pause_reply_mic_name = rospy.get_param("~pause_reply_mic_name", "Newmine")
        self.pause_reply_timeout = rospy.get_param("~pause_reply_timeout", 6.0)
        self.pause_reply_start_delay = rospy.get_param("~pause_reply_start_delay", 1.2)
        self.pause_reply_min_audio_sec = rospy.get_param("~pause_reply_min_audio_sec", 0.15)
        self.pause_reply_language = rospy.get_param("~pause_reply_language", "en")
        self.pause_reply_beam_size = rospy.get_param("~pause_reply_beam_size", 5)
        self.pause_reply_cooldown = rospy.get_param("~pause_reply_cooldown", 2.0)
        self.pause_reply_topic = rospy.get_param("~pause_reply_topic", "/person_following/pause_reply_text")
        self.pause_reply_text_input_enabled = rospy.get_param("~pause_reply_text_input_enabled", True)
        self.pause_reply_reask_on_unrecognized = rospy.get_param("~pause_reply_reask_on_unrecognized", True)
        self.pause_reply_reask_text = rospy.get_param(
            "~pause_reply_reask_text",
            "Can I beg you a pardon?",
        )
        self.pause_reply_reask_max_attempts = rospy.get_param("~pause_reply_reask_max_attempts", 1)
        self.pause_reply_reask_listen_delay = rospy.get_param("~pause_reply_reask_listen_delay", 1.1)

        # Parse recognized reply text for food items and persist to JSON.
        self.food_order_enabled = rospy.get_param("~food_order_enabled", True)
        default_food_order_json_file = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "food_orders.json")
        )
        self.food_order_json_file = rospy.get_param("~food_order_json_file", default_food_order_json_file)
        self.food_order_publish_topic = rospy.get_param(
            "~food_order_publish_topic",
            "/person_following/food_order_json",
        )
        self.food_order_confirm_enabled = rospy.get_param("~food_order_confirm_enabled", True)
        self.food_order_confirm_template = rospy.get_param(
            "~food_order_confirm_template",
            "OK, I'll get {foods} for you",
        )
        self.return_to_anchor_on_order_confirm = rospy.get_param(
            "~return_to_anchor_on_order_confirm",
            True,
        )
        default_return_anchor_json_file = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "return_anchor.json")
        )
        self.return_anchor_json_file = rospy.get_param(
            "~return_anchor_json_file",
            default_return_anchor_json_file,
        )
        self.return_anchor_bridge_republish = rospy.get_param("~return_anchor_bridge_republish", 2)
        self.return_table_approach_enabled = rospy.get_param("~return_table_approach_enabled", True)
        self.return_table_trigger_distance = rospy.get_param("~return_table_trigger_distance", 1.0)
        self.return_table_search_radius = rospy.get_param("~return_table_search_radius", 2.2)
        self.return_table_min_component_cells = rospy.get_param("~return_table_min_component_cells", 35)
        self.return_table_min_elongation = rospy.get_param("~return_table_min_elongation", 1.35)
        self.return_table_stop_offset = rospy.get_param("~return_table_stop_offset", 0.60)
        self.return_table_plan_retry_interval = rospy.get_param("~return_table_plan_retry_interval", 1.0)
        self.return_table_goal_republish = rospy.get_param("~return_table_goal_republish", 2)
        self.return_table_arrive_distance = rospy.get_param("~return_table_arrive_distance", 0.35)
        default_yolo_perception_dir = os.path.join("..", "26-WrightEagle.AI-YOLO-Perception")
        self.table_food_check_enabled = rospy.get_param("~table_food_check_enabled", True)
        self.table_food_check_delay = rospy.get_param("~table_food_check_delay", 1.0)
        self.table_food_detect_command = rospy.get_param(
            "~table_food_detect_command",
            "python3 realsenseinfer.py",
        )
        self.table_food_detect_use_shell = rospy.get_param("~table_food_detect_use_shell", False)
        self.table_food_detect_timeout = rospy.get_param("~table_food_detect_timeout", 5.0)
        self.table_food_detect_workdir = rospy.get_param(
            "~table_food_detect_workdir",
            default_yolo_perception_dir,
        )
        self.table_food_detection_json_file = rospy.get_param(
            "~table_food_detection_json_file",
            os.path.join(default_yolo_perception_dir, "detections.json"),
        )
        self.serving_target_enabled = rospy.get_param("~serving_target_enabled", True)
        default_serving_target_snapshot_file = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "serving_target_snapshot.json")
        )
        self.serving_target_snapshot_json_file = rospy.get_param(
            "~serving_target_snapshot_json_file",
            default_serving_target_snapshot_file,
        )
        self.serving_target_capture_topic = rospy.get_param(
            "~serving_target_capture_topic",
            "/person_following/serving_target_capture",
        )
        self.serving_target_capture_cmd_topic = rospy.get_param(
            "~serving_target_capture_cmd_topic",
            "/person_following/serving_target_capture_cmd",
        )
        self.customer_data_root = rospy.get_param(
            "~customer_data_root",
            os.path.abspath(os.path.join(os.path.dirname(__file__), "service_customers")),
        )
        self.active_customer_folder_topic = rospy.get_param(
            "~active_customer_folder_topic",
            "/person_following/active_customer_folder",
        )
        self.serving_customer_state_topic = rospy.get_param(
            "~serving_customer_state_topic",
            "/person_following/serving_customer_state",
        )
        self.navigate_request_topic = rospy.get_param(
            "~navigate_request_topic",
            "/person_following/navigate_request",
        )
        self.navigate_ack_topic = rospy.get_param(
            "~navigate_ack_topic",
            "/person_following/navigate_ack",
        )
        self.mhrc_nav_state_gating_enabled = rospy.get_param(
            "~mhrc_nav_state_gating_enabled",
            True,
        )
        self.mhrc_nav_force_accept = rospy.get_param("~mhrc_nav_force_accept", False)
        self.mhrc_nav_allow_locked = rospy.get_param("~mhrc_nav_allow_locked", False)
        self.mhrc_nav_request_ttl = rospy.get_param("~mhrc_nav_request_ttl", 30.0)
        self.mhrc_nav_debug_log_gating_decisions = rospy.get_param(
            "~mhrc_nav_debug_log_gating_decisions",
            False,
        )
        self.mhrc_nav_debug_state_override_enabled = rospy.get_param(
            "~mhrc_nav_debug_state_override_enabled",
            False,
        )
        self.mhrc_nav_debug_state_override_topic = rospy.get_param(
            "~mhrc_nav_debug_state_override_topic",
            "/person_following/debug_state_override",
        )
        self.gaze_stable_face_capture_enabled = rospy.get_param(
            "~gaze_stable_face_capture_enabled",
            True,
        )
        self.gaze_stable_face_hold_time = rospy.get_param("~gaze_stable_face_hold_time", 0.8)
        self.gaze_stable_capture_min_interval = rospy.get_param("~gaze_stable_capture_min_interval", 3.0)
        self.food_aliases = rospy.get_param(
            "~food_aliases",
            {
                "water": ["water", "bottle of water"],
                "coke": ["coke", "cola", "coca cola"],
                "juice": ["juice", "orange juice", "apple juice"],
                "coffee": ["coffee", "latte", "americano", "cappuccino"],
                "tea": ["tea", "black tea", "green tea", "milk tea"],
                "burger": ["burger", "hamburger", "cheeseburger"],
                "pizza": ["pizza"],
                "sandwich": ["sandwich"],
                "fried_rice": ["fried rice"],
                "noodles": ["noodles", "ramen"],
                "dumplings": ["dumpling", "dumplings"],
                "pasta": ["pasta", "spaghetti"],
                "fries": ["fries", "french fries", "chips"],
                "salad": ["salad"],
                "soup": ["soup"],
            },
        )
        self.food_qty_word_map = {
            "a": 1,
            "an": 1,
            "one": 1,
            "two": 2,
            "three": 3,
            "four": 4,
            "five": 5,
            "six": 6,
            "seven": 7,
            "eight": 8,
            "nine": 9,
            "ten": 10,
        }
        self.food_qty_bridge_tokens = {
            "x",
            "of",
            "piece",
            "pieces",
            "plate",
            "plates",
            "bowl",
            "bowls",
            "cup",
            "cups",
            "glass",
            "glasses",
            "bottle",
            "bottles",
            "can",
            "cans",
            "order",
            "orders",
            "portion",
            "portions",
        }
        self.food_patterns = self._build_food_patterns(self.food_aliases)
        self.food_alias_lookup = self._build_food_alias_lookup(self.food_aliases)
        self.food_alias_pairs = self._build_food_alias_pairs(self.food_aliases)
        self.food_catalog_set = set(self.food_alias_lookup.values())
        self.table_food_use_fuzzy_model = rospy.get_param("~table_food_use_fuzzy_model", True)
        self.table_food_fuzzy_backend = rospy.get_param(
            "~table_food_fuzzy_backend",
            "reuse_order_backend",
        )
        self.table_food_lexical_threshold = rospy.get_param("~table_food_lexical_threshold", 0.76)
        self.food_semantic_enabled = rospy.get_param("~food_semantic_enabled", True)
        self.food_semantic_backend = rospy.get_param("~food_semantic_backend", "ollama")
        self.food_semantic_command = rospy.get_param("~food_semantic_command", "")
        self.food_semantic_command_use_shell = rospy.get_param("~food_semantic_command_use_shell", False)
        self.food_semantic_timeout = rospy.get_param("~food_semantic_timeout", 8.0)
        self.food_semantic_model_path = rospy.get_param("~food_semantic_model_path", "")
        self.food_semantic_transformers_task = rospy.get_param(
            "~food_semantic_transformers_task",
            "text-generation",
        )
        self.food_semantic_device = rospy.get_param("~food_semantic_device", -1)
        self.food_semantic_max_new_tokens = rospy.get_param("~food_semantic_max_new_tokens", 120)
        self.food_semantic_temperature = rospy.get_param("~food_semantic_temperature", 0.0)
        self.food_semantic_ollama_url = rospy.get_param("~food_semantic_ollama_url", "")
        self.food_semantic_ollama_model = rospy.get_param("~food_semantic_ollama_model", "llama3.2:1b")
        self.food_semantic_ollama_keepalive = rospy.get_param("~food_semantic_ollama_keepalive", "5m")
        default_mhrc_src_dir = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "26-WrightEagle.AI-MHRC-planning",
                "src",
            )
        )
        self.food_semantic_mhrc_src_dir = rospy.get_param(
            "~food_semantic_mhrc_src_dir",
            default_mhrc_src_dir,
        )
        self.food_semantic_mhrc_base_url = rospy.get_param(
            "~food_semantic_mhrc_base_url",
            "http://localhost:11434/v1",
        )
        self.food_semantic_mhrc_api_key = rospy.get_param("~food_semantic_mhrc_api_key", "ollama")
        self.food_semantic_mhrc_model = rospy.get_param("~food_semantic_mhrc_model", "qwen2.5:3b")
        self.food_semantic_mhrc_temperature = rospy.get_param("~food_semantic_mhrc_temperature", 0.0)
        self.food_semantic_mhrc_max_tokens = rospy.get_param("~food_semantic_mhrc_max_tokens", 220)
        self.food_semantic_mhrc_async_workers = rospy.get_param("~food_semantic_mhrc_async_workers", 1)
        self.food_semantic_mhrc_timeout = rospy.get_param(
            "~food_semantic_mhrc_timeout",
            self.food_semantic_timeout,
        )
        self.food_semantic_mhrc_fuse_fail_threshold = rospy.get_param(
            "~food_semantic_mhrc_fuse_fail_threshold",
            3,
        )
        self.food_semantic_mhrc_fuse_cooldown = rospy.get_param(
            "~food_semantic_mhrc_fuse_cooldown",
            15.0,
        )
        self.food_semantic_mhrc_stats_log_interval = rospy.get_param(
            "~food_semantic_mhrc_stats_log_interval",
            30.0,
        )

        # Candidate sampling and scoring.
        self.min_candidate_radius = rospy.get_param("~min_candidate_radius", 0.9)
        self.max_candidate_radius = rospy.get_param("~max_candidate_radius", 1.6)
        self.radius_samples = rospy.get_param("~radius_samples", 5)
        self.front_angle_span_deg = rospy.get_param("~front_angle_span_deg", 90.0)
        self.angle_samples = rospy.get_param("~angle_samples", 13)
        self.weight_distance = rospy.get_param("~weight_distance", 2.4)
        self.weight_angle = rospy.get_param("~weight_angle", 1.2)
        self.weight_robot = rospy.get_param("~weight_robot", 0.8)

        # Hysteresis for goal switching: only switch when new goal is clearly better.
        self.switch_score_margin = rospy.get_param("~switch_score_margin", 0.35)
        self.switch_score_ratio = rospy.get_param("~switch_score_ratio", 0.15)

        # Collision constraints.
        self.robot_radius = rospy.get_param("~robot_radius", 0.34)
        self.path_check_step = rospy.get_param("~path_check_step", 0.05)
        self.occupancy_threshold = rospy.get_param("~occupancy_threshold", 50)
        self.unknown_is_occupied = rospy.get_param("~unknown_is_occupied", True)

        # Person heading estimation.
        self.min_heading_speed = rospy.get_param("~min_heading_speed", 0.05)

        # TF listener for frame conversion.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, PointStamped, queue_size=1)
        self.goal_point_pub = rospy.Publisher(self.goal_point_topic, PointStamped, queue_size=1)
        self.search_cmd_pub = rospy.Publisher(self.search_cmd_topic, Twist, queue_size=1)
        self.pause_prompt_pub = None
        if self.pause_prompt_topic:
            self.pause_prompt_pub = rospy.Publisher(self.pause_prompt_topic, String, queue_size=1)
        self.pause_prompt_mhrc_pub = None
        if self.pause_prompt_use_mhrc_speak and self.pause_prompt_mhrc_speak_topic:
            self.pause_prompt_mhrc_pub = rospy.Publisher(
                self.pause_prompt_mhrc_speak_topic,
                String,
                queue_size=10,
            )
        self.pause_reply_pub = None
        if self.pause_reply_topic:
            self.pause_reply_pub = rospy.Publisher(self.pause_reply_topic, String, queue_size=1)
        self.food_order_pub = None
        if self.food_order_publish_topic:
            self.food_order_pub = rospy.Publisher(self.food_order_publish_topic, String, queue_size=1)
        self.serving_target_capture_pub = None
        if self.serving_target_enabled and self.serving_target_capture_topic:
            self.serving_target_capture_pub = rospy.Publisher(
                self.serving_target_capture_topic,
                PointStamped,
                queue_size=1,
            )
        self.serving_target_capture_cmd_pub = None
        if self.serving_target_enabled and self.serving_target_capture_cmd_topic:
            self.serving_target_capture_cmd_pub = rospy.Publisher(
                self.serving_target_capture_cmd_topic,
                String,
                queue_size=1,
            )
        self.serving_customer_state_pub = None
        if self.serving_customer_state_topic:
            self.serving_customer_state_pub = rospy.Publisher(
                self.serving_customer_state_topic,
                String,
                queue_size=1,
                latch=True,
            )
        self.navigate_ack_pub = None
        if self.navigate_ack_topic:
            self.navigate_ack_pub = rospy.Publisher(
                self.navigate_ack_topic,
                String,
                queue_size=20,
            )

        rospy.Subscriber(self.person_topic, PointStamped, self.person_callback, queue_size=1)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber(self.active_customer_folder_topic, String, self.active_customer_folder_callback, queue_size=1)
        rospy.Subscriber(self.navigate_request_topic, String, self.navigate_request_callback, queue_size=20)
        if self.mhrc_speak_bridge_enabled and self.mhrc_speak_bridge_topic:
            rospy.Subscriber(
                self.mhrc_speak_bridge_topic,
                String,
                self.mhrc_speak_callback,
                queue_size=20,
            )
        if self.mhrc_nav_debug_state_override_enabled and self.mhrc_nav_debug_state_override_topic:
            rospy.Subscriber(
                self.mhrc_nav_debug_state_override_topic,
                String,
                self.debug_state_override_callback,
                queue_size=20,
            )
        if self.pause_reply_text_input_enabled and self.pause_reply_topic:
            rospy.Subscriber(
                self.pause_reply_topic,
                String,
                self.pause_reply_topic_callback,
                queue_size=20,
            )
        rospy.on_shutdown(self._on_shutdown)

        self.last_person_time = rospy.Time(0)
        self.last_goal_x = None
        self.last_goal_y = None
        self.last_goal_theta = None
        self.last_goal_score = None
        self.last_publish_time = rospy.Time(0)
        self.goal_reach_since = None
        self.goal_publish_paused = False
        self.paused_person_x = None
        self.paused_person_y = None
        self.paused_person_base_x = None
        self.paused_person_base_y = None
        self.paused_person_heading = None
        self.gaze_cmd_active = False
        self.latest_person_base_x = None
        self.latest_person_base_y = None
        self.latest_person_base_time = rospy.Time(0)
        self.last_pause_prompt_time = rospy.Time(0)
        self.last_local_mhrc_prompt_text = ""
        self.last_local_mhrc_prompt_mono = 0.0
        self.pause_speech_tts = None
        self.pause_speech_load_attempted = False
        self.pause_speech_lock = threading.Lock()
        self.pause_speech_asr = None
        self.pause_asr_load_attempted = False
        self.pause_asr_lock = threading.Lock()
        self.pause_reply_topic_lock = threading.Lock()
        self.pause_reply_thread = None
        self.last_pause_reply_time = rospy.Time(0)
        self.food_order_lock = threading.Lock()
        self.food_order_history = []
        self.current_needed_foods = []
        self.current_needed_foods_qty = {}
        self.return_to_anchor_active = False
        self.return_anchor_goal = None
        self.return_navigation_state = "IDLE"
        self.return_table_goal = None
        self.return_table_plan_attempt_time = rospy.Time(0)
        self.table_front_arrive_time = rospy.Time(0)
        self.table_food_check_done = False
        self.last_serving_target_capture_time = rospy.Time(0)
        self.gaze_stable_since = None
        self.last_gaze_stable_capture_time = rospy.Time(0)
        self.active_customer_folder = ""
        self.active_customer_id = ""
        self.active_customer_no = ""
        self.active_customer_state = "IDLE"
        self.food_semantic_pipeline = None
        self.food_semantic_load_attempted = False
        self.food_semantic_lock = threading.Lock()
        self.food_semantic_mhrc_client = None
        self.food_semantic_mhrc_load_attempted = False
        self.food_semantic_mhrc_lock = threading.Lock()
        self.food_semantic_mhrc_executor = ThreadPoolExecutor(
            max_workers=max(1, int(self.food_semantic_mhrc_async_workers))
        )
        self.food_semantic_mhrc_stats_lock = threading.Lock()
        self.food_semantic_mhrc_stats = {
            "submitted": 0,
            "success": 0,
            "failed": 0,
            "timeout": 0,
            "fuse_opened": 0,
            "fuse_blocked": 0,
            "fallback_used": 0,
            "latency_ms_sum": 0.0,
            "latency_ms_max": 0.0,
        }
        self.food_semantic_mhrc_fuse_open_until = 0.0
        self.food_semantic_mhrc_consecutive_failures = 0
        self.food_semantic_mhrc_last_stats_log = 0.0
        self.table_food_fuzzy_cache = {}
        self.table_food_fuzzy_lock = threading.Lock()

        self.last_person_x = None
        self.last_person_y = None
        self.last_person_stamp = None
        self.person_heading = None

        self.grid = None
        self._load_food_orders_from_json()

        rospy.loginfo(
            "person_goal_publisher started: person=%s goal=%s waypoint=%s goal_point=%s map=%s",
            self.person_topic,
            self.goal_topic,
            self.waypoint_topic,
            self.goal_point_topic,
            self.map_topic,
        )

    def map_callback(self, msg):
        self.grid = msg

    def _on_shutdown(self):
        try:
            self.food_semantic_mhrc_executor.shutdown(wait=False)
        except Exception:
            pass
        self._log_mhrc_stats_if_needed(force=True)

    def _customer_no_from_folder(self, folder):
        name = os.path.basename(str(folder or "").rstrip("/"))
        return str(name or "").strip()

    def _refresh_active_customer_no(self):
        folder_no = self._customer_no_from_folder(self.active_customer_folder)
        if folder_no:
            self.active_customer_no = folder_no
            if not str(self.active_customer_id or "").strip():
                self.active_customer_id = folder_no
            return self.active_customer_no

        cid = str(self.active_customer_id or "").strip()
        self.active_customer_no = cid
        return self.active_customer_no

    def _set_active_customer_context(self, folder=None, customer_id=None):
        folder_text = str(folder or "").strip()
        if folder_text:
            if not os.path.isdir(folder_text):
                return False
            self.active_customer_folder = folder_text
            folder_no = self._customer_no_from_folder(folder_text)
            cid = str(customer_id or "").strip()
            self.active_customer_id = cid or folder_no
            self._refresh_active_customer_no()
            return True

        cid = str(customer_id or "").strip()
        if cid:
            self.active_customer_id = cid
            self._refresh_active_customer_no()
            return True

        return False

    def _clear_active_customer_context(self):
        self.active_customer_folder = ""
        self.active_customer_id = ""
        self.active_customer_no = ""

    def active_customer_folder_callback(self, msg):
        folder = str(msg.data).strip()
        if not folder:
            return
        if not os.path.isdir(folder):
            rospy.logwarn("Active customer folder does not exist yet: %s", folder)
            return

        self._set_active_customer_context(folder=folder)
        self._load_food_orders_from_json()
        self._set_serving_customer_state("LOCKED", {"source": "active_customer_folder_topic"})

    def _set_serving_customer_state(self, state, extra=None):
        state_text = str(state or "IDLE").strip().upper() or "IDLE"
        self.active_customer_state = state_text

        if state_text != "IDLE":
            self._resolve_active_customer_folder()

        customer_no = ""
        if state_text != "IDLE":
            customer_no = self._refresh_active_customer_no()
            if not customer_no:
                rospy.logwarn_throttle(
                    2.0,
                    "Serving state %s has no customer No. tag yet (waiting active customer folder).",
                    state_text,
                )

        payload = {
            "timestamp": rospy.Time.now().to_sec(),
            "state": self.active_customer_state,
            "customer_id": self.active_customer_id,
            "customer_no": customer_no,
            "folder": self.active_customer_folder,
        }
        if isinstance(extra, dict):
            payload.update(extra)

        if self.serving_customer_state_pub is not None:
            try:
                self.serving_customer_state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            except Exception as exc:
                rospy.logwarn("Failed to publish serving customer state: %s", exc)

        folder = self.active_customer_folder
        if state_text != "IDLE" and (not folder or not os.path.isdir(folder)):
            folder = self._resolve_active_customer_folder()
        if folder and os.path.isdir(folder):
            path = os.path.join(folder, "customer_service_state.json")
            try:
                tmp_path = path + ".tmp"
                with open(tmp_path, "w", encoding="utf-8") as fp:
                    json.dump(payload, fp, indent=2, ensure_ascii=False)
                os.replace(tmp_path, path)
            except Exception as exc:
                rospy.logwarn("Failed to write customer service state file: %s", exc)

    def debug_state_override_callback(self, msg):
        if not self.mhrc_nav_debug_state_override_enabled:
            return

        raw = str(msg.data or "").strip()
        if not raw:
            return

        try:
            payload = json.loads(raw)
        except Exception as exc:
            rospy.logwarn("Invalid debug state override payload: %s", exc)
            return

        if not isinstance(payload, dict):
            rospy.logwarn("Debug state override payload must be a JSON object")
            return

        clear_customer_context = bool(payload.get("clear_customer_context", False))
        if clear_customer_context:
            self._clear_active_customer_context()

        folder = payload.get("active_customer_folder")
        if isinstance(folder, str) and folder.strip():
            folder = folder.strip()
            if os.path.isdir(folder):
                self._set_active_customer_context(folder=folder)
            else:
                rospy.logwarn("Debug state override folder does not exist: %s", folder)

        customer_id = payload.get("active_customer_id")
        if isinstance(customer_id, str) and customer_id.strip():
            self._set_active_customer_context(customer_id=customer_id.strip())

        return_state = payload.get("return_navigation_state")
        if isinstance(return_state, str) and return_state.strip():
            self.return_navigation_state = return_state.strip().upper()

        state = payload.get("active_customer_state")
        if isinstance(state, str) and state.strip():
            self._set_serving_customer_state(
                state.strip().upper(),
                {
                    "source": "debug_state_override_topic",
                    "return_navigation_state": self.return_navigation_state,
                },
            )

    def _refresh_mhrc_navigation_gate_config(self):
        self.mhrc_nav_state_gating_enabled = rospy.get_param(
            "~mhrc_nav_state_gating_enabled",
            self.mhrc_nav_state_gating_enabled,
        )
        self.mhrc_nav_force_accept = rospy.get_param(
            "~mhrc_nav_force_accept",
            self.mhrc_nav_force_accept,
        )
        self.mhrc_nav_allow_locked = rospy.get_param(
            "~mhrc_nav_allow_locked",
            self.mhrc_nav_allow_locked,
        )
        self.mhrc_nav_request_ttl = rospy.get_param(
            "~mhrc_nav_request_ttl",
            self.mhrc_nav_request_ttl,
        )
        self.mhrc_nav_debug_log_gating_decisions = rospy.get_param(
            "~mhrc_nav_debug_log_gating_decisions",
            self.mhrc_nav_debug_log_gating_decisions,
        )

    def _build_navigate_ack_payload(
        self,
        request_id,
        success,
        error_code="",
        message="",
        recommendation="",
        extra=None,
    ):
        customer_no = self._refresh_active_customer_no()
        payload = {
            "request_id": str(request_id or ""),
            "success": bool(success),
            "error_code": str(error_code or ""),
            "message": str(message or ""),
            "recommendation": str(recommendation or ""),
            "timestamp": rospy.Time.now().to_sec(),
            "customer_no": str(customer_no or ""),
            "active_customer_state": str(self.active_customer_state or "IDLE").upper(),
            "return_navigation_state": str(self.return_navigation_state or "IDLE").upper(),
        }
        if isinstance(extra, dict):
            payload.update(extra)
        return payload

    def _publish_navigate_ack(
        self,
        request_id,
        success,
        error_code="",
        message="",
        recommendation="",
        extra=None,
    ):
        if self.navigate_ack_pub is None:
            return
        payload = self._build_navigate_ack_payload(
            request_id=request_id,
            success=success,
            error_code=error_code,
            message=message,
            recommendation=recommendation,
            extra=extra,
        )
        try:
            self.navigate_ack_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        except Exception as exc:
            rospy.logwarn("Failed to publish navigate ACK: %s", exc)

    def _evaluate_mhrc_navigation_gate(self):
        self._refresh_mhrc_navigation_gate_config()
        active_state = str(self.active_customer_state or "IDLE").strip().upper()
        return_state = str(self.return_navigation_state or "IDLE").strip().upper()

        # Hard isolation rule: while Task5 is inside service workflow states,
        # MHRC navigate requests must not take over navigation writes.
        workflow_states = (
            "LOCKED",
            "TRACKING",
            "PAUSED_ORDERING",
            "ORDERED",
            "RETURNING",
            "TABLE_APPROACH",
            "AT_TABLE_FRONT",
        )

        if return_state != "IDLE":
            return {
                "allowed": False,
                "error_code": "busy_returning_navigation",
                "message": f"Task5 return navigation active ({return_state})",
                "recommendation": "wait_and_retry",
            }

        if active_state in workflow_states:
            return {
                "allowed": False,
                "error_code": "busy_service_workflow",
                "message": f"Task5 active workflow state {active_state}",
                "recommendation": "replan_wait_or_speak",
            }

        if self.mhrc_nav_force_accept:
            return {
                "allowed": True,
                "error_code": "",
                "message": "force_accept_enabled",
                "recommendation": "",
            }

        if not self.mhrc_nav_state_gating_enabled:
            return {
                "allowed": True,
                "error_code": "",
                "message": "state_gating_disabled",
                "recommendation": "",
            }

        if active_state == "IDLE":
            return {
                "allowed": True,
                "error_code": "",
                "message": "allowed_in_idle",
                "recommendation": "",
            }

        return {
            "allowed": False,
            "error_code": "stale_state",
            "message": f"Unsupported or unknown Task5 state {active_state}",
            "recommendation": "wait_and_retry",
        }

    def navigate_request_callback(self, msg):
        raw = str(msg.data or "").strip()
        request_id = ""

        if not raw:
            self._publish_navigate_ack(
                request_id="",
                success=False,
                error_code="invalid_request",
                message="empty navigate request payload",
                recommendation="fix_payload",
            )
            return

        try:
            payload = json.loads(raw)
        except Exception as exc:
            self._publish_navigate_ack(
                request_id="",
                success=False,
                error_code="invalid_request",
                message=f"navigate request is not valid JSON: {exc}",
                recommendation="fix_payload",
            )
            return

        if not isinstance(payload, dict):
            self._publish_navigate_ack(
                request_id="",
                success=False,
                error_code="invalid_request",
                message="navigate request payload must be a JSON object",
                recommendation="fix_payload",
            )
            return

        request_id = str(payload.get("request_id") or "").strip()
        action = str(payload.get("action") or "navigate").strip().lower()
        if action != "navigate":
            self._publish_navigate_ack(
                request_id=request_id,
                success=False,
                error_code="invalid_action",
                message=f"unsupported action {action}",
                recommendation="use_navigate_action",
            )
            return

        now_sec = rospy.Time.now().to_sec()
        ttl = max(1.0, float(self.mhrc_nav_request_ttl))
        try:
            request_ts = float(payload.get("timestamp", now_sec))
        except Exception:
            request_ts = now_sec
        if now_sec - request_ts > ttl:
            self._publish_navigate_ack(
                request_id=request_id,
                success=False,
                error_code="request_timeout",
                message=f"navigate request expired (>{ttl:.1f}s)",
                recommendation="retry_with_new_request",
            )
            return

        gate = self._evaluate_mhrc_navigation_gate()
        if self.mhrc_nav_debug_log_gating_decisions:
            rospy.loginfo(
                "MHRC navigate gate decision: allowed=%s active_state=%s return_state=%s",
                gate.get("allowed", False),
                str(self.active_customer_state or "IDLE").upper(),
                str(self.return_navigation_state or "IDLE").upper(),
            )

        if not gate.get("allowed", False):
            self._publish_navigate_ack(
                request_id=request_id,
                success=False,
                error_code=gate.get("error_code", "task5_fsm_active"),
                message=gate.get("message", "Task5 state gate rejected navigate request"),
                recommendation=gate.get("recommendation", "replan_wait_or_speak"),
            )
            return

        pose = payload.get("pose")
        if not isinstance(pose, dict):
            self._publish_navigate_ack(
                request_id=request_id,
                success=False,
                error_code="invalid_target",
                message="pose field missing or not object",
                recommendation="fix_target_pose",
            )
            return

        try:
            gx = float(pose.get("x"))
            gy = float(pose.get("y"))
            yaw = float(pose.get("yaw", 0.0))
        except Exception:
            self._publish_navigate_ack(
                request_id=request_id,
                success=False,
                error_code="invalid_target",
                message="pose requires numeric x/y[/yaw]",
                recommendation="fix_target_pose",
            )
            return

        if not (math.isfinite(gx) and math.isfinite(gy) and math.isfinite(yaw)):
            self._publish_navigate_ack(
                request_id=request_id,
                success=False,
                error_code="invalid_target",
                message="pose contains non-finite values",
                recommendation="fix_target_pose",
            )
            return

        frame_id = str(payload.get("frame_id") or self.global_frame).strip()
        if frame_id and frame_id != self.global_frame:
            self._publish_navigate_ack(
                request_id=request_id,
                success=False,
                error_code="invalid_frame",
                message=f"unsupported frame_id {frame_id}, expected {self.global_frame}",
                recommendation="use_global_frame",
            )
            return

        try:
            self._publish_manual_nav_goal(gx, gy, yaw, "mhrc-navigate-request")
        except Exception as exc:
            self._publish_navigate_ack(
                request_id=request_id,
                success=False,
                error_code="goal_publish_failed",
                message=str(exc),
                recommendation="retry_with_new_request",
            )
            return

        self._publish_navigate_ack(
            request_id=request_id,
            success=True,
            error_code="",
            message="navigate request accepted",
            recommendation="",
            extra={
                "accepted_target": {
                    "x": gx,
                    "y": gy,
                    "yaw": yaw,
                    "frame_id": self.global_frame,
                }
            },
        )

    def _read_customer_service_state(self, folder):
        state_file = os.path.join(folder, "customer_service_state.json")
        if not os.path.isfile(state_file):
            return "IDLE"

        try:
            with open(state_file, "r", encoding="utf-8") as fp:
                state_payload = json.load(fp)
            return str(state_payload.get("state", "IDLE")).upper()
        except Exception:
            return "IDLE"

    def _select_customer_folder_for_service(self, exclude_folder="", min_priority=0):
        root = self.customer_data_root
        if not root or not os.path.isdir(root):
            return ""

        priority = {
            "RETURNING": 6,
            "ORDERED": 5,
            "PAUSED_ORDERING": 4,
            "TRACKING": 3,
            "LOCKED": 2,
            "IDLE": 1,
        }

        exclude_real = ""
        if exclude_folder:
            try:
                exclude_real = os.path.realpath(str(exclude_folder))
            except Exception:
                exclude_real = ""

        best_folder = ""
        best_score = -1
        best_mtime = -1.0

        for name in os.listdir(root):
            folder = os.path.join(root, name)
            if not os.path.isdir(folder):
                continue

            if exclude_real:
                try:
                    if os.path.realpath(folder) == exclude_real:
                        continue
                except Exception:
                    pass

            state = self._read_customer_service_state(folder)

            score = priority.get(state, 0)
            if score < int(min_priority):
                continue

            mtime = os.path.getmtime(folder)
            if score > best_score or (score == best_score and mtime > best_mtime):
                best_score = score
                best_mtime = mtime
                best_folder = folder

        return best_folder

    def _resolve_active_customer_folder(self):
        if self.active_customer_folder and os.path.isdir(self.active_customer_folder):
            self._refresh_active_customer_no()
            return self.active_customer_folder

        selected = self._select_customer_folder_for_service()
        if selected:
            self._set_active_customer_context(folder=selected)
        return self.active_customer_folder

    def _resolve_customer_scoped_path(self, default_path):
        folder = self._resolve_active_customer_folder()
        if folder:
            try:
                os.makedirs(folder, exist_ok=True)
                return os.path.join(folder, os.path.basename(default_path))
            except Exception:
                pass
        return default_path

    def _resolve_customer_food_order_path(self):
        folder = self._resolve_active_customer_folder()
        if not folder:
            return ""

        folder_no = self._customer_no_from_folder(folder)
        active_no = self._refresh_active_customer_no()
        if active_no and folder_no and active_no != folder_no:
            rospy.logwarn(
                "Active customer No. (%s) differs from folder name (%s), forcing folder tag.",
                active_no,
                folder_no,
            )
            self.active_customer_id = folder_no
            self.active_customer_no = folder_no

        if not self.active_customer_no:
            rospy.logwarn("No active customer No. tag, skip resolving food order path.")
            return ""

        file_name = os.path.basename(str(self.food_order_json_file).strip())
        if not file_name:
            file_name = "food_orders.json"

        try:
            os.makedirs(folder, exist_ok=True)
        except Exception as exc:
            rospy.logwarn("Failed to prepare customer folder for food orders: %s", exc)
            return ""

        return os.path.join(folder, file_name)

    def _remove_legacy_food_order_file(self, active_path):
        legacy_path = str(self.food_order_json_file).strip()
        if not legacy_path:
            return

        try:
            if os.path.realpath(legacy_path) == os.path.realpath(active_path):
                return
        except Exception:
            return

        if not os.path.isfile(legacy_path):
            return

        try:
            os.remove(legacy_path)
            rospy.loginfo("Removed legacy global food order JSON: %s", legacy_path)
        except Exception as exc:
            rospy.logwarn("Failed to remove legacy global food order JSON (%s): %s", legacy_path, exc)

    def _clear_food_orders_cache(self):
        with self.food_order_lock:
            self.food_order_history = []
            self.current_needed_foods = []
            self.current_needed_foods_qty = {}

    def _reset_return_navigation_context(self):
        self.return_to_anchor_active = False
        self.return_anchor_goal = None
        self.return_navigation_state = "IDLE"
        self.return_table_goal = None
        self.return_table_plan_attempt_time = rospy.Time(0)
        self.table_front_arrive_time = rospy.Time(0)
        self.table_food_check_done = False
        self.goal_publish_paused = False
        self.goal_reach_since = None
        self.paused_person_x = None
        self.paused_person_y = None
        self.paused_person_base_x = None
        self.paused_person_base_y = None
        self.paused_person_heading = None

    def _complete_current_service_cycle_and_prepare_next(self, reason):
        current_folder = self._resolve_active_customer_folder()
        current_no = self._refresh_active_customer_no()

        if current_folder and os.path.isdir(current_folder):
            self._set_serving_customer_state(
                "IDLE",
                {
                    "reason": str(reason or "service_cycle_completed"),
                    "completed_customer_no": str(current_no or ""),
                    "completed_customer_folder": current_folder,
                },
            )

        self._reset_return_navigation_context()

        self._clear_active_customer_context()
        self._clear_food_orders_cache()
        rospy.loginfo(
            "Service cycle finished for customer No.=%s; switched to IDLE and waiting for next customer.",
            str(current_no or "unknown"),
        )

    def _load_customer_person_global_from_folder(self):
        candidates = []
        folder = self._resolve_active_customer_folder()
        if folder:
            candidates.append(os.path.join(folder, os.path.basename(self.serving_target_snapshot_json_file)))
            candidates.append(os.path.join(folder, "serving_target_snapshot.json"))
        candidates.append(self.serving_target_snapshot_json_file)

        for path in candidates:
            if not path or not os.path.isfile(path):
                continue
            try:
                with open(path, "r", encoding="utf-8") as fp:
                    payload = json.load(fp)
                person = payload.get("person_global", {})
                gx = float(person.get("x"))
                gy = float(person.get("y"))
                return gx, gy
            except Exception:
                continue

        if self.paused_person_x is not None and self.paused_person_y is not None:
            return float(self.paused_person_x), float(self.paused_person_y)
        return None

    def _identify_nearby_counter_or_table(self, anchor_x, anchor_y):
        if self.grid is None:
            return None

        center = self._world_to_map(anchor_x, anchor_y)
        if center is None:
            return None

        info = self.grid.info
        width = info.width
        height = info.height
        res = max(info.resolution, 1e-3)
        data = self.grid.data

        radius_cells = max(1, int(self.return_table_search_radius / res))
        cx, cy = center
        min_x = max(0, cx - radius_cells)
        max_x = min(width - 1, cx + radius_cells)
        min_y = max(0, cy - radius_cells)
        max_y = min(height - 1, cy + radius_cells)

        def _cell_to_world(mx, my):
            wx = info.origin.position.x + (mx + 0.5) * info.resolution
            wy = info.origin.position.y + (my + 0.5) * info.resolution
            return wx, wy

        visited = set()
        components = []

        for my in range(min_y, max_y + 1):
            for mx in range(min_x, max_x + 1):
                key = (mx, my)
                if key in visited:
                    continue

                idx = my * width + mx
                val = data[idx]
                if val < self.occupancy_threshold:
                    continue

                q = deque([key])
                visited.add(key)
                cells = []

                while q:
                    ux, uy = q.popleft()
                    cells.append((ux, uy))

                    for ny in range(uy - 1, uy + 2):
                        for nx in range(ux - 1, ux + 2):
                            if nx < min_x or ny < min_y or nx > max_x or ny > max_y:
                                continue
                            nkey = (nx, ny)
                            if nkey in visited:
                                continue
                            nidx = ny * width + nx
                            nval = data[nidx]
                            if nval >= self.occupancy_threshold:
                                visited.add(nkey)
                                q.append(nkey)

                if len(cells) < int(self.return_table_min_component_cells):
                    continue

                pts = np.array([_cell_to_world(mx0, my0) for (mx0, my0) in cells], dtype=np.float64)
                if pts.shape[0] < 3:
                    continue

                centroid = pts.mean(axis=0)
                cov = np.cov(pts[:, 0], pts[:, 1])
                if np.ndim(cov) != 2:
                    continue

                eigvals, eigvecs = np.linalg.eigh(cov)
                order = np.argsort(eigvals)
                minor_var = max(float(eigvals[order[0]]), 1e-6)
                major_var = max(float(eigvals[order[1]]), minor_var)
                major_vec = eigvecs[:, order[1]]

                major_len = 4.0 * math.sqrt(major_var)
                minor_len = 4.0 * math.sqrt(minor_var)
                elongation = major_len / max(minor_len, 1e-3)
                anchor_dist = math.hypot(float(centroid[0]) - anchor_x, float(centroid[1]) - anchor_y)

                components.append(
                    {
                        "centroid": (float(centroid[0]), float(centroid[1])),
                        "major_vec": (float(major_vec[0]), float(major_vec[1])),
                        "major_len": float(major_len),
                        "minor_len": float(minor_len),
                        "elongation": float(elongation),
                        "anchor_dist": float(anchor_dist),
                        "cells": len(cells),
                    }
                )

        if not components:
            return None

        elongated = [c for c in components if c["elongation"] >= float(self.return_table_min_elongation)]
        pool = elongated if elongated else components
        pool.sort(key=lambda c: (c["anchor_dist"], -c["elongation"], -c["cells"]))
        return pool[0]

    def _plan_table_front_goal(self, rx, ry):
        if self.return_anchor_goal is None:
            return None

        anchor_x, anchor_y, _ = self.return_anchor_goal
        component = self._identify_nearby_counter_or_table(anchor_x, anchor_y)
        if component is None:
            return None

        customer = self._load_customer_person_global_from_folder()
        if customer is None:
            customer = (anchor_x, anchor_y)

        cx, cy = component["centroid"]
        ux, uy = component["major_vec"]
        nx, ny = -uy, ux

        vx = customer[0] - cx
        vy = customer[1] - cy
        if vx * nx + vy * ny < 0.0:
            nx, ny = -nx, -ny

        base_offset = float(self.return_table_stop_offset) + 0.5 * float(component["minor_len"])
        best = None

        for k in range(8):
            dist = base_offset + 0.12 * float(k)
            sx = cx + nx * dist
            sy = cy + ny * dist
            if not self._is_pose_collision_free(sx, sy):
                continue
            if not self._is_segment_collision_free(rx, ry, sx, sy):
                continue

            theta = math.atan2(cy - sy, cx - sx)
            best = (sx, sy, theta)
            break

        if best is None:
            return None

        sx, sy, theta = best
        return {
            "goal": (sx, sy, theta),
            "component": component,
            "customer": customer,
        }

    def _world_to_map(self, x, y):
        if self.grid is None:
            return None
        info = self.grid.info
        ox = info.origin.position.x
        oy = info.origin.position.y
        mx = int((x - ox) / info.resolution)
        my = int((y - oy) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return mx, my

    def _cell_occupied(self, mx, my):
        idx = my * self.grid.info.width + mx
        val = self.grid.data[idx]
        if val < 0:
            return self.unknown_is_occupied
        return val >= self.occupancy_threshold

    def _is_pose_collision_free(self, x, y):
        if self.grid is None:
            return True

        center = self._world_to_map(x, y)
        if center is None:
            return False

        info = self.grid.info
        rad_cells = max(1, int(self.robot_radius / info.resolution))
        cx, cy = center

        for dx in range(-rad_cells, rad_cells + 1):
            for dy in range(-rad_cells, rad_cells + 1):
                if dx * dx + dy * dy > rad_cells * rad_cells:
                    continue
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or ny < 0 or nx >= info.width or ny >= info.height:
                    return False
                if self._cell_occupied(nx, ny):
                    return False
        return True

    def _is_segment_collision_free(self, x0, y0, x1, y1):
        if self.grid is None:
            return True

        dist = math.hypot(x1 - x0, y1 - y0)
        if dist < 1e-4:
            return self._is_pose_collision_free(x0, y0)

        steps = max(2, int(dist / self.path_check_step))
        for i in range(steps + 1):
            t = float(i) / float(steps)
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            if not self._is_pose_collision_free(x, y):
                return False
        return True

    def _estimate_person_front_heading(self, px, py, rx, ry, stamp):
        # Prefer person motion direction. Fallback to heading from person toward robot.
        if self.last_person_x is not None and self.last_person_y is not None and self.last_person_stamp is not None:
            dt = (stamp - self.last_person_stamp).to_sec()
            if dt > 1e-3:
                vx = (px - self.last_person_x) / dt
                vy = (py - self.last_person_y) / dt
                speed = math.hypot(vx, vy)
                if speed >= self.min_heading_speed:
                    self.person_heading = math.atan2(vy, vx)

        self.last_person_x = px
        self.last_person_y = py
        self.last_person_stamp = stamp

        if self.person_heading is not None:
            return self.person_heading

        return math.atan2(ry - py, rx - px)

    def _fallback_goal(self, px, py, rx, ry):
        # Without valid candidates, use simple follow target along robot->person ray.
        theta = math.atan2(py - ry, px - rx)
        gx = px - self.follow_distance * math.cos(theta)
        gy = py - self.follow_distance * math.sin(theta)
        return gx, gy, theta, None

    def _score_candidate(self, cx, cy, r, off, rx, ry, span):
        dist_pen = abs(r - self.follow_distance)
        angle_pen = abs(off) / max(span, 1e-3)
        robot_pen = math.hypot(cx - rx, cy - ry)
        return (
            self.weight_distance * dist_pen
            + self.weight_angle * angle_pen
            + self.weight_robot * robot_pen
        )

    def _select_goal(self, px, py, rx, ry, stamp):
        front_heading = self._estimate_person_front_heading(px, py, rx, ry, stamp)
        span = math.radians(self.front_angle_span_deg) * 0.5

        if self.radius_samples <= 1:
            radii = [self.follow_distance]
        else:
            radii = []
            for i in range(self.radius_samples):
                t = float(i) / float(self.radius_samples - 1)
                radii.append(self.min_candidate_radius + t * (self.max_candidate_radius - self.min_candidate_radius))

        if self.angle_samples <= 1:
            offsets = [0.0]
        else:
            offsets = []
            for i in range(self.angle_samples):
                t = float(i) / float(self.angle_samples - 1)
                offsets.append(-span + t * (2.0 * span))

        best = None

        for r in radii:
            for off in offsets:
                ang = front_heading + off
                cx = px + r * math.cos(ang)
                cy = py + r * math.sin(ang)

                if not self._is_pose_collision_free(cx, cy):
                    continue
                if not self._is_segment_collision_free(rx, ry, cx, cy):
                    continue
                if not self._is_segment_collision_free(px, py, cx, cy):
                    continue

                score = self._score_candidate(cx, cy, r, off, rx, ry, span)

                if best is None or score < best[0]:
                    best = (score, cx, cy, ang)

        if best is None:
            gx, gy, theta, score = self._fallback_goal(px, py, rx, ry)
            return gx, gy, theta, score, front_heading

        return best[1], best[2], best[3], best[0], front_heading

    def _evaluate_current_goal(self, px, py, rx, ry, front_heading):
        if self.last_goal_x is None or self.last_goal_y is None:
            return None

        gx = self.last_goal_x
        gy = self.last_goal_y

        if not self._is_pose_collision_free(gx, gy):
            return None
        if not self._is_segment_collision_free(rx, ry, gx, gy):
            return None
        if not self._is_segment_collision_free(px, py, gx, gy):
            return None

        vec_x = gx - px
        vec_y = gy - py
        r = math.hypot(vec_x, vec_y)
        if r < 1e-4:
            return None

        ang = math.atan2(vec_y, vec_x)
        off = math.atan2(math.sin(ang - front_heading), math.cos(ang - front_heading))
        span = math.radians(self.front_angle_span_deg) * 0.5
        score = self._score_candidate(gx, gy, r, off, rx, ry, span)
        return gx, gy, ang, score

    def _publish_goal(self, gx, gy, theta):
        # Ensure nav command is not overridden by stale gaze-track commands.
        self._stop_gaze_tracking_cmd()

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = self.global_frame
        pose_goal.header.stamp = rospy.Time.now()
        pose_goal.pose.position.x = gx
        pose_goal.pose.position.y = gy
        pose_goal.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)
        pose_goal.pose.orientation.x = qx
        pose_goal.pose.orientation.y = qy
        pose_goal.pose.orientation.z = qz
        pose_goal.pose.orientation.w = qw

        self.goal_pub.publish(pose_goal)

        if self.publish_waypoint_direct:
            waypoint = PointStamped()
            waypoint.header = pose_goal.header
            waypoint.point.x = gx
            waypoint.point.y = gy
            waypoint.point.z = 0.0
            self.waypoint_pub.publish(waypoint)
            self.goal_point_pub.publish(waypoint)

        self.last_goal_x = gx
        self.last_goal_y = gy
        self.last_goal_theta = theta
        self.last_goal_score = None
        self.last_publish_time = rospy.Time.now()
        self.goal_reach_since = None
        rospy.loginfo_throttle(0.5, "Published follow goal: (%.2f, %.2f)", gx, gy)

    def _stop_gaze_tracking_cmd(self):
        if not self.gaze_cmd_active:
            return
        self.search_cmd_pub.publish(Twist())
        self.gaze_cmd_active = False

    def _build_food_patterns(self, alias_map):
        patterns = []
        if not isinstance(alias_map, dict):
            return patterns

        for canonical, aliases in alias_map.items():
            key = str(canonical).strip().lower()
            if not key:
                continue
            names = [key]
            if isinstance(aliases, list):
                for item in aliases:
                    token = str(item).strip().lower()
                    if token:
                        names.append(token)
            elif aliases is not None:
                token = str(aliases).strip().lower()
                if token:
                    names.append(token)

            seen = set()
            for name in names:
                if name in seen:
                    continue
                seen.add(name)
                patterns.append(
                    (
                        key,
                        name,
                        re.compile(r"\\b" + re.escape(name) + r"\\b"),
                    )
                )

        # Match longer aliases first (e.g. "fried rice" before "rice").
        patterns.sort(key=lambda x: len(x[1]), reverse=True)
        return patterns

    def _build_food_alias_lookup(self, alias_map):
        lookup = {}
        if not isinstance(alias_map, dict):
            return lookup

        for canonical, aliases in alias_map.items():
            canonical_key = self._normalize_food_name(canonical)
            if not canonical_key:
                continue
            lookup[canonical_key] = canonical_key

            alias_list = []
            if isinstance(aliases, list):
                alias_list = aliases
            elif aliases is not None:
                alias_list = [aliases]

            for alias in alias_list:
                alias_key = self._normalize_food_name(alias)
                if alias_key:
                    lookup[alias_key] = canonical_key

        return lookup

    def _build_food_alias_pairs(self, alias_map):
        pairs = []
        if not isinstance(alias_map, dict):
            return pairs

        seen = set()
        for canonical, aliases in alias_map.items():
            canonical_key = self._normalize_food_name(canonical)
            if not canonical_key:
                continue

            alias_items = [canonical_key]
            if isinstance(aliases, list):
                alias_items.extend(aliases)
            elif aliases is not None:
                alias_items.append(aliases)

            for alias in alias_items:
                alias_key = self._normalize_food_name(alias)
                if not alias_key:
                    continue
                pair = (alias_key, canonical_key)
                if pair in seen:
                    continue
                seen.add(pair)
                pairs.append(pair)

        return pairs

    def _normalize_food_name(self, name):
        if name is None:
            return ""
        text = str(name).strip().lower()
        text = re.sub(r"[^a-z0-9_\\s]", " ", text)
        text = re.sub(r"\\s+", " ", text).strip()
        if text in ("none", "null"):
            return ""
        return text

    def _canonicalize_food_name(self, name):
        key = self._normalize_food_name(name)
        if not key:
            return None
        if key in self.food_alias_lookup:
            return self.food_alias_lookup[key]
        return key.replace(" ", "_")

    def _best_lexical_food_alias_match(self, key):
        if not key:
            return None

        best_name = None
        best_score = 0.0
        threshold = float(self.table_food_lexical_threshold)

        for alias_key, canonical in self.food_alias_pairs:
            score = difflib.SequenceMatcher(None, key, alias_key).ratio()
            if key in alias_key or alias_key in key:
                score = max(score, 0.90)
            if score > best_score:
                best_score = score
                best_name = canonical

        if best_name and best_score >= threshold:
            return best_name
        return None

    def _build_table_food_fuzzy_prompt(self, detected_name):
        known_foods = ", ".join(sorted(self.food_catalog_set))
        return (
            "You are a food-label matcher for a restaurant robot. "
            "Map one detected object label to a canonical ordered-food name. "
            "Return strict JSON only with schema: {\"name\":\"canonical_or_empty\"}. "
            "If no match, return {\"name\":\"\"}. "
            "Canonical names: "
            + known_foods
            + ". "
            + "Detected label: "
            + str(detected_name)
        )

    def _run_table_food_fuzzy_semantic_backend(self, prompt):
        backend = str(self.table_food_fuzzy_backend).strip().lower()
        if backend in (
            "",
            "reuse_order_backend",
            "same_as_order",
            "same",
            "order",
            "follow_order",
        ):
            backend = str(self.food_semantic_backend).strip().lower()

        raw = ""
        if backend in ("auto", ""):
            raw = self._run_food_semantic_command(prompt)
            if not raw:
                raw = self._run_food_semantic_ollama(prompt, prompt_override=prompt)
            if not raw:
                raw = self._run_food_semantic_transformers(prompt, prompt_override=prompt)
        elif backend in ("command", "cmd", "local-command"):
            raw = self._run_food_semantic_command(prompt)
        elif backend in ("ollama", "http", "remote"):
            raw = self._run_food_semantic_ollama(prompt, prompt_override=prompt)
        elif backend in ("transformers", "hf", "huggingface"):
            raw = self._run_food_semantic_transformers(prompt, prompt_override=prompt)
        elif backend in ("mhrc", "mhrc-planning", "mhrc_planning"):
            raw = self._run_food_semantic_mhrc(prompt, prompt_override=prompt)
        elif backend in ("off", "none", "disabled", "false", "0"):
            raw = ""
        else:
            rospy.logwarn_throttle(5.0, "Unsupported table food fuzzy backend: %s", backend)

        return str(raw or "").strip()

    def _parse_table_food_fuzzy_semantic_result(self, raw_text):
        if not raw_text:
            return None

        candidates = []
        blob = self._extract_json_blob(raw_text)
        if blob:
            try:
                payload = json.loads(blob)
                if isinstance(payload, dict):
                    for key in ("name", "canonical", "food", "item", "match"):
                        value = payload.get(key)
                        if isinstance(value, str):
                            candidates.append(value)
                elif isinstance(payload, list):
                    for item in payload:
                        if isinstance(item, str):
                            candidates.append(item)
                        elif isinstance(item, dict):
                            for key in ("name", "canonical", "food", "item", "match"):
                                value = item.get(key)
                                if isinstance(value, str):
                                    candidates.append(value)
            except Exception:
                pass

        if not candidates:
            candidates.append(raw_text)

        for name in candidates:
            canonical = self._canonicalize_food_name(name)
            if canonical and canonical in self.food_catalog_set:
                return canonical

        return None

    def _best_model_food_alias_match(self, key):
        if not self.table_food_use_fuzzy_model:
            return None

        with self.table_food_fuzzy_lock:
            cached = self.table_food_fuzzy_cache.get(key)
        if cached is not None:
            return cached or None

        prompt = self._build_table_food_fuzzy_prompt(key)
        raw = self._run_table_food_fuzzy_semantic_backend(prompt)
        matched = self._parse_table_food_fuzzy_semantic_result(raw)

        with self.table_food_fuzzy_lock:
            self.table_food_fuzzy_cache[key] = matched or ""

        return matched

    def _canonicalize_detected_food_name(self, name):
        key = self._normalize_food_name(name)
        if not key:
            return None

        direct = self.food_alias_lookup.get(key)
        if direct:
            return direct

        lexical = self._best_lexical_food_alias_match(key)
        if lexical:
            return lexical

        return self._best_model_food_alias_match(key)

    def _coerce_positive_qty(self, value):
        if isinstance(value, bool):
            return 1
        if isinstance(value, (int, float)):
            qty = int(value)
            return qty if qty > 0 else 1

        text = str(value).strip().lower()
        if not text:
            return 1
        if text in self.food_qty_word_map:
            return int(self.food_qty_word_map[text])
        if text.startswith("x") and text[1:].isdigit():
            qty = int(text[1:])
            return qty if qty > 0 else 1
        if text.isdigit():
            qty = int(text)
            return qty if qty > 0 else 1
        return 1

    def _build_food_semantic_prompt(self, text):
        known_foods = ", ".join(sorted(self.food_aliases.keys()))
        return (
            "You are an English food-order information extractor. "
            "Input sentence is in English. "
            "Return strict JSON only, with no extra text. "
            "Schema: {\"items\":[{\"name\":\"food_name\",\"qty\":1}]}. "
            "qty must be integer >= 1. If quantity is missing, use 1. "
            "Do not invent items that are not mentioned. "
            "Prefer canonical names from: "
            + known_foods
            + ". "
            + "Example: 'I want two cokes and a burger' -> "
            + "{\"items\":[{\"name\":\"coke\",\"qty\":2},{\"name\":\"burger\",\"qty\":1}]}. "
            + "Sentence: "
            + text
        )

    def _extract_json_blob(self, text):
        if not text:
            return ""

        raw = text.strip()
        if raw.startswith("```"):
            raw = re.sub(r"^```[a-zA-Z0-9_-]*\\s*", "", raw)
            raw = re.sub(r"\\s*```$", "", raw)
            raw = raw.strip()

        candidates = [raw]
        obj_start = raw.find("{")
        obj_end = raw.rfind("}")
        if obj_start >= 0 and obj_end > obj_start:
            candidates.append(raw[obj_start : obj_end + 1])

        arr_start = raw.find("[")
        arr_end = raw.rfind("]")
        if arr_start >= 0 and arr_end > arr_start:
            candidates.append(raw[arr_start : arr_end + 1])

        for candidate in candidates:
            try:
                json.loads(candidate)
                return candidate
            except Exception:
                continue
        return ""

    def _parse_semantic_food_json(self, raw_text):
        blob = self._extract_json_blob(raw_text)
        if not blob:
            return {}, []

        try:
            payload = json.loads(blob)
        except Exception:
            return {}, []

        if isinstance(payload, dict):
            items = (
                payload.get("items")
                or payload.get("foods")
                or payload.get("orders")
                or payload.get("result")
                or []
            )
        elif isinstance(payload, list):
            items = payload
        else:
            items = []

        summary = {}
        mentions = []
        for item in items:
            raw_name = None
            raw_qty = 1
            if isinstance(item, str):
                raw_name = item
            elif isinstance(item, dict):
                raw_name = (
                    item.get("name")
                    or item.get("food")
                    or item.get("item")
                    or item.get("dish")
                )
                raw_qty = (
                    item.get("qty")
                    or item.get("quantity")
                    or item.get("count")
                    or 1
                )

            canonical = self._canonicalize_food_name(raw_name)
            if not canonical:
                continue

            qty = self._coerce_positive_qty(raw_qty)
            summary[canonical] = summary.get(canonical, 0) + qty
            mentions.append(
                {
                    "name": canonical,
                    "alias": self._normalize_food_name(raw_name),
                    "qty": qty,
                    "source": "semantic",
                }
            )

        return summary, mentions

    def _run_food_semantic_command(self, text):
        template = self.food_semantic_command
        if not template:
            return ""

        timeout = max(0.5, float(self.food_semantic_timeout))
        use_shell = bool(self.food_semantic_command_use_shell)

        try:
            if "{text}" in template:
                command = template.replace("{text}", text)
                if use_shell:
                    proc = subprocess.run(
                        command,
                        shell=True,
                        capture_output=True,
                        text=True,
                        timeout=timeout,
                    )
                else:
                    proc = subprocess.run(
                        shlex.split(command),
                        capture_output=True,
                        text=True,
                        timeout=timeout,
                    )
            else:
                if use_shell:
                    proc = subprocess.run(
                        template,
                        shell=True,
                        input=text,
                        capture_output=True,
                        text=True,
                        timeout=timeout,
                    )
                else:
                    proc = subprocess.run(
                        shlex.split(template),
                        input=text,
                        capture_output=True,
                        text=True,
                        timeout=timeout,
                    )

            if proc.returncode != 0:
                rospy.logwarn_throttle(
                    3.0,
                    "Food semantic command returned code %d",
                    proc.returncode,
                )
            return (proc.stdout or "").strip()
        except Exception as exc:
            rospy.logwarn_throttle(3.0, "Food semantic command failed: %s", exc)
            return ""

    def _update_mhrc_stats(self, key=None, latency_ms=None):
        with self.food_semantic_mhrc_stats_lock:
            if key:
                self.food_semantic_mhrc_stats[key] = self.food_semantic_mhrc_stats.get(key, 0) + 1
            if latency_ms is not None and latency_ms >= 0.0:
                self.food_semantic_mhrc_stats["latency_ms_sum"] += float(latency_ms)
                self.food_semantic_mhrc_stats["latency_ms_max"] = max(
                    self.food_semantic_mhrc_stats["latency_ms_max"],
                    float(latency_ms),
                )

    def _log_mhrc_stats_if_needed(self, force=False):
        interval = max(1.0, float(self.food_semantic_mhrc_stats_log_interval))
        now_mono = time.monotonic()

        with self.food_semantic_mhrc_stats_lock:
            if not force and (now_mono - self.food_semantic_mhrc_last_stats_log) < interval:
                return
            self.food_semantic_mhrc_last_stats_log = now_mono
            stats = dict(self.food_semantic_mhrc_stats)
            fuse_open_until = float(self.food_semantic_mhrc_fuse_open_until)

        avg_latency = 0.0
        if stats["success"] > 0:
            avg_latency = stats["latency_ms_sum"] / float(stats["success"])

        fuse_remaining = max(0.0, fuse_open_until - now_mono)
        rospy.loginfo(
            "MHRC semantic stats: submitted=%d success=%d failed=%d timeout=%d "
            "fuse_blocked=%d fuse_opened=%d fallback=%d avg_latency_ms=%.1f max_latency_ms=%.1f "
            "fuse_remaining_s=%.1f",
            int(stats["submitted"]),
            int(stats["success"]),
            int(stats["failed"]),
            int(stats["timeout"]),
            int(stats["fuse_blocked"]),
            int(stats["fuse_opened"]),
            int(stats["fallback_used"]),
            float(avg_latency),
            float(stats["latency_ms_max"]),
            float(fuse_remaining),
        )

    def _is_mhrc_fuse_open(self):
        now_mono = time.monotonic()
        with self.food_semantic_mhrc_stats_lock:
            return now_mono < float(self.food_semantic_mhrc_fuse_open_until)

    def _record_mhrc_success(self, latency_ms):
        with self.food_semantic_mhrc_stats_lock:
            self.food_semantic_mhrc_stats["success"] += 1
            self.food_semantic_mhrc_stats["latency_ms_sum"] += float(latency_ms)
            self.food_semantic_mhrc_stats["latency_ms_max"] = max(
                self.food_semantic_mhrc_stats["latency_ms_max"],
                float(latency_ms),
            )
            self.food_semantic_mhrc_consecutive_failures = 0

    def _record_mhrc_failure(self, is_timeout=False):
        trip_fuse = False
        now_mono = time.monotonic()
        threshold = max(1, int(self.food_semantic_mhrc_fuse_fail_threshold))
        cooldown = max(1.0, float(self.food_semantic_mhrc_fuse_cooldown))

        with self.food_semantic_mhrc_stats_lock:
            self.food_semantic_mhrc_stats["failed"] += 1
            if is_timeout:
                self.food_semantic_mhrc_stats["timeout"] += 1

            self.food_semantic_mhrc_consecutive_failures += 1
            if self.food_semantic_mhrc_consecutive_failures >= threshold:
                self.food_semantic_mhrc_consecutive_failures = 0
                self.food_semantic_mhrc_fuse_open_until = now_mono + cooldown
                self.food_semantic_mhrc_stats["fuse_opened"] += 1
                trip_fuse = True

        if trip_fuse:
            rospy.logwarn(
                "MHRC semantic fuse opened for %.1fs after repeated failures",
                cooldown,
            )

    def _record_mhrc_fallback(self):
        self._update_mhrc_stats("fallback_used")

    def _run_food_semantic_mhrc_sync(self, prompt):
        client = self._load_food_semantic_mhrc_client()
        if client is None:
            return ""

        response = client.chat(
            [{"role": "user", "content": str(prompt)}],
            temperature=float(self.food_semantic_mhrc_temperature),
            max_tokens=max(64, int(self.food_semantic_mhrc_max_tokens)),
        )
        return str(response or "").strip()

    def _load_food_semantic_mhrc_client(self):
        with self.food_semantic_mhrc_lock:
            if self.food_semantic_mhrc_client is not None:
                return self.food_semantic_mhrc_client
            if self.food_semantic_mhrc_load_attempted:
                return None

            self.food_semantic_mhrc_load_attempted = True

            src_dir = os.path.abspath(str(self.food_semantic_mhrc_src_dir).strip())
            if not src_dir or not os.path.isdir(src_dir):
                rospy.logwarn("MHRC src dir not found: %s", src_dir)
                return None

            if src_dir not in sys.path:
                sys.path.insert(0, src_dir)

            try:
                from modules.planning.llm_client import LLMClient
            except Exception as exc:
                rospy.logwarn("Failed to import MHRC LLMClient: %s", exc)
                return None

            timeout = max(0.5, float(self.food_semantic_mhrc_timeout))
            config = {
                "base_url": str(self.food_semantic_mhrc_base_url).strip(),
                "api_key": str(self.food_semantic_mhrc_api_key).strip() or "ollama",
                "model": str(self.food_semantic_mhrc_model).strip() or "qwen2.5:3b",
                "temperature": float(self.food_semantic_mhrc_temperature),
                "max_tokens": max(64, int(self.food_semantic_mhrc_max_tokens)),
                "timeout": timeout,
            }

            if not config["base_url"]:
                rospy.logwarn("MHRC backend disabled: empty base_url")
                return None

            try:
                self.food_semantic_mhrc_client = LLMClient(config=config)
                rospy.loginfo(
                    "Food semantic MHRC backend ready: model=%s base_url=%s",
                    config["model"],
                    config["base_url"],
                )
                return self.food_semantic_mhrc_client
            except Exception as exc:
                rospy.logwarn("Failed to initialize MHRC LLMClient: %s", exc)
                return None

    def _run_food_semantic_mhrc(self, text, prompt_override=None):
        prompt = prompt_override if prompt_override is not None else self._build_food_semantic_prompt(text)
        if not prompt:
            return ""

        self._update_mhrc_stats("submitted")

        if self._is_mhrc_fuse_open():
            self._update_mhrc_stats("fuse_blocked")
            self._record_mhrc_fallback()
            rospy.logwarn_throttle(5.0, "MHRC semantic request blocked: fuse is open")
            self._log_mhrc_stats_if_needed()
            return ""

        timeout = max(0.5, float(self.food_semantic_mhrc_timeout))
        start = time.monotonic()
        future = self.food_semantic_mhrc_executor.submit(self._run_food_semantic_mhrc_sync, prompt)

        try:
            raw = future.result(timeout=timeout)
            latency_ms = (time.monotonic() - start) * 1000.0
            if raw:
                self._record_mhrc_success(latency_ms)
                self._log_mhrc_stats_if_needed()
                return raw

            self._record_mhrc_failure(is_timeout=False)
            self._record_mhrc_fallback()
            self._log_mhrc_stats_if_needed()
            return ""
        except FuturesTimeoutError:
            future.cancel()
            self._record_mhrc_failure(is_timeout=True)
            self._record_mhrc_fallback()
            rospy.logwarn_throttle(3.0, "Food semantic MHRC request timed out (%.1fs)", timeout)
            self._log_mhrc_stats_if_needed()
            return ""
        except Exception as exc:
            self._record_mhrc_failure(is_timeout=False)
            self._record_mhrc_fallback()
            rospy.logwarn_throttle(3.0, "Food semantic MHRC request failed: %s", exc)
            self._log_mhrc_stats_if_needed()
            return ""

    def _load_food_semantic_pipeline(self):
        with self.food_semantic_lock:
            if self.food_semantic_pipeline is not None:
                return self.food_semantic_pipeline
            if self.food_semantic_load_attempted:
                return None

            self.food_semantic_load_attempted = True
            model_path = self.food_semantic_model_path
            if not model_path:
                return None
            if not os.path.isdir(model_path):
                rospy.logwarn("Food semantic model path not found: %s", model_path)
                return None

            try:
                from transformers import pipeline

                self.food_semantic_pipeline = pipeline(
                    self.food_semantic_transformers_task,
                    model=model_path,
                    tokenizer=model_path,
                    device=int(self.food_semantic_device),
                )
                rospy.loginfo(
                    "Food semantic local model loaded: %s (%s)",
                    model_path,
                    self.food_semantic_transformers_task,
                )
                return self.food_semantic_pipeline
            except Exception as exc:
                rospy.logwarn("Food semantic local model load failed: %s", exc)
                return None

    def _run_food_semantic_transformers(self, text, prompt_override=None):
        pipe = self._load_food_semantic_pipeline()
        if pipe is None:
            return ""

        prompt = prompt_override if prompt_override is not None else self._build_food_semantic_prompt(text)
        max_new_tokens = max(16, int(self.food_semantic_max_new_tokens))
        do_sample = float(self.food_semantic_temperature) > 1e-4
        kwargs = {
            "max_new_tokens": max_new_tokens,
            "do_sample": do_sample,
        }
        if do_sample:
            kwargs["temperature"] = max(0.1, float(self.food_semantic_temperature))

        try:
            outputs = pipe(prompt, **kwargs)
            if not outputs:
                return ""
            item = outputs[0]
            if isinstance(item, dict):
                raw = item.get("generated_text") or item.get("summary_text") or ""
            else:
                raw = str(item)
            if raw.startswith(prompt):
                raw = raw[len(prompt) :]
            return raw.strip()
        except Exception as exc:
            rospy.logwarn_throttle(3.0, "Food semantic local inference failed: %s", exc)
            return ""

    def _run_food_semantic_ollama(self, text, prompt_override=None):
        base_url = str(self.food_semantic_ollama_url).strip()
        if not base_url:
            return ""

        endpoint = base_url.rstrip("/") + "/api/generate"
        payload = {
            "model": self.food_semantic_ollama_model,
            "prompt": prompt_override if prompt_override is not None else self._build_food_semantic_prompt(text),
            "stream": False,
        }
        keepalive = str(self.food_semantic_ollama_keepalive).strip()
        if keepalive:
            payload["keep_alive"] = keepalive

        timeout = max(0.5, float(self.food_semantic_timeout))
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            endpoint,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                raw = resp.read().decode("utf-8", errors="ignore")
            result = json.loads(raw)
            text_out = result.get("response", "")
            return str(text_out).strip()
        except Exception as exc:
            rospy.logwarn_throttle(3.0, "Food semantic ollama request failed: %s", exc)
            return ""

    def _extract_food_items_semantic(self, text):
        if not self.food_semantic_enabled:
            return {}, []

        backend = str(self.food_semantic_backend).strip().lower()
        raw = ""

        if backend in ("auto", ""):
            raw = self._run_food_semantic_command(text)
            if not raw:
                raw = self._run_food_semantic_ollama(text)
            if not raw:
                raw = self._run_food_semantic_transformers(text)
        elif backend in ("command", "cmd", "local-command"):
            raw = self._run_food_semantic_command(text)
        elif backend in ("ollama", "http", "remote"):
            raw = self._run_food_semantic_ollama(text)
        elif backend in ("mhrc", "mhrc-planning", "mhrc_planning"):
            raw = self._run_food_semantic_mhrc(text)
        elif backend in ("transformers", "hf", "huggingface"):
            raw = self._run_food_semantic_transformers(text)
        else:
            rospy.logwarn_throttle(5.0, "Unsupported food semantic backend: %s", backend)

        if not raw:
            return {}, []

        return self._parse_semantic_food_json(raw)

    def _extract_food_items(self, text):
        if not text:
            return {}, []

        semantic_summary, semantic_mentions = self._extract_food_items_semantic(text)
        if semantic_summary:
            return semantic_summary, semantic_mentions

        normalized = re.sub(r"[^a-z0-9\\s]", " ", text.lower())
        normalized = re.sub(r"\\s+", " ", normalized).strip()
        if not normalized:
            return {}, []

        token_matches = list(re.finditer(r"\\S+", normalized))
        tokens = [m.group(0) for m in token_matches]

        def _qty_from_token(token):
            tk = token.strip().lower()
            if not tk:
                return None
            if tk.startswith("x") and len(tk) > 1 and tk[1:].isdigit():
                return max(1, int(tk[1:]))
            if tk.isdigit():
                return max(1, int(tk))
            return self.food_qty_word_map.get(tk)

        def _qty_near_span(s0, s1):
            left_idx = None
            right_idx = None
            for idx, tm in enumerate(token_matches):
                if tm.end() <= s0:
                    continue
                if tm.start() >= s1:
                    continue
                if left_idx is None:
                    left_idx = idx
                right_idx = idx

            if left_idx is None or right_idx is None:
                return 1

            # Check tokens before alias, nearest first: "two burgers".
            for idx in range(left_idx - 1, max(-1, left_idx - 6), -1):
                tk = tokens[idx]
                qty = _qty_from_token(tk)
                if qty is not None:
                    return qty
                if tk in self.food_qty_bridge_tokens:
                    continue
                break

            # Check tokens after alias: "burger x2" or "burger 2".
            for idx in range(right_idx + 1, min(len(tokens), right_idx + 6)):
                tk = tokens[idx]
                qty = _qty_from_token(tk)
                if qty is not None:
                    return qty
                if tk in self.food_qty_bridge_tokens:
                    continue
                break

            return 1

        mentions = []
        used_spans = []
        for canonical, alias, pattern in self.food_patterns:
            for match in pattern.finditer(normalized):
                s0, s1 = match.span()
                overlap = False
                for u0, u1 in used_spans:
                    if not (s1 <= u0 or s0 >= u1):
                        overlap = True
                        break
                if overlap:
                    continue
                used_spans.append((s0, s1))

                qty = _qty_near_span(s0, s1)
                mentions.append(
                    {
                        "name": canonical,
                        "alias": alias,
                        "qty": int(qty),
                        "source": "keyword",
                    }
                )

        summary = {}
        for m in mentions:
            name = m["name"]
            qty = max(1, int(m.get("qty", 1)))
            summary[name] = summary.get(name, 0) + qty

        return summary, mentions

    def _load_food_orders_from_json(self):
        if not self.food_order_enabled:
            return

        path = self._resolve_customer_food_order_path()
        if not path:
            self._clear_food_orders_cache()
            rospy.logwarn_throttle(2.0, "No active customer folder; skip loading food order JSON.")
            return

        if not os.path.isfile(path):
            self._clear_food_orders_cache()
            return

        try:
            with open(path, "r", encoding="utf-8") as fp:
                payload = json.load(fp)
            history = payload.get("order_history", [])
            needed = payload.get("current_needed_foods", [])
            needed_qty = payload.get("current_needed_foods_qty", {})

            loaded_history = []
            loaded_needed = []
            loaded_needed_qty = {}

            if isinstance(history, list):
                loaded_history = history
            if isinstance(needed, list):
                loaded_needed = [str(x) for x in needed]
            if isinstance(needed_qty, dict):
                parsed = {}
                for key, val in needed_qty.items():
                    try:
                        qty = int(val)
                    except Exception:
                        continue
                    if qty > 0:
                        parsed[str(key)] = qty
                loaded_needed_qty = parsed

            with self.food_order_lock:
                self.food_order_history = loaded_history
                self.current_needed_foods = loaded_needed
                self.current_needed_foods_qty = loaded_needed_qty

            self._remove_legacy_food_order_file(path)
        except Exception as exc:
            rospy.logwarn("Failed to load food order JSON: %s", exc)

    def _format_foods_with_qty_for_speech(self, foods_with_qty):
        if not isinstance(foods_with_qty, list):
            return ""

        parts = []
        for item in foods_with_qty:
            if not isinstance(item, dict):
                continue
            name = str(item.get("name", "")).strip().replace("_", " ")
            if not name:
                continue
            qty = self._coerce_positive_qty(item.get("qty", 1))
            if qty > 1:
                parts.append("%d %s" % (qty, name))
            else:
                parts.append(name)

        if not parts:
            return ""
        if len(parts) == 1:
            return parts[0]
        if len(parts) == 2:
            return parts[0] + " and " + parts[1]
        return ", ".join(parts[:-1]) + ", and " + parts[-1]

    def _build_food_order_confirmation_text(self, payload):
        if not self.food_order_confirm_enabled:
            return ""
        if not isinstance(payload, dict):
            return ""

        entry = payload.get("last_entry", {})
        foods_with_qty = []
        if isinstance(entry, dict):
            foods_with_qty = entry.get("foods_with_qty", [])

        foods_text = self._format_foods_with_qty_for_speech(foods_with_qty)
        if not foods_text:
            return ""

        template = str(self.food_order_confirm_template).strip()
        if not template:
            template = "OK, I'll get {foods} for you"

        try:
            return template.format(foods=foods_text)
        except Exception:
            return "OK, I'll get %s for you" % foods_text

    def _publish_serving_target_capture_request(self, gx, gy, reason):
        if not self.serving_target_enabled:
            return

        now = rospy.Time.now()
        if self.serving_target_capture_pub is not None:
            req = PointStamped()
            req.header.stamp = now
            req.header.frame_id = self.global_frame
            req.point.x = float(gx)
            req.point.y = float(gy)
            req.point.z = 0.0
            self.serving_target_capture_pub.publish(req)

        if self.serving_target_capture_cmd_pub is not None:
            cmd_payload = {
                "timestamp": now.to_sec(),
                "frame_id": self.global_frame,
                "reason": str(reason),
                "person_global": {
                    "x": float(gx),
                    "y": float(gy),
                    "z": 0.0,
                },
                "customer_id": self.active_customer_id,
                "customer_folder": self.active_customer_folder,
            }
            try:
                self.serving_target_capture_cmd_pub.publish(
                    String(data=json.dumps(cmd_payload, ensure_ascii=False))
                )
            except Exception as exc:
                rospy.logwarn("Failed to publish serving target capture cmd: %s", exc)

    def _record_serving_target_snapshot(self, px, py, px_base, py_base):
        if not self.serving_target_enabled:
            return False

        self._resolve_active_customer_folder()

        try:
            gx = float(px)
            gy = float(py)
        except Exception:
            rospy.logwarn("Serving target snapshot skipped: invalid person coordinates")
            return False

        now = rospy.Time.now()
        payload = {
            "timestamp": now.to_sec(),
            "frame_id": self.global_frame,
            "person_global": {
                "x": gx,
                "y": gy,
                "z": 0.0,
            },
            "current_goal": {
                "x": self.last_goal_x,
                "y": self.last_goal_y,
                "theta": self.last_goal_theta,
            },
        }
        if px_base is not None and py_base is not None:
            payload["person_base"] = {
                "frame_id": self.base_frame,
                "x": float(px_base),
                "y": float(py_base),
                "z": 0.0,
            }

        payload["customer_id"] = self.active_customer_id
        payload["customer_folder"] = self.active_customer_folder

        path = self._resolve_customer_scoped_path(self.serving_target_snapshot_json_file)
        if path:
            try:
                parent = os.path.dirname(path)
                if parent:
                    os.makedirs(parent, exist_ok=True)
                tmp_path = path + ".tmp"
                with open(tmp_path, "w", encoding="utf-8") as fp:
                    json.dump(payload, fp, indent=2, ensure_ascii=False)
                os.replace(tmp_path, path)
            except Exception as exc:
                rospy.logwarn("Failed to write serving target snapshot JSON: %s", exc)

        self._publish_serving_target_capture_request(gx, gy, "pause_enter")
        self._set_serving_customer_state("PAUSED_ORDERING")

        self.last_serving_target_capture_time = now
        rospy.loginfo("Serving target snapshot saved and face capture requested.")
        return True

    def _load_return_anchor_goal(self):
        path = self.return_anchor_json_file
        if not path or not os.path.isfile(path):
            rospy.logwarn("Return anchor JSON not found: %s", path)
            return None

        try:
            with open(path, "r", encoding="utf-8") as fp:
                payload = json.load(fp)
        except Exception as exc:
            rospy.logwarn("Failed to read return anchor JSON: %s", exc)
            return None

        position = payload.get("position", {})
        orientation = payload.get("orientation", {})

        try:
            gx = float(position.get("x"))
            gy = float(position.get("y"))
        except Exception:
            rospy.logwarn("Return anchor JSON missing valid position x/y")
            return None

        qx = float(orientation.get("x", 0.0))
        qy = float(orientation.get("y", 0.0))
        qz = float(orientation.get("z", 0.0))
        qw = float(orientation.get("w", 1.0))
        theta = euler_from_quaternion([qx, qy, qz, qw])[2]
        return gx, gy, theta

    def _publish_manual_nav_goal(self, gx, gy, theta, reason):
        self._stop_gaze_tracking_cmd()

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = self.global_frame
        pose_goal.header.stamp = rospy.Time.now()
        pose_goal.pose.position.x = gx
        pose_goal.pose.position.y = gy
        pose_goal.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)
        pose_goal.pose.orientation.x = qx
        pose_goal.pose.orientation.y = qy
        pose_goal.pose.orientation.z = qz
        pose_goal.pose.orientation.w = qw
        self.goal_pub.publish(pose_goal)

        # Always publish bridge points so far_planner receives a return target.
        waypoint = PointStamped()
        waypoint.header = pose_goal.header
        waypoint.point.x = gx
        waypoint.point.y = gy
        waypoint.point.z = 0.0
        self.waypoint_pub.publish(waypoint)
        self.goal_point_pub.publish(waypoint)

        self.last_goal_x = gx
        self.last_goal_y = gy
        self.last_goal_theta = theta
        self.last_goal_score = None
        self.last_publish_time = rospy.Time.now()
        self.goal_reach_since = None

        rospy.loginfo(
            "Published %s goal: (%.2f, %.2f)",
            str(reason),
            gx,
            gy,
        )

    def _publish_return_anchor_goal(self, gx, gy, theta):
        self._publish_manual_nav_goal(gx, gy, theta, "return-to-start")

    def _trigger_return_to_anchor(self):
        if not self.return_to_anchor_on_order_confirm:
            return False
        if self.return_to_anchor_active:
            return True

        anchor = self._load_return_anchor_goal()
        if anchor is None:
            return False

        gx, gy, theta = anchor
        self.return_anchor_goal = anchor
        self.return_to_anchor_active = True
        self.return_navigation_state = "GO_TO_ANCHOR"
        self.return_table_goal = None
        self.return_table_plan_attempt_time = rospy.Time(0)
        self.table_front_arrive_time = rospy.Time(0)
        self.table_food_check_done = False
        self.goal_publish_paused = True
        self._set_serving_customer_state("RETURNING")

        republish = max(1, int(self.return_anchor_bridge_republish))
        for _ in range(republish):
            self._publish_return_anchor_goal(gx, gy, theta)

        return True

    def _run_return_navigation_cycle(self):
        if not self.return_to_anchor_active:
            return
        if self.return_anchor_goal is None:
            return

        pose = self._lookup_robot_pose()
        if pose is None:
            return

        rx, ry, _ = pose
        ax, ay, _ = self.return_anchor_goal
        dist_to_anchor = math.hypot(rx - ax, ry - ay)

        if self.return_navigation_state == "GO_TO_ANCHOR":
            if not self.return_table_approach_enabled:
                if dist_to_anchor <= self.return_table_trigger_distance:
                    self._complete_current_service_cycle_and_prepare_next(
                        "service_cycle_completed_at_anchor",
                    )
                return
            if dist_to_anchor > self.return_table_trigger_distance:
                return

            now = rospy.Time.now()
            if self.return_table_plan_attempt_time != rospy.Time(0):
                if (now - self.return_table_plan_attempt_time).to_sec() < self.return_table_plan_retry_interval:
                    return
            self.return_table_plan_attempt_time = now

            plan = self._plan_table_front_goal(rx, ry)
            if plan is None:
                rospy.logwarn_throttle(1.0, "No nearby table/bar candidate found near return anchor yet.")
                return

            sx, sy, theta = plan["goal"]
            comp = plan["component"]
            customer = plan["customer"]

            self.return_table_goal = (sx, sy, theta)
            self.return_navigation_state = "TABLE_APPROACH"
            self.table_front_arrive_time = rospy.Time(0)
            self.table_food_check_done = False
            self._set_serving_customer_state("TABLE_APPROACH")

            republish = max(1, int(self.return_table_goal_republish))
            for _ in range(republish):
                self._publish_manual_nav_goal(sx, sy, theta, "table-front")

            rospy.loginfo(
                "Table/bar long-edge front goal selected (elong=%.2f, customer=(%.2f, %.2f)).",
                comp["elongation"],
                customer[0],
                customer[1],
            )
            return

        if self.return_navigation_state == "TABLE_APPROACH" and self.return_table_goal is not None:
            sx, sy, _ = self.return_table_goal
            dist_to_table = math.hypot(rx - sx, ry - sy)
            if dist_to_table <= self.return_table_arrive_distance:
                self.return_navigation_state = "AT_TABLE_FRONT"
                self.table_front_arrive_time = rospy.Time.now()
                self.table_food_check_done = False
                self._set_serving_customer_state("AT_TABLE_FRONT")
                rospy.loginfo("Arrived at table/bar front serving position.")

        if self.return_navigation_state == "AT_TABLE_FRONT":
            self._run_table_food_missing_check_cycle()

    def _store_food_order(self, source_text, food_summary, food_mentions):
        if not self.food_order_enabled or not food_summary:
            return None, False

        self._resolve_active_customer_folder()
        customer_no = self._refresh_active_customer_no()
        if not customer_no:
            rospy.logwarn("No active customer No. tag; skip storing food order JSON.")
            return None, False

        now = rospy.Time.now().to_sec()
        ts = rospy.Time.from_sec(now).to_sec()
        foods_with_qty = []
        for name in sorted(food_summary.keys()):
            foods_with_qty.append({"name": name, "qty": int(food_summary[name])})

        entry = {
            "timestamp": ts,
            "source": "pause_reply",
            "customer_no": str(customer_no),
            "customer_folder": str(self.active_customer_folder or ""),
            "recognized_text": source_text,
            "foods": [item["name"] for item in foods_with_qty],
            "foods_with_qty": foods_with_qty,
            "food_mentions": food_mentions,
        }

        with self.food_order_lock:
            self.food_order_history.append(entry)
            if len(self.food_order_history) > 200:
                self.food_order_history = self.food_order_history[-200:]

            for item in food_summary.keys():
                if item not in self.current_needed_foods:
                    self.current_needed_foods.append(item)
                self.current_needed_foods_qty[item] = (
                    int(self.current_needed_foods_qty.get(item, 0)) + int(food_summary[item])
                )

            payload = {
                "customer_no": str(customer_no),
                "customer_folder": str(self.active_customer_folder or ""),
                "current_needed_foods": self.current_needed_foods,
                "current_needed_foods_qty": self.current_needed_foods_qty,
                "last_entry": entry,
                "order_history": self.food_order_history,
            }

        stored_to_file = False
        customer_path = self._resolve_customer_food_order_path()
        if not customer_path:
            rospy.logwarn("No active customer folder; skip storing food order JSON.")
        else:
            try:
                parent = os.path.dirname(customer_path)
                if parent:
                    os.makedirs(parent, exist_ok=True)
                tmp_path = customer_path + ".tmp"
                with open(tmp_path, "w", encoding="utf-8") as fp:
                    json.dump(payload, fp, indent=2, ensure_ascii=False)
                os.replace(tmp_path, customer_path)
                stored_to_file = True
                self._remove_legacy_food_order_file(customer_path)
            except Exception as exc:
                rospy.logwarn("Failed to write food order JSON (%s): %s", customer_path, exc)

        if self.food_order_pub is not None:
            try:
                self.food_order_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            except Exception as exc:
                rospy.logwarn("Failed to publish food order JSON: %s", exc)

        return payload, stored_to_file

    def _resolve_table_food_detection_json_path(self):
        raw_path = str(self.table_food_detection_json_file).strip()
        if not raw_path:
            return ""

        try:
            raw_path = raw_path.format(
                customer_folder=self.active_customer_folder,
                customer_id=self.active_customer_id,
            )
        except Exception:
            pass

        if os.path.isabs(raw_path):
            return raw_path

        workdir = str(self.table_food_detect_workdir).strip()
        if workdir:
            return os.path.join(workdir, raw_path)
        return os.path.abspath(raw_path)

    def _append_detected_food_name(self, raw_name, detected_set):
        if not isinstance(detected_set, set):
            return
        canonical = self._canonicalize_detected_food_name(raw_name)
        if canonical:
            detected_set.add(canonical)

    def _extract_detected_foods_from_payload(self, payload):
        detected = set()

        entries = []
        if isinstance(payload, list):
            entries.extend(payload)
        elif isinstance(payload, dict):
            for key in ("objects", "items", "detections", "results"):
                value = payload.get(key)
                if isinstance(value, list):
                    entries.extend(value)

            for key in ("labels", "detected_labels", "foods", "detected_foods"):
                value = payload.get(key)
                if isinstance(value, list):
                    entries.extend(value)
                elif isinstance(value, str):
                    entries.append(value)

            if not entries:
                entries.append(payload)

        for item in entries:
            if isinstance(item, str):
                self._append_detected_food_name(item, detected)
                continue

            if not isinstance(item, dict):
                continue

            for key in ("label", "name", "class", "category", "object", "food"):
                value = item.get(key)
                if isinstance(value, str) and value.strip():
                    self._append_detected_food_name(value, detected)
                    break

        return detected

    def _extract_detected_foods_from_text(self, text):
        detected = set()
        raw = str(text or "")
        if not raw:
            return detected

        blob = self._extract_json_blob(raw)
        if blob:
            try:
                payload = json.loads(blob)
                detected.update(self._extract_detected_foods_from_payload(payload))
            except Exception:
                pass

        for match in re.finditer(r"(?:类别|class|label)\s*[:：]\s*([^\n|]+)", raw, flags=re.IGNORECASE):
            token = str(match.group(1)).strip()
            if token:
                self._append_detected_food_name(token, detected)

        return detected

    def _load_detected_foods_from_json_file(self, path):
        detected = set()
        if not path or not os.path.isfile(path):
            return detected

        latest_payload = None
        try:
            with open(path, "r", encoding="utf-8") as fp:
                tail_lines = deque(fp, maxlen=200)

            for line in reversed(tail_lines):
                row = str(line).strip()
                if not row:
                    continue
                try:
                    latest_payload = json.loads(row)
                    break
                except Exception:
                    continue

            if latest_payload is None:
                with open(path, "r", encoding="utf-8") as fp:
                    latest_payload = json.load(fp)
        except Exception as exc:
            rospy.logwarn("Failed to parse detection JSON file (%s): %s", path, exc)
            return detected

        detected.update(self._extract_detected_foods_from_payload(latest_payload))
        return detected

    def _run_table_food_detection(self):
        detected = set()
        detection_json_path = self._resolve_table_food_detection_json_path()

        cmd_template = str(self.table_food_detect_command).strip()
        if cmd_template:
            try:
                command = cmd_template.format(
                    customer_folder=self.active_customer_folder,
                    customer_id=self.active_customer_id,
                    detection_json=detection_json_path,
                    yolo_repo=self.table_food_detect_workdir,
                )
            except Exception:
                command = cmd_template

            timeout_sec = max(0.5, float(self.table_food_detect_timeout))
            workdir = str(self.table_food_detect_workdir).strip()
            cwd = workdir if (workdir and os.path.isdir(workdir)) else None

            try:
                if self.table_food_detect_use_shell:
                    proc = subprocess.run(
                        command,
                        shell=True,
                        cwd=cwd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        timeout=timeout_sec,
                    )
                else:
                    proc = subprocess.run(
                        shlex.split(command),
                        shell=False,
                        cwd=cwd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        timeout=timeout_sec,
                    )

                output_text = ((proc.stdout or "") + "\n" + (proc.stderr or "")).strip()
                detected.update(self._extract_detected_foods_from_text(output_text))

                if proc.returncode != 0:
                    rospy.logwarn("Table food detect command exit code %s", proc.returncode)
            except subprocess.TimeoutExpired as exc:
                timeout_output = ((exc.stdout or "") + "\n" + (exc.stderr or "")).strip()
                detected.update(self._extract_detected_foods_from_text(timeout_output))
                rospy.logwarn("Table food detect command timed out after %.1fs", timeout_sec)
            except Exception as exc:
                rospy.logwarn("Table food detect command failed: %s", exc)

        if detection_json_path:
            detected.update(self._load_detected_foods_from_json_file(detection_json_path))

        return detected

    def _get_ordered_foods_for_table_check(self):
        ordered = []
        self._resolve_active_customer_folder()
        self._load_food_orders_from_json()

        with self.food_order_lock:
            if isinstance(self.current_needed_foods_qty, dict) and self.current_needed_foods_qty:
                for name, qty in self.current_needed_foods_qty.items():
                    try:
                        if int(qty) <= 0:
                            continue
                    except Exception:
                        continue
                    canonical = self._canonicalize_food_name(name)
                    if canonical:
                        ordered.append(canonical)
            elif isinstance(self.current_needed_foods, list):
                for name in self.current_needed_foods:
                    canonical = self._canonicalize_food_name(name)
                    if canonical:
                        ordered.append(canonical)

        unique_ordered = []
        seen = set()
        for food in ordered:
            if food in seen:
                continue
            seen.add(food)
            unique_ordered.append(food)
        return unique_ordered

    def _run_table_food_missing_check_cycle(self):
        if not self.table_food_check_enabled:
            self.table_food_check_done = True
            return
        if self.table_food_check_done:
            return
        if self.return_navigation_state != "AT_TABLE_FRONT":
            return

        now = rospy.Time.now()
        if self.table_front_arrive_time == rospy.Time(0):
            self.table_front_arrive_time = now
            return

        if (now - self.table_front_arrive_time).to_sec() < max(0.0, float(self.table_food_check_delay)):
            return

        self.table_food_check_done = True
        ordered_foods = self._get_ordered_foods_for_table_check()
        if not ordered_foods:
            rospy.logwarn("Table food check skipped: no ordered foods in active customer folder.")
            return

        detected_foods = self._run_table_food_detection()
        missing_foods = [food for food in ordered_foods if food not in detected_foods]

        if not missing_foods:
            rospy.loginfo("Table food check passed: all ordered foods were detected.")
            return

        missing_names = [name.replace("_", " ") for name in missing_foods]
        announce = "The customer wants %s" % ", ".join(missing_names)
        self._announce_text_prompt(announce, force=True)
        rospy.loginfo("Table food check missing items: %s", ", ".join(missing_names))

    def _load_pause_speech_tts(self):
        if not self.pause_prompt_use_speech_module:
            return None

        with self.pause_speech_lock:
            if self.pause_speech_tts is not None:
                return self.pause_speech_tts
            if self.pause_speech_load_attempted:
                return None

            self.pause_speech_load_attempted = True
            module_file = self.pause_prompt_speech_module_file

            if not module_file or not os.path.isfile(module_file):
                rospy.logwarn("Pause prompt speech module file not found: %s", module_file)
                return None

            try:
                spec = importlib.util.spec_from_file_location("task5_pause_speech_synthesizer", module_file)
                if spec is None or spec.loader is None:
                    rospy.logwarn("Cannot build import spec for speech module: %s", module_file)
                    return None

                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)

                tts_cls = getattr(module, self.pause_prompt_speech_class)
                tts_obj = tts_cls()
                speak_fn = getattr(tts_obj, "speak", None)
                if not callable(speak_fn):
                    rospy.logwarn(
                        "Speech class %s has no callable speak(text)",
                        self.pause_prompt_speech_class,
                    )
                    return None

                self.pause_speech_tts = tts_obj
                rospy.loginfo(
                    "Pause prompt speech module loaded: %s (%s)",
                    module_file,
                    self.pause_prompt_speech_class,
                )
                return self.pause_speech_tts
            except Exception as exc:
                rospy.logwarn("Pause prompt speech module load failed: %s", exc)
                return None

    def _speak_with_pause_speech_module(self, text):
        tts_obj = self._load_pause_speech_tts()
        if tts_obj is None:
            return False

        def _worker():
            try:
                with self.pause_speech_lock:
                    tts_obj.speak(text)
            except Exception as exc:
                rospy.logwarn("Pause prompt speech module speak failed: %s", exc)

        threading.Thread(target=_worker, daemon=True).start()
        return True

    def _load_pause_speech_asr(self):
        if not (self.pause_reply_listen_enabled and self.pause_reply_use_speech_module):
            return None

        with self.pause_asr_lock:
            if self.pause_speech_asr is not None:
                return self.pause_speech_asr
            if self.pause_asr_load_attempted:
                return None

            self.pause_asr_load_attempted = True
            module_file = self.pause_reply_speech_module_file

            if not module_file or not os.path.isfile(module_file):
                rospy.logwarn("Pause reply ASR module file not found: %s", module_file)
                return None

            try:
                spec = importlib.util.spec_from_file_location("task5_pause_speech_asr", module_file)
                if spec is None or spec.loader is None:
                    rospy.logwarn("Cannot build import spec for ASR module: %s", module_file)
                    return None

                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)

                asr_cls = getattr(module, self.pause_reply_speech_class)
                try:
                    asr_obj = asr_cls(
                        self.pause_reply_mic_name,
                        ros_publish_enabled=False,
                    )
                except TypeError:
                    asr_obj = asr_cls(self.pause_reply_mic_name)

                stream = getattr(asr_obj, "stream", None)
                vad_iterator = getattr(asr_obj, "vad_iterator", None)
                model = getattr(asr_obj, "model", None)
                if stream is None or vad_iterator is None or model is None:
                    rospy.logwarn(
                        "ASR class %s missing required fields: stream/vad_iterator/model",
                        self.pause_reply_speech_class,
                    )
                    return None

                self.pause_speech_asr = asr_obj
                rospy.loginfo(
                    "Pause reply ASR module loaded: %s (%s)",
                    module_file,
                    self.pause_reply_speech_class,
                )
                return self.pause_speech_asr
            except Exception as exc:
                rospy.logwarn("Pause reply ASR module load failed: %s", exc)
                return None

    def _transcribe_pause_reply_audio(self, asr_obj, audio_16k):
        try:
            import numpy as np
        except Exception as exc:
            rospy.logwarn("Pause reply ASR import numpy failed: %s", exc)
            return ""

        kwargs = {
            "beam_size": self.pause_reply_beam_size,
            "without_timestamps": True,
        }
        if self.pause_reply_language:
            kwargs["language"] = self.pause_reply_language

        try:
            segments, _ = asr_obj.model.transcribe(audio_16k.astype(np.float32), **kwargs)
            text = "".join(seg.text.strip() + " " for seg in segments).strip()
            return text
        except Exception as exc:
            rospy.logwarn("Pause reply transcription failed: %s", exc)
            return ""

    def _listen_pause_reply_once(self):
        asr_obj = self._load_pause_speech_asr()
        if asr_obj is None:
            return ""

        try:
            import numpy as np
            import resampy
        except Exception as exc:
            rospy.logwarn("Pause reply ASR dependencies unavailable: %s", exc)
            return ""

        stream = asr_obj.stream
        vad_iterator = asr_obj.vad_iterator
        target_rate = int(getattr(asr_obj, "target_rate", 16000))
        device_rate = int(getattr(asr_obj, "device_rate", 48000))
        frame_samples_device = int(getattr(asr_obj, "frame_samples_device", int(device_rate * 32 / 1000)))
        frame_samples_16k = int(getattr(asr_obj, "frame_samples_16k", int(target_rate * 32 / 1000)))

        min_audio_samples = max(1, int(target_rate * self.pause_reply_min_audio_sec))
        deadline = rospy.Time.now() + rospy.Duration(max(0.5, self.pause_reply_timeout))
        buffer_16k = np.array([], dtype=np.float32)
        is_speaking = False

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            try:
                data = stream.read(frame_samples_device, exception_on_overflow=False)
            except Exception as exc:
                rospy.logwarn("Pause reply audio read failed: %s", exc)
                return ""

            audio_dev = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
            try:
                audio_16k = resampy.resample(
                    audio_dev,
                    sr_orig=device_rate,
                    sr_new=target_rate,
                    filter="kaiser_best",
                )
            except Exception:
                audio_16k = resampy.resample(audio_dev, sr_orig=device_rate, sr_new=target_rate)

            if len(audio_16k) > frame_samples_16k:
                audio_16k = audio_16k[:frame_samples_16k]
            elif len(audio_16k) < frame_samples_16k:
                pad = frame_samples_16k - len(audio_16k)
                audio_16k = np.concatenate([audio_16k, np.zeros(pad, dtype=np.float32)])

            speech_dict = vad_iterator(audio_16k, return_seconds=False)
            if speech_dict is not None and "start" in speech_dict:
                is_speaking = True
                buffer_16k = np.array([], dtype=np.float32)

            if is_speaking:
                buffer_16k = np.concatenate([buffer_16k, audio_16k])

            if speech_dict is not None and "end" in speech_dict:
                is_speaking = False
                if len(buffer_16k) >= min_audio_samples:
                    return self._transcribe_pause_reply_audio(asr_obj, buffer_16k)

        if is_speaking and len(buffer_16k) >= min_audio_samples:
            return self._transcribe_pause_reply_audio(asr_obj, buffer_16k)

        return ""

    def _process_pause_reply_text(self, text, publish_topic=True):
        if not text:
            return False

        active_state = str(self.active_customer_state or "IDLE").strip().upper()
        if active_state not in ("PAUSED_ORDERING", "TRACKING"):
            rospy.loginfo(
                "Pause reply ignored in state %s (allowed: PAUSED_ORDERING/TRACKING)",
                active_state,
            )
            return False

        self._resolve_active_customer_folder()

        rospy.loginfo("Pause reply recognized: %s", text)
        if publish_topic and self.pause_reply_pub is not None:
            self.pause_reply_pub.publish(String(data=text))

        food_summary, food_mentions = self._extract_food_items(text)
        if food_summary:
            payload, stored_to_file = self._store_food_order(text, food_summary, food_mentions)
            summary_text = ", ".join(
                ["%s x%d" % (k, food_summary[k]) for k in sorted(food_summary.keys())]
            )
            rospy.loginfo("Extracted requested foods: %s", summary_text)

            if stored_to_file and payload is not None:
                latest_state = str(self.active_customer_state or "IDLE").strip().upper()
                if latest_state in ("PAUSED_ORDERING", "TRACKING"):
                    self._set_serving_customer_state("ORDERED")
                confirm_text = self._build_food_order_confirmation_text(payload)
                if confirm_text:
                    self._announce_text_prompt(confirm_text, force=True)
                self._trigger_return_to_anchor()

            return True

        rospy.loginfo("Reply recognized but not understood as a food order")
        return False

    def _pause_reply_worker(self):
        try:
            if self.pause_reply_start_delay > 1e-3:
                rospy.sleep(self.pause_reply_start_delay)
            text = self._listen_pause_reply_once()
            if not text:
                rospy.loginfo("Pause reply not recognized within %.1fs", self.pause_reply_timeout)
                return

            if self._process_pause_reply_text(text):
                return

            if not self.pause_reply_reask_on_unrecognized:
                return

            max_attempts = max(0, int(self.pause_reply_reask_max_attempts))
            for attempt in range(max_attempts):
                prompt_text = self.pause_reply_reask_text
                if prompt_text:
                    self._announce_text_prompt(prompt_text, force=True)

                if self.pause_reply_reask_listen_delay > 1e-3:
                    rospy.sleep(self.pause_reply_reask_listen_delay)

                retry_text = self._listen_pause_reply_once()
                if not retry_text:
                    rospy.loginfo(
                        "Pause reply retry %d/%d not recognized within %.1fs",
                        attempt + 1,
                        max_attempts,
                        self.pause_reply_timeout,
                    )
                    continue

                if self._process_pause_reply_text(retry_text):
                    return

                rospy.loginfo(
                    "Pause reply retry %d/%d still not understood",
                    attempt + 1,
                    max_attempts,
                )
        except Exception as exc:
            rospy.logwarn("Pause reply worker failed: %s", exc)
        finally:
            self._cleanup_pause_reply_asr()

    def _trigger_pause_reply_listen(self):
        if not self.pause_reply_listen_enabled:
            return

        now = rospy.Time.now()
        if self.last_pause_reply_time != rospy.Time(0):
            if (now - self.last_pause_reply_time).to_sec() < self.pause_reply_cooldown:
                return

        if self.pause_reply_thread is not None and self.pause_reply_thread.is_alive():
            return

        self.last_pause_reply_time = now
        self.pause_reply_thread = threading.Thread(target=self._pause_reply_worker, daemon=True)
        self.pause_reply_thread.start()

    def _cleanup_pause_reply_asr(self):
        with self.pause_asr_lock:
            if self.pause_speech_asr is None:
                return

            cleanup_fn = getattr(self.pause_speech_asr, "cleanup", None)
            if callable(cleanup_fn):
                try:
                    cleanup_fn()
                except Exception:
                    pass
            self.pause_speech_asr = None
            self.pause_asr_load_attempted = False

    def _publish_prompt_to_mhrc_topic(self, text):
        if self.pause_prompt_mhrc_pub is None:
            return False

        try:
            if self.pause_prompt_mhrc_require_subscriber:
                min_connections = 1
                if (
                    self.mhrc_speak_bridge_enabled
                    and self.mhrc_speak_bridge_topic == self.pause_prompt_mhrc_speak_topic
                ):
                    # One connection can be this node's own bridge subscriber.
                    min_connections = 2

                if self.pause_prompt_mhrc_pub.get_num_connections() < min_connections:
                    rospy.logwarn_throttle(
                        5.0,
                        "MHRC speak topic has no external subscribers: %s",
                        self.pause_prompt_mhrc_speak_topic,
                    )
                    return False

            self.pause_prompt_mhrc_pub.publish(String(data=text))
            self.last_local_mhrc_prompt_text = text
            self.last_local_mhrc_prompt_mono = time.monotonic()
            return True
        except Exception as exc:
            rospy.logwarn("Failed to publish MHRC speak prompt: %s", exc)
            return False

    def mhrc_speak_callback(self, msg):
        text = str(msg.data or "").strip()
        if not text:
            return

        try:
            header = getattr(msg, "_connection_header", {}) or {}
            caller_id = str(header.get("callerid") or "").strip()
        except Exception:
            caller_id = ""

        # Ignore loopback from this node and recently mirrored same-text publishes.
        if caller_id == rospy.get_name():
            return
        if (
            text == self.last_local_mhrc_prompt_text
            and (time.monotonic() - self.last_local_mhrc_prompt_mono) < 1.0
        ):
            return

        self._announce_text_prompt(text, force=True, source="mhrc_bridge")

    def pause_reply_topic_callback(self, msg):
        text = str(msg.data or "").strip()
        if not text:
            return

        try:
            header = getattr(msg, "_connection_header", {}) or {}
            caller_id = str(header.get("callerid") or "").strip()
        except Exception:
            caller_id = ""

        # Ignore self-published loopback when internal ASR mode is enabled.
        if caller_id == rospy.get_name():
            return

        with self.pause_reply_topic_lock:
            self._process_pause_reply_text(text, publish_topic=False)

    def _announce_text_prompt(self, text, force=False, source="task5"):
        if not self.pause_prompt_enabled:
            return False

        text = str(text).strip()
        if not text:
            return False

        now = rospy.Time.now()
        if not force and self.last_pause_prompt_time != rospy.Time(0):
            if (now - self.last_pause_prompt_time).to_sec() < self.pause_prompt_cooldown:
                return False

        announced = False
        prefer_task5 = bool(self.pause_prompt_task5_priority_higher or source == "mhrc_bridge")
        allow_mhrc_publish = source != "mhrc_bridge"

        if prefer_task5:
            if self.pause_prompt_use_speech_module:
                announced = self._speak_with_pause_speech_module(text)

            if not announced and self.pause_prompt_pub is not None:
                self.pause_prompt_pub.publish(String(data=text))
                announced = True

            if not announced and allow_mhrc_publish:
                announced = self._publish_prompt_to_mhrc_topic(text)
        else:
            if allow_mhrc_publish:
                announced = self._publish_prompt_to_mhrc_topic(text)

            if not announced and self.pause_prompt_use_speech_module:
                announced = self._speak_with_pause_speech_module(text)

            if not announced and self.pause_prompt_pub is not None:
                self.pause_prompt_pub.publish(String(data=text))
                announced = True

        if not announced and self.pause_prompt_command:
            try:
                cmd = self.pause_prompt_command.format(text=text)
                if self.pause_prompt_use_shell:
                    subprocess.Popen(cmd, shell=True)
                else:
                    subprocess.Popen(shlex.split(cmd))
                announced = True
            except Exception as exc:
                rospy.logwarn("Pause prompt command failed: %s", exc)

        if not announced:
            try:
                subprocess.Popen(["espeak", text])
                announced = True
            except Exception:
                pass

        if not announced:
            rospy.loginfo("Pause prompt: %s", text)

        self.last_pause_prompt_time = now
        return announced

    def _announce_pause_prompt(self):
        return self._announce_text_prompt(self.pause_prompt_text, force=False)

    def _angle_diff_abs(self, a, b):
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    def _gaze_motion_allowed(self, rx, ry, robot_yaw, linear_x, linear_y):
        motion_norm = math.hypot(linear_x, linear_y)
        if motion_norm < 1e-4 or self.grid is None:
            return True

        # Convert desired robot-frame motion direction into global frame.
        dir_x = linear_x / motion_norm
        dir_y = linear_y / motion_norm
        world_dx = dir_x * math.cos(robot_yaw) - dir_y * math.sin(robot_yaw)
        world_dy = dir_x * math.sin(robot_yaw) + dir_y * math.cos(robot_yaw)

        probe = self.gaze_collision_probe_distance
        step = max(0.03, self.path_check_step * 0.6)
        steps = max(2, int(probe / step))

        # Multi-point probe for higher-frequency and more conservative collision checks.
        for i in range(1, steps + 1):
            d = probe * float(i) / float(steps)
            tx = rx + d * world_dx
            ty = ry + d * world_dy
            if not self._is_pose_collision_free(tx, ty):
                return False

        tx = rx + probe * world_dx
        ty = ry + probe * world_dy
        return self._is_segment_collision_free(rx, ry, tx, ty)

    def _lookup_robot_pose(self):
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.2),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(1.0, "Robot TF lookup failed: %s", exc)
            return None

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y
        q = robot_tf.transform.rotation
        robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return rx, ry, robot_yaw

    def _publish_gaze_tracking_cmd(self, px_base, py_base, rx, ry, robot_yaw):
        cmd = Twist()

        heading = math.atan2(py_base, px_base)
        yaw_deadband = math.radians(self.gaze_yaw_deadband_deg)
        if abs(heading) > yaw_deadband:
            cmd.angular.z = max(
                -self.gaze_max_angular,
                min(self.gaze_max_angular, self.gaze_kp_angular * heading),
            )

        if self.gaze_track_linear:
            dist = math.hypot(px_base, py_base)
            err = dist - self.gaze_target_distance
            heading_gate = math.radians(self.gaze_linear_heading_gate_deg)
            if abs(err) > self.gaze_distance_tolerance and abs(heading) < heading_gate:
                linear_v = self.gaze_kp_linear * err
                linear_v = min(self.gaze_max_forward, max(-self.gaze_max_reverse, linear_v))
                cmd.linear.x = linear_v

        if self.gaze_track_lateral:
            lateral_gate = math.radians(self.gaze_lateral_heading_gate_deg)
            if abs(heading) < lateral_gate and abs(py_base) > self.gaze_lateral_deadband:
                lateral_v = self.gaze_kp_lateral * py_base
                lateral_v = max(-self.gaze_max_lateral, min(self.gaze_max_lateral, lateral_v))
                cmd.linear.y = lateral_v

        # Check combined translational motion; degrade gracefully if needed.
        if not self._gaze_motion_allowed(rx, ry, robot_yaw, cmd.linear.x, cmd.linear.y):
            if abs(cmd.linear.y) > 1e-4 and self._gaze_motion_allowed(rx, ry, robot_yaw, 0.0, cmd.linear.y):
                cmd.linear.x = 0.0
            elif abs(cmd.linear.x) > 1e-4 and self._gaze_motion_allowed(rx, ry, robot_yaw, cmd.linear.x, 0.0):
                cmd.linear.y = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0

        self.search_cmd_pub.publish(cmd)
        self.gaze_cmd_active = True

    def _maybe_capture_gaze_stable_face(self, px_base, py_base, rx, ry, robot_yaw):
        if not self.gaze_stable_face_capture_enabled:
            self.gaze_stable_since = None
            return

        heading = math.atan2(py_base, max(px_base, 1e-4))
        dist = math.hypot(px_base, py_base)
        heading_ok = abs(heading) <= math.radians(max(self.gaze_yaw_deadband_deg, 2.0))
        dist_ok = abs(dist - self.gaze_target_distance) <= max(self.gaze_distance_tolerance, 0.08)

        now = rospy.Time.now()
        if heading_ok and dist_ok:
            if self.gaze_stable_since is None:
                self.gaze_stable_since = now
                return

            held = (now - self.gaze_stable_since).to_sec()
            if held < self.gaze_stable_face_hold_time:
                return

            if self.last_gaze_stable_capture_time != rospy.Time(0):
                if (now - self.last_gaze_stable_capture_time).to_sec() < self.gaze_stable_capture_min_interval:
                    return

            gx = rx + px_base * math.cos(robot_yaw) - py_base * math.sin(robot_yaw)
            gy = ry + px_base * math.sin(robot_yaw) + py_base * math.cos(robot_yaw)
            self._publish_serving_target_capture_request(gx, gy, "gaze_stable")
            self.last_gaze_stable_capture_time = now
            self.gaze_stable_since = now
            rospy.loginfo("Gaze stable, requested target face capture.")
            return

        self.gaze_stable_since = None

    def _run_gaze_tracking_cycle(self):
        if self.return_to_anchor_active:
            self.gaze_stable_since = None
            self._stop_gaze_tracking_cmd()
            return

        if not (self.gaze_tracking_on_pause and self.goal_publish_paused):
            self.gaze_stable_since = None
            return

        if self.latest_person_base_time == rospy.Time(0):
            self.gaze_stable_since = None
            self._stop_gaze_tracking_cmd()
            return

        if (rospy.Time.now() - self.latest_person_base_time).to_sec() > self.gaze_person_timeout:
            self.gaze_stable_since = None
            self._stop_gaze_tracking_cmd()
            return

        pose = self._lookup_robot_pose()
        if pose is None:
            self.gaze_stable_since = None
            self._stop_gaze_tracking_cmd()
            return

        rx, ry, robot_yaw = pose
        self._publish_gaze_tracking_cmd(
            self.latest_person_base_x,
            self.latest_person_base_y,
            rx,
            ry,
            robot_yaw,
        )
        self._maybe_capture_gaze_stable_face(
            self.latest_person_base_x,
            self.latest_person_base_y,
            rx,
            ry,
            robot_yaw,
        )

    def _update_reach_pause_state(self, rx, ry, px, py, px_base, py_base, now):
        if not self.stop_publish_on_reach:
            return False

        # Resume goal updates only when person has moved significantly.
        if self.goal_publish_paused:
            if self.paused_person_x is None or self.paused_person_y is None:
                return True

            moved = math.hypot(px - self.paused_person_x, py - self.paused_person_y)
            if moved >= self.person_reacquire_distance:
                self.goal_publish_paused = False
                self.goal_reach_since = None
                self._set_serving_customer_state("TRACKING")
                rospy.loginfo("Person moved globally %.2fm, resume goal publishing.", moved)
                return False

            # Base-frame criteria make pause exit more responsive for near-field motion.
            if (
                px_base is not None
                and py_base is not None
                and self.paused_person_base_x is not None
                and self.paused_person_base_y is not None
            ):
                forward_shift = abs(px_base - self.paused_person_base_x)
                lateral_shift = abs(py_base - self.paused_person_base_y)
                heading_now = math.atan2(py_base, max(px_base, 1e-4))
                heading_shift = 0.0
                if self.paused_person_heading is not None:
                    heading_shift = self._angle_diff_abs(heading_now, self.paused_person_heading)

                if (
                    forward_shift >= self.person_reacquire_forward
                    or lateral_shift >= self.person_reacquire_lateral
                    or heading_shift >= math.radians(self.person_reacquire_heading_deg)
                ):
                    self.goal_publish_paused = False
                    self.goal_reach_since = None
                    self._set_serving_customer_state("TRACKING")
                    rospy.loginfo(
                        "Person moved near-field (dx=%.2f, dy=%.2f, dtheta=%.1fdeg), resume goal publishing.",
                        forward_shift,
                        lateral_shift,
                        math.degrees(heading_shift),
                    )
                return False

            return True

        if self.last_goal_x is None or self.last_goal_y is None:
            self.goal_reach_since = None
            return False

        dist_to_goal = math.hypot(rx - self.last_goal_x, ry - self.last_goal_y)
        if dist_to_goal <= self.goal_reach_distance:
            if self.goal_reach_since is None:
                self.goal_reach_since = now
            held = (now - self.goal_reach_since).to_sec()
            if held >= self.goal_reach_hold_time:
                self.goal_publish_paused = True
                self.paused_person_x = px
                self.paused_person_y = py
                self.paused_person_base_x = px_base
                self.paused_person_base_y = py_base
                if px_base is not None and py_base is not None:
                    self.paused_person_heading = math.atan2(py_base, max(px_base, 1e-4))
                else:
                    self.paused_person_heading = None
                rospy.loginfo(
                    "Goal reached (dist=%.2fm, held=%.2fs). Pause goal publishing until person moves.",
                    dist_to_goal,
                    held,
                )
                self._record_serving_target_snapshot(px, py, px_base, py_base)
                self._announce_pause_prompt()
                self._trigger_pause_reply_listen()
                return True
        else:
            self.goal_reach_since = None

        return False

    def person_callback(self, msg):
        now = rospy.Time.now()
        self.last_person_time = now

        if self.return_to_anchor_active:
            self._stop_gaze_tracking_cmd()
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                msg.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.3),
            )
            person_in_global = tf2_geometry_msgs.do_transform_point(msg, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(1.0, "TF transform failed: %s", exc)
            return

        person_in_base = None
        try:
            if msg.header.frame_id == self.base_frame:
                person_in_base = msg
            else:
                base_tf = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    msg.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(0.2),
                )
                person_in_base = tf2_geometry_msgs.do_transform_point(msg, base_tf)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            person_in_base = None

        if person_in_base is not None:
            self.latest_person_base_x = person_in_base.point.x
            self.latest_person_base_y = person_in_base.point.y
            self.latest_person_base_time = now

        pose = self._lookup_robot_pose()
        if pose is None:
            return

        rx, ry, robot_yaw = pose
        px = person_in_global.point.x
        py = person_in_global.point.y
        px_base = None if person_in_base is None else person_in_base.point.x
        py_base = None if person_in_base is None else person_in_base.point.y

        stamp = now

        # If robot already reached a stable goal, stop publishing until person moves.
        if self._update_reach_pause_state(rx, ry, px, py, px_base, py_base, stamp):
            if not self.gaze_tracking_on_pause:
                self._stop_gaze_tracking_cmd()
            return

        self._stop_gaze_tracking_cmd()

        gx, gy, theta, cand_score, front_heading = self._select_goal(px, py, rx, ry, stamp)

        # Hysteresis: keep current valid goal unless candidate is clearly better.
        current_eval = self._evaluate_current_goal(px, py, rx, ry, front_heading)
        if current_eval is not None and cand_score is not None:
            _, _, cur_theta, cur_score = current_eval
            improvement = cur_score - cand_score
            ratio = improvement / max(cur_score, 1e-3)
            if improvement < self.switch_score_margin or ratio < self.switch_score_ratio:
                gx = self.last_goal_x
                gy = self.last_goal_y
                theta = cur_theta
                cand_score = cur_score

        now = rospy.Time.now()
        if (now - self.last_publish_time).to_sec() < self.min_publish_interval:
            return

        if self.last_goal_x is not None and self.last_goal_y is not None:
            move = math.hypot(gx - self.last_goal_x, gy - self.last_goal_y)
            if move < self.goal_update_distance:
                return

        self._publish_goal(gx, gy, theta)
        self.last_goal_score = cand_score

    def run(self):
        rate = rospy.Rate(self.run_rate_hz)
        try:
            while not rospy.is_shutdown():
                self._run_return_navigation_cycle()
                self._run_gaze_tracking_cycle()
                if (rospy.Time.now() - self.last_person_time).to_sec() > self.person_timeout:
                    rospy.logwarn_throttle(1.0, "Person target timeout, waiting for fresh detections.")
                    self._stop_gaze_tracking_cmd()
                if self.grid is None:
                    rospy.logwarn_throttle(2.0, "No occupancy grid yet on %s, using fallback goals.", self.map_topic)
                rate.sleep()
        finally:
            self._cleanup_pause_reply_asr()


if __name__ == "__main__":
    node = PersonGoalPublisher()
    node.run()
