#!/usr/bin/env python3
# person detection + raised-hand gesture + voice call detection

import json
import os
import threading
import time
import queue

import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from std_msgs.msg import String
from ultralytics import YOLO

try:
    import pyaudio
    from faster_whisper import WhisperModel
    import resampy
    from silero_vad import load_silero_vad, VADIterator
    VOICE_ENABLED = True
except ImportError:
    VOICE_ENABLED = False
    rospy.logwarn("Voice recognition dependencies not available. Voice detection disabled.")


class VoiceCallDetector(threading.Thread):
    """Background thread for continuous speech recognition and call detection."""
    def __init__(self, call_keywords=None, model_size="small", device="cpu"):
        super().__init__(daemon=True)
        self.call_keywords = call_keywords or ["waiter", "robot", "assistant"]
        self.call_queue = queue.Queue(maxsize=1)
        self.last_call_time = None
        self.model_size = model_size
        self.device = device
        self.is_running = False
        
        if not VOICE_ENABLED:
            rospy.logwarn("Voice recognition disabled: dependencies missing")
            return
            
        self.p = None
        self.stream = None
        self.whisper_model = None
        self.vad_model = None
        self.vad_iterator = None
        self._init_models()

    def _init_models(self):
        if not VOICE_ENABLED:
            return
        try:
            rospy.loginfo("Loading Whisper model (%s)...", self.model_size)
            self.whisper_model = WhisperModel(
                f"{self.model_size}.en",
                device=self.device,
                compute_type="int8"
            )
            rospy.loginfo("Whisper model loaded")
            
            rospy.loginfo("Loading Silero VAD model...")
            self.vad_model = load_silero_vad()
            self.vad_iterator = VADIterator(self.vad_model, threshold=0.5)
            rospy.loginfo("VAD model loaded")
        except Exception as e:
            rospy.logerr("Failed to load models: %s", e)
            VOICE_ENABLED = False

    def run(self):
        if not VOICE_ENABLED or self.whisper_model is None:
            return
        
        try:
            self.is_running = True
            self.p = pyaudio.PyAudio()
            
            # Find microphone
            device_index = self.p.get_default_input_device_info()["index"]
            device_info = self.p.get_device_info_by_index(device_index)
            device_rate = int(device_info.get("defaultSampleRate", 48000))
            
            self.stream = self.p.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=device_rate,
                input=True,
                frames_per_buffer=int(device_rate * 32 / 1000),
                input_device_index=device_index
            )
            
            rospy.loginfo("Voice detector listening...")
            target_rate = 16000
            frame_samples_16k = int(target_rate * 32 / 1000)
            buffer_16k = np.array([], dtype=np.float32)
            is_speaking = False
            
            while self.is_running:
                try:
                    data = self.stream.read(int(device_rate * 32 / 1000), exception_on_overflow=False)
                    audio_dev = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
                    
                    # Resample to 16k
                    audio_16k = resampy.resample(audio_dev, sr_orig=device_rate, sr_new=target_rate)
                    if len(audio_16k) > frame_samples_16k:
                        audio_16k = audio_16k[:frame_samples_16k]
                    elif len(audio_16k) < frame_samples_16k:
                        audio_16k = np.concatenate([audio_16k, np.zeros(frame_samples_16k - len(audio_16k))])
                    
                    speech_dict = self.vad_iterator(audio_16k, return_seconds=False)
                    
                    if speech_dict is not None:
                        if 'start' in speech_dict:
                            is_speaking = True
                            buffer_16k = np.array([], dtype=np.float32)
                        if 'end' in speech_dict:
                            is_speaking = False
                            if len(buffer_16k) > int(target_rate * 0.1):
                                self._transcribe(buffer_16k)
                    
                    if is_speaking:
                        buffer_16k = np.concatenate([buffer_16k, audio_16k])
                        
                except Exception as e:
                    rospy.logerr("Voice recognition error: %s", e)
                    break
                    
        except Exception as e:
            rospy.logerr("Voice detector initialization failed: %s", e)
        finally:
            self._cleanup()

    def _transcribe(self, audio):
        try:
            segments, _ = self.whisper_model.transcribe(
                audio.astype(np.float32),
                beam_size=5,
                language="en",
                without_timestamps=True
            )
            text = "".join(seg.text.strip() + " " for seg in segments).strip().lower()
            if text:
                rospy.loginfo("Recognized: %s", text)
                for kw in self.call_keywords:
                    if kw in text:
                        try:
                            self.call_queue.put_nowait(True)
                        except queue.Full:
                            pass
                        self.last_call_time = rospy.Time.now().to_sec()
                        rospy.loginfo("Call detected: %s", kw)
                        break
        except Exception as e:
            rospy.logwarn("Transcription error: %s", e)

    def check_call(self, timeout=1.0):
        try:
            self.call_queue.get_nowait()
            if self.last_call_time and rospy.Time.now().to_sec() - self.last_call_time <= timeout:
                return True
        except queue.Empty:
            pass
        return False

    def stop(self):
        self.is_running = False

    def _cleanup(self):
        try:
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
            if self.p:
                self.p.terminate()
        except:
            pass


class RobotController:
    def __init__(self, enabled=True):
        self.search_cmd_vel_topic = rospy.get_param("~search_cmd_vel_topic", "/person_following/search_cmd_vel")
        self.vel_pub = rospy.Publisher(self.search_cmd_vel_topic, Twist, queue_size=10)
        self.enabled = enabled
        self.is_searching = False
        self.search_start_time = None
        self.max_search_time = rospy.get_param("~max_search_time", 5.0)
        self.search_angular_speed = rospy.get_param("~search_angular_speed", 0.3)
        self.costmap_topic = rospy.get_param("~costmap_topic", "/person_following/occupancy_grid")
        self.costmap_timeout = rospy.get_param("~costmap_timeout", 1.0)
        self.costmap_obstacle_radius = rospy.get_param("~costmap_obstacle_radius", 1.4)
        self.costmap_occupied_threshold = rospy.get_param("~costmap_occupied_threshold", 55)
        self.search_direction = rospy.get_param("~search_direction", 1.0)
        self.default_search_direction = self.search_direction
        self.last_costmap = None
        self.last_costmap_time = None
        self.last_obstacle_state = False

        rospy.Subscriber(self.costmap_topic, OccupancyGrid, self.costmap_callback, queue_size=1)

    def costmap_callback(self, msg):
        self.last_costmap = msg
        self.last_costmap_time = rospy.Time.now()

    def _fresh_costmap(self):
        return (
            self.last_costmap is not None
            and self.last_costmap_time is not None
            and (rospy.Time.now() - self.last_costmap_time).to_sec() <= self.costmap_timeout
        )

    def _robot_center_index(self):
        if self.last_costmap is None:
            return None
        width = self.last_costmap.info.width
        height = self.last_costmap.info.height
        if width <= 0 or height <= 0:
            return None
        return width // 2, height // 2

    def _count_near_obstacles(self):
        if not self._fresh_costmap():
            return 0

        grid = self.last_costmap
        center = self._robot_center_index()
        if center is None:
            return 0

        cx, cy = center
        res = max(grid.info.resolution, 1e-3)
        radius_cells = max(1, int(self.costmap_obstacle_radius / res))
        width = grid.info.width
        height = grid.info.height
        occupied = 0

        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                idx = ny * width + nx
                val = grid.data[idx]
                if val < 0 or val >= self.costmap_occupied_threshold:
                    occupied += 1
        return occupied

    def _obstacle_ahead(self):
        # For a rolling local grid, near occupied cells around robot imply the search spin
        # could sweep into nearby obstacles. Use a conservative occupancy count threshold.
        return self._count_near_obstacles() > 20

    def _flip_search_direction_if_needed(self):
        obstacle_ahead = self._obstacle_ahead()
        if obstacle_ahead and not self.last_obstacle_state:
            self.search_direction *= -1.0
            self.last_obstacle_state = True
            return True
        if not obstacle_ahead:
            self.last_obstacle_state = False
            self.search_direction = self.default_search_direction
        return False

    def rotate_to_search(self):
        if not self.enabled:
            return False
        if not self.is_searching:
            self.is_searching = True
            self.search_start_time = time.time()

        # If the local costmap shows nearby obstacles, spin in the opposite direction.
        self._flip_search_direction_if_needed()

        twist = Twist()
        twist.angular.z = self.search_direction * self.search_angular_speed
        self.vel_pub.publish(twist)

        if time.time() - self.search_start_time > self.max_search_time:
            self.stop_rotation()
            return False
        return True

    def stop_rotation(self):
        if not self.enabled:
            return
        twist = Twist()
        self.vel_pub.publish(twist)
        self.is_searching = False


def initialize_realsense(width=640, height=480, fps=30):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    align = rs.align(rs.stream.color)
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    intrinsics = color_profile.get_intrinsics()
    return pipeline, align, depth_scale, intrinsics


def get_median_depth_in_roi(depth_frame, depth_scale, x1, y1, x2, y2):
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    x1 = max(0, int(x1))
    y1 = max(0, int(y1))
    x2 = min(width - 1, int(x2))
    y2 = min(height - 1, int(y2))

    if x2 <= x1 or y2 <= y1:
        return None

    depth_data = np.asanyarray(depth_frame.get_data())
    roi = depth_data[y1:y2, x1:x2]
    roi_meters = roi.astype(np.float32) * depth_scale
    valid_depths = roi_meters[(roi_meters > 0.1) & (roi_meters < 8.0)]
    if len(valid_depths) == 0:
        return None
    return float(np.median(valid_depths))


def get_3d_coordinates(depth_frame, intrinsics, pixel_x, pixel_y, depth_value):
    if depth_value is None or depth_value <= 0:
        return None
    if pixel_x < 0 or pixel_y < 0 or pixel_x >= intrinsics.width or pixel_y >= intrinsics.height:
        return None
    point = rs.rs2_deproject_pixel_to_point(intrinsics, [float(pixel_x), float(pixel_y)], float(depth_value))
    return np.array(point, dtype=np.float32)


def is_raised_hand(keypoints_xy, keypoints_conf, conf_th=0.35, y_margin=12.0):
    l_sh, r_sh, l_wr, r_wr = 5, 6, 9, 10

    left_ok = (
        keypoints_conf[l_sh] > conf_th
        and keypoints_conf[l_wr] > conf_th
        and keypoints_xy[l_wr][1] < (keypoints_xy[l_sh][1] - y_margin)
    )
    right_ok = (
        keypoints_conf[r_sh] > conf_th
        and keypoints_conf[r_wr] > conf_th
        and keypoints_xy[r_wr][1] < (keypoints_xy[r_sh][1] - y_margin)
    )
    return left_ok or right_ok


def body_center_from_keypoints_or_box(keypoints_xy, keypoints_conf, box_xyxy, conf_th=0.25):
    idx = [5, 6, 11, 12]
    pts = [keypoints_xy[i] for i in idx if keypoints_conf[i] > conf_th]
    if pts:
        pts = np.array(pts)
        return float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))
    x1, y1, x2, y2 = box_xyxy
    return float((x1 + x2) * 0.5), float((y1 + y2) * 0.5)


def match_track_id(tracks, cx, cy, now_sec, max_dist_px, track_timeout):
    best_id = None
    best_dist = 1e9
    for tid, t in tracks.items():
        if now_sec - t["last_seen"] > track_timeout:
            continue
        dx = cx - t["cx"]
        dy = cy - t["cy"]
        dist = (dx * dx + dy * dy) ** 0.5
        if dist < best_dist and dist <= max_dist_px:
            best_dist = dist
            best_id = tid
    return best_id


def choose_rightmost(candidates):
    if not candidates:
        return None
    return max(candidates, key=lambda c: c["cx"])


def choose_nearest(candidates):
    """Choose person closest to camera (smallest depth)."""
    if not candidates:
        return None
    return min(candidates, key=lambda c: c["depth"])


def main():
    rospy.init_node("person_detection_with_voice")

    model_path = rospy.get_param("~model_path", "yolov8n-pose.pt")
    detect_conf = rospy.get_param("~detect_conf", 0.35)
    person_topic = rospy.get_param("~person_topic", "/person/base_link_3d_position")
    frame_id = rospy.get_param("~frame_id", "base_link")
    show_debug = rospy.get_param("~show_debug", True)
    enable_search_rotation = rospy.get_param("~enable_search_rotation", True)
    track_match_px = rospy.get_param("~track_match_px", 80.0)
    lock_match_px = rospy.get_param("~lock_match_px", 120.0)
    track_timeout = rospy.get_param("~track_timeout", 1.0)
    call_timeout = rospy.get_param("~call_timeout", 1.0)
    enable_voice = rospy.get_param("~enable_voice", True)
    whisper_model = rospy.get_param("~whisper_model", "small")

    # Record first trigger location (voice call or raised hand) as return anchor.
    return_anchor_enabled = rospy.get_param("~return_anchor_enabled", True)
    return_anchor_topic = rospy.get_param("~return_anchor_topic", "/person_following/return_anchor")
    return_anchor_frame = rospy.get_param("~return_anchor_frame", "map")
    return_anchor_base_frame = rospy.get_param("~return_anchor_base_frame", frame_id)
    return_anchor_lookup_timeout = rospy.get_param("~return_anchor_lookup_timeout", 0.2)
    default_return_anchor_file = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "return_anchor.json")
    )
    return_anchor_json_file = rospy.get_param("~return_anchor_json_file", default_return_anchor_file)

    # Capture target face when pause-state capture request arrives.
    serving_target_capture_topic = rospy.get_param(
        "~serving_target_capture_topic",
        "/person_following/serving_target_capture",
    )
    default_serving_target_face_image_file = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "serving_target_face.jpg")
    )
    serving_target_face_image_file = rospy.get_param(
        "~serving_target_face_image_file",
        default_serving_target_face_image_file,
    )
    default_serving_target_face_meta_file = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "serving_target_face_meta.json")
    )
    serving_target_face_meta_file = rospy.get_param(
        "~serving_target_face_meta_file",
        default_serving_target_face_meta_file,
    )
    serving_target_face_top_ratio = rospy.get_param("~serving_target_face_top_ratio", 0.50)
    serving_target_capture_timeout = rospy.get_param("~serving_target_capture_timeout", 2.0)
    serving_target_capture_cmd_topic = rospy.get_param(
        "~serving_target_capture_cmd_topic",
        "/person_following/serving_target_capture_cmd",
    )

    customer_data_root = rospy.get_param(
        "~customer_data_root",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "service_customers")),
    )
    active_customer_folder_topic = rospy.get_param(
        "~active_customer_folder_topic",
        "/person_following/active_customer_folder",
    )
    serving_customer_state_topic = rospy.get_param(
        "~serving_customer_state_topic",
        "/person_following/serving_customer_state",
    )

    cam_to_base_x = rospy.get_param("~cam_to_base_x", 0.0)
    cam_to_base_y = rospy.get_param("~cam_to_base_y", 0.0)
    cam_to_base_z = rospy.get_param("~cam_to_base_z", 0.0)

    pub = rospy.Publisher(person_topic, PointStamped, queue_size=1)
    active_customer_folder_pub = rospy.Publisher(active_customer_folder_topic, String, queue_size=1, latch=True)
    serving_customer_state_pub = rospy.Publisher(serving_customer_state_topic, String, queue_size=1, latch=True)
    return_anchor_pub = None
    if return_anchor_enabled and return_anchor_topic:
        return_anchor_pub = rospy.Publisher(return_anchor_topic, PoseStamped, queue_size=1, latch=True)

    tf_buffer = None
    tf_listener = None
    if return_anchor_enabled:
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

    controller = RobotController(enabled=enable_search_rotation)

    voice_detector = None
    if enable_voice and VOICE_ENABLED:
        voice_detector = VoiceCallDetector(model_size=whisper_model)
        voice_detector.start()
        rospy.loginfo("Voice call detector started")

    model = YOLO(model_path)
    pipeline, align, depth_scale, intrinsics = initialize_realsense()

    rate = rospy.Rate(15)
    rospy.loginfo("Person detector (raised-hand + voice call) started, model=%s", model_path)

    tracks = {}
    next_track_id = 1
    locked_track_id = None
    return_anchor_recorded = False
    return_anchor_trigger_reason = None
    return_anchor_trigger_track_id = None
    active_customer_folder = ""
    active_customer_id = ""
    active_customer_no = ""
    serving_customer_state = "IDLE"
    first_lock_face_saved = False
    serving_target_capture_pending = False
    serving_target_capture_request = None
    serving_target_capture_time = None

    def _try_record_return_anchor(reason, track_id):
        nonlocal return_anchor_recorded
        if not return_anchor_enabled or return_anchor_recorded:
            return False
        if tf_buffer is None:
            return False

        try:
            tf_msg = tf_buffer.lookup_transform(
                return_anchor_frame,
                return_anchor_base_frame,
                rospy.Time(0),
                rospy.Duration(return_anchor_lookup_timeout),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(1.0, "Return anchor TF lookup failed: %s", exc)
            return False

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = return_anchor_frame
        pose.pose.position.x = tf_msg.transform.translation.x
        pose.pose.position.y = tf_msg.transform.translation.y
        pose.pose.position.z = tf_msg.transform.translation.z
        pose.pose.orientation.x = tf_msg.transform.rotation.x
        pose.pose.orientation.y = tf_msg.transform.rotation.y
        pose.pose.orientation.z = tf_msg.transform.rotation.z
        pose.pose.orientation.w = tf_msg.transform.rotation.w

        if return_anchor_pub is not None:
            return_anchor_pub.publish(pose)

        payload = {
            "timestamp": pose.header.stamp.to_sec(),
            "frame_id": pose.header.frame_id,
            "trigger": reason,
            "track_id": int(track_id) if track_id is not None else None,
            "position": {
                "x": pose.pose.position.x,
                "y": pose.pose.position.y,
                "z": pose.pose.position.z,
            },
            "orientation": {
                "x": pose.pose.orientation.x,
                "y": pose.pose.orientation.y,
                "z": pose.pose.orientation.z,
                "w": pose.pose.orientation.w,
            },
        }

        if return_anchor_json_file:
            try:
                parent = os.path.dirname(return_anchor_json_file)
                if parent:
                    os.makedirs(parent, exist_ok=True)
                tmp_path = return_anchor_json_file + ".tmp"
                with open(tmp_path, "w", encoding="utf-8") as fp:
                    json.dump(payload, fp, indent=2, ensure_ascii=False)
                os.replace(tmp_path, return_anchor_json_file)
            except Exception as exc:
                rospy.logwarn("Failed to write return anchor JSON: %s", exc)

        return_anchor_recorded = True
        rospy.loginfo(
            "Return anchor recorded at (%.2f, %.2f, %.2f), trigger=%s, track_id=%s",
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            reason,
            str(track_id),
        )
        return True

    def _publish_serving_customer_state(state, extra=None):
        nonlocal serving_customer_state
        nonlocal active_customer_no
        serving_customer_state = str(state)

        folder_no = ""
        if active_customer_folder and os.path.isdir(active_customer_folder):
            folder_no = os.path.basename(active_customer_folder.rstrip("/"))
        if folder_no:
            active_customer_no = folder_no
        elif active_customer_id and not active_customer_no:
            active_customer_no = str(active_customer_id)

        payload = {
            "timestamp": rospy.Time.now().to_sec(),
            "state": serving_customer_state,
            "customer_id": active_customer_id,
            "customer_no": active_customer_no,
            "folder": active_customer_folder,
        }
        if isinstance(extra, dict):
            payload.update(extra)

        try:
            serving_customer_state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        except Exception:
            pass

        if active_customer_folder and os.path.isdir(active_customer_folder):
            state_path = os.path.join(active_customer_folder, "customer_service_state.json")
            try:
                tmp_path = state_path + ".tmp"
                with open(tmp_path, "w", encoding="utf-8") as fp:
                    json.dump(payload, fp, indent=2, ensure_ascii=False)
                os.replace(tmp_path, state_path)
            except Exception:
                pass

    def _ensure_customer_folder(reason, track_id):
        nonlocal active_customer_folder
        nonlocal active_customer_id
        nonlocal active_customer_no

        if active_customer_folder and os.path.isdir(active_customer_folder):
            return active_customer_folder

        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        customer_id = "customer_%s_%s" % (ts, str(track_id))
        folder = os.path.join(customer_data_root, customer_id)
        try:
            os.makedirs(os.path.join(folder, "faces"), exist_ok=True)
        except Exception as exc:
            rospy.logwarn("Failed to create customer folder: %s", exc)
            return ""

        active_customer_folder = folder
        active_customer_id = customer_id
        active_customer_no = os.path.basename(folder.rstrip("/"))

        profile = {
            "timestamp": rospy.Time.now().to_sec(),
            "customer_id": active_customer_id,
            "customer_no": active_customer_no,
            "trigger": reason,
            "track_id": int(track_id) if track_id is not None else None,
            "folder": active_customer_folder,
        }
        profile_path = os.path.join(active_customer_folder, "customer_profile.json")
        try:
            with open(profile_path, "w", encoding="utf-8") as fp:
                json.dump(profile, fp, indent=2, ensure_ascii=False)
        except Exception as exc:
            rospy.logwarn("Failed to write customer profile: %s", exc)

        try:
            active_customer_folder_pub.publish(String(data=active_customer_folder))
        except Exception:
            pass

        _publish_serving_customer_state("LOCKED", {"reason": reason})
        rospy.loginfo("Created service customer folder: %s", active_customer_folder)
        return active_customer_folder

    def _face_paths(reason):
        safe_reason = str(reason).strip().lower().replace(" ", "_")
        if not safe_reason:
            safe_reason = "capture"
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())

        if active_customer_folder and os.path.isdir(active_customer_folder):
            face_dir = os.path.join(active_customer_folder, "faces")
            os.makedirs(face_dir, exist_ok=True)
            image_path = os.path.join(face_dir, "%s_%s.jpg" % (stamp, safe_reason))
            meta_path = os.path.join(face_dir, "%s_%s.json" % (stamp, safe_reason))
            return image_path, meta_path

        return serving_target_face_image_file, serving_target_face_meta_file

    def _save_customer_face_snapshot(color_img, box, track_id, x_base, y_base, z_base, reason):
        nonlocal serving_target_capture_pending
        nonlocal serving_target_capture_request
        nonlocal serving_target_capture_time

        if color_img is None or box is None:
            return False

        h, w = color_img.shape[:2]
        x1, y1, x2, y2 = [int(v) for v in box]
        x1 = max(0, min(x1, w - 1))
        x2 = max(0, min(x2, w - 1))
        y1 = max(0, min(y1, h - 1))
        y2 = max(0, min(y2, h - 1))
        if x2 <= x1 or y2 <= y1:
            rospy.logwarn("Serving target face capture failed: invalid bbox")
            return False

        top_ratio = min(0.9, max(0.2, float(serving_target_face_top_ratio)))
        head_y2 = y1 + int((y2 - y1) * top_ratio)
        head_y2 = max(y1 + 1, min(head_y2, y2))

        face_crop = color_img[y1:head_y2, x1:x2].copy()
        if face_crop.size == 0:
            face_crop = color_img[y1:y2, x1:x2].copy()
        if face_crop.size == 0:
            rospy.logwarn("Serving target face capture failed: empty crop")
            return False

        try:
            image_path, meta_path = _face_paths(reason)
            image_parent = os.path.dirname(image_path)
            if image_parent:
                os.makedirs(image_parent, exist_ok=True)
            ok = cv2.imwrite(image_path, face_crop)
            if not ok:
                rospy.logwarn("Failed to write serving target face image")
                return False

            meta = {
                "timestamp": rospy.Time.now().to_sec(),
                "customer_id": active_customer_id,
                "track_id": int(track_id) if track_id is not None else None,
                "image_path": image_path,
                "reason": reason,
                "bbox": {
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                },
                "face_bbox": {
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": head_y2,
                },
                "person_base": {
                    "frame_id": frame_id,
                    "x": float(x_base),
                    "y": float(y_base),
                    "z": float(z_base),
                },
                "capture_request": serving_target_capture_request,
            }

            meta_parent = os.path.dirname(meta_path)
            if meta_parent:
                os.makedirs(meta_parent, exist_ok=True)
            tmp_meta = meta_path + ".tmp"
            with open(tmp_meta, "w", encoding="utf-8") as fp:
                json.dump(meta, fp, indent=2, ensure_ascii=False)
            os.replace(tmp_meta, meta_path)

            serving_target_capture_pending = False
            serving_target_capture_request = None
            serving_target_capture_time = None
            _publish_serving_customer_state("TRACKING", {"last_face_reason": reason})
            rospy.loginfo("Serving target face saved: %s", image_path)
            return True
        except Exception as exc:
            rospy.logwarn("Serving target face save failed: %s", exc)
            return False

    def _serving_target_capture_callback(msg):
        nonlocal serving_target_capture_pending
        nonlocal serving_target_capture_request
        nonlocal serving_target_capture_time

        serving_target_capture_pending = True
        serving_target_capture_request = {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id,
            "x": msg.point.x,
            "y": msg.point.y,
            "z": msg.point.z,
            "reason": "pause_enter",
        }
        serving_target_capture_time = rospy.Time.now()
        rospy.loginfo(
            "Serving target capture requested at (%.2f, %.2f) in %s",
            msg.point.x,
            msg.point.y,
            msg.header.frame_id,
        )

    def _serving_target_capture_cmd_callback(msg):
        nonlocal serving_target_capture_pending
        nonlocal serving_target_capture_request
        nonlocal serving_target_capture_time
        nonlocal active_customer_folder
        nonlocal active_customer_id
        nonlocal active_customer_no

        raw = str(msg.data).strip()
        if not raw:
            return

        payload = None
        try:
            payload = json.loads(raw)
        except Exception:
            payload = None

        reason = "capture"
        frame = frame_id
        x = 0.0
        y = 0.0
        z = 0.0

        if isinstance(payload, dict):
            reason = str(payload.get("reason", "capture"))
            pos = payload.get("person_global", {})
            if isinstance(pos, dict):
                try:
                    x = float(pos.get("x", 0.0))
                    y = float(pos.get("y", 0.0))
                    z = float(pos.get("z", 0.0))
                except Exception:
                    x, y, z = 0.0, 0.0, 0.0
            frame = str(payload.get("frame_id", frame_id))

            folder = str(payload.get("customer_folder", "")).strip()
            if folder and os.path.isdir(folder):
                active_customer_folder = folder
                active_customer_id = os.path.basename(folder.rstrip("/"))
                active_customer_no = active_customer_id

            cid = str(payload.get("customer_id", "")).strip()
            if cid and not active_customer_id:
                active_customer_id = cid
            if not active_customer_no and active_customer_id:
                active_customer_no = str(active_customer_id)

        serving_target_capture_pending = True
        serving_target_capture_request = {
            "stamp": rospy.Time.now().to_sec(),
            "frame_id": frame,
            "x": x,
            "y": y,
            "z": z,
            "reason": reason,
        }
        serving_target_capture_time = rospy.Time.now()
        rospy.loginfo("Serving target capture cmd received: reason=%s", reason)

    def _serving_customer_state_sync_callback(msg):
        nonlocal locked_track_id
        nonlocal active_customer_folder
        nonlocal active_customer_id
        nonlocal active_customer_no
        nonlocal serving_customer_state
        nonlocal first_lock_face_saved
        nonlocal serving_target_capture_pending
        nonlocal serving_target_capture_request
        nonlocal serving_target_capture_time

        raw = str(msg.data or "").strip()
        if not raw:
            return

        payload = None
        try:
            payload = json.loads(raw)
        except Exception:
            payload = None

        state = ""
        if isinstance(payload, dict):
            state = str(payload.get("state", "")).strip().upper()
            folder = str(payload.get("folder", "")).strip()
            cid = str(payload.get("customer_id", "")).strip()
            cno = str(payload.get("customer_no", "")).strip()

            if folder and os.path.isdir(folder):
                active_customer_folder = folder
                active_customer_no = os.path.basename(folder.rstrip("/"))
                if cid:
                    active_customer_id = cid
                elif active_customer_no:
                    active_customer_id = active_customer_no
            elif cid and not active_customer_id:
                active_customer_id = cid

            if cno:
                active_customer_no = cno
            elif not active_customer_no and active_customer_id:
                active_customer_no = str(active_customer_id)
        else:
            state = raw.upper()

        if state:
            serving_customer_state = state

        if state != "IDLE":
            return

        locked_track_id = None
        first_lock_face_saved = False
        serving_target_capture_pending = False
        serving_target_capture_request = None
        serving_target_capture_time = None
        active_customer_folder = ""
        active_customer_id = ""
        active_customer_no = ""
        rospy.loginfo_throttle(2.0, "Serving state switched to IDLE, unlock detector for next customer.")

    rospy.Subscriber(
        serving_target_capture_topic,
        PointStamped,
        _serving_target_capture_callback,
        queue_size=1,
    )
    rospy.Subscriber(
        serving_target_capture_cmd_topic,
        String,
        _serving_target_capture_cmd_callback,
        queue_size=1,
    )
    rospy.Subscriber(
        serving_customer_state_topic,
        String,
        _serving_customer_state_sync_callback,
        queue_size=5,
    )

    try:
        while not rospy.is_shutdown():
            if (
                return_anchor_enabled
                and not return_anchor_recorded
                and return_anchor_trigger_reason is not None
            ):
                _try_record_return_anchor(return_anchor_trigger_reason, return_anchor_trigger_track_id)

            if serving_target_capture_pending and serving_target_capture_time is not None:
                if (rospy.Time.now() - serving_target_capture_time).to_sec() > serving_target_capture_timeout:
                    rospy.logwarn("Serving target face capture request timeout")
                    serving_target_capture_pending = False
                    serving_target_capture_request = None
                    serving_target_capture_time = None

            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                rate.sleep()
                continue

            color_img = np.asanyarray(color_frame.get_data())
            results = model.predict(color_img, conf=detect_conf, verbose=False)

            all_candidates = []
            raised_candidates = []
            now_sec = rospy.Time.now().to_sec()

            for res in results:
                if res.boxes is None or res.keypoints is None:
                    continue
                boxes = res.boxes.xyxy.cpu().numpy()
                kxy = res.keypoints.xy.cpu().numpy()
                kcf = res.keypoints.conf.cpu().numpy()
                cls = res.boxes.cls.cpu().numpy().astype(int)

                for i in range(len(boxes)):
                    if cls[i] != 0:
                        continue

                    cx, cy = body_center_from_keypoints_or_box(kxy[i], kcf[i], boxes[i])
                    roi_half = 12
                    depth = get_median_depth_in_roi(
                        depth_frame,
                        depth_scale,
                        cx - roi_half,
                        cy - roi_half,
                        cx + roi_half,
                        cy + roi_half,
                    )
                    if depth is None:
                        continue

                    tid = match_track_id(tracks, cx, cy, now_sec, track_match_px, track_timeout)
                    if tid is None:
                        tid = next_track_id
                        next_track_id += 1
                        tracks[tid] = {
                            "cx": cx,
                            "cy": cy,
                            "last_seen": now_sec,
                        }
                    else:
                        tracks[tid]["cx"] = cx
                        tracks[tid]["cy"] = cy
                        tracks[tid]["last_seen"] = now_sec

                    item = {
                        "track_id": tid,
                        "cx": cx,
                        "cy": cy,
                        "box": boxes[i],
                        "depth": depth,
                    }
                    all_candidates.append(item)
                    if is_raised_hand(kxy[i], kcf[i]):
                        raised_candidates.append(item)

            stale_ids = [tid for tid, t in tracks.items() if now_sec - t["last_seen"] > track_timeout]
            for tid in stale_ids:
                del tracks[tid]

            selected = None
            if locked_track_id is None:
                voice_triggered = voice_detector and voice_detector.check_call(call_timeout)
                if raised_candidates:
                    selected = choose_rightmost(raised_candidates)
                    rospy.loginfo("Locked raised-hand target: track_id=%d", selected["track_id"])
                    locked_track_id = selected["track_id"]
                    _ensure_customer_folder("raised_hand", selected["track_id"])
                    first_lock_face_saved = False
                    if return_anchor_trigger_reason is None:
                        return_anchor_trigger_reason = "raised_hand"
                        return_anchor_trigger_track_id = selected["track_id"]
                        _try_record_return_anchor(return_anchor_trigger_reason, return_anchor_trigger_track_id)
                elif voice_triggered and all_candidates:
                    selected = choose_nearest(all_candidates)
                    rospy.loginfo("Locked voice-call target (nearest): track_id=%d", selected["track_id"])
                    locked_track_id = selected["track_id"]
                    _ensure_customer_folder("voice_call", selected["track_id"])
                    first_lock_face_saved = False
                    if return_anchor_trigger_reason is None:
                        return_anchor_trigger_reason = "voice_call"
                        return_anchor_trigger_track_id = selected["track_id"]
                        _try_record_return_anchor(return_anchor_trigger_reason, return_anchor_trigger_track_id)
            else:
                for c in all_candidates:
                    if c["track_id"] == locked_track_id:
                        selected = c
                        break

                if selected is None and locked_track_id in tracks:
                    last = tracks[locked_track_id]
                    best = None
                    best_d = 1e9
                    for c in all_candidates:
                        dx = c["cx"] - last["cx"]
                        dy = c["cy"] - last["cy"]
                        d = (dx * dx + dy * dy) ** 0.5
                        if d < best_d and d <= lock_match_px:
                            best = c
                            best_d = d
                    if best is not None:
                        selected = best
                        tracks[locked_track_id]["cx"] = best["cx"]
                        tracks[locked_track_id]["cy"] = best["cy"]
                        tracks[locked_track_id]["last_seen"] = now_sec

            if selected is None:
                controller.rotate_to_search()
                if show_debug:
                    cv2.imshow("person_detector", color_img)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                rate.sleep()
                continue

            controller.stop_rotation()

            cx = selected["cx"]
            cy = selected["cy"]
            box = selected["box"]
            depth = selected["depth"]
            track_id = locked_track_id if locked_track_id is not None else selected["track_id"]
            p_cam = get_3d_coordinates(depth_frame, intrinsics, cx, cy, depth)
            if p_cam is None:
                rate.sleep()
                continue

            x_base = float(p_cam[2] + cam_to_base_x)
            y_base = float(-p_cam[0] + cam_to_base_y)
            z_base = float(-p_cam[1] + cam_to_base_z)

            msg = PointStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_id
            msg.point.x = x_base
            msg.point.y = y_base
            msg.point.z = z_base
            pub.publish(msg)

            if (not first_lock_face_saved) and active_customer_folder:
                if _save_customer_face_snapshot(color_img, box, track_id, x_base, y_base, z_base, "first_lock"):
                    first_lock_face_saved = True

            if serving_target_capture_pending:
                reason = "capture"
                if isinstance(serving_target_capture_request, dict):
                    reason = str(serving_target_capture_request.get("reason", "capture"))
                _save_customer_face_snapshot(color_img, box, track_id, x_base, y_base, z_base, reason)

            if show_debug:
                x1, y1, x2, y2 = [int(v) for v in box]
                cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(color_img, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                text = "LOCKED ID {} d={:.2f}m".format(track_id, depth)
                cv2.putText(color_img, text, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("person_detector", color_img)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            rate.sleep()

    finally:
        pipeline.stop()
        controller.stop_rotation()
        if voice_detector:
            voice_detector.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
