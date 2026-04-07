#!/usr/bin/env python3
# person detection + raised-hand gesture filtering + 3D target publishing

import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import PointStamped, Twist
from ultralytics import YOLO


class RobotController:
    def __init__(self):
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.is_searching = False
        self.search_start_time = None
        self.max_search_time = rospy.get_param("~max_search_time", 5.0)
        self.search_angular_speed = rospy.get_param("~search_angular_speed", 0.3)

    def rotate_to_search(self):
        if not self.is_searching:
            self.is_searching = True
            self.search_start_time = time.time()

        twist = Twist()
        twist.angular.z = self.search_angular_speed
        self.vel_pub.publish(twist)

        if time.time() - self.search_start_time > self.max_search_time:
            self.stop_rotation()
            return False
        return True

    def stop_rotation(self):
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
    # COCO keypoints: 5=LShoulder, 6=RShoulder, 9=LWrist, 10=RWrist
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
    # Prefer shoulder/hip center, fallback to bbox center.
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


def main():
    rospy.init_node("person_detection_raised_hand")

    model_path = rospy.get_param("~model_path", "yolov8n-pose.pt")
    detect_conf = rospy.get_param("~detect_conf", 0.35)
    person_topic = rospy.get_param("~person_topic", "/person/base_link_3d_position")
    frame_id = rospy.get_param("~frame_id", "base_link")
    show_debug = rospy.get_param("~show_debug", True)
    track_match_px = rospy.get_param("~track_match_px", 80.0)
    lock_match_px = rospy.get_param("~lock_match_px", 120.0)
    track_timeout = rospy.get_param("~track_timeout", 1.0)

    # Approximate camera-optical to base_link conversion offsets (meters)
    cam_to_base_x = rospy.get_param("~cam_to_base_x", 0.0)
    cam_to_base_y = rospy.get_param("~cam_to_base_y", 0.0)
    cam_to_base_z = rospy.get_param("~cam_to_base_z", 0.0)

    pub = rospy.Publisher(person_topic, PointStamped, queue_size=1)
    controller = RobotController()

    model = YOLO(model_path)
    pipeline, align, depth_scale, intrinsics = initialize_realsense()

    rate = rospy.Rate(15)
    rospy.loginfo("Raised-hand detector started, model=%s", model_path)

    tracks = {}
    next_track_id = 1
    locked_track_id = None

    try:
        while not rospy.is_shutdown():
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
                    if cls[i] != 0:  # person only
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
                selected = choose_rightmost(raised_candidates)
                if selected is not None:
                    locked_track_id = selected["track_id"]
                    rospy.loginfo("Locked raised-hand target: track_id=%d", locked_track_id)
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
                    cv2.imshow("raised_hand_detection", color_img)
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

            # Convert from camera optical frame to base_link-like coordinates.
            # camera: x right, y down, z forward
            # base_link-like: x forward, y left, z up
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

            if show_debug:
                x1, y1, x2, y2 = [int(v) for v in box]
                cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(color_img, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                text = "LOCKED ID {} d={:.2f}m".format(track_id, depth)
                cv2.putText(color_img, text, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("raised_hand_detection", color_img)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            rate.sleep()

    finally:
        pipeline.stop()
        controller.stop_rotation()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
