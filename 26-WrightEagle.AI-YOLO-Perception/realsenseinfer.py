# realsenseinfer.py
import time
import cv2
import numpy as np
import pyrealsense2 as rs
import json

from ultralytics import YOLO
MODEL_PATH = "yolov12x-seg.pt"     # ← 换成你的分割模型路径
CONF_THRES = 0.5
IOU_THRES = 0.45

# 目标距离的鲁棒统计参数
MAX_VALID_DIST = 10.0      # 过滤 >10m 的深度点（可按需要调整）
MIN_MASK_PIX = 200         # 掩膜像素太少则忽略（避免噪点）

# 可选：限制分辨率与 FPS 以保证实时性
COLOR_W, COLOR_H, FPS = 640, 480, 30

def main():
    # 1) 初始化 RealSense 管线与对齐
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, COLOR_W, COLOR_H, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)

    profile = pipeline.start(config)

    # 获取深度尺度（以米为单位）
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()  # 例如 0.001
    print(f"[Info] Depth scale: {depth_scale} meters per unit")

    color_stream = profile.get_stream(rs.stream.color)
    intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
    # 将深度对齐到彩色
    align = rs.align(rs.stream.color)

    # 2) 加载 YOLO 分割模型
    print("[Info] Loading YOLO model...")
    model = YOLO(MODEL_PATH)
    print("[Info] Model loaded.")

    fps_t0, fps_cnt = time.time(), 0
    fps_val = 0.0

    frame_id = 0

    try:
        while True:
            # 3) 获取并对齐帧
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color = np.asanyarray(color_frame.get_data())  # HxWx3, BGR
            depth = np.asanyarray(depth_frame.get_data())  # HxW, uint16
            depth_m = depth * depth_scale                   # 转成米

            # 4) YOLO 推理（分割）
            # 如果你的显卡负载高，可设置 imgsz、半精度等参数（参考 ultralytics 文档）
            results = model.predict(
                color,
                conf=CONF_THRES,
                iou=IOU_THRES,
                stream=False,
                verbose=False,
            )

            annotated = color.copy()
            detected_objects = []

            if len(results) > 0:
                r = results[0]
                names = r.names if hasattr(r, "names") else {}

                if r.masks is None:
                    # 仅检测框
                    for box in r.boxes:
                        xyxy = box.xyxy[0].cpu().numpy().astype(int)
                        x1, y1, x2, y2 = xyxy
                        cls = int(box.cls[0].cpu().numpy()) if box.cls is not None else -1
                        conf = float(box.conf[0].cpu().numpy()) if box.conf is not None else 0.0

                        cx = int((x1 + x2) / 2)
                        cy = int((y1 + y2) / 2)
                        cx = np.clip(cx, 0, COLOR_W - 1)
                        cy = np.clip(cy, 0, COLOR_H - 1)

                        dist = depth_m[cy, cx]
                        label = names.get(cls, str(cls))

                        # 像素转相机坐标
                        X, Y, Z = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], dist)


                        detected_objects.append({
                            "label": label,
                            "conf": conf,
                            "xyz": (X, Y, Z),
                        })
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        put_text(annotated, f"{label} {conf:.2f} | {dist:.2f}m", (x1, max(0, y1 - 8)))

                else:
                    # 分割掩膜
                    masks = r.masks.data.cpu().numpy()
                    orig_h, orig_w = r.masks.orig_shape

                    boxes = r.boxes
                    classes = boxes.cls.cpu().numpy().astype(int) if boxes is not None else None
                    confs = boxes.conf.cpu().numpy() if boxes is not None else None

                    for i, m in enumerate(masks):
                        mask = cv2.resize(m, (orig_w, orig_h), interpolation=cv2.INTER_NEAREST)
                        mask_bin = mask > 0.5
                        if mask_bin.sum() < MIN_MASK_PIX:
                            continue

                        region_depth = depth_m[mask_bin]
                        valid = region_depth[(region_depth > 0) & (region_depth < MAX_VALID_DIST)]
                        if valid.size == 0:
                            continue
                        dist_med = float(np.median(valid))

                        ys, xs = np.nonzero(mask_bin)
                        cx, cy = int(xs.mean()), int(ys.mean())

                        # 转相机坐标
                        X, Y, Z = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], dist_med)

                        # 获取外接矩形框（便于估计物体大小）
                        x, y, w, h = cv2.boundingRect(mask_bin.astype(np.uint8))

                        label = "obj"
                        conf = None
                        if classes is not None and i < len(classes):
                            label = names.get(int(classes[i]), str(int(classes[i])))
                        if confs is not None and i < len(confs):
                            conf = float(confs[i])

                        detected_objects.append({
                            "label": label,
                            "conf": conf,
                            "xyz": (X, Y, Z),
                            "box": {"x": x, "y": y, "w": w, "h": h}
                        })

                        txt = f"{label}"
                        if conf is not None:
                            txt += f" {conf:.2f}"
                        txt += f" | {dist_med:.2f} m"

                        contours, _ = cv2.findContours(mask_bin.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(annotated, contours, -1, (0, 255, 0), 2)
                        put_text(annotated, txt, (cx, cy))

            # === 保存到 JSON 文件 ===
            frame_id += 1
            if detected_objects:
                out = {
                    "frame_id": frame_id,
                    # 使用 RealSense 帧时间戳（毫秒）
                    "timestamp_ms": color_frame.get_timestamp(),
                    "objects": detected_objects
                }
                with open("detections.json", "a", encoding="utf-8") as f:
                    f.write(json.dumps(out, ensure_ascii=False) + "\n")

                #print("Detected objects:", out)

            # 5) 计算并显示 FPS
            fps_cnt += 1
            if fps_cnt >= 10:
                now = time.time()
                fps_val = fps_cnt / (now - fps_t0)
                fps_t0, fps_cnt = now, 0
            put_text(annotated, f"FPS: {fps_val:.1f}", (10, 25))

            cv2.imshow("YOLO Seg + RealSense Depth (meters)", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # q 或 ESC 退出
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

def put_text(img, text, org):
    # 统一的文字绘制（带描边更清晰）
    x, y = org
    cv2.putText(img, text, (x+1, y+1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (x, y),     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

if __name__ == "__main__":
    main()
