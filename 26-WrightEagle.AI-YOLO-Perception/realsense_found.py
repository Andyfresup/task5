# realsense_found.py
import cv2
import numpy as np
from ultralytics import YOLO
from sentence_transformers import SentenceTransformer
import time
import pyrealsense2 as rs

# -----------------------------
# SBERT 文本相似度
# -----------------------------
sbert_model = SentenceTransformer("all-MiniLM-L6-v2")
embedding_cache = {}

def text_similarity(a, b):
    if a not in embedding_cache:
        embedding_cache[a] = sbert_model.encode(a, convert_to_numpy=True)
    if b not in embedding_cache:
        embedding_cache[b] = sbert_model.encode(b, convert_to_numpy=True)
    emb_a = embedding_cache[a]
    emb_b = embedding_cache[b]
    sim = float(np.dot(emb_a, emb_b) / (np.linalg.norm(emb_a)*np.linalg.norm(emb_b)))
    return max(0.0, min(1.0, sim))

# -----------------------------
# 提取物体信息
# -----------------------------
def extract_objects(result):
    objs = []
    names = result.names
    for i, box in enumerate(result.boxes):
        cls_id = int(box.cls[0])
        name = names[cls_id] if isinstance(names, (list, dict)) else str(cls_id)
        x1, y1, x2, y2 = map(float, box.xyxy[0])
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        objs.append({
            "name": name,
            "xmin": int(x1),
            "xmax": int(x2),
            "center": (cx, cy),
            "y": cy,
            "width": x2 - x1
        })
    return objs

# -----------------------------
# 分层
# -----------------------------
def assign_layers(objects, eps=100):
    sorted_objs = sorted(objects, key=lambda o: o["y"])
    layers = []
    layer_id = 0
    if not sorted_objs:
        return []
    layers.append(layer_id)
    last_y = sorted_objs[0]["y"]
    for obj in sorted_objs[1:]:
        if abs(obj["y"] - last_y) > eps:
            layer_id += 1
            last_y = obj["y"]
        layers.append(layer_id)
    for i, obj in enumerate(sorted_objs):
        obj["layer"] = layers[i]
    return sorted_objs

# -----------------------------
# 估计目标宽度
# -----------------------------
def estimate_width(objects_on_layer, target_name):
    sims = [text_similarity(o["name"], target_name) for o in objects_on_layer]
    similar_objs = [o for o, s in zip(objects_on_layer, sims) if s > 0.5]
    if similar_objs:
        widths = [o["width"] for o in similar_objs]
        return np.median(widths)
    widths = [o["width"] for o in objects_on_layer]
    return np.median(widths) if widths else 100

# -----------------------------
# 找 gap
# -----------------------------
def find_best_gap(objects_on_layer, target_name, img_width=None):
    if not objects_on_layer:
        return None
    objs = sorted(objects_on_layer, key=lambda o: o["center"][0])
    target_width = estimate_width(objects_on_layer, target_name)

    # 1. 尝试放在两个物体中间
    gaps = []
    for i in range(len(objs) - 1):
        left, right = objs[i], objs[i + 1]
        gap_width = right["xmin"] - left["xmax"]
        if gap_width >= target_width:
            cx = int((left["xmax"] + right["xmin"]) / 2)
            cy = int((left["center"][1] + right["center"][1]) / 2)
            gaps.append((gap_width, cx, cy))
    if gaps:
        gaps.sort(reverse=True)
        _, cx, cy = gaps[0]
        return cx, cy

    # 2. 没有 gap 时，放最左或最右
    leftmost = objs[0]
    rightmost = objs[-1]
    cy = int(np.median([o["center"][1] for o in objs]))

    if img_width:
        left_space = leftmost["xmin"]
        right_space = img_width - rightmost["xmax"]

        if left_space >= target_width:
            cx = int(leftmost["xmin"] - target_width / 2)
            return cx, cy
        elif right_space >= target_width:
            cx = int(rightmost["xmax"] + target_width / 2)
            return cx, cy
    else:
        cx = int(rightmost["xmax"] + target_width / 2)
        return cx, cy

    cx = int(leftmost["xmin"] - target_width / 2)
    return cx, cy

# -----------------------------
# 计算放置点
# -----------------------------
def compute_placement(result, target_name, exclude_classes=None, img_width=None):
    if exclude_classes is None:
        exclude_classes = []

    objects = extract_objects(result)
    objects = [o for o in objects if o["name"] not in exclude_classes]
    if not objects:
        return None, None

    objects = assign_layers(objects, eps=100)

    # 选择层：语义相似度加权
    layer_scores = {}
    for o in objects:
        sim = text_similarity(o["name"], target_name)
        layer_scores[o["layer"]] = layer_scores.get(o["layer"], 0) + sim
    if not layer_scores:
        return None, None
    best_layer = max(layer_scores.items(), key=lambda x: x[1])[0]

    layer_objs = [o for o in objects if o["layer"] == best_layer]

    return find_best_gap(layer_objs, target_name, img_width), best_layer

# -----------------------------
# 可视化
# -----------------------------
def draw_point(img_path, point, output="output2.jpg"):
    img = cv2.imread(img_path)
    if img is None: return
    cx, cy = point
    cv2.circle(img, (int(cx), int(cy)), 15, (0,0,255), -1)
    cv2.putText(img, "Place Here", (int(cx)-50, int(cy)-20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
    cv2.imwrite(output, img)
    print(f"已保存: {output}")

# -----------------------------
# 示例
# -----------------------------
if __name__ == "__main__":
    # ------------------------
    # 1. 初始化 RealSense 管线
    # ------------------------
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # ------------------------
    # 2. 加载 YOLO
    # ------------------------
    model = YOLO("yolo11x-seg.pt")
    target_item = "orange"
    exclude = ["potted plant"]

    print("开始实时推理，按 Q 退出")

    while True:
        # ------------------------
        # 3. 获取 RealSense 图像
        # ------------------------
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        img_height, img_width = img.shape[:2]

        # ------------------------
        # 4. YOLO 推理
        # ------------------------
        results = model.predict(img, conf=0.25, iou=0.5, verbose=False)
        result = results[0]

        # ------------------------
        # 5. 计算放置点
        # ------------------------
        pos, layer = compute_placement(
            result,
            target_name=target_item,
            exclude_classes=exclude,
            img_width=img_width
        )

        # ------------------------
        # 6. 在 RealSense 图像上绘制结果
        # ------------------------
        if pos:
            cx, cy = pos
            cv2.circle(img, (int(cx), int(cy)), 10, (0, 0, 255), -1)
            cv2.putText(img, f"{target_item}: {pos} (Layer {layer})",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)
        else:
            cv2.putText(img, "No placement found",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)

        # ------------------------
        # 7. 显示实时画面
        # ------------------------
        cv2.imshow("RealSense YOLO Placement", img)

        # ------------------------
        # 8. 按 Q 退出
        # ------------------------
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # ------------------------
    # 9. 清理
    # ------------------------
    pipeline.stop()
    cv2.destroyAllWindows()
