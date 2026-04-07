# yolo_molmo_predict.py
import os
os.environ["HF_ENDPOINT"] = "https://hf-mirror.com"
os.environ["TRANSFORMERS_OFFLINE"] = "1"

import torch
from ultralytics import YOLO
from transformers import AutoModelForCausalLM, AutoProcessor, GenerationConfig
from PIL import Image, ImageDraw
import re
import random

# ======================
# Config
# ======================
IMAGE_PATH = "pic4test2.jpg"
OUTPUT_PATH = "pic4test_with_point.jpg"

# YOLO
YOLO_MODEL = "yolo11x-seg.pt"
YOLO_CONF = 0.25
YOLO_IOU = 0.5
YOLO_DEVICE = "cuda"

# Molmo
MOLMO_MODEL = "./Molmo-7B-D-0924"
GRID_SIZE = 64
DEVICE = "cuda"

# ======================
# Utils
# ======================
def bbox_to_grid64(bbox, W, H, grid=64):
    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2

    gx = int(cx / W * grid)
    gy = int(cy / H * grid)

    gx = max(0, min(grid - 1, gx))
    gy = max(0, min(grid - 1, gy))
    return gx, gy

# ======================
# 1. YOLO inference
# ======================
print("Running YOLO...")
yolo = YOLO(YOLO_MODEL)

results = yolo.predict(
    source=IMAGE_PATH,
    conf=YOLO_CONF,
    iou=YOLO_IOU,
    device=YOLO_DEVICE,
    save=False,
    verbose=False
)

image = Image.open(IMAGE_PATH).convert("RGB")
W, H = image.size

detected_objects = []

for result in results:
    names = yolo.names
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cls_id = int(box.cls[0])
        cls_name = names[cls_id]

        gx, gy = bbox_to_grid64((x1, y1, x2, y2), W, H, GRID_SIZE)

        detected_objects.append({
            "name": cls_name,
            "grid": [gx, gy]
        })

print("YOLO detected objects:")
for obj in detected_objects:
    print(f"- {obj['name']} @ grid {obj['grid']}")

# ======================
# 2. Build Molmo prompt
# ======================
object_text = ""
for obj in detected_objects:
    object_text += f"- {obj['name']}: center at [{obj['grid'][0]}, {obj['grid'][1]}]\n"
occupied = ", ".join(
    [f"({o['grid'][0]},{o['grid'][1]})" for o in detected_objects]
)

prompt = f"""
You are given an image divided into a {GRID_SIZE} x {GRID_SIZE} grid.

Grid coordinates:
- Top-left: (0, 0)
- Bottom-right: ({GRID_SIZE - 1}, {GRID_SIZE - 1})

Exsiting objects and their approximate locations (for reference only), which are NOT allowed for placement:
{object_text}

Rules:
- The output [x, y] MUST NOT be any occupied grid cell.
- Use object locations only as spatial reference, NOT as output candidates.
- Place the apple on a flat, empty surface visible in the image.
- Do NOT copy coordinates from the occupied list.

Only output:
[x, y]
"""


# ======================
# 3. Molmo inference
# ======================
print("Running Molmo...")
processor = AutoProcessor.from_pretrained(
    MOLMO_MODEL,
    trust_remote_code=True,
    torch_dtype="auto"
)

model = AutoModelForCausalLM.from_pretrained(
    MOLMO_MODEL,
    trust_remote_code=True,
    torch_dtype="auto"
).to(DEVICE)

inputs = processor.process(images=[image], text=prompt)
inputs = {k: v.to(DEVICE).unsqueeze(0) for k, v in inputs.items()}

with torch.inference_mode():
    output = model.generate_from_batch(
        inputs,
        GenerationConfig(
            max_new_tokens=50,
            stop_strings="<|endoftext|>"
        ),
        tokenizer=processor.tokenizer
    )

generated_tokens = output[0, inputs["input_ids"].size(1):]
generated_text = processor.tokenizer.decode(
    generated_tokens, skip_special_tokens=True
)

print("Molmo output:", generated_text)

# -------- parse grid coordinates (random pick one) --------
matches = re.findall(r"\[\s*(\d+)\s*,\s*(\d+)\s*\]", generated_text)
if not matches:
    raise ValueError(f"Failed to parse grid coords from: {generated_text}")

# 转成 int
candidates = [(int(x), int(y)) for x, y in matches]

# 画面中心（grid 坐标）
cx = (GRID_SIZE - 1) / 2
cy = (GRID_SIZE - 1) / 2

# 按到中心的欧氏距离排序
candidates.sort(key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)

# 取前 K 个，再随机选一个
K = min(6, len(candidates))   
gx, gy = random.choice(candidates[:K])


gx = max(0, min(GRID_SIZE - 1, gx))
gy = max(0, min(GRID_SIZE - 1, gy))

x_px = int((gx + 0.5) / GRID_SIZE * W)
y_px = int((gy + 0.5) / GRID_SIZE * H)

print(f"Final placement: grid ({gx}, {gy}) -> pixel ({x_px}, {y_px})")

draw = ImageDraw.Draw(image)
r = 8
draw.ellipse((x_px - r, y_px - r, x_px + r, y_px + r), fill="red")

image.save(OUTPUT_PATH)
print(f"Saved result to {OUTPUT_PATH}")
