# yolo_seg_infer.py
from ultralytics import YOLO
import argparse
import os
import numpy as np


def main(args):
    # 加载模型
    model = YOLO(args.model)

    # 做分割推理
    results = model.predict(
        source=args.source,
        save=True,               # 保存结果图像
        save_txt=args.save_txt,  # 保存mask坐标（如需要）
        conf=args.conf,          # 置信度阈值
        iou=args.iou,            # IoU阈值
        show=args.show,          # 是否显示窗口
        device=args.device       # 指定设备：cuda:0 / cpu
    )
    for result in results:
        names = model.names   # 获取类别名映射字典 {0: 'person', 1: 'cup', ...}

        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2

            cls_id = int(box.cls[0])     # 类别 ID
            cls_name = names[cls_id]     # 类别名称

            print(f"类别: {cls_name} | 中心坐标: ({cx:.1f}, {cy:.1f})")



    print(f"Inference done. Results saved to 'runs/segment/predict'.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLOv11 Image Segmentation Inference Script")
    parser.add_argument("--model", type=str, default="yolo11x-seg.pt", help="Path to YOLOv11 segmentation model")
    parser.add_argument("--source", type=str, default="R3.jpg", help="Image path or folder path")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--iou", type=float, default=0.5, help="IoU threshold")
    parser.add_argument("--show", action="store_true", help="Show result in window")
    parser.add_argument("--save_txt", action="store_true", help="Save segmentation masks to TXT")
    parser.add_argument("--device", type=str, default="cuda", help="Device to use: 'cuda', 'cuda:0', or 'cpu'")

    args = parser.parse_args()
    main(args)

