# yolo-perception
The toolkit built on YOLO instance segmentation, Intel RealSense, and multimodal reasoning models focuses on detecting objects, understanding spatial layouts, and predicting feasible object placement locations in images or real-time camera streams.

## Modules Overview

### 1. YOLO Instance Segmentation (`yolo_seg_infer.py`)

YOLO-based object detection and instance segmentation module for single RGB images.

#### Features

* YOLO instance segmentation inference
* Object class and bounding box prediction
* Object center point extraction
* Optional saving of segmentation results
* Configurable confidence and IoU thresholds

#### Usage

```bash
python yolo_seg_infer.py --source image.jpg
```

---

### 2. YOLO-based Placement Prediction (`yolo_found.py`)

Rule-based object placement prediction module based on geometric reasoning and semantic similarity.

#### Features

* Object extraction from YOLO segmentation results
* Scene layering based on vertical object positions
* Semantic similarity matching using SBERT
* Gap-based placement location estimation
* Visualization of recommended placement points

This module does **not** rely on large language models and serves as a lightweight baseline.

#### Usage

```bash
python yolo_found.py
```

---

### 3. Real-time Placement Prediction with RealSense (`realsense_found.py`)

Real-time object perception and placement prediction using Intel RealSense RGB streams.

#### Features

* Live RGB image acquisition from RealSense cameras
* Real-time YOLO instance segmentation
* Online placement point computation
* Visualization of placement results on live video
* Interactive exit control

#### Usage

```bash
python realsense_found.py
```

---

### 4. YOLO Segmentation with RealSense Depth (`realsenseinfer.py`)

Advanced perception module combining YOLO instance segmentation with RealSense depth sensing.

#### Features

* RGB–depth frame alignment
* Robust depth estimation inside segmentation masks
* Pixel-to-camera coordinate deprojection
* 3D object position estimation
* Detection results saved in JSON format
* Real-time FPS display

#### Usage

```bash
python realsenseinfer.py
```

---

### 5. YOLO + Molmo Multimodal Placement Prediction (`yolo_molmo_predict.py`)

Multimodal object placement prediction module combining YOLO perception with Molmo reasoning.

#### Features

* YOLO-based object detection and spatial grounding
* Image discretization into a fixed grid (e.g., 64 × 64)
* Structured prompt construction for Molmo
* Multimodal reasoning for placement location prediction
* Placement result visualization on the original image

This module demonstrates a complete **perception–reasoning–placement** pipeline.

#### Usage

```bash
python yolo_molmo_predict.py
```

---

## Environment Setup

### Requirements

* Python ≥ 3.9
* PyTorch
* CUDA-capable GPU (recommended for Molmo inference)
* Intel RealSense SDK (for RealSense-related modules)

---

### Create Environment

```bash
conda create -n yolo_place python=3.9
conda activate yolo_place
```

---

### Install Required Packages

```bash
pip install ultralytics torch torchvision opencv-python numpy pillow
pip install sentence-transformers transformers
pip install pyrealsense2
```

---

## Configuration

* YOLO model paths can be modified directly in each script
* Molmo models are loaded from a local directory
* RealSense resolution and frame rate can be adjusted in the scripts
* Default placement prediction operates in image space unless depth is explicitly used

---

## Notes

* This repository is intended as a **research prototype and demonstration toolkit**
* The focus is on **pipeline integration and capability demonstration**, rather than benchmark performance
* Multiple placement strategies (heuristic and multimodal) are provided for comparison
* The codebase is intentionally kept **script-based and easy to modify**

---
