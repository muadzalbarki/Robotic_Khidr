# UPDATE LOG - Robotik Khidr

## Format
- **Date**: YYYY-MM-DD
- **Type**: [FEATURE/FIX/REFACTOR/BREAKING]
- **Files**: List of modified files
- **Description**: Detailed description of changes
- **Rollback**: How to rollback if needed

---

## 2025-10-21
### [BREAKING] Migration to OpenCV-only Vision System
**Files**: 
- `raspi/src/vision.py` - Complete rewrite
- `raspi/src/main.py` - Updated to use OpenCV detection
- `raspi/src/config.py` - Removed YOLO config, added LiDAR mapping constants

**Changes**:
1. **Removed YOLO/Ultralytics dependency completely**
   - Deleted `ultralytics`, `torch`, `torchvision` dependencies (~1.5GB freed)
   - Removed `YOLO_MODEL_PATH`, `YOLO_BOTTLE_CLASS_ID`, `YOLO_CONFIDENCE_THRESHOLD` from config
   - Rewrote `vision.py` to use pure OpenCV HSV contour detection

2. **Vision System Rewrite** (`vision.py`)
   - New `detect_objects()` using HSV contour detection
   - Dynamic class detection via `COLOR_RANGES` config
   - Zone logic: LEFT/CENTER/RIGHT based on frame thirds
   - Removed all YOLO/ultralytics imports
   - Fixed camera initialization recursion bug
   - Added `capture_frame()` method

2. **Config Updates** (`config.py`)
   - Removed: `YOLO_MODEL_PATH`, `YOLO_BOTTLE_CLASS_ID`, `YOLO_CONFIDENCE_THRESHOLD`
   - Ultrasonic grab distance: **5cm → 6cm**
   - Added LiDAR mapping constants:
     - `LIDAR_WALL_FOLLOW_DIST = 200` (mm, 20cm target)
     - `LIDAR_WARNING_THRESHOLD = 700` (mm, increased from 500)
     - `LIDAR_CRITICAL_THRESHOLD = 300` (mm)
     - `LIDAR_WALL_FOLLOW_DIST = 200` (mm)
     - `LIDAR_WALL_FOLLOW_TOLERANCE = 30` (mm)

3. **Main.py Updates**
   - Updated to use `vision.detect_objects()` instead of `detect_bottles()`
   - Updated state machine logic for new detection format
   - State machine: EXPLORE → SCANNING_BOTTLE → APPROACHING_BOTTLE → GRIPPING → NAV_ZONE → PLACING_BOTTLE → EXPLORE

3. **Dependencies Removed** (~1.5GB freed)
   - `torch`, `torchvision`, `ultralytics`, `ultralytics-thop`, `ultralytics-platform`
   - `nvidia-cudnn-cu13`, `nvidia-cusparselt-cu13`, `nvidia-nccl-cu13`, `nvidia-nvshmem-cu13`
   - `nvidia-cublas`, `nvidia-cuda-*`, `nvidia-cufft`, `nvidia-cufile`, `nvidia-cuda-cupti`
   - `nvidia-curand`, `nvidia-cusolver`, `nvidia-cusparse`, `nvidia-nvjitlink`
   - `nvidia-cufile`, `nvidia-cuda-cupti`, `nvidia-curand`, `nvidia-cusolver`
   - `nvidia-cusparse`, `nvidia-nvtx`, `nvidia-nvjitlink`, `nvidia-cufile`
   - `nvidia-cuda-cupti`, `nvidia-curand`, `nvidia-cusolver`, `nvidia-cusparse`
   - `nvidia-nvtx`, `nvidia-nvjitlink`, `nvidia-cufile`
   - `triton`, `cuda-toolkit`, `cuda-bindings`, `cuda-pathfinder`

4. **Model Path Update**
   - `YOLO_MODEL_PATH` updated to `/home/khidr/khidr/Robotic_Khidr/raspi/models/my_model.pt` (not used anymore)

**Rollback**: 
```bash
git checkout 422b675 -- raspi/src/config.py raspi/src/vision.py raspi/src/main.py
pip install -r raspi/requirements.txt
```

---

## 2025-10-21 (Complete)
### [FEATURE] Complete OpenCV Migration & LiDAR Navigation Implementation
**Files**: 
- `raspi/src/vision.py` - Complete OpenCV rewrite
- `raspi/src/main.py` - Full state machine implementation
- `raspi/src/lidar_nav.py` - LiDAR odometry with scan matching, wall-following PID
- `raspi/src/mapper.py` - ArenaMap with landmark tracking, odometry integration
- `raspi/src/main.py` - Complete state machine
- `raspi/src/config.py` - Updated constants
- `raspi/src/mapper.py` - ArenaMap with landmark tracking

**Changes**:
1. **Complete OpenCV Migration** (removing YOLO/Ultralytics completely)
   - `vision.py`: Full OpenCV HSV contour detection with zone logic
   - `main.py`: Full state machine implementation
   - Removed all YOLO/Ultralytics dependencies

2. **LiDAR Navigation System** (`lidar_nav.py`)
   - Scan matching ICP for odometry
   - Wall-following PID control
   - Motor speed control with obstacle avoidance
   - Sector-based distance monitoring

2. **Arena Mapping** (`mapper.py`)
   - ArenaMap class with landmark tracking
   - Odometry integration with pose correction
   - Landmark/watering zone/station/home_base storage
   - JSON persistence

3. **Complete State Machine** (`main.py`)
   - EXPLORE → SCANNING_BOTTLE → APPROACHING_BOTTLE → GRIPPING → NAV_ZONE → PLACING_BOTTLE → EXPLORE
   - Watering zone detection and mapping
   - Ultrasonic threshold at 6cm for grabbing
   - Wall following with LiDAR

4. **Config Updates** (`config.py`)
   - Ultrasonic grab distance: 6cm
   - LiDAR constants for wall following and mapping
   - Removed all YOLO references

**Rollback**: 
```bash
git checkout 0e6e6cc -- raspi/src/lidar_nav.py raspi/src/mapper.py raspi/src/main.py
```

---

## 2025-10-21 (Complete)
### [FEATURE] Complete OpenCV Migration & LiDAR Navigation Implementation
**Files**: 
- `raspi/src/vision.py` - Complete OpenCV rewrite
- `raspi/src/main.py` - Full state machine implementation
- `raspi/src/lidar_nav.py` - LiDAR odometry with scan matching, wall-following PID
- `raspi/src/mapper.py` - ArenaMap with landmark tracking, odometry integration
- `raspi/src/main.py` - Complete state machine
- `raspi/src/config.py` - Updated constants
- `raspi/src/mapper.py` - ArenaMap with landmark tracking

**Changes**:
1. **Complete OpenCV Migration** (removing YOLO/Ultralytics completely)
   - `vision.py`: Full OpenCV HSV contour detection with zone logic
   - `main.py`: Full state machine implementation
   - Removed all YOLO/Ultralytics dependencies

2. **LiDAR Navigation System** (`lidar_nav.py`)
   - Scan matching ICP for odometry
   - Wall-following PID control
   - Motor speed control with obstacle avoidance
   - Sector-based distance monitoring

2. **Arena Mapping** (`mapper.py`)
   - ArenaMap class with landmark tracking
   - Odometry integration with pose correction
   - Landmark/watering zone/station/home_base storage
   - JSON persistence

3. **Complete State Machine** (`main.py`)
   - EXPLORE → SCANNING_BOTTLE → APPROACHING_BOTTLE → GRIPPING → NAV_ZONE → PLACING_BOTTLE → EXPLORE
   - Watering zone detection and mapping
   - Ultrasonic threshold at 6cm for grabbing
   - Wall following with LiDAR

4. **Config Updates** (`config.py`)
   - Ultrasonic grab distance: 6cm
   - LiDAR constants for wall following and mapping
   - Removed all YOLO references

**Rollback**: 
```bash
git checkout 0e6e6cc -- raspi/src/lidar_nav.py raspi/src/mapper.py raspi/src/main.py
```

---

## Rollback Commands Summary

```bash
# Full rollback to before OpenCV migration
git checkout 860c03d -- raspi/src/config.py raspi/src/vision.py raspi/src/main.py
pip install -r raspi/requirements.txt

# Rollback to before LiDAR mapping work
git checkout 0e6e6cc -- raspi/src/lidar_nav.py raspi/src/mapper.py raspi/src/main.py
```