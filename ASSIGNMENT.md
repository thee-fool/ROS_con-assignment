# 🤖 ROS Bot — Recruitment Assignment

## Overview

This is a ROS 2 (Jazzy) autonomous robot project that runs in **Gazebo Harmonic**. The robot can:
- **Explore** an environment autonomously
- **Detect** objects using computer vision
- **Navigate** towards detected objects
- **Pick up** objects with a robot arm
- **Deliver** them to a person

However, key parts of the pipeline are **missing** — your job is to implement them!

---

## What's Already Working

✅ Robot spawning in Gazebo with full sensor suite (camera, depth camera, LiDAR, IMU, GPS)  
✅ SLAM-based mapping (slam_toolbox)  
✅ Nav2 navigation stack  
✅ Frontier-based autonomous exploration  
✅ Robot arm with inverse kinematics & pick-up sequence  
✅ 3D pose estimation from depth camera  
✅ TF2 transforms between all frames  
✅ ROS 2 service for changing target object at runtime  

---

## What You Need to Implement

### Task 1: Object Detection Node ⭐⭐⭐ (Primary)

**File:** `src/bme_gazebo_sensors_py/bme_gazebo_sensors_py/object_detection.py`

The `identify_image()` method and model loading are stubbed out with `TODO` comments. You must:

1. **Load an object detection model** (we recommend YOLOv8 via `ultralytics`, but any method works)
2. **Detect the target object** (`self.target_object`) in camera frames
3. **Publish detection signals:**
   - `Bool(True)` on `/object_detected` when target is found
   - `Point(x=cx, y=cy)` on `/centroid` with the pixel centroid
4. **Navigate towards the object:**
   - Use `self.send_incremental_goal(angle)` for Nav2-based approach (provided)
   - Or use `self.publisher.publish(Twist(...))` for direct velocity control
5. **Stop and signal the arm** when close enough:
   - Check depth via `self.latest_depth_frame[cy, cx]`
   - When `depth < self.stopping_distance_2`, publish `Bool(True)` on `/arm_controller/found_object`
6. **Handle object loss:** publish `Bool(False)` on `/object_detected` when target disappears
7. **Publish annotated frames** on `/detection` for RViz visualization

**Relevant topics and services:**
| Topic/Service | Type | Direction | Purpose |
|---|---|---|---|
| `/camera/image` | `sensor_msgs/Image` | Subscribe | RGB camera feed |
| `/camera/depth_image` | `sensor_msgs/Image` | Subscribe | Depth camera (float32, meters) |
| `/object_detected` | `std_msgs/Bool` | Publish | Signal exploration controller |
| `/centroid` | `geometry_msgs/Point` | Publish | Pixel coordinates to arm |
| `/arm_controller/found_object` | `std_msgs/Bool` | Publish | Trigger arm pickup |
| `/cmd_vel` | `geometry_msgs/Twist` | Publish | Direct velocity control |
| `set_target_object` | `SetTargetObject` | Service Server | Change target at runtime |

---

### Task 2: Exploration Controller ⭐⭐ (Required)

**File:** `src/object_finder/object_finder/controller_node.py`

The `detected_callback()` method is stubbed. You must:

1. When `/object_detected` publishes `True` → publish `Bool(False)` on `/explore/resume` to **pause** exploration
2. When `/object_detected` publishes `False` → publish `Bool(True)` on `/explore/resume` to **resume** exploration

---

### Task 3: Post-Pickup Exploration Reset ⭐ (Bonus)

**File:** `src/explore/src/explore.cpp`

Three C++ methods are stubbed:
- `found_callback()` — handle `/arm_controller/picked_object` signal
- `resetExplorationState()` — clear costmaps, reset SLAM, prepare for new exploration
- `spinGoal()` — spin the robot in place to rebuild the map

Read the TODO comments in each method for detailed guidance.

---

## Setup & Running

### Prerequisites
- ROS 2 Jazzy
- Gazebo Harmonic
- Python packages: `ultralytics`, `cvzone`, `numpy<2`

### Steps

```bash
# Build
colcon build
source install/local_setup.sh

# Terminal 1: Spawn robot in Gazebo
ros2 launch bme_ros2_navigation spawn_robot.launch.py

# Terminal 2: Start SLAM + Navigation
ros2 launch bme_ros2_navigation navigation_with_slam.launch.py

# Terminal 3: Start exploration + your detection + controller
ros2 launch object_finder full_launch.py

# Terminal 4: Set target object
ros2 run bme_gazebo_sensors_py set_target_client
# Enter: fire hydrant

# Terminal 5: Start arm pickup node
ros2 run bme_ros2_simple_arm_py object_pickup

# Terminal 6: Start arm 3D pose estimation
ros2 run bme_ros2_simple_arm_py object_detection
```

> **Note:** In Gazebo, add a standing person and a fire hydrant (or object of your choice that exists in the YOLO model) to the world.

### Testing your detection in RViz
Open RViz → Add Display → By Topic → `/detection/image` to see your detection output.

---

## Evaluation Criteria

| Criteria | Weight |
|---|---|
| Object detection accuracy and reliability | 30% |
| Navigation towards detected object | 25% |
| Exploration pause/resume integration | 20% |
| Code quality and documentation | 15% |
| Bonus: Post-pickup reset (Task 3) | 10% |

---

## Architecture Reference

```
spawn_robot.launch.py → Gazebo + URDF + Sensor Bridges
navigation_with_slam.launch.py → SLAM Toolbox + Nav2
full_launch.py → explore_lite + YOUR detection node + YOUR controller
set_target_client → tells detection what to find
object_pickup → arm IK + pick sequence (listens to /Object_position)
object_detection (arm pkg) → depth→3D transform (listens to /centroid)
```

**Good luck! 🚀**
