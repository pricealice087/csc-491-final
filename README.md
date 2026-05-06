# CSC-491 Final — Autonomous Basketball Delivery Robot

A TurtleBot3 Waffle in Gazebo that detects a basketball using YOLOv8, navigates to it using A* path planning, and delivers it to a target location (LeBron James statue).

---

## How to Run

**Requires 4 terminals.** Run each in the project root (`~/ai_robotics/csc-491-final`).

### Terminal 1 — Simulation
```bash
chmod +x run.sh   # only needed once
./run.sh
```
Launches Gazebo with room1.world, spawns the robot, starts AMCL localization and map server.

### Terminal 2 — Basketball Detector (needs Python venv)
```bash
source /opt/ros/jazzy/setup.bash
source yolo_env/bin/activate
python3 scripts/basketball_depth_detector_node.py
```
Detects the basketball using YOLOv8 + depth camera. Publishes heading and distance to `/basketball/detection`.

### Terminal 3 — A\* Path Planner
```bash
source /opt/ros/jazzy/setup.bash
python3 src/astar_planner/astar_node.py
```
Listens for a goal on `/goal_pose`, plans a path using the occupancy map, publishes waypoints to `/astar_path`.

### Terminal 4 — Delivery Controller
```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/controller_node.py
```
Runs the state machine that ties everything together.

---

## State Machine

```
SEARCHING → APPROACHING_BALL → PICKUP → DELIVERING → DELIVERED
```

| State | Behaviour | Transition |
|-------|-----------|------------|
| SEARCHING | Spins in place | Ball detected with confidence ≥ 0.3 → saves world position |
| APPROACHING_BALL | Follows A* path to saved ball position | Within 1m of ball → PICKUP |
| PICKUP | Stops for 2 seconds | After 2s → DELIVERING |
| DELIVERING | Follows A* path to LeBron at (2.0, 0.0) | Within 1m of LeBron → DELIVERED |
| DELIVERED | Stops | — |

---

## Setting Up the Python venv

```bash
python3 -m venv yolo_env --system-site-packages
source yolo_env/bin/activate
pip install -r requirements.txt
```

---

## Useful Commands

```bash
# Keyboard control
ros2 run turtlebot3_teleop teleop_keyboard

# View robot camera
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw

# Check topics
ros2 topic list
ros2 topic hz /basketball/detection
ros2 topic echo /astar_path

# Check nodes
ros2 node list

```