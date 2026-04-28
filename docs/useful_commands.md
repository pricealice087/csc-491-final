
# Useful Commands

### Compiles and installs ROS2 packages
```bash
colcon build
source install/setup.bash
```


### Launches the Gazebo Sim using the launch file
```bash
cd csc-491-final
chmod +x run.sh         # ONLY ONCE to make the script executable
./run.sh                # To launch the gz sim
# If you want to lanuch with a specific world enter the world file afterwards 
# EXAMPLE:
./run.sh room2.world

```

### For keyboard control in separate terminal
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```


## AMCL Node Launch
```bash
# Terminal 1
./run.sh

# Terminal 2
ros2 launch nav2_bringup localization_launch.py \
    map:=$HOME/ai_robotics/csc-491-final/maps/room1_map.yaml

# Don't forget to set inital 2d Pose Estimate at the top of RVIZ

# Terminal 3
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

```