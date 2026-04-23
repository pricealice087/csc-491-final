
# Useful Commands

Compiles and installs ROS2 packages
```bash
colcon build
source install/setup.bash
```


Launches the Gazebo Sim using the launch file
```bash
source /opt/ros/jazzy/setup.bash           
export TURTLEBOT3_MODEL=waffle
ros2 launch launch/gazebo.launch.py
```

For keyboard control in separate terminal
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
