
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
