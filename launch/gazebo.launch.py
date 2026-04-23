import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get TurtleBot3 launch directory
    turtlebot3_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch'
    )
    
    # Get ros_gz_sim package directory (CHANGED from gazebo_ros)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Get path to YOUR world file (relative to this launch file)
    project_root = Path(__file__).parent.parent
    world = str(project_root / 'worlds' / 'delivery_world.world')
    
    # Gazebo server (CHANGED to use ros_gz_sim)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world}'}.items()
    )
    
    # Robot state publisher (from TurtleBot3 package)
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'robot_state_publisher.launch.py')
        )
    )
    
    # Spawn TurtleBot3 (from TurtleBot3 package)
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'spawn_turtlebot3.launch.py')
        )
    )
    
    # Create launch description and add all actions
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    
    return ld