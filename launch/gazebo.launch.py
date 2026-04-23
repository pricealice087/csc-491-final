import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='room1.world',
        description='World file name (must be in worlds/ folder)'
    )
    
    turtlebot3_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch'
    )
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Get ABSOLUTE path to project root at runtime
    project_root = str(Path(__file__).parent.parent.resolve())
    worlds_dir = os.path.join(project_root, 'worlds')
    
    # Get world filename from launch argument
    world_file = LaunchConfiguration('world')
    
    # Build absolute path using PathJoinSubstitution
    world_path = PathJoinSubstitution([worlds_dir, world_file])
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items()
    )
    
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'robot_state_publisher.launch.py')
        )
    )
    
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'spawn_turtlebot3.launch.py')
        )
    )
    
    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(gzserver_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    
    return ld