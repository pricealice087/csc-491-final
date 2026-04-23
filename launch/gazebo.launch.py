import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Declare argument for world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='delivery_world.world',
        description='World file name (must be in worlds/ folder)'
    )
    
    # Get TurtleBot3 launch directory
    turtlebot3_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch'
    )
    
    # Get ros_gz_sim package directory
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Get path to world file
    project_root = Path(__file__).parent.parent
    worlds_dir = str(project_root / 'worlds')
    world_file = LaunchConfiguration('world')
    
    # Build full path using PathJoinSubstitution
    world_path = PathJoinSubstitution([worlds_dir, world_file])
    
    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'robot_state_publisher.launch.py')
        )
    )
    
    # Spawn TurtleBot3
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'spawn_turtlebot3.launch.py')
        )
    )
    
    # Create launch description
    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(gzserver_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    
    return ld