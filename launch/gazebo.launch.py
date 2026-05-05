import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


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
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    project_root = str(Path(__file__).parent.parent.resolve())
    worlds_dir = os.path.join(project_root, 'worlds')
    models_dir = os.path.join(project_root, 'models')

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + models_dir
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_dir

    world_path = PathJoinSubstitution([worlds_dir, LaunchConfiguration('world')])

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'robot_state_publisher.launch.py')
        )
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'waffle',
            '-file', os.path.join(models_dir, 'turtlebot3_depth', 'model.sdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
        ],
        output='screen',
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={os.path.join(tb3_gazebo_pkg, "params", "turtlebot3_waffle_bridge.yaml")}',
        ],
        output='screen',
    )

    rgb_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    depth_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen',
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(gzserver_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot)
    ld.add_action(ros_gz_bridge)
    ld.add_action(rgb_image_bridge)
    ld.add_action(depth_image_bridge)
    ld.add_action(slam_node)
    return ld