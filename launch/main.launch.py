import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    """
    """
    # -- parameters
    path_gazebo_ros = get_package_share_directory('gazebo_ros')
    path_current_pkg = get_package_share_directory('my_ros2_robot_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x = LaunchConfiguration('x', default="0.0")
    y = LaunchConfiguration('y', default="0.0")

    world_name = ("cloister", "cloister_asphalt", "gallery", "playpen", "playpen_asphalt")[1]
    path_world = os.path.join(path_current_pkg, 'worlds', f'{world_name}.world')

    robot_name = "mrobot"
    xacro_file = os.path.join(path_current_pkg, 'urdf', robot_name, 'main.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # -- IncludeLaunchDescription and Node
    launch_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': path_world}.items()
    )

    launch_gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    node_spawn = Node(
        package="my_ros2_robot_gazebo",
        executable="spawn",
        parameters=[{
            "robot_name": robot_name,
            "x": x,
            "y": y
        }]
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_gzserver)
    ld.add_action(launch_gzclient)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_spawn)

    return ld
