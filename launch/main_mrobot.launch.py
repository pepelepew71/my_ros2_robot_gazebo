import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    """
    Launch gazebo with world
    """
    # -- configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # -- parameters
    path_gazebo_ros = get_package_share_directory('gazebo_ros')
    path_my_ros2_robot_gazebo = get_package_share_directory('my_ros2_robot_gazebo')

    world = os.path.join(
        path_my_ros2_robot_gazebo,
        'worlds',
        # 'empty.world',
        'cloister.world',
    )

    # -- xacro
    xacro_file = os.path.join(get_package_share_directory('my_ros2_robot_gazebo'), 'urdf', 'mrobot', 'main.xacro')
    robot_desc = xacro.process_file(xacro_file)
    robot_desc = robot_desc.toxml()

    # -- IncludeLaunchDescription
    launch_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )

    launch_gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # -- Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
    )

    node_spawn = Node(
        package="my_ros2_robot_gazebo",
        executable="spawn",
        parameters=[{"robot_name": "mrobot"}]
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_gzserver)
    ld.add_action(launch_gzclient)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_spawn)

    return ld

if __name__ == "__main__":

    generate_launch_description()
