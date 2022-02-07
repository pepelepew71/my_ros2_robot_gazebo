import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    """
    # -- vars
    path_current_pkg = get_package_share_directory('my_ros2_robot_gazebo')
    path_slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_params_file = os.path.join(path_current_pkg, 'config', 'mapper_params_online_sync.yaml')

    # -- IncludeLaunchDescription
    launch_slam = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(path_slam_toolbox, 'launch', 'online_sync_launch.py')),
        launch_arguments={"slam_params_file": slam_params_file}.items()
    )

    node_rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(path_current_pkg, 'config', 'slam.rviz')]
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_slam)
    ld.add_action(node_rviz2)

    return ld
