"""
Spawn a robot by gazebo's service with xacro
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    """
    # -- vars
    folder_robot = get_package_share_directory('my_ros2_robot_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    name = LaunchConfiguration('name', default="robot")
    ns = LaunchConfiguration('ns', default="")
    x = LaunchConfiguration('x', default="0.0")
    y = LaunchConfiguration('y', default="0.0")
    qz = LaunchConfiguration('qz', default="0.0")
    qw = LaunchConfiguration('qw', default="1.0")

    # -- IncludeLaunchDescription
    launch_spawn = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(folder_robot, 'launch', '_spawn_by_xacro.launch.py')),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "name": name,
            "ns": ns,
            "x": x,
            "y": y,
            "z": "0.2",
            "qx": "0.0",
            "qy": "0.0",
            "qz": qz,
            "qw": qw,
        }.items()
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_spawn)

    return ld
