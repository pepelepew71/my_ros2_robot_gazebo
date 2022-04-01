"""

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    """
    # -- vars
    folder_robot = get_package_share_directory('my_ros2_robot_gazebo')

    # -- IncludeLaunchDescription
    # -- world_name: cloister, cloister_asphalt, gallery, playpen, playpen_asphalt
    launch_world = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(folder_robot, 'launch', '_gazebo.launch.py')),
        launch_arguments={"name": "cloister"}.items()
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_world)

    return ld
