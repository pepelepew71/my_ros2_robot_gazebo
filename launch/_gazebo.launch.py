import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    """
    """
    # -- vars
    world_name = LaunchConfiguration(variable_name="world_name").perform(context=context)

    path_gazebo_ros = get_package_share_directory('gazebo_ros')
    path_current_pkg = get_package_share_directory('my_ros2_robot_gazebo')
    path_world = os.path.join(path_current_pkg, 'worlds', f'{world_name}.world')

    # -- IncludeLaunchDescription
    launch_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': path_world}.items()
    )

    launch_gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    return [launch_gzserver, launch_gzclient]

def generate_launch_description():
    """
    """
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(name="world_name"))
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
