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
    path_current_pkg = get_package_share_directory('my_ros2_robot_gazebo')
    world_name = ("cloister", "cloister_asphalt", "gallery", "playpen", "playpen_asphalt")[0]
    robot_name = "mrobot"
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x = LaunchConfiguration('x', default="0.0")
    y = LaunchConfiguration('y', default="0.0")

    # -- IncludeLaunchDescription
    launch_world = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(path_current_pkg, 'launch', '_world.launch.py')),
        launch_arguments={"world_name": world_name}.items()
    )

    launch_spawn = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(path_current_pkg, 'launch', '_spawn.launch.py')),
        launch_arguments={
            "robot_name": robot_name,
            "use_sim_time": use_sim_time,
            "x": x,
            "y": y
        }.items()
    )

    # -- LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_world)
    ld.add_action(launch_spawn)

    return ld
