"""

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
import xacro

def launch_setup(context, *args, **kwargs):
    """
    """
    # -- vars
    use_sim_time = LaunchConfiguration(variable_name='use_sim_time', default='true')
    name = LaunchConfiguration(variable_name='name', default='true').perform(context=context)
    x = LaunchConfiguration(variable_name='x', default="0.0")
    y = LaunchConfiguration(variable_name='y', default="0.0")

    path_current_pkg = get_package_share_directory('my_ros2_robot_gazebo')
    sdf = os.path.join(path_current_pkg, 'urdf', name, 'main.xacro')
    sdf = xacro.process_file(sdf).toxml()

    # -- Node
    # -- robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': sdf
        }]
    )

    # -- spawn robot by xacro
    node_spawn = Node(
        package="my_ros2_robot_gazebo",
        executable="spawn_by_xacro",
        parameters=[{
            "name": name,
            "x": x,
            "y": y
        }]
    )

    return [node_robot_state_publisher, node_spawn]

def generate_launch_description():
    """
    """
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(name="use_sim_time", default_value="true"))
    ld.add_action(DeclareLaunchArgument(name="name", default_value=""))
    ld.add_action(DeclareLaunchArgument(name="declare_x", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument(name="declare_y", default_value="0.0"))
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
