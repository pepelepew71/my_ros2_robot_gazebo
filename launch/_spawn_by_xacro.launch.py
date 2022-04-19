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
    ns = LaunchConfiguration(variable_name='ns', default='').perform(context=context)
    x = LaunchConfiguration(variable_name='x')
    y = LaunchConfiguration(variable_name='y')
    z = LaunchConfiguration(variable_name='z')
    qx = LaunchConfiguration(variable_name='qx')
    qy = LaunchConfiguration(variable_name='qy')
    qz = LaunchConfiguration(variable_name='qz')
    qw = LaunchConfiguration(variable_name='qw')

    folder_current_pkg = get_package_share_directory('my_ros2_robot_gazebo')
    path_xacro_file = os.path.join(folder_current_pkg, 'urdf', "mrobot", 'main.xacro')
    sdf = xacro.process_file(path_xacro_file, mappings={'ns' : ns}) if ns else xacro.process_file(path_xacro_file)
    sdf = sdf.toxml()

    # -- Node
    # -- robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace = ns,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': sdf
        }]
    )

    # -- spawn robot by xacro
    node_spawn = Node(
        package="my_ros2_robot_gazebo",
        executable="by_xacro",
        parameters=[{
            "name": name,
            "ns": ns,
            "x": x,
            "y": y,
            "z": z,
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "qw": qw,
        }]
    )

    return [node_robot_state_publisher, node_spawn]

def generate_launch_description():
    """
    """
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(name="use_sim_time"))
    ld.add_action(DeclareLaunchArgument(name="name"))
    ld.add_action(DeclareLaunchArgument(name="ns"))
    ld.add_action(DeclareLaunchArgument(name="x" ))
    ld.add_action(DeclareLaunchArgument(name="y" ))
    ld.add_action(DeclareLaunchArgument(name="z" ))
    ld.add_action(DeclareLaunchArgument(name="qx"))
    ld.add_action(DeclareLaunchArgument(name="qy"))
    ld.add_action(DeclareLaunchArgument(name="qz"))
    ld.add_action(DeclareLaunchArgument(name="qw"))
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
