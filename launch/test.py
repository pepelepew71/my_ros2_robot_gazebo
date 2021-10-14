import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # -- launch gazebo with world
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(
        get_package_share_directory('my_ros2_robot_gazebo'),
        'worlds',
        # 'empty.world'
        'cloister.world',
        # 'turtlebot3_world.world',
    )
    print(world)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # -- spawn robot and state publisher
    # launch_file_dir = os.path.join(get_package_share_directory('my_ros2_robot_gazebo'), 'launch')
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # x_pose = LaunchConfiguration('x_pose', default='0.0')
    # y_pose = LaunchConfiguration('y_pose', default='0.0')

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    return ld

if __name__ == "__main__":

    generate_launch_description()
