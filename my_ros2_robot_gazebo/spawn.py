"""
spawn_turtlebot.py

Script used to spawn a turtlebot in a generic position
"""

import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import xacro

def main():
    """
    Use service /spawn_entity to spawn robot.
    """
    rclpy.init()
    node = rclpy.create_node("entity_spawner")

    # -- args
    node.declare_parameter(name="robot_name", value=None)
    node.declare_parameter(name="x", value=None)
    node.declare_parameter(name="y", value=None)

    robot_name = node.get_parameter('robot_name').value
    x = node.get_parameter('x').value
    y = node.get_parameter('y').value

    # -- call service to spawn robot
    if robot_name is not None:

        # -- parse xacro to urdf
        xacro_file = os.path.join(get_package_share_directory('my_ros2_robot_gazebo'), 'urdf', f'{robot_name}', 'main.xacro')
        robot_desc = xacro.process_file(xacro_file)
        robot_desc = robot_desc.toxml()

        # -- set data for request
        request = SpawnEntity.Request()
        request.name = f'{robot_name}'
        request.xml = robot_desc
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = 0.2

        # -- service client
        node.get_logger().info("Connecting to `/spawn_entity` service...")
        client = node.create_client(SpawnEntity, "/spawn_entity")
        if not client.service_is_ready():
            client.wait_for_service()
            node.get_logger().info("...connected!")

        # -- call service
        node.get_logger().info("Sending service request to `/spawn_entity`")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            print('response: %r' % future.result())
        else:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception())
    else:
        node.get_logger().info("robot_name is not defined.")

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
