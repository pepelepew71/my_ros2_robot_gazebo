"""
Use Gazebo's service /spawn_entity to spawn robot in a generic position
"""

import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import xacro

def _get_sdf_from_xacro(ns: str="") -> str:
    folder_current_pkg = get_package_share_directory('my_ros2_robot_gazebo')
    path_xacro_file = os.path.join(folder_current_pkg, 'urdf', "mrobot", 'main.xacro')
    sdf = xacro.process_file(path_xacro_file, mappings={'ns' : ns}) if ns else xacro.process_file(path_xacro_file)
    sdf = sdf.toxml()
    return sdf

def _get_request(name: str, sdf: str, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> SpawnEntity.Request:
    req = SpawnEntity.Request()
    req.name = name
    req.xml = sdf
    req.initial_pose.position.x = x
    req.initial_pose.position.y = y
    req.initial_pose.position.z = z
    req.initial_pose.orientation.x = qx
    req.initial_pose.orientation.y = qy
    req.initial_pose.orientation.z = qz
    req.initial_pose.orientation.w = qw
    return req

def _call_service(node, request: SpawnEntity.Request):
    # -- client
    node.get_logger().info("Connecting to `/spawn_entity` service...")
    client = node.create_client(SpawnEntity, "/spawn_entity")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    # -- send
    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())


class Spawner:

    @staticmethod
    def by_xacro():

        rclpy.init()
        node = rclpy.create_node("entity_spawner")

        # -- args
        node.declare_parameter(name="name", value="robot")
        node.declare_parameter(name="ns", value="")
        node.declare_parameter(name="x", value=0.0)
        node.declare_parameter(name="y", value=0.0)
        node.declare_parameter(name="z", value=0.2)
        node.declare_parameter(name="qx", value=0.0)
        node.declare_parameter(name="qy", value=0.0)
        node.declare_parameter(name="qz", value=0.0)
        node.declare_parameter(name="qw", value=1.0)

        name = node.get_parameter('name').value
        ns = node.get_parameter('ns').value
        x = node.get_parameter('x').value
        y = node.get_parameter('y').value
        z = node.get_parameter('z').value
        qx = node.get_parameter('qx').value
        qy = node.get_parameter('qy').value
        qz = node.get_parameter('qz').value
        qw = node.get_parameter('qw').value

        # -- parse xacro to urdf
        sdf = _get_sdf_from_xacro(ns=ns)

        # -- get request
        request = _get_request(name=name, sdf=sdf, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw)

        # -- call service
        _call_service(node=node, request=request)

        # -- close node
        node.get_logger().info("Done! Shutting down node.")
        node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def by_sdf():

        rclpy.init()
        node = rclpy.create_node("entity_spawner")

        # -- args
        path, name, x, y = sys.argv[1:]

        # -- parse xacro to urdf
        sdf = open(path, 'r').read()

        # -- get request
        request = _get_request(name=name, x=float(x), y=float(y), sdf=sdf)

        # -- call service
        _call_service(node=node, request=request)

        # -- close node
        node.get_logger().info("Done! Shutting down node.")
        node.destroy_node()
        rclpy.shutdown()
