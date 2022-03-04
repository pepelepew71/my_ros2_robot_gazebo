import xacro

if __name__ == "__main__":
    xacro_file = "/home/ych/ros2_ws/src/my_ros2_robot_gazebo/urdf/mrobot/main.xacro"
    robot_desc = xacro.process_file(xacro_file)
    robot_desc = robot_desc.toxml()

    with open("test.sdf", "w") as fp:
        fp.write(robot_desc)
