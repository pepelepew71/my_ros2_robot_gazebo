import os
import pathlib

import xacro

if __name__ == "__main__":

    folder_pkg = str(pathlib.Path(__file__).parent.parent.absolute())
    xacro_file = os.path.join(folder_pkg, "urdf", "mrobot", "main.xacro")
    robot_desc = xacro.process_file(xacro_file)
    robot_desc = robot_desc.toxml()

    folder_sdf = os.path.join(folder_pkg, "sdf")
    if not os.path.exists(folder_sdf):
        os.mkdir(folder_sdf)

    path_sdf = os.path.join(folder_sdf, "mrobot.sdf")
    with open(path_sdf, "w") as fp:
        fp.write(robot_desc)
