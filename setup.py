import os
from glob import glob
from setuptools import setup

package_name = 'my_ros2_robot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib',   package_name, 'lib'), glob('lib/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf/accessory'), glob('urdf/accessory/*.xacro')),
        (os.path.join('share', package_name, 'urdf/mrobot'), glob('urdf/mrobot/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ych',
    maintainer_email='yc.huang71@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'by_xacro = my_ros2_robot_gazebo.spawner:Spawner.by_xacro',
            'by_sdf = my_ros2_robot_gazebo.spawner:Spawner.by_sdf',
        ],
    },
)
