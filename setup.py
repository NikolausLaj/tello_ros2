from setuptools import find_packages, setup
import glob
package_name = 'tello_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/tello_bridge.yaml']),
        ('share/' + package_name + '/launch', ['launch/tello_bridge.launch.py']),
        (f'share/{package_name}/msg', glob.glob('msg/*.msg')),
    ],
    install_requires=['setuptools', 'djitellopy'],
    package_data={
        '': ['*.msg'],
    },
    zip_safe=True,
    maintainer='nikolaus',
    maintainer_email='nikolaus.lajtai@gmx.at',
    description='Tello ROS2 Bridge Node',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tello_bridge_node = tello_ros2.tello_bridge_node:main',
            'teleop_twist_rpyt_keyboard = tello_ros2.teleop_twist_rpyt_keyboard:main'
        ],
    },
)
