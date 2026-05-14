from glob import glob

from setuptools import find_packages, setup

package_name = 'vr_udp_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hsh',
    maintainer_email='hsh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'udp_to_pose = vr_udp_bridge.udp_to_pose:main',
            'quest2ros_adapter = vr_udp_bridge.quest2ros_adapter:main',
            'lerobot_piper_recorder = vr_udp_bridge.lerobot_piper_recorder:main',
            'vr_dual_arm_teleop = vr_udp_bridge.vr_dual_arm_teleop:main',
            'vr_piper_anchor_teleop = vr_udp_bridge.vr_piper_anchor_teleop:main',
            'vr_piper_joint_ik_axis_gripper_teleop = vr_udp_bridge.vr_piper_joint_ik_axis_gripper_teleop:main',
            'vr_piper_joint_ik_teleop = vr_udp_bridge.vr_piper_joint_ik_teleop:main',
            'vr_piper_controller_base = vr_udp_bridge.vr_piper_controller_base:main',
            'vr_piper_controller_ee = vr_udp_bridge.vr_piper_controller_ee:main',
            'vr_piper_pose_teleop = vr_udp_bridge.vr_piper_pose_teleop:main',
            'vr_piper_gazebo_joint_teleop = vr_udp_bridge.vr_piper_gazebo_joint_teleop:main',
        ],
    },
)
