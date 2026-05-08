import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():

    obstacleavoidance = Node(
        package='Projet_CDVDM_IB',
        executable='obstacleavoidance',
        name='Projet_CDVDM_IB',
        output='screen',
        on_exit=Shutdown(),
    )

    # Create a launch description for the robot state publisher
    robot_state_pub = Node(
        package='Projet_CDVDM_IB',
        executable='corridornavigation',
        name='Projet_CDVDM_IB',
        output='screen',
    )

    return LaunchDescription([

        obstacleavoidance,
        robot_state_pub
    ])
