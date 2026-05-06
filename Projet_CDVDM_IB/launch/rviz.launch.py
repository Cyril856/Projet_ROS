import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():

    # Get the URDF file name
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    print('urdf_file_name : {}'.format(urdf_file_name))

    # Get the URDF file path
    urdf_path = os.path.join(
        get_package_share_directory('projet2025'),
        'urdf',
        urdf_file_name)

    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Create a launch description for RVIZ2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        on_exit=Shutdown(),
        #To tell rviz2 to load the rviz configuration file at launch.
        arguments =['-d' + os.path.join(get_package_share_directory('Projet_CDVDM_IB'),
                            'rviz', 'config.rviz')]
    )

    # Create a launch description for the robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
                'robot_description': robot_desc
        }],
    )

    # Create a launch description for the joint state publisher gui
    # joint state publisher gui node proposes a graphical interface aiming at controlling through some sliders the different robot joint positions. 
    joint_state_pub = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    return LaunchDescription([

        rviz2,
        #robot_state_pub,
        #joint_state_pub

    ])
