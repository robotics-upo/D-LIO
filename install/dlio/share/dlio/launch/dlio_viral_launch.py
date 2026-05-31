from launch.events import matchers
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import math


def generate_launch_description():

    bag_path = LaunchConfiguration('bag_path')

    play_bag = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'ros2', 'bag', 'play', bag_path, '--rate', '0.00001'
        ],
        output='screen',
        condition=IfCondition(PythonExpression(['"', bag_path, '" != ""']))
    )

    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(launch_file_dir)
    rviz_config_file = os.path.join(launch_file_dir, 'default.rviz')
    config_file = os.path.join(pkg_dir, 'config', 'dlio_viral.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description='Full path to the .db3 file to play with ros2 bag. If not provided, the launch will wait for external IMU and LiDAR data to arrive on the corresponding topics.'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=rviz_config_file,
            description='Full path to the RViz config file.'
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', LaunchConfiguration('rviz_config_file')],
            output='screen'
        ),


        # IMU: T_Body_Imu = Identidad
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_tf_base_link_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu'],
            output='screen'),

        # LiDAR horizontal (os1_cloud_node1): R=I, t=(-0.050, 0, 0.055)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_tf_base_link_to_sensor1',
            arguments=['-0.050', '0.0', '0.055', '0', '0', '0',
                    'base_link', 'sensor1/os_sensor'],
            output='screen'),

        # LiDAR vertical (os1_cloud_node2): yaw=pi, pitch=0, roll=pi/2, t=(-0.550, 0.030, 0.050)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_tf_base_link_to_sensor2',
            arguments=['-0.550', '0.030', '0.050',
                    str(math.pi), '0.0', str(math.pi/2),
                    'base_link', 'sensor2/os_sensor'],
            output='screen'),

        Node(
            package='dlio',
            executable='dlio_node',
            name='dlio_node',
            output='screen',
            remappings=[
                ('/dlio_node/initial_pose', '/initialpose')
            ],
            parameters=[config_file],
            arguments=[
                '--ros-args',
                '--log-level', 'dlio_node:=DEBUG'
            ]
        ),

        play_bag
    ])
