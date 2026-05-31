import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    bag_path = LaunchConfiguration('bag_path')

    bag_play = ExecuteProcess(
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
    config_file = os.path.join(pkg_dir, 'config', 'dlio_colosseo.yaml')


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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_L',
            arguments=['0.0', '0.0', '0.0',
                    '0.0', '0.0', '0.0', '1.0',
                    'base_link', 'os_sensor'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_I',
            arguments=['-0.04563025', '-0.00784853', '-0.60841501',
                    '0.00445442', '-0.00342322', '-0.01598836', '0.99985640',
                    'base_link', 'imu_link_ned'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_I2',
            arguments=['0.006253', '-0.011775', '0.007645',
                    '0.00445442', '-0.00342322', '-0.01598836', '0.99985637',
                    'base_link', 'os_imu'],
            output='screen'
        ),

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
                '--log-level', 'dlio_node:=INFO'
            ]
        ),

        bag_play
    ])
