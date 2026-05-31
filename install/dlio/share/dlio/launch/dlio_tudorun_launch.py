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
            'ros2', 'bag', 'play', bag_path, '--rate', '1.0'
        ],
        output='screen',
        condition=IfCondition(PythonExpression(['"', bag_path, '" != ""']))
    )
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(launch_file_dir)
    rviz_config_file = os.path.join(launch_file_dir, 'default.rviz')
    config_file = os.path.join(pkg_dir, 'config', 'dlio_tudorun.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description='Full path to the .db3 file to play with ros2 bag.'
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
            name='base_to_livox',  
            arguments=[
                "0.0", "0.0", "0.0",           
                "0.0", "0.0", "0.0", "1.0",        
                "base_link",                       
                "livox_frame"                        
            ],
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
