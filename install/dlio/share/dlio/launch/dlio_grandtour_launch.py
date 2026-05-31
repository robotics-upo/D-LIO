import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

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
    config_file = os.path.join(pkg_dir, 'config', 'dlio_grandtour.yaml')

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
            name='tf_base_to_box_base',
            arguments=[
                "0.0764038", "-0.0361", "0.280279",                 
                "0.9999999991989279", "0.0", "0.0", "4.0026794885936924e-05", 
                "base_link",      
                "box_base"  
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_box_base_to_hesai',
            arguments=[
                "0.30252311281815153", "-0.042546924756587536", "-0.012840842673182529", 
                "0.7093301791430758", "-0.7048516039326548", "-0.003921407292290494", "-0.004419949690193552",
                "box_base",    
                "hesai_lidar" 
            ],
            output='screen'
        ),
       
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='anymal_imu_broadcaster',
            arguments=[
                "-0.25565", "0.00255", "0.07672",                                
                "6.123233995736766e-17", "1.0", "6.123233995736766e-17", "3.749399456654644e-33", 
                "base", "anymal_imu"                                           
            ],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_box_base_to_stim320',
            arguments=[
                "0.2860923615483028", 
                "-0.0705237149536422", 
                "0.15822743993874794",
                "0.999969253444768", 
                "0.005117285966486087", 
                "-0.0009724399227390018", 
                "-0.005861732683034935",
                "box_base",    
                "stim320_imu" 
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

        play_bag
    ])
