from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os

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
    rviz_config_file = os.path.join(launch_file_dir, 'default.rviz')

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
            name='Body_T_Xt32',
            arguments=[
                "0.0", "0.0", "0.0",
                "0.0", "0.0", "0.7071067811865475", "0.7071067811865475",
                "base_link",
                "PandarXT-32"
            ],
            output='screen'
        ),
       
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='x36d',
            arguments=[
                "-0.06","0.0","-0.16",
                "-0.0007904608601149109","-0.003950257535560073","-0.0007041813888545112","0.9999916373478345",
                'base_link',
                'x36d'
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
            parameters=[
                {'in_cloud_aux': '/os1_cloud_node2/points'},
                {'in_cloud': '/hesai/pandar'},
                {'hz_cloud': 10.0},
                {'in_imu': '/mti3dk/imu'},
                {'hz_imu': 100.0},
                {'calibration_time': 5.0},
                {'aux_lidar_en': False},
                {'gyr_dev': 5.236e-4},
                {'gyr_rw_dev': 3.4e-05},
                {'acc_dev': 7e-3},
                {'acc_rw_dev': 2.5e-9},
                {'base_frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'map_frame_id': 'map'},
                {'keyframe_dist':2.0},
                {'keyframe_rot': 0.5},
                {'tdfGridSizeX_low': -30.0},
                {'tdfGridSizeX_high': 100.0},
                {'tdfGridSizeY_low': -100.0},
                {'tdfGridSizeY_high': 100.0},
                {'tdfGridSizeZ_low': -100.0},
                {'tdfGridSizeZ_high': 100.0},
                {'solver_max_iter': 200},
                {'solver_max_threads': 20},
                {'min_range': 1.0},
                {'max_range': 200.0},
                {'pc_downsampling': 1},
                {'robust_kernel_scale': 1.0},
                {'kGridMarginFactor': 0.8},
                {'maxload': 200.0},
                {'maxCells': 350000},
                {'lidar_type': "ouster"},
                {'leaf_size': -1.0}
            ],
            arguments=[
                '--ros-args',
                '--log-level', 'dlio_node:=INFO'
            ]
        ),

        play_bag
    ])
