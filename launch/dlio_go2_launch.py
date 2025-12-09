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
        condition=IfCondition(
            PythonExpression(["'", bag_path, "' != ''"])
        )
    )

    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    rviz_config_file = os.path.join(launch_file_dir, 'default.rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description=(
                'Full path to the .db3 file to play with ros2 bag. '
                'If empty, no bag will be played and se esperará datos externos.'
            )
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=rviz_config_file,
            description='Full path to the RViz config file.'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rviz2', 'rviz2',
                '-d', LaunchConfiguration('rviz_config_file')
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_os_lidar',
            arguments=[
                '0.0', '0.0', '0.03618',
                '0.0', '0.0', '1.0', '0.0',
                'base_link', 'os_lidar'
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_os_imu',
            arguments=[
                '0.006253', '-0.011775', '0.007645',
                '0.0', '0.0', '0.0', '1.0',
                'base_link', 'os_imu'
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
                {'in_cloud_aux': '/os_cloud_node/points'},
                {'in_cloud': '/ouster/points'},
                {'hz_cloud': 10.0},
                {'in_imu': '/ouster/imu'},
                {'hz_imu': 100.0},
                {'calibration_time': 4.0},
                {'aux_lidar_en': False},
                {'gyr_dev': 0.008},
                {'gyr_rw_dev': 0.000244},
                {'acc_dev': 0.08},
                {'acc_rw_dev': 0.000157},
                {'base_frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'map_frame_id': 'map'},
                {'keyframe_dist': 1.0},
                {'keyframe_rot': 25.0},
                {'tdfGridSizeX_low': -50.0},
                {'tdfGridSizeX_high': 50.0},
                {'tdfGridSizeY_low': -50.0},
                {'tdfGridSizeY_high':50.0},
                {'tdfGridSizeZ_low': -10.0},
                {'tdfGridSizeZ_high': 50.0},
                {'solver_max_iter': 500},
                {'solver_max_threads': 20},
                {'min_range': 0.5},
                {'max_range': 500.0},
                {'pc_downsampling': 1},
                {'robust_kernel_scale': 10.0},
                {'kGridMarginFactor': 0.8},
                {'maxload': 25.0},
                {'maxCells': 200000},
                {'lidar_type': "ouster"},
                {'leaf_size': -1.0}
            ],
            arguments=[
                '--ros-args',
                '--log-level', 'dlio_node:=INFO'
            ]
        ),

        bag_play
    ])


