from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os

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

        # Iniciar RViz con el archivo de configuración
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', LaunchConfiguration('rviz_config_file')],
            output='screen'
        ),
        # Static Tf
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_L',
            # x y z qx qy qz qw  parent child
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
            name='static_tf_base_link_to_I',
            arguments=['0.006253', '-0.011775', '0.007645',
                    '0.00445442', '-0.00342322', '-0.01598836', '0.99985637',
                    'base_link', 'os_imu'],
            output='screen'
        ),

        # DLO3D Node
        Node(
            package='dlio',
            executable='dlo3d_node',
            name='dll3d_node',
            output='screen',
            remappings=[
                ('/dll3d_node/initial_pose', '/initialpose')
            ],
            parameters=[
                {'in_cloud_aux': '/nada'},
                {'in_cloud': '/ouster/points'},
                {'hz_cloud': 10.0},
                {'in_imu': '/imu/data'},
                {'hz_imu': 100.0},
                {'calibration_time': 6.0},
                {'aux_lidar_en': False},
                {'gyr_dev':  0.0814929939543128923}, #0514929939543128923
                {'gyr_rw_dev': 1.0106430876706567e-05},
                {'acc_dev': 0.0818800668934690885}, #051800668934690885
                {'acc_rw_dev': 1.4361954439703917e-05},
                {'base_frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'map_frame_id': 'map'},
                {'keyframe_dist': 1.0},
                {'keyframe_rot': 1.698},
                {'tdfGridSizeX_low': -100.0},
                {'tdfGridSizeX_high': 500.0},
                {'tdfGridSizeY_low': -200.0},
                {'tdfGridSizeY_high': 200.0},
                {'tdfGridSizeZ_low': -50.0},
                {'tdfGridSizeZ_high': 50.0},
                {'solver_max_iter': 1000},
                {'solver_max_threads': 20},
                {'min_range': 1.0},
                {'max_range': 1000.0},
                {'pc_downsampling': 1},
                {'robust_kernel_scale': 0.3},
                {'tdf_grid_res': 0.05},
                {'maxload': 50.0}, #60
                {'maxCells': 80000}
            ],
            arguments=['--ros-args', '--log-level', 'INFO'] #DEBUG o INFO
        ),

        bag_play
    ])
