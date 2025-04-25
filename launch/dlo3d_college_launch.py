from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    bag_path = LaunchConfiguration('bag_path')

    bag_play = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 
            'ros2', 'bag', 'play', bag_path, '--rate', '0.00001'
        ],
        output='screen',
        condition=IfCondition(bag_path)
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
            name='static_tf_base_link_to_base_laser_link',
            arguments=['-0.0', '-0.00', '0.0','0.0', '0.0', '0.0', 'base_link', 'os_sensor'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_base_imu_link',
            arguments=['-0.0', '-0.00', '0.0', '0.0', '0.0', '0.0', 'base_link', 'os_imu'],
            output='screen'
        ),

        # DLO3D Node
        Node(
            package='dll3d',
            executable='dlo3d_node',
            name='dll3d_node',
            output='screen',
            remappings=[
                ('/dll3d_node/initial_pose', '/initialpose')
            ],
            parameters=[
                {'in_cloud_aux': '/nada'},
                {'in_cloud': '/os_cloud_node/points'},
                {'hz_cloud': 10.0},
                {'in_imu': '/os_cloud_node/imu'},
                {'hz_imu': 100.0},
                {'calibration_time': 1.0},
                {'aux_lidar_en': False},
                {'gyr_dev':  0.00117396706572},
                {'gyr_rw_dev': 2.66e-07},
                {'acc_dev': 0.0115432018302},
                {'acc_rw_dev': 0.0000333},
                {'base_frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'map_frame_id': 'map'},
                {'keyframe_dist': 1.0},
                {'keyframe_rot': 25.0},
                {'tdfGridSizeX_low': -10.0},
                {'tdfGridSizeX_high': 70.0},
                {'tdfGridSizeY_low': -30.0},
                {'tdfGridSizeY_high': 30.0},
                {'tdfGridSizeZ_low': -5.0},
                {'tdfGridSizeZ_high': 30.0},
                {'solver_max_iter': 500},
                {'solver_max_threads': 20},
                {'min_range': 2.0},
                {'max_range': 100.0},
                {'pc_downsampling': 4},
                {'robust_kernel_scale': 1.0}
            ]
        ),

        bag_play
    ])
