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

        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', LaunchConfiguration('rviz_config_file')],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_base_laser_link',
            arguments=['0.0', '0.00', '0.0','0.0', '0.0', '0.0', 'base_link', 'sensor1/os_sensor'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_base_imu_link',
            arguments=['0.0', '0.00', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu'], 
            output='screen'
        ),

        Node(
            package='dlio',
            executable='dlo3d_node',
            name='dll3d_node',
            output='screen',
            remappings=[
                ('/dll3d_node/initial_pose', '/initialpose')
            ],
            parameters=[
                {'in_cloud_aux': '/os_cloud_node/points'},        # Auxiliar LiDAR topic     
                {'in_cloud': '/os_cloud_node/points'},            # Main LiDAR topic
                {'hz_cloud': 10.0},                               # LiDAR Frecuency
                {'in_imu': '/os_cloud_node/imu'},                 # IMU topic   
                {'hz_imu': 100.0},                                # IMU Frecuency
                {'calibration_time': 1.0},                        # Calibration time (set to 0.0 for no calibration.) -> Calibrate if the vehicle is stationary during calibration_time
                {'aux_lidar_en': False},                
                {'gyr_dev':  0.00117396706572},                   # Deviations and random walk of the IMU (gyroscope and accelerometer)
                {'gyr_rw_dev': 2.66e-07},
                {'acc_dev': 0.0115432018302},
                {'acc_rw_dev': 0.0000333},
                {'base_frame_id': 'base_link'},                   # Frames of the vehicle and odometry
                {'odom_frame_id': 'odom'},
                {'keyframe_dist': 1.0},                           # Keyframing thresholds -> Determines when the update of the map  is performed        
                {'keyframe_rot': 25.0},
                {'tdfGridSizeX_low': -10.0},                      # Dimensions of the 1m³-resolution map
                {'tdfGridSizeX_high': 70.0},
                {'tdfGridSizeY_low': -30.0},
                {'tdfGridSizeY_high': 30.0},
                {'tdfGridSizeZ_low': -5.0},
                {'tdfGridSizeZ_high': 30.0},
                {'solver_max_iter': 100},                        # Solver max iterations
                {'solver_max_threads': 20},                      # CPU cores -> Det depending on the computer employed
                {'min_range': 1.0},                              # Distance Filter (TDF + Optimization)
                {'max_range': 100.0},
                {'robust_kernel_scale': 1.0},                   
                {'kGridMarginFactor': 0.8},
                {'maxload': 100.0},
                {'maxCells': 100000},                           # Size of the High-resolution buffer
                {'lidar_type': "ouster"},                       # Lidar Sensor (ouster - hesai)
                {'leaf_size': -1.0},                            # VoxelFilter leaf size. Set to negative (e.g. -1.0) for no downsampling.
                {'timestamp_mode':"START_OF_SCAN"}              # Indicates if timestamp marks scan start or end. Defaults to 'START_OF_SCAN'; use 'END_OF_SCAN' only if required.
            ],
            arguments=[
                '--ros-args',
                '--log-level', 'dlio_node:=INFO'                # Set to DEBUG to see unwarping, downsampling, and timing information of each scan
            ]
        ),

        bag_play
    ])
