from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Declare LaunchConfiguration for bag_path
    bag_path = LaunchConfiguration('bag_path')

    # Play the bag if bag_path is provided (evaluates to true if bag_path is non-empty)
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

        # Iniciar RViz con el archivo de configuración
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', LaunchConfiguration('rviz_config_file')],
            output='screen'
        ),

        # Static Tf
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_base_laser_link_1',
            arguments=['0.0', '0.0', '0.0', '1.5708','0.0', '0.0', 'base_link', 'PandarXT-32'],
            output='screen'
        ),

       
       Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=['0.00173',      # x
                '0.00799',     # y
                '0.197481',     # z
                '0.18476',
                '-0.0947',     # pitch 
                '0.0421',    # roll
                'base_link',
                'mti3dk'
            ],
            output='screen' 
        ),

        # DLO3D node
        Node(
            package='D-LIO',
            executable='dlo3d_node',
            name='dlo3d_node',
            output='screen',
            remappings=[
                ('/dll3d_node/initial_pose', '/initialpose')
            ],
            parameters=[
                {'in_cloud_aux': '/os1_cloud_node2/points'},
                {'in_cloud': '/hesai/pandar'},
                {'hz_cloud': 10.0},
                {'in_imu': '/mti3dk/imu'},
                {'hz_imu': 90.0},
                {'calibration_time': 1.0},
                {'aux_lidar_en': False},
                {'gyr_dev': 5.236e-5},
                {'gyr_rw_dev': 3.4e-05},
                {'acc_dev': 7e-4},
                {'acc_rw_dev': 2.5e-9},
                {'base_frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'map_frame_id': 'map'},
                {'keyframe_dist':2.0},
                {'keyframe_rot': 30.0},
                {'tdfGridSizeX_low': -5.0},
                {'tdfGridSizeX_high': 90.0},
                {'tdfGridSizeY_low': -50.0},
                {'tdfGridSizeY_high': 20.0},
                {'tdfGridSizeZ_low': -15.0},
                {'tdfGridSizeZ_high': 15.0},
                {'solver_max_iter': 200},
                {'solver_max_threads': 16},
                {'min_range': 1.0},
                {'max_range': 100.0},
                {'pc_downsampling': 1},
                {'robust_kernel_scale': 1.0},
                {'kGridMarginFactor': 0.8},
                {'maxload': 100.0},
                {'maxCells': 100000}
            ]
        ),

        play_bag
    ])
