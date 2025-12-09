from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Declare LaunchConfiguration for bag_path
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
            parameters=[
                {'in_cloud_aux': '/os1_cloud_node2/points'},
                {'in_cloud': '/boxi/hesai/points'},
                {'hz_cloud': 10.0},
                {'in_imu': '/boxi/stim320/imu'},
                {'hz_imu': 500.0},
                {'calibration_time': 1.0},
                {'aux_lidar_en': False},
                {'gyr_dev': 0.05},
                {'gyr_rw_dev': 3.4e-09},
                {'acc_dev': 0.05},
                {'acc_rw_dev': 2.5e-9},
                {'base_frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'map_frame_id': 'map'},
                {'keyframe_dist': 1.0},
                {'keyframe_rot': 1.7},
                {'tdfGridSizeX_low': -30.0},
                {'tdfGridSizeX_high': 100.0},
                {'tdfGridSizeY_low': -100.0},
                {'tdfGridSizeY_high': 100.0},
                {'tdfGridSizeZ_low': -100.0},
                {'tdfGridSizeZ_high': 100.0},
                {'solver_max_iter': 200},
                {'solver_max_threads': 20},
                {'min_range': 0.5},
                {'max_range': 200.0},
                {'pc_downsampling': 1},
                {'robust_kernel_scale': 15.0},
                {'kGridMarginFactor': 0.8},
                {'maxload':15.0},
                {'maxCells': 200000},
                {'lidar_type': "hesai"},
                {'leaf_size': -1.0}
            ],
            arguments=[
                '--ros-args',
                '--log-level', 'dlio_node:=INFO'
            ]
        ),

        play_bag
    ])
