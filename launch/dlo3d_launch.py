from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    map = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 
            'ros2', 'bag', 'play', '/home/upo/Datasets/eee_01/eee_01.db3', '--rate', '0.000001'
        ],
        output='screen',
        name='bag_play_process'
    )
    
    return LaunchDescription([
    
        # Tf 
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_base_laser_link',
            arguments=['-0.050', '0.0', '-0.055', '0', '0', '3.1412', 'base_link', 'sensor1/os_sensor'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_base_laser_link',
            arguments=['-0.55', '-0.03', '-0.05', '3.1412', '0.0', '-1.570', 'base_link', 'sensor2/os_sensor'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='new_link2',
            arguments=['0.0', '0.0', '0.0', '0', '0', '3.1412', 'base_link', 'imu'],
            output='screen'
        ),


        # DLO3D
        Node(
            package='dlo3d',
            executable='dlo3d_node',
            name='dlo3d_node',
            output='screen',
            remappings=[
                ('/dlo3d_node/initial_pose', '/initialpose')
            ],
            parameters=[
                {'in_cloud_aux': '/os1_cloud_node2/points'},
                {'in_cloud': '/os1_cloud_node1/points'},
                {'hz_cloud': 10.0},
                {'in_imu': '/imu/imu'},
                {'hz_imu': 388.0},
                {'aux_lidar_en': False},
                {'base_frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'map_frame_id': 'map'},
                {'keyframe_dist': 2.0},
                {'keyframe_rot': 25.0},
                {'tdf_grid_size_x': 80.0},
                {'tdf_grid_size_y': 60.0},
                {'tdf_grid_size_z': 40.0},
                {'solver_max_iter': 200},
                {'solver_max_threads': 20},
                {'min_range': 1.0},
                {'max_range': 100.0},
                {'pc_downsampling': 1},
                {'robust_kernel_scale': 1.0}   
            ]
        ),

        map

    ])
