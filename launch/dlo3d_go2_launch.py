import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # Parámetro para la ruta del bag
    bag_path = LaunchConfiguration('bag_path')

    # Proceso para reproducir el bag sólo si bag_path != ''
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

    # Directorio donde está este launch file
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    rviz_config_file = os.path.join(launch_file_dir, 'default.rviz')

    return LaunchDescription([

        # Argumento para ruta del bag
        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description=(
                'Full path to the .db3 file to play with ros2 bag. '
                'If empty, no bag will be played and se esperará datos externos.'
            )
        ),

        # Argumento para ruta del RViz config
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=rviz_config_file,
            description='Full path to the RViz config file.'
        ),

        # Lanzar RViz con la configuración indicada
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rviz2', 'rviz2',
                '-d', LaunchConfiguration('rviz_config_file')
            ],
            output='screen'
        ),

        # Publicar transform estático base_link → os1_lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_base_laser_link',
            arguments=[
                '0.0', '0.0', '0.0',    # xyz
                '0.0', '0.0', '0.0',    # rpy
                'base_link', 'os_lidar'
            ],
            output='screen'
        ),

        # Publicar transform estático base_link → imu
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_base_imu_link',
            arguments=[
                '0.0', '0.0', '0.0',    # xyz
                '0.0', '0.0', '0.0',    # rpy
                'base_link', 'os_imu'
            ],
            output='screen'
        ),

        # Nodo principal DLO3D
        Node(
            package='dlo3d',
            executable='dlo3d_node',
            name='dlo3d_node',
            output='screen',
            remappings=[
                ('/dll3d_node/initial_pose', '/initialpose')
            ],
            parameters=[
                {'in_cloud_aux': '/os_cloud_node/points'},
                {'in_cloud': '/ouster/points'},
                {'hz_cloud': 10.0},
                {'in_imu': '/ouster/imu'},
                {'hz_imu': 100.0},
                {'calibration_time': 2.0},
                {'aux_lidar_en': False},
                {'gyr_dev': 0.00345},
                {'gyr_rw_dev': 0.00244},
                {'acc_dev': 0.0222},
                {'acc_rw_dev': 0.00157},
                {'base_frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'map_frame_id': 'map'},
                {'keyframe_dist': 2.0},
                {'keyframe_rot': 25.0},
                {'tdfGridSizeX_low': -30.0},
                {'tdfGridSizeX_high': 30.0},
                {'tdfGridSizeY_low': -40.0},
                {'tdfGridSizeY_high': 40.0},
                {'tdfGridSizeZ_low': -10.0},
                {'tdfGridSizeZ_high': 15.0},
                {'solver_max_iter': 500},
                {'solver_max_threads': 20},
                {'min_range': 1.0},
                {'max_range': 100.0},
                {'pc_downsampling': 1},
                {'robust_kernel_scale': 1.0},
                {'kGridMarginFactor': 0.8},
                {'maxload': 100.0},
                {'maxCells': 100000}
            ]
        ),

        # Finalmente, reproducir el bag si se ha proporcionado ruta
        bag_play
    ])
