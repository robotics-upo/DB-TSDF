from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os


def generate_launch_description():

    bag_path   = LaunchConfiguration('bag_path')
    rviz_cfg   = LaunchConfiguration('rviz_config_file')

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    default_rviz = os.path.join(launch_dir, 'default.rviz')

    bag_play = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'ros2', 'bag', 'play', bag_path,
            '--clock',             
            '--rate', '1.0'
        ],
        output='screen',
        condition=IfCondition(PythonExpression(['"', bag_path, '" != ""']))
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description='Ruta al bag .db3 o carpeta. Si se deja vacío, el nodo espera datos en vivo.'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz,
            description='Archivo RViz a cargar.'
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_cfg],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_vicon_to_camera',
            #   x     y      z       qx     qy     qz     qw    frame_id  child_frame_id
            arguments=[
                '0.00114049', '0.0450936', '0.0430765',
                '0.0924132',  '0.0976455', '0.0702949', '0.988425',
                'kinect',
                'camera_optical'
            ],
            output='screen'
        ),

        Node(
            package='dlo3d',
            executable='dlo3d_mapper',
            name='dlo3d_mapper',
            output='screen',
            parameters=[
                {'use_sim_time':      True},

                # Dataset: cow_and_ladyhumble  wlenfkljhnewf
                {'in_cloud':          '/camera/depth_registered/points'},
                {'in_tf':            '/kinect/vrpn_client/estimated_transform'},
                {'use_tf':       True},
                {'pc_downsampling':   1},
                {'min_range':         0.0},
                {'max_range':        100.0},
                {'base_frame_id':     'base_link'},
                {'odom_frame_id':     'odom'},
                # {'tdfGridSizeX_low':  -10.0},
                # {'tdfGridSizeX_high':  10.0},
                # {'tdfGridSizeY_low':  -10.0},
                # {'tdfGridSizeY_high':  10.0},
                # {'tdfGridSizeZ_low':   -5.0},
                # {'tdfGridSizeZ_high':  5.0},
                # {'tdf_grid_res':      0.02},

                {'tdfGridSizeX_low':  -8.0},
                {'tdfGridSizeX_high':  7.0},
                {'tdfGridSizeY_low':  -5.0},
                {'tdfGridSizeY_high':  5.0},
                {'tdfGridSizeZ_low':   -2.0},
                {'tdfGridSizeZ_high':  5.0},
                {'tdf_grid_res':      0.01},
                
                ]
        ),

        bag_play
    ])
