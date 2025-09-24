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

    gt_pub = ExecuteProcess(
        cmd=[
            'python3','-u','-c',
            (
                "import rclpy, math\n"
                "from rclpy.node import Node\n"
                "from geometry_msgs.msg import TransformStamped\n"
                "from sensor_msgs.msg import PointCloud2\n"
                "POSE_FILE='/home/ros/ros2_ws/mai_city/bin/poses/00.txt'\n"
                "poses=[]\n"
                "with open(POSE_FILE,'r') as f:\n"
                "  for line in f:\n"
                "    vals=[float(x) for x in line.strip().split()]\n"
                "    if len(vals)==12: poses.append(vals)\n"
                "def mat_to_quat(r00,r01,r02,r10,r11,r12,r20,r21,r22):\n"
                "  tr=r00+r11+r22\n"
                "  if tr>0:\n"
                "    S=math.sqrt(tr+1.0)*2.0\n"
                "    qw=0.25*S\n"
                "    qx=(r21-r12)/S\n"
                "    qy=(r02-r20)/S\n"
                "    qz=(r10-r01)/S\n"
                "  elif r00>r11 and r00>r22:\n"
                "    S=math.sqrt(1.0+r00-r11-r22)*2.0\n"
                "    qw=(r21-r12)/S\n"
                "    qx=0.25*S\n"
                "    qy=(r01+r10)/S\n"
                "    qz=(r02+r20)/S\n"
                "  elif r11>r22:\n"
                "    S=math.sqrt(1.0-r00+r11-r22)*2.0\n"
                "    qw=(r02-r20)/S\n"
                "    qx=(r01+r10)/S\n"
                "    qy=0.25*S\n"
                "    qz=(r12+r21)/S\n"
                "  else:\n"
                "    S=math.sqrt(1.0-r00-r11+r22)*2.0\n"
                "    qw=(r10-r01)/S\n"
                "    qx=(r02+r20)/S\n"
                "    qy=(r12+r21)/S\n"
                "    qz=0.25*S\n"
                "  return qx,qy,qz,qw\n"
                "class GTPub(Node):\n"
                "  def __init__(self):\n"
                "    super().__init__('gt_pub')\n"
                "    self.i=0\n"
                "    self.pub=self.create_publisher(TransformStamped,'/gt/transform',100)\n"
                "    self.sub=self.create_subscription(PointCloud2,'/velodyne_points',self.cb,10)\n"
                "  def cb(self,msg):\n"
                "    if not poses: return\n"
                "    i=min(self.i,len(poses)-1)\n"
                "    r00,r01,r02,tx, r10,r11,r12,ty, r20,r21,r22,tz = poses[i]\n"
                "    qx,qy,qz,qw = mat_to_quat(r00,r01,r02,r10,r11,r12,r20,r21,r22)\n"
                "    t=TransformStamped()\n"
                "    t.header.stamp=msg.header.stamp\n"
                "    t.header.frame_id='odom'\n"
                "    t.child_frame_id='base_link'\n"
                "    t.transform.translation.x=tx\n"
                "    t.transform.translation.y=ty\n"
                "    t.transform.translation.z=tz\n"
                "    t.transform.rotation.x=qx\n"
                "    t.transform.rotation.y=qy\n"
                "    t.transform.rotation.z=qz\n"
                "    t.transform.rotation.w=qw\n"
                "    self.pub.publish(t)\n"
                "    self.i+=1\n"
                "rclpy.init(); n=GTPub(); rclpy.spin(n)\n"
            )
        ],
        output='screen'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description='Ruta al bag .db3 o carpeta. Si se deja vacío, el nodo espera datos en vivo.'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value='/home/ros/ros2_ws/src/phd_dev/launch/default.rviz',
            description='Archivo RViz a cargar.'
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_cfg],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_to_laser',
            arguments=[
                # xyz  rpy  frame_id  child_frame_id
                '0', '0', '0',   '0', '0', '0',   'base_link', 'os_sensor'
            ],
            output='screen'
        ),

        Node(
            package='db_tsdf',
            executable='db_tsdf_mapper',
            name='db_tsdf_mapper',
            output='screen',
            parameters=[
                {'use_sim_time':      True},

                {'in_cloud':       '/velodyne_points'},
                {'in_tf':         '/gt/transform'},
                {'use_tf':       True},
                {'pc_downsampling':   1},
                {'min_range':         0.0},
                {'max_range':        100.0},
                {'base_frame_id':     'base_link'},
                {'odom_frame_id':     'odom'},
                {'tdfGridSizeX_low':  -20.0},
                {'tdfGridSizeX_high':  100.0},
                {'tdfGridSizeY_low':  -30.0},
                {'tdfGridSizeY_high':  30.0},
                {'tdfGridSizeZ_low':   -3.5},
                {'tdfGridSizeZ_high':  3.5},
                {'tdf_grid_res':      0.05},
                ]
        ),

        gt_pub,
        bag_play
    ])
