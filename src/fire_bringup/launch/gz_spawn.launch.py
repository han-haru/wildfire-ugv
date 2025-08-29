from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro

def launch_setup(context, *args, **kwargs):
    desc_pkg    = get_package_share_directory('fire_description')
    bringup_pkg = get_package_share_directory('fire_bringup')

    rviz_cfg   = PathJoinSubstitution([bringup_pkg, 'rviz', 'robot.rviz']).perform(context)
    mesh_scale = LaunchConfiguration('mesh_scale').perform(context)

    xacro_file = os.path.join(desc_pkg, 'urdf', 'fire_robot.xacro')
    urdf_doc   = xacro.process_file(xacro_file, mappings={'mesh_scale': mesh_scale})
    robot_desc = urdf_doc.toxml()

    world_file = os.path.join(bringup_pkg, 'worlds', 'test_route.sdf')
    if not os.path.exists(world_file):
        raise RuntimeError(f"[fire_bringup] World file not found: {world_file}")

    # Save for debug
    with open('/tmp/fire_robot.urdf', 'w') as f:
        f.write(robot_desc)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        output='screen'
    )

    # Start Fortress with the world file
    gz = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '3', world_file],
        output='screen'
    )

    # Spawn with a fixed name (no renaming)
    spawn = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'test_world',
                '-name', 'fire_robot',
                '-z', '0.2',
                '-string', robot_desc
            ],
            output='screen'
        )]
    )

    # Bridge real topics (model-scoped)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # One-way GZ->ROS for clock (safer)
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/mag@sensor_msgs/msg/MagneticField@ignition.msgs.Magnetometer'
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        output='screen'
    )

    return [gz, rsp, spawn, bridge, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mesh_scale', default_value='0.001 0.001 0.001'),
        OpaqueFunction(function=launch_setup)
    ])