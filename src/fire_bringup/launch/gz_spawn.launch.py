# fire_bringup/launch/gz_spawn_fortress.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro  # ← 중요: python3-xacro(ros-humble-xacro) 패키지 필요

def launch_setup(context, *args, **kwargs):
    # 패키지 경로
    desc_pkg    = get_package_share_directory('fire_description')
    bringup_pkg = get_package_share_directory('fire_bringup')

    # 런치 인자 실제 값 읽기
    use_rviz   = LaunchConfiguration('use_rviz').perform(context)
    rviz_cfg   = PathJoinSubstitution([bringup_pkg, 'rviz', 'robot.rviz']).perform(context)
    mesh_scale = LaunchConfiguration('mesh_scale').perform(context)   # 예: "0.001 0.001 0.001"

    # 1) xacro를 파이썬에서 직접 실행 → 문자열로 URDF 얻기 (토큰 분리 이슈 0%)
    xacro_file = os.path.join(desc_pkg, 'urdf', 'fire_robot.xacro')
    # xacro arg는 dict로 넘김: 이름 → 문자열
    urdf_doc   = xacro.process_file(xacro_file, mappings={'mesh_scale': mesh_scale})
    robot_desc = urdf_doc.toxml()

    # xacro 처리 후
    with open('/tmp/fire_robot.urdf', 'w') as f:
        f.write(robot_desc)

    # 2) robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        output='screen'
    )

    # 3) joint_state_publisher 바로 아래에 추가
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        output='screen'
    )

    # 4) Gazebo Fortress 실행
    gz = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '3', 'empty.sdf'],
        output='screen'
    )

    # 5) 스폰: -string에 **그냥 URDF 문자열**을 바로 넣기 (Command 안 씀)
    spawn = TimerAction(
        period=1.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'empty',
                '-name', 'fire_robot',
                '-allow_renaming', 'true',
                '-z', '0.2',
                '-string', robot_desc
            ],
            output='screen'
        )]
    )

    # 6) clock 브리지 
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/mag@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer'
        ],
        output='screen'
    )

    # 7) RViz 
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 반환: 액션 리스트
    actions = [gz, rsp, spawn, clock_bridge, rviz]
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=''),  # 필요 시 덮어쓰기용
        DeclareLaunchArgument('mesh_scale', default_value='0.001 0.001 0.001'),
        OpaqueFunction(function=launch_setup)  # ← 여기서 전부 세팅
    ])
