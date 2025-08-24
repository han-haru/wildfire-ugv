# fire_bringup/launch/controllers.launch.py
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    bringup_pkg = get_package_share_directory('fire_bringup')
    ros2_control_params = os.path.join(bringup_pkg, 'config', 'controller.yaml')

    cm = '/controller_manager'
    cm_arg = ['--controller-manager', cm]
    cm_timeout = ['--controller-manager-timeout', '60']

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'] + cm_arg + cm_timeout,
        output='screen'
    )

    arm = TimerAction(period=0.5, actions=[Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_position_controller',
            '--param-file', ros2_control_params,
        ] + cm_arg + cm_timeout,
        output='screen'
    )])

    blade = TimerAction(period=1.0, actions=[Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'blade_velocity_controller',
            '--param-file', ros2_control_params,
        ] + cm_arg + cm_timeout,
        output='screen'
    )])

    #velocity_controllers/JointGroupVelocityController
    #★ 중요: diff_drive는 타입까지 강제
    # diff = TimerAction(period=1.5, actions=[Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'diff_drive_controller',
    #         #'--controller-type', 'diff_drive_controller/DiffDriveController',
    #         '--param-file', ros2_control_params,
    #     ] + cm_arg + cm_timeout,
    #     output='screen'
    # )])

    diff = TimerAction(period=1.5, actions=[Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'wheel_test_controller',
            #'--controller-type', 'diff_drive_controller/DiffDriveController',
            '--param-file', ros2_control_params,
        ] + cm_arg + cm_timeout,
        output='screen'
    )])


    return LaunchDescription([jsb, arm, blade, diff])

    # bringup_pkg = get_package_share_directory('fire_bringup')
    # ros2_control_params = os.path.join(bringup_pkg, 'config', 'controller.yaml')

    # # 공통 옵션들
    # cm_node = '/controller_manager'
    # cm_arg = ['--controller-manager', cm_node]
    # cm_timeout = ['--controller-manager-timeout', '60']  # 필요시 60초로 넉넉히


    # jsb = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )
    # arm = TimerAction(period=0.5, actions=[Node(
    #     package='controller_manager', executable='spawner',
    #     arguments=['arm_position_controller', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )])
    # blade = TimerAction(period=1.0, actions=[Node(
    #     package='controller_manager', executable='spawner',
    #     arguments=['blade_velocity_controller', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )])
    # # 4) Diff drive controller  ← 여기서도 --param-file 꼭 전달
    # diff = TimerAction(period=1.5, actions=[Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'diff_drive_controller',
    #         '--param-file', ros2_control_params
    #     ] + cm_arg + cm_timeout,
    #     output='screen'
    # )])
    # return LaunchDescription([jsb, arm, blade, diff])
