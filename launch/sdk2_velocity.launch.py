from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    network_interface_arg = DeclareLaunchArgument(
        'network_interface', default_value='eth0',
        description='Network interface for Unitree DDS, e.g., eth0/enp3s0')

    control_rate_arg = DeclareLaunchArgument(
        'control_rate_hz', default_value='50',
        description='Control loop rate in Hz')

    stale_timeout_arg = DeclareLaunchArgument(
        'stale_timeout_s', default_value='1.0',
        description='Stop if no cmd_vel received for this many seconds')

    auto_stand_arg = DeclareLaunchArgument(
        'auto_stand', default_value='true',
        description='Call StandUp on start')

    max_vx_arg = DeclareLaunchArgument('max_vx', default_value='2.0')
    max_vy_arg = DeclareLaunchArgument('max_vy', default_value='0.5')
    max_wz_arg = DeclareLaunchArgument('max_wz', default_value='1.5')

    node = Node(
        package='unitree_control',
        executable='go2_node',
        name='go2_node',
        output='screen',
        parameters=[{
            'network_interface': LaunchConfiguration('network_interface'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            'stale_timeout_s': LaunchConfiguration('stale_timeout_s'),
            'auto_stand': LaunchConfiguration('auto_stand'),
            'max_vx': LaunchConfiguration('max_vx'),
            'max_vy': LaunchConfiguration('max_vy'),
            'max_wz': LaunchConfiguration('max_wz'),
        }]
    )

    return LaunchDescription([
        network_interface_arg,
        control_rate_arg,
        stale_timeout_arg,
        auto_stand_arg,
        max_vx_arg,
        max_vy_arg,
        max_wz_arg,
        node
    ])
