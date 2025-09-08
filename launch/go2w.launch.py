from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('legged_control')
    config_file_path = PathJoinSubstitution([package_path, 'config', 'sdk2_config_go2w.ini'])

    network_interface_arg = DeclareLaunchArgument(
        'network_interface', default_value='eth0',
        description='Network interface for Unitree DDS, e.g., eth0/enp3s0')

    timeout_arg = DeclareLaunchArgument(
        'timeout_s', default_value='3.0',
        description='sport client timeout in seconds')

    control_rate_arg = DeclareLaunchArgument(
        'control_rate_hz', default_value='15',
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
        package='legged_control',
        executable='go2_node',
        name='go2_node',
        output='screen',
        parameters=[{
            'network_interface': LaunchConfiguration('network_interface'),
            'timeout_s': LaunchConfiguration('timeout_s'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            'stale_timeout_s': LaunchConfiguration('stale_timeout_s'),
            'auto_stand': LaunchConfiguration('auto_stand'),
            'max_vx': LaunchConfiguration('max_vx'),
            'max_vy': LaunchConfiguration('max_vy'),
            'max_wz': LaunchConfiguration('max_wz'),
            'config_path': config_file_path,
        }]
    )

    return LaunchDescription([
        network_interface_arg,
        timeout_arg,
        control_rate_arg,
        stale_timeout_arg,
        auto_stand_arg,
        max_vx_arg,
        max_vy_arg,
        max_wz_arg,
        node
    ])
