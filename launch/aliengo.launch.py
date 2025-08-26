from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('legged_control')
    config_file_path = PathJoinSubstitution([package_path, 'config', 'sdk1_config.ini'])

    target_ip_arg = DeclareLaunchArgument(
        'target_ip', default_value='192.168.123.10',
        description='Target robot IP address for UDP communication')
    
    target_port_arg = DeclareLaunchArgument(
        'target_port', default_value='8007',
        description='Target robot port for UDP communication')
    
    local_port_arg = DeclareLaunchArgument(
        'local_port', default_value='8082',
        description='Local port for UDP communication')
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate_hz', default_value='15',
        description='Control loop rate in Hz (50Hz for policy control)')
    
    stale_timeout_arg = DeclareLaunchArgument(
        'stale_timeout_s', default_value='0.5',
        description='Stop if no cmd_vel received for this many seconds')
    
    max_vx_arg = DeclareLaunchArgument('max_vx', default_value='0.8')
    max_vy_arg = DeclareLaunchArgument('max_vy', default_value='0.5')
    max_wz_arg = DeclareLaunchArgument('max_wz', default_value='1.0')
    
    node = Node(
        package='legged_control',
        executable='aliengo_node',
        name='aliengo_node',
        output='screen',
        parameters=[{
            'target_ip': LaunchConfiguration('target_ip'),
            'target_port': LaunchConfiguration('target_port'),
            'local_port': LaunchConfiguration('local_port'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            'stale_timeout_s': LaunchConfiguration('stale_timeout_s'),
            'max_vx': LaunchConfiguration('max_vx'),
            'max_vy': LaunchConfiguration('max_vy'),
            'max_wz': LaunchConfiguration('max_wz'),
            'config_path': config_file_path,
        }]
    )
    
    return LaunchDescription([
        target_ip_arg,
        target_port_arg,
        local_port_arg,
        control_rate_arg,
        stale_timeout_arg,
        max_vx_arg,
        max_vy_arg,
        max_wz_arg,
        node
    ])
