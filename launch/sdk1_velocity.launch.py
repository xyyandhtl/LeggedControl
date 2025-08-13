from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
        'control_rate_hz', default_value='500',
        description='Control loop rate in Hz (500Hz for low-level control)')
    
    stale_timeout_arg = DeclareLaunchArgument(
        'stale_timeout_s', default_value='0.5',
        description='Stop if no cmd_vel received for this many seconds')
    
    max_vx_arg = DeclareLaunchArgument('max_vx', default_value='2.0')
    max_vy_arg = DeclareLaunchArgument('max_vy', default_value='1.0')
    max_wz_arg = DeclareLaunchArgument('max_wz', default_value='2.0')
    
    node = Node(
        package='unitree_control',
        executable='sdk1_velocity_node',
        name='sdk1_velocity_node',
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
