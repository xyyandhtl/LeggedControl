from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('legged_control')

    # Declare launch arguments
    network_interface_arg = DeclareLaunchArgument(
        'network_interface', default_value='enx0826ae330a17',
        description='Network interface for Unitree DDS')

    control_mode_arg = DeclareLaunchArgument(
        'control_mode', default_value='sport',
        description="Control mode to launch: 'sport' or 'policy'")

    config_name_arg = DeclareLaunchArgument(
        'config_name', default_value='sdk2_config_go2.ini',
        description='Config file name for policy mode')

    # Define the node
    control_node = Node(
        package='legged_control',
        executable='go2_node',
        name='go2_control_node', # Node name can be generic
        output='screen',
        parameters=[
            # Pass all parameters that might be used by either controller
            {'network_interface': LaunchConfiguration('network_interface')},
            {'control_mode': LaunchConfiguration('control_mode')},
            {'config_path': PathJoinSubstitution([package_path, 'config', LaunchConfiguration('config_name')])},

            # Common parameters that both controllers might use
            {'timeout_s': 3.0},
            {'stale_timeout_s': 1.0},
            {'max_vx': 2.0},
            {'max_vy': 0.5},
            {'max_wz': 1.5},

            # Sport mode specific (will be ignored by policy controller)
            {'auto_stand': False},
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value=''),
        SetEnvironmentVariable(name='CYCLONEDDS_URI', value=''),
        network_interface_arg,
        control_mode_arg,
        config_name_arg,
        control_node
    ])