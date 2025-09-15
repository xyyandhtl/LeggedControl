from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    go2_launch_file = PathJoinSubstitution([
        get_package_share_directory('legged_control'),
        'launch',
        'go2.launch.py'
    ])

    control_mode_arg = DeclareLaunchArgument(
        'control_mode', default_value='sport',
        description="Control mode for Go2W: 'sport' or 'policy'")

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(go2_launch_file),
        launch_arguments={
            'control_mode': LaunchConfiguration('control_mode'),
            'config_name': 'sdk2_config_go2w.ini'
        }.items()
    )

    return LaunchDescription([
        control_mode_arg,
        included_launch
    ])
