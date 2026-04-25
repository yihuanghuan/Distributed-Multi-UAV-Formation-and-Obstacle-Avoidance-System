from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ladrc_controller')
    config_file = os.path.join(pkg_share, 'config', 'ladrc_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=config_file,
            description='Path to LADRC parameters file'
        ),
        
        Node(
            package='ladrc_controller',
            executable='ladrc_position_controller_node',
            name='ladrc_position_controller',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),
    ])