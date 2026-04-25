from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # [修改] 寻找到新的 ladrc_controller 包
    ladrc_controller_pkg = get_package_share_directory('ladrc_controller')
    
    # [修改] 定义 ladrc_controller 的启动文件路径
    ladrc_controller_launch_path = os.path.join(
        ladrc_controller_pkg,
        'launch',
        'ladrc_controller.launch.py' 
    )

    # [修改] 加载 ladrc_controller 的启动文件
    ladrc_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ladrc_controller_launch_path)
    )

    # minisnap 规划器节点
    planner_node = Node(
        package='trajectory_planner_py',
        executable='planner_node',
        name='planner_node',
        output='screen' # 增加输出
    )

    return LaunchDescription([
        ladrc_controller_launch,  # <-- [修改] 启动 LADRC
        planner_node,             # <-- [修改] 立即启动规划器 (不再需要 TimerAction)
    ])