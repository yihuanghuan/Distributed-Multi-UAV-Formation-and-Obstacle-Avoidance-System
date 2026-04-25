from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 假设我们启动 3 架无人机
    num_uavs = 3

    # 全局可视化节点只需要启动一次
    visualizer_node = Node(
        package='trajectory_planner_py',
        executable='visualizer_node',
        name='trajectory_visualizer',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    for i in range(num_uavs):
        # PX4 多机 SITL 默认生成的命名空间是 px4_1, px4_2, px4_3...
        # 对应的 MAVLink SYS_ID 通常是 1, 2, 3...
        namespace = f'px4_{i+1}'
        sys_id = i + 1

        # 1. 启动 LADRC 控制器节点
        controller_node = Node(
            package='ladrc_controller',
            executable='ladrc_position_controller_node',
            namespace=namespace,
            name='ladrc_controller',
            parameters=[{
                'sys_id': sys_id,
                'hover_thrust': 0.55,  # 如果单机测试时改过这个值，请同步修改
                'omega_o_x': 8.0,
                'omega_o_y': 8.0,
                'omega_c_x': 2.2,
                'omega_c_y': 2.2,
                'omega_c_z': 2.8,
                'max_acceleration_x': 4.5,
                'max_acceleration_y': 4.5,
            }],
            output='screen'
        )

        # 2. 启动 Minisnap 规划器节点
        planner_node = Node(
            package='trajectory_planner_py',
            executable='planner_node',
            namespace=namespace,
            name='trajectory_planner',
            output='screen'
        )

        ld.add_action(controller_node)
        ld.add_action(planner_node)

    ld.add_action(visualizer_node)
    ld.add_action(rviz_node)

    return ld