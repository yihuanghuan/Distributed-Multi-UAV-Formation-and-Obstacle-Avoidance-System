#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class TrajectoryVisualizer(Node):
    def __init__(self, num_uavs=3):
        super().__init__('trajectory_visualizer')
        self.paths = {}
        self.pubs = {}
        self.subs = {}
        self.initial_positions = {}
        self.reference_initial = None
        self.current_positions = {}

        self.marker_pub = self.create_publisher(MarkerArray, '/rviz_uav_markers', 10)

        # UAV 颜色表 (r, g, b)
        self.uav_colors = {
            1: (0.98, 0.30, 0.30),
            2: (0.22, 0.58, 0.98),
            3: (0.15, 0.78, 0.35),
        }

        # 当多机局部坐标都从 (0,0,0) 开始时，使用该间距在 RViz 中拉开显示
        self.declare_parameter('fallback_spacing', 2.0)
        self.fallback_spacing = float(self.get_parameter('fallback_spacing').value)

        # 如果初始偏移过小，认为多机坐标系重合，启用兜底拉开
        self.overlap_threshold_xy = 0.2

        # 遍历为每架无人机创建独立的订阅器和发布器
        for i in range(1, num_uavs + 1):
            ns = f'/px4_{i}'
            
            # 发布给 RViz 的 Path 话题
            self.pubs[i] = self.create_publisher(Path, f'{ns}/rviz_path', 10)
            
            # 初始化 Path 对象
            self.paths[i] = Path()
            self.paths[i].header.frame_id = 'map' # RViz 中的全局固定坐标系

            # 订阅底层飞控里程计
            self.subs[i] = self.create_subscription(
                VehicleOdometry,
                f'{ns}/fmu/out/vehicle_odometry',
                lambda msg, uav_id=i: self.odom_callback(msg, uav_id),
                rclpy.qos.qos_profile_sensor_data
            )

    def to_enu(self, msg):
        # PX4 NED -> RViz ENU
        return [
            float(msg.position[1]),
            float(msg.position[0]),
            float(-msg.position[2]),
        ]

    def get_display_offset(self, uav_id):
        init = self.initial_positions.get(uav_id)
        if init is None:
            return [0.0, 0.0, 0.0]

        if self.reference_initial is None:
            self.reference_initial = init.copy()

        # 优先使用相对首机(参考机)的初始偏移
        dx = init[0] - self.reference_initial[0]
        dy = init[1] - self.reference_initial[1]
        dz = init[2] - self.reference_initial[2]

        # 若初始偏移几乎重合（例如每架机都在自身局部原点），使用固定间距拉开
        if uav_id != 1 and abs(dx) + abs(dy) < self.overlap_threshold_xy:
            dx = self.fallback_spacing * float(uav_id - 1)
            dy = 0.0

        return [dx, dy, dz]

    def odom_callback(self, msg, uav_id):
        enu = self.to_enu(msg)

        if uav_id not in self.initial_positions:
            self.initial_positions[uav_id] = enu.copy()

        init = self.initial_positions[uav_id]
        offset = self.get_display_offset(uav_id)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # 以各自初始点为局部零点，再按初始偏移映射到同一显示坐标系
        pose.pose.position.x = (enu[0] - init[0]) + offset[0]
        pose.pose.position.y = (enu[1] - init[1]) + offset[1]
        pose.pose.position.z = (enu[2] - init[2]) + offset[2]

        self.current_positions[uav_id] = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )

        # 将当前点追加到历史路径中
        self.paths[uav_id].poses.append(pose)
        self.paths[uav_id].header.stamp = pose.header.stamp
        
        # 发布更新后的路径
        self.pubs[uav_id].publish(self.paths[uav_id])
        self.publish_markers(pose.header.stamp)

    def publish_markers(self, stamp):
        marker_array = MarkerArray()

        for uav_id in sorted(self.current_positions.keys()):
            x, y, z = self.current_positions[uav_id]
            r, g, b = self.uav_colors.get(uav_id, (1.0, 1.0, 1.0))

            body = Marker()
            body.header.frame_id = 'map'
            body.header.stamp = stamp
            body.ns = 'uav_body'
            body.id = uav_id
            body.type = Marker.SPHERE
            body.action = Marker.ADD
            body.pose.position.x = x
            body.pose.position.y = y
            body.pose.position.z = z
            body.pose.orientation.w = 1.0
            body.scale.x = 0.35
            body.scale.y = 0.35
            body.scale.z = 0.20
            body.color.a = 0.95
            body.color.r = r
            body.color.g = g
            body.color.b = b

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = stamp
            label.ns = 'uav_label'
            label.id = 100 + uav_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 0.45
            label.pose.orientation.w = 1.0
            label.scale.z = 0.30
            label.color.a = 1.0
            label.color.r = r
            label.color.g = g
            label.color.b = b
            label.text = f'UAV {uav_id}'

            marker_array.markers.append(body)
            marker_array.markers.append(label)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()