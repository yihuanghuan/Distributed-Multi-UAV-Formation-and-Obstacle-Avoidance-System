#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped, PoseStamped, TwistStamped, AccelStamped

from trajectory_planner_py.min_snap_trajectory_generators import (
    optimizeTrajectory,
    YawMinAccTrajectory,
)
from trajectory_planner_py.waypoints import figure_8


class TrajectoryNode(Node):

    def __init__(self) -> None:
        super().__init__("trajectory_generator")

        self._pose_pub = self.create_publisher(PoseStamped, "target_pose", 10)
        self._twist_pub = self.create_publisher(TwistStamped, "target_twist", 10)
        self._accel_pub = self.create_publisher(AccelStamped, "target_accel", 10)

        self.create_subscription(Vector3Stamped, "att_rpy", self._rpy_cb, 10)
        self.create_subscription(Vector3Stamped, "position", self._pos_cb, 10)

        self._curr_pos = np.zeros(3)   # current vehicle position  (x,y,z)
        self._curr_rpy = np.zeros(3)   # current vehicle attitude (roll, pitch, yaw)

        # trajectory generators
        t_wp, xyz_wp, rpy_wp = figure_8()
        self._xyz_traj = optimizeTrajectory(
            xyz_wp,
            t_wp,
            optim_target="poly-coeff",
            poly_order=7,
            floating_cubes=None,
            t_cubes=None,
        )
        self._yaw_traj = YawMinAccTrajectory(
            yaw_waypoints=rpy_wp[:, 2],
            t_waypoints=t_wp,
        )

        self._trajectory_duration = t_wp[-1]
        self.get_logger().info(f"Trajectory initialized with duration: {self._trajectory_duration} seconds.")

        self._hover_duration = 15.0
        self.get_logger().info(f"Waiting {self._hover_duration}s for controller takeoff...")

        self._start_ns = self.get_clock().now().nanoseconds
        self._log_counter = 0
        update_hz = 100.0
        self.create_timer(1.0 / update_hz, self._on_timer)

    def _on_timer(self) -> None:
        t_elapsed = (self.get_clock().now().nanoseconds - self._start_ns) * 1e-9

        # [修改] “悬停起飞，然后执行” 逻辑
        if t_elapsed < self._hover_duration:
            # 阶段 1: 正在悬停。控制器正在起飞。
            # 始终发送 t=0 (即 [0,0,-2]) 的指令
            t_now = 0.0
        else:
            # 阶段 2: 起飞完成。开始8字轨迹。
            t_mission = t_elapsed - self._hover_duration
            if self._trajectory_duration > 0:
                t_now = t_mission % self._trajectory_duration
            else:
                t_now = 0.0 # 避免除以零

        pos, vel, acc, _, _ = self._xyz_traj.eval(t_now)
        yaw, _, _ = self._yaw_traj.eval(t_now, des_pos=pos, curr_pos=self._curr_pos)
        target_rpy = np.array([0.0, 0.0, yaw])

        self._publish_pose(pos, target_rpy)
        self._publish_twist(vel)
        self._publish_accel(acc)

        # [修改] 增加日志节流
        self._log_counter += 1
        # 因为 update_hz = 100.0, 所以我们需要计数到 100
        if self._log_counter >= 100: 
            # self.get_logger().info(f"Planner publishing commands: position={pos.round(3)}, attitude={target_rpy.round(3)}")
            self._log_counter = 0

    def _rpy_cb(self, msg: Vector3Stamped) -> None:
        """Store the vehicle’s current roll-pitch-yaw."""
        self._curr_rpy[:] = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _pos_cb(self, msg: Vector3Stamped) -> None:
        """Store the vehicle’s current XYZ position."""
        self._curr_pos[:] = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _publish_twist(self, lin_vel: np.ndarray) -> TwistStamped:
        msg = TwistStamped()
        msg.twist.linear.x = lin_vel[0]
        msg.twist.linear.y = lin_vel[1]
        msg.twist.linear.z = lin_vel[2]
        self._twist_pub.publish(msg)
        return msg

    def _publish_accel(self, lin_acc: np.ndarray) -> AccelStamped:
        msg = AccelStamped()
        msg.accel.linear.x = lin_acc[0]
        msg.accel.linear.y = lin_acc[1]
        msg.accel.linear.z = lin_acc[2]
        self._accel_pub.publish(msg)
        return msg

    def _publish_pose(self, xyz: np.ndarray, rpy: np.ndarray) -> PoseStamped:
        quat_xyzw = Rotation.from_euler("XYZ", rpy).as_quat()  # (x,y,z,w)
        
        # 创建 *一个* 消息对象
        msg = PoseStamped()
        
        # 填充 *这一个* 对象
        msg.pose.position.x = xyz[0]
        msg.pose.position.y = xyz[1]
        msg.pose.position.z = xyz[2]
        msg.pose.orientation.w = quat_xyzw[3]
        msg.pose.orientation.x = quat_xyzw[0]
        msg.pose.orientation.y = quat_xyzw[1]
        msg.pose.orientation.z = quat_xyzw[2]
        
        # 发布 *这一个* 对象
        self._pose_pub.publish(msg)
        return msg

def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
