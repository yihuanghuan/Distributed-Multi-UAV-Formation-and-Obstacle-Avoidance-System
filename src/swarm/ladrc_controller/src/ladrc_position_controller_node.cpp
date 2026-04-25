#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp> // <-- [核心修改 1] 使用底层的姿态推力指令
#include <px4_msgs/msg/vehicle_command.hpp>
#include "ladrc_controller/ladrc_core.hpp"

#include <Eigen/Dense> // <-- [核心修改 2] 引入 Eigen 进行底层姿态映射解算
#include <cmath>
#include <chrono>
#include <atomic>

using namespace std::chrono_literals;

enum class FlightState { INIT, ARMING, SETTING_OFFBOARD, RUNNING_TRAJECTORY };

class LADRCPositionControllerNode : public rclcpp::Node
{
public:
  LADRCPositionControllerNode() : Node("ladrc_position_controller")
  {
    this->declare_parameter("control_frequency", 50.0);
    this->declare_parameter("omega_o_x", 5.0);
    this->declare_parameter("omega_o_y", 5.0);
    this->declare_parameter("omega_o_z", 14.0);
    this->declare_parameter("omega_c_x", 1.5);
    this->declare_parameter("omega_c_y", 1.5);
    this->declare_parameter("omega_c_z", 3.2);
    this->declare_parameter("b0_x", 1.0);
    this->declare_parameter("b0_y", 1.0);
    this->declare_parameter("b0_z", 0.7);
    this->declare_parameter("max_acceleration_x", 3.0);
    this->declare_parameter("max_acceleration_y", 3.0);
    this->declare_parameter("max_acceleration_z", 4.5);
    
    // [新增] 悬停推力基础参数 (0~1之间，与具体的无人机模型相关，iris通常为0.5~0.6)
    this->declare_parameter("hover_thrust", 0.55); 

    // [多机改造 1] 声明系统 ID 参数，用于区分不同的无人机
    this->declare_parameter("sys_id", 1);

    double control_freq = this->get_parameter("control_frequency").as_double();
    dt_ = 1.0 / control_freq;

    initializeControllers();

    reference_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "target_pose", rclcpp::QoS(10), std::bind(&LADRCPositionControllerNode::poseCallback, this, std::placeholders::_1));
    reference_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "target_twist", rclcpp::QoS(10), std::bind(&LADRCPositionControllerNode::twistCallback, this, std::placeholders::_1));
    reference_accel_sub_ = this->create_subscription<geometry_msgs::msg::AccelStamped>(
        "target_accel", rclcpp::QoS(10), std::bind(&LADRCPositionControllerNode::accelCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(), std::bind(&LADRCPositionControllerNode::odomCallback, this, std::placeholders::_1));

    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 10);
    
    // [核心修改 3] 发布底层姿态与推力节点
    attitude_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("fmu/in/vehicle_attitude_setpoint", 10);
    
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/in/vehicle_command", 10);

    control_timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&LADRCPositionControllerNode::controlLoop, this));
    command_timer_ = this->create_wall_timer(100ms, std::bind(&LADRCPositionControllerNode::stateMachine, this));

    flight_state_ = FlightState::INIT;
    offboard_setpoint_counter_ = 0;

    RCLCPP_INFO(this->get_logger(), "LADRC -> 姿态推力映射 控制器已启动！(Step 1)");
  }

private:
  void initializeControllers()
  {
    ladrc_controller::LADRCParams params_x, params_y, params_z;
    double max_acc_x = this->get_parameter("max_acceleration_x").as_double();
    double max_acc_y = this->get_parameter("max_acceleration_y").as_double();
    double max_acc_z = this->get_parameter("max_acceleration_z").as_double();

    auto setParams = [&](ladrc_controller::LADRCParams& p, const std::string& axis, double max_acc) {
      p.omega_o = this->get_parameter("omega_o_" + axis).as_double();
      p.omega_c = this->get_parameter("omega_c_" + axis).as_double();
      p.kp = p.omega_c * p.omega_c;
      p.kd = 2.0 * p.omega_c;
      p.b0 = this->get_parameter("b0_" + axis).as_double();
      p.dt = dt_;
      p.max_output = max_acc;
      p.min_output = -max_acc;
    };

    setParams(params_x, "x", max_acc_x);
    setParams(params_y, "y", max_acc_y);
    setParams(params_z, "z", max_acc_z);

    ladrc_x_ = std::make_unique<ladrc_controller::LADRCController>(params_x);
    ladrc_y_ = std::make_unique<ladrc_controller::LADRCController>(params_y);
    ladrc_z_ = std::make_unique<ladrc_controller::LADRCController>(params_z);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { reference_pose_ = *msg; has_pose_ = true; }
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) { reference_twist_ = *msg; has_twist_ = true; }
  void accelCallback(const geometry_msgs::msg::AccelStamped::SharedPtr msg) { reference_accel_ = *msg; has_accel_ = true; }
  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    current_odom_ = *msg;
    px4_timestamp_us_.store(msg->timestamp, std::memory_order_relaxed);
    has_odom_ = true;
  }

  uint64_t getPx4TimestampUs()
  {
    const uint64_t ts = px4_timestamp_us_.load(std::memory_order_relaxed);
    if (ts > 0) {
      return ts;
    }
    return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
  }

  void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param7 = 0.0)
  {
    if (!has_odom_) return; 

    px4_msgs::msg::VehicleCommand msg{};
    
    msg.command = command;
    msg.param1 = param1; msg.param2 = param2; msg.param7 = param7;
    
    // [核心修复]
    msg.target_system = 0;     // 0 表示“广播”。因为我们 ROS 2 话题已经隔离了命名空间 (px4_1 等)，
                              // 消息绝不会串台。设为 0 可以完美绕过飞控底层的严格 ID 校验。
    msg.target_component = 0;  // 同样广播到所有组件
    
    msg.source_system = 1;     // 恢复为 1！告诉飞控：“我是你头顶上的机载电脑”，而不是 255 (地面站)
    msg.source_component = 1;
    msg.from_external = true;
    
    msg.timestamp = getPx4TimestampUs(); 
    vehicle_command_pub_->publish(msg);
  }

  void stateMachine()
  {
    if (!has_odom_) {
      return;
    }

    switch (flight_state_.load()) {
    case FlightState::INIT:
      if (++offboard_setpoint_counter_ * 100 > 10000) {
        // 多机/无遥控器场景下，先切 Offboard，再尝试解锁
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
        flight_state_ = FlightState::SETTING_OFFBOARD;
        offboard_setpoint_counter_ = 0;
      }
      break;
    case FlightState::SETTING_OFFBOARD:
      if (++offboard_setpoint_counter_ * 100 > 2000) {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        flight_state_ = FlightState::ARMING;
        offboard_setpoint_counter_ = 0;
      }
      break;
    case FlightState::ARMING:
      if (++offboard_setpoint_counter_ * 100 > 1000) {
        flight_state_ = FlightState::RUNNING_TRAJECTORY; command_timer_->cancel();
      }
      break;
    case FlightState::RUNNING_TRAJECTORY:
      command_timer_->cancel();
      break;
    }
  }

  void controlLoop()
  {
    // 持续发布 offboard 模式心跳
    publishOffboardControlMode();

    // [核心修复] 在状态机未进入正式飞行前，必须持续向 PX4 发送待机 Setpoint！
    // 否则 PX4 会因为“未接收到设定点”而拒绝解锁和进入 Offboard 模式。
    if (flight_state_.load() != FlightState::RUNNING_TRAJECTORY || !has_pose_ || !has_twist_ || !has_accel_ || !has_odom_) {
      
      px4_msgs::msg::VehicleAttitudeSetpoint idle_msg{};
      idle_msg.timestamp = getPx4TimestampUs();
      // 保持水平姿态 (四元数 w=1, x=0, y=0, z=0)
      idle_msg.q_d[0] = 1.0; 
      idle_msg.q_d[1] = 0.0; 
      idle_msg.q_d[2] = 0.0; 
      idle_msg.q_d[3] = 0.0;
      // 推力设置为 0
      idle_msg.thrust_body[0] = 0.0;
      idle_msg.thrust_body[1] = 0.0;
      idle_msg.thrust_body[2] = 0.0; 
      
      attitude_setpoint_pub_->publish(idle_msg);

      // 打印等待状态的日志
      if (++log_counter_ >= 50) {
        RCLCPP_INFO(this->get_logger(), "等待状态机激活或等待话题... 当前状态: %d", (int)flight_state_.load());
        log_counter_ = 0;
      }
      return;
    }

    // ===============================================================
    // 以下为进入 RUNNING_TRAJECTORY 后的正式飞行控制代码 (保持不变)
    // ===============================================================

    // 1. 获取测量值 (Odom) ENU
    double x_meas = current_odom_.position[1];
    double y_meas = current_odom_.position[0];
    double z_meas = -current_odom_.position[2];

    // 2. 获取参考值 (Target) ENU
    double x_ref = reference_pose_.pose.position.x;
    double y_ref = reference_pose_.pose.position.y;
    double z_ref = -reference_pose_.pose.position.z;
    double vx_ref = reference_twist_.twist.linear.x;
    double vy_ref = reference_twist_.twist.linear.y;
    double vz_ref = -reference_twist_.twist.linear.z;
    double ax_ref = reference_accel_.accel.linear.x;
    double ay_ref = reference_accel_.accel.linear.y;
    double az_ref = -reference_accel_.accel.linear.z;

    // 3. 计算 LADRC 虚拟外环力 (ax, ay, az)
    double ax_cmd_enu = ladrc_x_->update(x_ref, vx_ref, ax_ref, x_meas);
    double ay_cmd_enu = ladrc_y_->update(y_ref, vy_ref, ay_ref, y_meas);
    double az_cmd_enu = ladrc_z_->update(z_ref, vz_ref, az_ref, z_meas);

    // 4. 获取期望偏航角 Yaw (ENU 转 NED)
    double qx = reference_pose_.pose.orientation.x;
    double qy = reference_pose_.pose.orientation.y;
    double qz = reference_pose_.pose.orientation.z;
    double qw = reference_pose_.pose.orientation.w;
    double yaw_enu = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    double yaw_ned = -yaw_enu + M_PI / 2.0;

    // 5. U -> Tau 映射 (外环加速度转换为底层姿态与推力)
    publishAttitudeSetpoint(ax_cmd_enu, ay_cmd_enu, az_cmd_enu, yaw_ned);
    
    if (++log_counter_ >= 50) {
      RCLCPP_INFO(this->get_logger(), "LADRC 追踪中 -> Ref: [%.2f, %.2f, %.2f], Pos: [%.2f, %.2f, %.2f]", 
                  x_ref, y_ref, z_ref, x_meas, y_meas, z_meas);
      log_counter_ = 0;
    }
  }

  void publishOffboardControlMode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = getPx4TimestampUs();
    msg.position = false;
    msg.velocity = false;      
    msg.acceleration = false; 
    msg.attitude = true;      // <-- [核心修改 4] 开启底层姿态控制，绕过前三环
    msg.body_rate = false;
    offboard_mode_pub_->publish(msg);
  }

  // === 核心修改 5：微分平坦算法 (U to Tau Map + 防死亡翻滚) ===
  void publishAttitudeSetpoint(double ax_enu, double ay_enu, double az_enu, double yaw_ned)
  {
    // 1. 将 ENU 加速度指令转换为 PX4(NED) 坐标系
    Eigen::Vector3d a_ned(ay_enu, ax_enu, -az_enu);

    // 2. 加上重力补偿，计算最终期望的总推力向量 (NED 坐标系下，重力向下为 +9.81)
    // T/m = a - g -> T_ned = a_ned - g_ned
    Eigen::Vector3d g_ned(0.0, 0.0, 9.81);
    Eigen::Vector3d T_ned = a_ned - g_ned;

    // 3. 防“死亡翻滚”保护 (Anti-Death-Roll)
    // 在 NED 中，Z轴向下。为了保持无人机朝上飞行，推力向量必须指向天空（Z必须是负值）。
    // 如果算出 T_ned(2) 大于 -1.0，说明飞机想往下掉甚至想翻过来，强制兜底！
    if (T_ned(2) > -1.0) {
        T_ned(2) = -1.0; 
    }

    // 4. 计算期望的机体坐标系各轴朝向 (X_b, Y_b, Z_b)
    // 机体的 Z 轴指向底部，而推力向量 T_ned 指向上方，所以 Z_b = - T_ned 的归一化
    Eigen::Vector3d z_b = -T_ned.normalized();

    // 根据偏航角计算期望机头大致指向
    Eigen::Vector3d x_c(std::cos(yaw_ned), std::sin(yaw_ned), 0.0);

    // 通过叉乘生成正交的机体右轴(Y_b) 和 机体正前轴(X_b)
    Eigen::Vector3d y_b = z_b.cross(x_c).normalized();
    Eigen::Vector3d x_b = y_b.cross(z_b).normalized();

    // 组装旋转矩阵并转为四元数
    Eigen::Matrix3d R;
    R.col(0) = x_b;
    R.col(1) = y_b;
    R.col(2) = z_b;
    Eigen::Quaterniond q_des(R);

    // 5. 提取并映射归一化推力 (防推力气球效应)
    double hover_thrust = this->get_parameter("hover_thrust").as_double();
    // 推力大小 / 重力常数 * 悬停推力系数
    double norm_thrust = (T_ned.norm() / 9.81) * hover_thrust;
    
    // 限幅推力到 0 ~ 1 之间
    norm_thrust = std::max(0.0, std::min(norm_thrust, 1.0));

    // 6. 发布底层姿态与推力设定点
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = getPx4TimestampUs();
    
    msg.q_d[0] = q_des.w();
    msg.q_d[1] = q_des.x();
    msg.q_d[2] = q_des.y();
    msg.q_d[3] = q_des.z();
    
    // PX4 v1.14 中 thrust_body 为 float[3]。Z轴由于指向机体下方，推力是反向的，所以带负号
    msg.thrust_body[0] = 0.0;
    msg.thrust_body[1] = 0.0;
    msg.thrust_body[2] = -static_cast<float>(norm_thrust);

    attitude_setpoint_pub_->publish(msg);
  }

  std::unique_ptr<ladrc_controller::LADRCController> ladrc_x_;
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_y_;
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_z_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr reference_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr reference_accel_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_; // <--- [修改]
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  geometry_msgs::msg::PoseStamped reference_pose_;
  geometry_msgs::msg::TwistStamped reference_twist_;
  geometry_msgs::msg::AccelStamped reference_accel_;
  px4_msgs::msg::VehicleOdometry current_odom_;

  bool has_pose_ = false, has_twist_ = false, has_accel_ = false, has_odom_ = false;

  int log_counter_ = 0;
  std::atomic<uint64_t> px4_timestamp_us_{0};

  std::atomic<FlightState> flight_state_;
  std::atomic<uint64_t> offboard_setpoint_counter_;
  double dt_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LADRCPositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}