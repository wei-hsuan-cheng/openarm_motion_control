#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_16.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <unordered_map>
#include <vector>
#include <string>
#include <chrono>
#include <iostream>

using RM = RMUtils;
using std::placeholders::_1;

class RobotJointDynamics : public rclcpp::Node {
public:
  RobotJointDynamics()
  : rclcpp::Node("robot_joint_dynamics")
  {
    loadYAMLParams();
    initJointConfig();
    initJointStates();
    initJointDynamics();
    initTimeSpec();
    initROSComponents();
    // Init logging
    RCLCPP_INFO(get_logger(), "Starting [RobotJointDynamics] . . .\nConfigured with %d joints @ %.1f Hz.",
                n_, fs_);
  }

private:
  // ---------- YAML schema (same as your other nodes) ----------
  void loadYAMLParams() {
    declare_parameter<std::vector<std::string>>("joint_names", {});
    declare_parameter<int>("num_joints", 0);
    declare_parameter<std::vector<double>>("joint_limits_lower",   {});
    declare_parameter<std::vector<double>>("joint_limits_upper",   {});
    declare_parameter<std::vector<double>>("joint_limits_velocity",{});
    declare_parameter<std::vector<double>>("joint_limits_effort",  {});
    declare_parameter<double>("fs", 500.0);

    // declare_parameter<std::vector<double>>("initial_joint_position", {}); // vector fallback
    // declare_parameter<std::vector<double>>("initial_joint_velocity", {}); // vector fallback
  }

  void initJointConfig() {
    get_parameter("joint_names", joint_names_);
    get_parameter("num_joints", num_joints_param_);

    if (joint_names_.empty()) {
      RCLCPP_FATAL(get_logger(), "joint_names is empty.");
      throw std::runtime_error("joint_names empty");
    }
    n_ = static_cast<int>(joint_names_.size());
    if (num_joints_param_ != 0 && num_joints_param_ != n_) {
      RCLCPP_WARN(get_logger(), "num_joints (%d) != joint_names.size() (%d). Using %d.",
                  num_joints_param_, n_, n_);
    }

    // Joint limits → Eigen (n×4) [ll, ul, vel, eff]
    std::vector<double> jl_lower, jl_upper, jl_vel, jl_eff;
    get_parameter("joint_limits_lower",    jl_lower);
    get_parameter("joint_limits_upper",    jl_upper);
    get_parameter("joint_limits_velocity", jl_vel);
    get_parameter("joint_limits_effort",   jl_eff);
    auto ensure_size = [&](std::vector<double>& v, const char* name){
      if ((int)v.size() != n_) {
        RCLCPP_WARN(get_logger(), "%s size %zu != n (%d). Resizing/padding with 0.",
                    name, v.size(), n_);
        v.resize(n_, 0.0);
      }
    };
    ensure_size(jl_lower, "joint_limits_lower");
    ensure_size(jl_upper, "joint_limits_upper");
    ensure_size(jl_vel,   "joint_limits_velocity");
    ensure_size(jl_eff,   "joint_limits_effort");

    joint_limits_.resize(n_, 4);
    for (int i = 0; i < n_; ++i) {
      joint_limits_(i,0) = jl_lower[i];
      joint_limits_(i,1) = jl_upper[i];
      joint_limits_(i,2) = jl_vel[i];
      joint_limits_(i,3) = jl_eff[i];
    }

  }

  void initJointStates() {
    q_.setZero(n_);
    qd_.setZero(n_);

    // Name-keyed params: initial_joint_position.<joint>, initial_joint_velocity.<joint>
    for (int i = 0; i < n_; ++i) {
      const std::string &name = joint_names_[i];
      // Declare per-joint params so rclcpp will accept them from YAML
      const std::string pos_key = "initial_joint_position." + name;
      const std::string vel_key = "initial_joint_velocity." + name;
      declare_parameter<double>(pos_key, std::numeric_limits<double>::quiet_NaN());
      declare_parameter<double>(vel_key, std::numeric_limits<double>::quiet_NaN());

      double vpos, vvel;
      if (get_parameter(pos_key, vpos) && std::isfinite(vpos)) q_(i)  = vpos;
      if (get_parameter(vel_key, vvel) && std::isfinite(vvel)) qd_(i) = vvel;
    }

    is_first_loop_ = true;
    last_log_ = std::chrono::steady_clock::now();
  }

  void initJointDynamics() {
    /* First-order joint dynamics */
    received_cmd_ = false;
    // Init joint velocity command
    qd_cmd_.setZero(n_);
    a_ = 100.0;                                   // first-order joint velocity response
    vel_limit_ = 0.0;                            // |qdot| clamp [rad/s]
  }

  void initTimeSpec()
  {
    get_parameter("fs", fs_);
    if (fs_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "Invalid fs parameter: %f", fs_);
      throw std::runtime_error("Bad fs param");
    }
    Ts_ = 1.0 / fs_;
    t_ = 0.0;
    k_ = 0;
  }

  void initROSComponents() {
    // sub: desired joint velocity
    sub_joint_velocity_cmd_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_velocity_command",
      rclcpp::SensorDataQoS(),
      std::bind(&RobotJointDynamics::onCmd, this, _1)
    );

    // pub: joint states
    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());

    // control loop
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(Ts_)),
      std::bind(&RobotJointDynamics::onTimer, this));
  }

  // ---------- subs / loop ----------
  void onCmd(const sensor_msgs::msg::JointState& msg)
  {
    if (msg.name.empty() || msg.velocity.size() < msg.name.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Bad /joint_velocity_command (empty or velocities < names). Ignoring.");
      return;
    }

    // Build quick lookup table: joint name -> index in message
    std::unordered_map<std::string, std::size_t> idx;
    idx.reserve(msg.name.size());
    for (std::size_t k = 0; k < msg.name.size(); ++k) idx[msg.name[k]] = k;
    // Fill qd_ only for the n_ arm joints, ignore unknown/extras (e.g., gripper)
    for (int i = 0; i < n_; ++i) {
      auto it = idx.find(joint_names_[i]);
      if (it == idx.end()) continue;  // this joint not present in the msg
      std::size_t k = it->second;

      // Bounds check for velocity arrays
      if (k < msg.velocity.size()) qd_cmd_(i) = msg.velocity[k];
    }

    received_cmd_ = true;
  }

  // Helpers
  bool everyTimeInterval(const std::chrono::steady_clock::time_point& last_log, const double& t = 2.0) {
    auto now_steady = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_steady - last_log).count() >= t) return true;
    return false;
  }

  void periodicLogging(double t = 2.0) {
    if (everyTimeInterval(last_log_, t)) 
    {
      // Logging
      last_log_ = std::chrono::steady_clock::now();
    }
  }

  void publishJointStates() {
    // Publish joint state
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = joint_names_;
    js.position.resize(n_);
    js.velocity.resize(n_);
    for (int i = 0; i < n_; ++i) {
      js.position[i] = q_(i);
      js.velocity[i] = qd_(i);
    }
    pub_joint_states_->publish(js);
  }

  void solveJointDynamics()
  {
    // Saturate joint velocity command
    const double vl = vel_limit_;
    if (vl > 0.0) {
      for (int i = 0; i < n_; ++i) {
        qd_cmd_(i) = std::max(-vl, std::min(vl, qd_cmd_(i)));
      }
    }

    // ===== First-order joint velocity dynamics =====
    // Governing equation: qdd + a * qd = q̇d_cmd 
    // => qdd = q̇d_cmd − a * qd
    Eigen::VectorXd qdd = qd_cmd_ - a_ * qd_;

    // Propagate joint velocity
    qd_ += qdd * Ts_;
    // Clamp joint velocity
    if (vl > 0.0) {
      for (int i = 0; i < n_; ++i) {
        qd_(i) = std::max(-vl, std::min(vl, qd_(i)));
      }
    }

    // Propagate joint position
    q_  += qd_ * Ts_;
  }

  void onTimer() {
    if (!received_cmd_) {
      if (everyTimeInterval(last_log_, 1.0)) {
        RCLCPP_WARN(get_logger(), "Waiting for /joint_velocity_command and message...");
        last_log_ = std::chrono::steady_clock::now();
      }
    }

    if (is_first_loop_)
    {
        k_ = 0;
        is_first_loop_ = false;
    }

    solveJointDynamics();
    publishJointStates();
    periodicLogging(2.0);

    k_++;
    t_ += Ts_;
  }

private:
  // ---- params / robot ----
  std::vector<std::string> joint_names_;
  int n_{0}, num_joints_param_{0};
  MatrixXd joint_limits_; // n x 4: [ll, ul, v, e]

  // ---- control state ----
  Eigen::VectorXd q_;              // (n)
  Eigen::VectorXd qd_;             // (n)
  Eigen::VectorXd qd_cmd_;         // (n)
  bool received_cmd_;

  // timing
  double fs_, Ts_, k_, t_;
  bool is_first_loop_;
  double a_;
  double vel_limit_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_velocity_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        pub_joint_states_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

  // logging
  std::chrono::steady_clock::time_point last_log_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RobotJointDynamics>());
  } catch (const std::exception& e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
