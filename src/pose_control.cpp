#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_15.hpp"

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

class PoseControl : public rclcpp::Node {
public:
  PoseControl()
  : rclcpp::Node("pose_control")
  {
    RCLCPP_INFO(get_logger(), "Starting [PoseControl] . . .");
    loadYAMLParams();
    initRobotConfig();
    initSystemDynamics();
    initControlState();
    initControlParams();
    initTimeSpec();
    initROS();
    RCLCPP_INFO(get_logger(), "Configured with %d joints @ %.1f Hz. base=%s, ee=%s",
                n_, fs_, base_link_.c_str(), ee_link_.c_str());
  }

private:
  // ---------- YAML schema (same as your other nodes) ----------
  void loadYAMLParams() {
    declare_parameter<std::string>("robot_name", "");
    declare_parameter<std::string>("base_link", "");
    declare_parameter<std::string>("ee_link", "");
    declare_parameter<std::string>("screw_representation", "body");
    declare_parameter<std::vector<std::string>>("joint_names", {});
    declare_parameter<int>("num_joints", 0);
    declare_parameter<std::vector<double>>("joint_limits_lower",   {});
    declare_parameter<std::vector<double>>("joint_limits_upper",   {});
    declare_parameter<std::vector<double>>("joint_limits_velocity",{});
    declare_parameter<std::vector<double>>("joint_limits_effort",  {});
    declare_parameter<std::vector<double>>("M_position", {});        // [x,y,z]
    declare_parameter<std::vector<double>>("M_quaternion_wxyz", {}); // [w,x,y,z]
    declare_parameter<double>("fs", 500.0);}

  void initRobotConfig() {
    get_parameter("robot_name", robot_name_);
    get_parameter("base_link", base_link_);
    get_parameter("ee_link", ee_link_);
    get_parameter("screw_representation", rep_);
    get_parameter("joint_names", joint_names_);
    get_parameter("num_joints", num_joints_param_);
    get_parameter("M_position", M_pos_);
    get_parameter("M_quaternion_wxyz", M_qwxyz_);

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

    // Validate home pose
    if (M_pos_.size() != 3 || M_qwxyz_.size() != 4) {
      RCLCPP_FATAL(get_logger(), "M_position must be length 3 and M_quaternion_wxyz length 4 (w,x,y,z).");
      throw std::runtime_error("Bad M param sizes");
    }
    M_.pos  = Vector3d(M_pos_[0], M_pos_[1], M_pos_[2]);
    M_.quat = Quaterniond(M_qwxyz_[0], M_qwxyz_[1], M_qwxyz_[2], M_qwxyz_[3]); // (w,x,y,z)
    M_.quat.normalize();

    // Build screw list matrix 6 x n  (order of [v, w])
    S_.resize(6, n_);
    for (int j = 0; j < n_; ++j) {
      const std::string pname = "screw_list." + joint_names_[j];
      declare_parameter<std::vector<double>>(pname, {});
      std::vector<double> arr;
      if (!get_parameter(pname, arr) || arr.size() != 6) {
        RCLCPP_FATAL(get_logger(), "Param %s must be length-6 [v(0:2), w(3:5)].", pname.c_str());
        throw std::runtime_error("Bad screw param");
      }
      S_.col(j) << arr[0], arr[1], arr[2], arr[3], arr[4], arr[5];
    }
    
    // Init ScrewList object
    screws_ = ScrewList(S_, M_);
    screws_.setMeta(
      robot_name_,
      ScrewList::ParseRep(rep_),
      joint_names_,
      base_link_.empty() ? "base_link" : base_link_,
      ee_link_.empty()   ? "ee_link"   : ee_link_,
      joint_limits_
    );
    // Print screw list
    screws_.PrintList();
  }

  void initSystemDynamics() {
    a_ = 100.0;                                   // first-order joint vel response
    vel_limit_ = 0.0;                            // |qdot| clamp [rad/s]
  }

  void initControlState() {
    // Initial joint state
    q_.setZero(n_);
    q_ << -0.955, -0.674, 1.163, 1.321, 0.756, -0.590, -0.909;
    qd_.setZero(n_);
    qd_cmd_.setZero(n_);

    // Desired EE pose (init = home)
    pos_quat_b_e_ = RM::FKPoE(screws_, q_);     // current pose (home pose)
    pos_quat_b_e_cmd_ = RM::FKPoE(screws_, q_); // Desired pose
    have_target_ = false;

    last_log_ = std::chrono::steady_clock::now();

    // Optional: start from mid of joint limits if you want
    // for (int i=0;i<n_;++i) q_(i) = 0.5*(joint_limits_(i,0)+joint_limits_(i,1));
  }

  void initControlParams()
  {
    is_first_loop_ = true;
    pbvs_target_reached_ = false;

    // Pose controller params
    double pos_mult = 2.5 * std::pow(10.0, 2.0);
    double so3_mult = 2.0 * std::pow(10.0, 2.0);
    Vector3d kp_pos(1.0, 1.0, 1.0), kp_so3(1.0, 1.0, 1.0);
    kp_pos_so3_ = Matrix6d::Identity();
    kp_pos_so3_.topLeftCorner(3, 3)     = (pos_mult * kp_pos).asDiagonal();
    kp_pos_so3_.bottomRightCorner(3, 3) = (so3_mult * kp_so3).asDiagonal();
    error_norm_thresh_ << 1e-3, (1.0 * M_PI / 180.0);  // [m, rad]
    // error_norm_thresh_ << 5e-4, (1.0 * M_PI / 180.0);  // [m, rad]

    // S-curve params
    scur_ = SCurveParams{1.0};

    // Error buffers
    window_size_vision_ = 10;
    twist_e_buffer_.clear();
    error_norm_buffer_.clear();

    // RCLCPP_INFO(this->get_logger(), "Pose controller parameters updated.");
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

  void initROS() {
    // sub: desired EE pose
    sub_pose_cmd_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/openarm_left/ee_pose_command", rclcpp::QoS(10),
      std::bind(&PoseControl::onPoseCmd, this, _1));

    // pub: measured joints
    pub_js_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());

    // control loop
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(Ts_)),
      std::bind(&PoseControl::onTimer, this));
  }

  // ---------- subs / loop ----------
  void onPoseCmd(const geometry_msgs::msg::PoseStamped& msg) {
    Vector3d pos = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    Quaterniond quat = Eigen::Quaterniond(msg.pose.orientation.w,
                                          msg.pose.orientation.x,
                                          msg.pose.orientation.y,
                                          msg.pose.orientation.z);
    pos_quat_b_e_cmd_ = PosQuat(pos, quat);
    have_target_ = true;
  }

  void onTimer() {
    if (!have_target_) return;

    if (is_first_loop_)
    {
        k_ = 0;
        is_first_loop_ = false;
    }

    // ===== FK for current pose =====
    pos_quat_b_e_ = RM::FKPoE(screws_, q_);

    // ===== Pose controller =====
    // Controller input (pose command and error)
    PosQuat pos_quat_m_cmd = RM::PosQuats2RelativePosQuat(pos_quat_b_e_, pos_quat_b_e_cmd_);
    pos_so3_m_cmd_ = RM::PosQuat2Posso3(pos_quat_m_cmd);

    // P-control for trajectory tracking
    Vector6d twist_cmd_raw = RM::KpPosso3(pos_so3_m_cmd_, kp_pos_so3_, pbvs_target_reached_);

    // Twist S-curve for smoothing
    // Vector6d twist_e_mavg = RM::MAvg(twist_e_, twist_e_buffer_, window_size_vision_);
    Vector6d twist_e_mavg = Vector6d::Zero();
    Vector6d twist_cmd  = RM::SCurve(twist_cmd_raw, twist_e_mavg, scur_.lambda, k_ * Ts_, scur_.T);

    // Error thresholding
    Vector2d error_norm = Vector2d(RM::Norm(pos_so3_m_cmd_.head(3)), RM::Norm(pos_so3_m_cmd_.tail(3)));
    Vector2d error_norm_mavg = RM::MAvg(error_norm, error_norm_buffer_, window_size_vision_);
    auto [twist_e_cmd_, pbvs_target_reached_] = RM::ErrorThreshold(error_norm_mavg, error_norm_thresh_, twist_cmd); 

    // // Print twist command
    // RM::PrintVec(twist_e_cmd_, "twist_e_cmd_");

    // ===== Jacobian J(q) (6×n) and DLS pseudo-inverse mapping =====
    // J_e(θ)
    MatrixXd Je = RM::Jacob(screws_, q_); // 6 x n

    // Map from twist command to joint velocity command
    double lambda = 1e-2; // DLS damping
    if (lambda > 0.0) {
        // Damped Least Squares: J⁺ = Jᵀ (J Jᵀ + λ² I)^{-1}
        MatrixXd A = Je * Je.transpose() + (lambda * lambda) * MatrixXd::Identity(6,6);
        qd_cmd_ = Je.transpose() * A.inverse() * twist_e_cmd_;
    } else {
        qd_cmd_ = Je.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(twist_e_cmd_);
    }

    // Saturate joint velocity command
    const double vl = vel_limit_;
    if (vl > 0.0) {
      for (int i = 0; i < n_; ++i) {
        qd_cmd_(i) = std::max(-vl, std::min(vl, qd_cmd_(i)));
      }
    }

    // ===== First-order joint velocity dynamics =====
    // q̈ = q̇_cmd − a q̇
    Eigen::VectorXd qdd = qd_cmd_ - a_ * qd_;

    // integrate (semi-implicit)
    qd_ += qdd * Ts_;
    if (vl > 0.0) {
      for (int i = 0; i < n_; ++i) {
        qd_(i) = std::max(-vl, std::min(vl, qd_(i)));
      }
    }
    q_  += qd_ * Ts_;

    // ===== Publish joint state =====
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = joint_names_;
    js.position.resize(n_);
    js.velocity.resize(n_);
    for (int i = 0; i < n_; ++i) {
      js.position[i] = q_(i);
      js.velocity[i] = qd_(i);
    }
    pub_js_->publish(js);

    // periodic logging
    auto now_steady = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_steady - last_log_).count() >= 2.0) {
      last_log_ = now_steady;
      RCLCPP_INFO(get_logger(), "pos_so3_error [m, rad]: [%.4f, %.4f]", error_norm(0), error_norm(1));
    }

    k_++;
    t_ += Ts_;
  }

private:
  // ---- params / robot ----
  std::string robot_name_, base_link_, ee_link_, rep_;
  std::vector<std::string> joint_names_;
  int n_{0}, num_joints_param_{0};

  // Screw list params
  std::vector<double> M_pos_, M_qwxyz_;
  MatrixXd S_;
  MatrixXd joint_limits_; // n x 4: [ll, ul, v, e]
  PosQuat M_;
  ScrewList screws_;

  // ---- control state ----
  Eigen::VectorXd q_;              // (n)
  Eigen::VectorXd qd_;             // (n)
  Eigen::VectorXd qd_cmd_;         // (n)
  PosQuat pos_quat_b_e_;
  Vector6d twist_e_;
  
  // Pose controller params
  bool have_target_{false};
  PosQuat pos_quat_b_e_cmd_;
  Vector6d pos_so3_m_cmd_;
  Matrix6d kp_pos_so3_;
  Vector6d twist_e_cmd_;
  bool pbvs_target_reached_;

  struct SCurveParams
  {
    SCurveParams(double T_in = 1.0)
    : T(T_in),
      lambda((4.0 / T_in) * 5.0)
    {}
    double T;      
    double lambda; 
  };
  SCurveParams scur_;         
  Vector2d error_norm_thresh_;

  // Buffers
  std::deque<Vector6d> twist_e_buffer_;
  std::deque<Vector2d> error_norm_buffer_;
  std::size_t window_size_vision_;

  // timing
  double fs_, Ts_, k_, t_;
  bool is_first_loop_;
  double a_;
  double vel_limit_;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        pub_js_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

  // logging
  std::chrono::steady_clock::time_point last_log_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PoseControl>());
  } catch (const std::exception& e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
