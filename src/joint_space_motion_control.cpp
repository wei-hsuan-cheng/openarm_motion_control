#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>

#include <unordered_map>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <limits>

using std::placeholders::_1;

class JointSpaceMotionControl : public rclcpp::Node {
public:
  JointSpaceMotionControl()
  : rclcpp::Node("joint_space_motion_control"),
    fs_(500.0), Ts_(1.0 / fs_), Kp_(100.0), a_(100.0), VEL_LIMIT_(0.0)
  {
    // Init project
    RCLCPP_INFO(get_logger(), "Starting [JointSpaceMotionControl] . . .");
    // Load ROS 2 parameters from yaml file
    loadYAMLParams();
    // Init
    initRobotConfig();
    initControlState();
    initTimeSpec();
    initROSComponents();

    RCLCPP_INFO(get_logger(), "Configured with %d joints @ %.1f Hz. base_link=%s, ee_link=%s",
                n_, fs_, base_link_.c_str(), ee_link_.c_str());
  }

private:
  // -------- load params (schema identical to PoseCommandLeft) --------
  void loadYAMLParams()
  {
    // Robot config params
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

    declare_parameter<double>("fs", 200.0);
  }

  void initRobotConfig()
  {
    // Fetch core config
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

    // Extract joint limits into an n x 4 Eigen matrix
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

    // (Optional) Print joint limits
    std::cout << "\n-- Left arm joint limits [ll, ul, vel, eff] [rad, rad, rad/s, Nm] -->\n";
    for (int i = 0; i < n_; ++i) {
      std::cout << "  " << joint_names_[i] << ": ["
                << joint_limits_(i,0) << ", "
                << joint_limits_(i,1) << ", "
                << joint_limits_(i,2) << ", "
                << joint_limits_(i,3) << "]\n";
    }
    std::cout << std::endl;
  }

  void initControlState()
  {
    // Initial joint state
    q_.setZero(n_);
    q_ << -0.955, -0.674, 1.163, 1.321, 0.756, -0.590, -0.909;
    qd_.setZero(n_);
    qcmd_.setZero(n_);

    // name → index map for incoming commands
    name_to_idx_.clear();
    for (int i = 0; i < n_; ++i) name_to_idx_[joint_names_[i]] = i;
  }

  void initTimeSpec()
  {
    t_last_log_ = std::chrono::steady_clock::now();
  }

  void initROSComponents()
  {
    // Publishers / Subscribers
    pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());

    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_command",
      rclcpp::SensorDataQoS(),
      std::bind(&JointSpaceMotionControl::onCmd, this, _1)
    );

    // Control loop
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(Ts_)),
      std::bind(&JointSpaceMotionControl::onTimer, this)
    );
  }

  // -------- callbacks --------
  void onCmd(const sensor_msgs::msg::JointState& msg)
  {
    if (msg.name.empty() || msg.position.size() < msg.name.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Bad /joint_command (empty or positions < names). Ignoring.");
      return;
    }

    // Update entries in Eigen qcmd_ for any joints we model
    const size_t m = msg.name.size();
    for (size_t k = 0; k < m; ++k) {
      auto it = name_to_idx_.find(msg.name[k]);
      if (it == name_to_idx_.end()) continue; // ignore unknown (e.g., gripper)
      const int i = it->second;
      if (k < msg.position.size()) {
        qcmd_(i) = msg.position[k];
      }
    }
  }

  void onTimer()
  {
    // First-order velocity response around a P position loop (Eigen math):
    // qdot_cmd = Kp * (qcmd - q)
    // qdd      = qdot_cmd - a * qdot
    // integrate semi-implicit Euler
    const double vl = (VEL_LIMIT_ == 0.0) ? 1e6 : VEL_LIMIT_;

    Eigen::VectorXd qe     = qcmd_ - q_;
    Eigen::VectorXd qd_cmd = Kp_ * qe;

    // pre-saturate commanded velocity
    qd_cmd = qd_cmd.cwiseMax(-vl).cwiseMin(vl);

    Eigen::VectorXd qdd = qd_cmd - a_ * qd_;

    // integrate
    qd_ += qdd * Ts_;
    qd_  = qd_.cwiseMax(-vl).cwiseMin(vl);
    q_  += qd_ * Ts_;

    // // Debug (optional)
    // std::cout << "\nqe = [";
    // for (int i = 0; i < n_; ++i) {
    //   std::cout << qe(i);
    //   if (i < n_-1) std::cout << ", ";
    // }
    // std::cout << "]\n" << std::flush;

    // Publish measured joints (convert Eigen → std::vector)
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = joint_names_;

    js.position.resize(n_);
    js.velocity.resize(n_);
    for (int i = 0; i < n_; ++i) {
      js.position[i] = q_(i);
      js.velocity[i] = qd_(i);
    }
    pub_->publish(js);

    // lightweight log
    auto now_steady = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_steady - t_last_log_).count() >= 2.0) {
      t_last_log_ = now_steady;
      const double max_qe = qe.cwiseAbs().maxCoeff();
      const double rmse   = std::sqrt(qe.squaredNorm() / std::max(1, n_));
      RCLCPP_INFO(get_logger(), "Track err: max=%.4f rad, rmse=%.4f rad", max_qe, rmse);
    }
  }

  // -------- members --------
  // params / state (copied schema)
  std::string robot_name_, base_link_, ee_link_, rep_;
  std::vector<std::string> joint_names_;
  int n_{0}, num_joints_param_{0};
  std::vector<double> M_pos_, M_qwxyz_;

  // joint limits packed to n x 4 (Eigen)
  Eigen::MatrixXd joint_limits_; // [ll, ul, vel, eff]

  // control timing & gains (fixed for minimal node)
  const double fs_;        // 500 Hz
  const double Ts_;        // 1/fs
  const double Kp_;        // 100
  const double a_;         // 100
  const double VEL_LIMIT_; // 0 -> treated as very large

  // simulated states (Eigen)
  Eigen::VectorXd q_, qd_, qcmd_;
  std::unordered_map<std::string, int> name_to_idx_;

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // logging
  std::chrono::steady_clock::time_point t_last_log_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<JointSpaceMotionControl>());
  } catch (const std::exception& e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}