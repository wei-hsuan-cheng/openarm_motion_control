#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_14.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <chrono>

using RM = RMUtils;
using std::placeholders::_1;

class JointCommand : public rclcpp::Node {
public:
  JointCommand() : rclcpp::Node("joint_command") 
  {
    // Init project
    RCLCPP_INFO(get_logger(), "Starting [JointCommand]. . .");
    // Load ROS 2 parameters from yaml file
    loadYAMLParams();
    // Init
    initRobotConfig();
    initMotionParams();
    initRobotControl();
    initTimeSpec();
    initROSComponents();
    // Init logging
    RCLCPP_INFO(get_logger(), "Configured with %d joints @ %.1f Hz. base_link=%s, ee_link=%s",
                n_, fs_, base_link_.c_str(), ee_link_.c_str());

  }

  void loadYAMLParams()
  {
    // -------- Robot config params --------
    declare_parameter<std::string>("base_link", "");
    declare_parameter<std::string>("ee_link", "");
    declare_parameter<std::string>("screw_representation", "body");
    declare_parameter<std::vector<std::string>>("joint_names", {});
    declare_parameter<int>("num_joints", 0);
    declare_parameter<std::vector<double>>("M_position", {});        // [x,y,z]
    declare_parameter<std::vector<double>>("M_quaternion_wxyz", {}); // [w,x,y,z]

    // -------- Motion profile params --------
    declare_parameter<double>("fs", 200.0);
    declare_parameter<std::vector<double>>("offset_rad", {});   // q0
    declare_parameter<std::vector<double>>("amplitude_rad", {});// A
    declare_parameter<std::vector<double>>("frequency_hz", {}); // f
    declare_parameter<std::vector<double>>("phase_rad", {});    // φ
  }

  void initRobotConfig()
  {
    // -------- Robot config params --------
    get_parameter("base_link", base_link_);
    get_parameter("ee_link", ee_link_);
    get_parameter("screw_representation", rep_);
    get_parameter("joint_names", joint_names_);
    get_parameter("num_joints", num_joints_param_);
    get_parameter("M_position", M_pos_);
    get_parameter("M_quaternion_wxyz", M_qwxyz_);

    // Validate joint names
    if (joint_names_.empty()) {
      RCLCPP_FATAL(get_logger(), "joint_names is empty.");
      throw std::runtime_error("joint_names empty");
    }

    // Number of joints
    n_ = static_cast<int>(joint_names_.size());
    if (num_joints_param_ != 0 && num_joints_param_ != n_) {
      RCLCPP_WARN(get_logger(), "num_joints (%d) != joint_names.size() (%d). Using %d.", num_joints_param_, n_, n_);
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
  }

  void initMotionParams() 
  {
    // -------- Motion profile params --------
    get_parameter("fs", fs_);
    get_parameter("offset_rad", q0_);
    get_parameter("amplitude_rad", A_);
    get_parameter("frequency_hz", f_);
    get_parameter("phase_rad", phi_);

    // Expand scalars or pad vectors to n_
    auto expand_vec = [&](std::vector<double>& v, double def_scalar)
    {
      if (v.empty()) v = std::vector<double>(1, def_scalar);
      if (v.size() == 1) v = std::vector<double>(n_, v[0]);
      if ((int)v.size() != n_) {
        RCLCPP_WARN(get_logger(), "Vector param resized to n=%d from %zu", n_, v.size());
        v.resize(n_, v.empty() ? def_scalar : v.back());
      }
    };

    expand_vec(q0_, 0.0);            // default 0 offset
    expand_vec(A_,  0.25);           // default 0.25 rad amplitude
    expand_vec(f_,  0.10);           // default 0.10 Hz
    expand_vec(phi_, 0.0);           // default 0 phase
  }

  void initRobotControl()
  {
    joint_angles_cmd_.resize(n_);
    pos_quat_b_e_ = PosQuat(Vector3d::Zero(), Quaterniond::Identity());
  }

  void initTimeSpec()
  {
    if (fs_ <= 0.0) fs_ = 200.0;
    Ts_ = 1.0 / fs_;
    t_ = 0.0;
  }

  void initROSComponents()
  {
    // -------- Publishers --------
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
    ee_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", 10);

    // -------- Timer loop --------
    timer_period_ = std::chrono::duration<double>(Ts_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period_),
      std::bind(&JointCommand::timerCallback, this)
    );

    // -------- Record start time --------
    start_time_ = now_time();
  }

  rclcpp::Time now_time() 
  { 
    return this->get_clock()->now(); 
  }

  void getJointCommand()
  {
    // // θ(t) = q0 + A ⊙ sin(2π f t + φ)
    // for (int i = 0; i < n_; ++i) {
    //   joint_angles_cmd_(i) = q0_[i] + A_[i] * std::sin(2.0 * M_PI * f_[i] * t_ + phi_[i]);
    // }

    // Ground-truth pose
    // Joints: [-0.551, 0.462, 0.569, 1.479, 1.078, 0.301, 0.892]
    // Translation: [0.330, 0.018, 0.321]
    // Rotation: in Quaternion [0.358, 0.425, -0.049, 0.830] // (x,y,z,w)
    joint_angles_cmd_ << -0.551, 0.462, 0.569, 1.479, 1.078, 0.301, 0.892;
  }

  void solveFK()
  {
    // Compute FK with PoE and publish EE pose w.r.t. base
    pos_quat_b_e_ = RM::FKPoE(screws_, joint_angles_cmd_);
  }

  void publishStates()
  {
    // Publish joint angles
    sensor_msgs::msg::JointState js;
    js.header.stamp = now_time();
    js.name = joint_names_;
    js.position.resize(n_);
    for (int i = 0; i < n_; ++i) js.position[i] = joint_angles_cmd_(i);
    joint_pub_->publish(js);

    // Publish ee pose
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = js.header.stamp;
    ps.header.frame_id = base_link_.empty() ? "base_link" : base_link_;
    ps.pose.position.x = pos_quat_b_e_.pos.x();
    ps.pose.position.y = pos_quat_b_e_.pos.y();
    ps.pose.position.z = pos_quat_b_e_.pos.z();
    ps.pose.orientation.x = pos_quat_b_e_.quat.x();
    ps.pose.orientation.y = pos_quat_b_e_.quat.y();
    ps.pose.orientation.z = pos_quat_b_e_.quat.z();
    ps.pose.orientation.w = pos_quat_b_e_.quat.w();
    ee_pose_pub_->publish(ps);
  }

private:
  void timerCallback() 
  {
    t_ = (now_time() - start_time_).seconds();
    getJointCommand();
    solveFK();
    publishStates();
  }


  // Params / state
  std::string base_link_, ee_link_, rep_;
  std::vector<std::string> joint_names_;
  int num_joints_param_{0}, n_{0};
  std::vector<double> M_pos_, M_qwxyz_;

  double fs_{200.0};
  double Ts_;
  double t_;
  std::vector<double> q0_, A_, f_, phi_;
  std::chrono::duration<double> timer_period_;
  rclcpp::Time start_time_;

  MatrixXd S_;
  PosQuat M_;
  ScrewList screws_;
  VectorXd joint_angles_cmd_;
  PosQuat pos_quat_b_e_;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<JointCommand>());
  } catch (const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
