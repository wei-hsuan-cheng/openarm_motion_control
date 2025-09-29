#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_16.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <vector>
#include <string>
#include <chrono>

using RM = RMUtils;
using std::placeholders::_1;

class PoseVisualizationLeft : public rclcpp::Node {
public:
  PoseVisualizationLeft() : rclcpp::Node("pose_visualization_left") 
  {
    // Init project
    RCLCPP_INFO(get_logger(), "Starting [PoseVisualizationLeft]. . .");
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
    get_parameter("robot_name", robot_name_);
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

    // Extract joint limits
    // --- NEW: fetch joint limits arrays and pack to n×4 matrix [ll, ul, v, e] ---
    std::vector<double> jl_lower, jl_upper, jl_vel, jl_eff;
    get_parameter("joint_limits_lower",    jl_lower);
    get_parameter("joint_limits_upper",    jl_upper);
    get_parameter("joint_limits_velocity", jl_vel);
    get_parameter("joint_limits_effort",   jl_eff);

    // size checks (warn + pad/truncate to n_)
    auto fix_size = [&](std::vector<double>& v, const char* name){
      if ((int)v.size() != n_) {
        RCLCPP_WARN(get_logger(), "%s size %zu != n (%d). Resizing.", name, v.size(), n_);
        v.resize(n_, 0.0);
      }
    };
    fix_size(jl_lower, "joint_limits_lower");
    fix_size(jl_upper, "joint_limits_upper");
    fix_size(jl_vel,   "joint_limits_velocity");
    fix_size(jl_eff,   "joint_limits_effort");

    joint_limits_.resize(n_, 4);
    for (int i = 0; i < n_; ++i) {
      joint_limits_(i, 0) = jl_lower[i];
      joint_limits_(i, 1) = jl_upper[i];
      joint_limits_(i, 2) = jl_vel[i];
      joint_limits_(i, 3) = jl_eff[i];
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

  void initMotionParams() 
  {
    // -------- Motion profile params --------
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
    pos_quat_b_e_cmd_ = PosQuat(Vector3d::Zero(), Quaterniond::Identity());
    gripper_pos_cmd_ = 0.0;
    theta_sol_ = VectorXd::Zero(n_); // Temp IK solution
    joint_angles_cmd_ = VectorXd::Zero(n_); // Joint command to the robot

    // IK initial guess
    theta_sol_ << -0.955, -0.674, 1.163, 1.321, 0.756, -0.590, -0.909;
    joint_angles_cmd_ << -0.955, -0.674, 1.163, 1.321, 0.756, -0.590, -0.909;

    pos_quat_b_e_ik_ = PosQuat(Vector3d::Zero(), Quaterniond::Identity());
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
  }

  void initROSComponents()
  {
    // -------- Publishers --------
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
    ee_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/ee_pose", 10);

    // joint_cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>("/openarm_left/joint_command", rclcpp::SensorDataQoS());
    // ee_pose_cmd_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/openarm_left/ee_pose_command", 10);

    // -------- TF broadcaster --------
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // -------- Timer loop --------
    timer_period_ = std::chrono::duration<double>(Ts_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period_),
      std::bind(&PoseVisualizationLeft::timerCallback, this)
    );

    // -------- Record start time --------
    start_time_ = now_time();
  }

  rclcpp::Time now_time() 
  { 
    return this->get_clock()->now(); 
  }

  bool everyTimeInterval(double t = 2.0) {
    static auto last_call = std::chrono::steady_clock::now();

    auto now = std::chrono::steady_clock::now();
    double elapsed_sec =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - last_call).count();

    if (elapsed_sec >= t) {
      last_call = now;
      return true;
    }
    return false;
  }

  void getPoseCommand()
  {
    // Ground-truth nominal (centroid) pose
    // Joints: [-0.955, -0.674, 1.163, 1.321, 0.756, -0.590, -0.909]
    // Translation: [0.324, 0.093, 0.515]
    // Rotation: in Quaternion [-0.085, -0.191, 0.879, -0.429] // (w,x,y,z)

    pos_quat_b_e_cmd_.pos = Vector3d(0.25, 0.15, 0.5); // [m]
    pos_quat_b_e_cmd_.quat = Quaterniond(-0.085, -0.191, 0.879, -0.429); // (w,x,y,z)

    // Time varying pose command
    // double r = 0.175; // [m]
    double r = 0.1; // [m]
    // double offset_x = r * cos(2.0 * M_PI * f_[0] * t_); // [m]
    double offset_x = 0.0; // [m]
    double offset_y = 0.0; // [m]
    double offset_z = r * sin(2.0 * M_PI * f_[1] * t_); // [m]
    double offset_thx = 0.0; // [rad]
    double offset_thy = 0.0; // [rad]
    double offset_thz = 0.0; // [rad]

    PosQuat pos_quat_offset = PosQuat(Vector3d(offset_x, offset_y, offset_z), 
                                      RM::zyxEuler2Quat(Vector3d(offset_thz, offset_thy, offset_thx)));
    pos_quat_b_e_cmd_ = RM::TransformPosQuats({pos_quat_b_e_cmd_, pos_quat_offset});
  }

  void solveIK()
  {
    // === Solve IK problem ===
    theta_sol_ = joint_angles_cmd_;
    const double eomg = 1e-7;      // orientation tol (‖ω‖) [rad]
    const double ev   = 1e-7;      // position tol (‖v‖) [m]
    int cur_iter = 0; // current iteration (for debugging)
    const int    max_iter = 200;
    const double lambda   = 1e-2;  // DLS damping (set 0.0 to disable)
    const double step_clip = 0.0;  // set >0.0 (e.g., 0.2) to cap per-step |Δθ|
    const bool   wrap_pi   = true;

    auto t_start = std::chrono::high_resolution_clock::now();
    bool ok = RM::IKNum(screws_, pos_quat_b_e_cmd_, theta_sol_, cur_iter, eomg, ev, max_iter, lambda, step_clip, wrap_pi);
    auto t_end   = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_ms = t_end - t_start;

    // === Manipulability / Singularity check ===
    MatrixXd J_e = RM::Jacob(screws_, theta_sol_);
    const double w         = RM::ManipulabilityIndex(J_e);
    const double log10w    = RM::ManipulabilityIndexLog10(J_e);
    const double sigma_min = RM::MinSingularValue(J_e);
    const double sigma_min_threshold = 1.5 * pow(10, -2);
    const bool   near_sing = RM::NearSingular(J_e, sigma_min_threshold);

    // if (everyTimeInterval(1.0))
    // {
    //   std::cout << "\n===== Left arm pose command =====\n";
      
    //   // Print IK results
    //   std::cout << "-- IK target pose pos_quat_b_e_cmd_ [m, quat_wxyz] -->\n" 
    //     << pos_quat_b_e_cmd_.pos.x() << ", " << pos_quat_b_e_cmd_.pos.y() << ", " << pos_quat_b_e_cmd_.pos.z() << ", " 
    //     << pos_quat_b_e_cmd_.quat.w() << ", " << pos_quat_b_e_cmd_.quat.x() << ", " << pos_quat_b_e_cmd_.quat.y() << ", " << pos_quat_b_e_cmd_.quat.z() << "\n";
    //   std::cout << "-- IK success -->\n" << (ok ? "[SUCCEEDED]" : "[FAILED]") << "\n";
    //   std::cout << "-- theta_sol_ [rad] -->\n" << theta_sol_.transpose() << "\n";
    //   std::cout << "-- IK computation iteration/time/rate [idx, ms, Hz] -->\n" << cur_iter << ", " << elapsed_ms.count() << ", " << (1000.0 / elapsed_ms.count()) << std::endl;
      
    //   // Print manipulability index
    //   std::cout << "\n----- Manipulability / Singularity check -----\n";
    //   std::cout << "w          = " << w << "   (log10(w) = " << log10w << ")\n";
    //   std::cout << "sigma_min  = " << sigma_min << "\n";
    //   std::cout << "nearSing? (thresh = " + std::to_string(sigma_min_threshold) + ")  = " << (near_sing ? "YES" : "NO") << "\n";
    //   if (near_sing) {
    //       std::cout << "[WARN] Jacobian near singular; DLS (lambda=" << lambda
    //                 << ") may be critical here.\n";
    //   }

    // }

    // // Print manipulability index
    // // std::cout << "----- Manipulability / Singularity check -----\n";
    // // std::cout << "w          = " << w << "   (log10(w) = " << log10w << ")\n";
    // // std::cout << "nearSing? (thresh = " + std::to_string(sigma_min_threshold) + ")  = " << (near_sing ? "YES" : "NO") << "\n";
    // if (true)
    // {
    //   if (near_sing) {
    //     std::cout << "w          = " << w << "   (log10(w) = " << log10w << ")\n";
    //     std::cout << "nearSing? (thresh = " + std::to_string(sigma_min_threshold) + ")  = " << (near_sing ? "YES" : "NO") << "\n";  
    //   }
    // }
    

    // Update joint command from IK solution
    joint_angles_cmd_ = theta_sol_;
  }

  void solveFK()
  {
    // Compute FK with PoE and publish EE pose w.r.t. base
    pos_quat_b_e_ik_ = RM::FKPoE(screws_, joint_angles_cmd_);

    // std::cout << "\n-- FK result pose pos_quat_b_e_ik_ -->\n";
    // std::cout << "pos [m]: " << pos_quat_b_e_ik_.pos.transpose() << "\n";
    // std::cout << "quat (w,x,y,z): " << pos_quat_b_e_ik_.quat.w() << ", " << pos_quat_b_e_ik_.quat.x() << ", " << pos_quat_b_e_ik_.quat.y() << ", " << pos_quat_b_e_ik_.quat.z() << "\n";

  }

  void gripperControl()
  {
    double gripper_pos_max = 0.044; // [m]
    // Time-varying gripper command
    gripper_pos_cmd_ = 0.5 * (1.0 + sin(2.0 * M_PI * f_[0] * t_)) * gripper_pos_max; // normalized position
  }
  
  // void publishCommands()
  // {
  //   // Publish joint commands (including gripper position)
  //   sensor_msgs::msg::JointState jcmd;
  //   jcmd.header.stamp = now_time();
  //   jcmd.name = joint_names_;
  //   jcmd.name.push_back("openarm_left_finger_joint1");
  //   jcmd.position.resize(n_ + 1);
  //   for (int i = 0; i < n_; ++i) jcmd.position[i] = joint_angles_cmd_(i);
  //   jcmd.position[n_] = gripper_pos_cmd_;
  //   joint_cmd_pub_->publish(jcmd);

  //   // Publish ee pose
  //   geometry_msgs::msg::PoseStamped pcmd;
  //   pcmd.header.stamp = jcmd.header.stamp;
  //   pcmd.header.frame_id = base_link_.empty() ? "base_link" : base_link_;
  //   pcmd.pose.position.x = pos_quat_b_e_ik_.pos.x();
  //   pcmd.pose.position.y = pos_quat_b_e_ik_.pos.y();
  //   pcmd.pose.position.z = pos_quat_b_e_ik_.pos.z();
  //   pcmd.pose.orientation.x = pos_quat_b_e_ik_.quat.x();
  //   pcmd.pose.orientation.y = pos_quat_b_e_ik_.quat.y();
  //   pcmd.pose.orientation.z = pos_quat_b_e_ik_.quat.z();
  //   pcmd.pose.orientation.w = pos_quat_b_e_ik_.quat.w();
  //   ee_pose_cmd_pub_->publish(pcmd);
  // }

  void publishStates()
  {
    // Publish joint angles (including gripper position)
    sensor_msgs::msg::JointState js;
    js.header.stamp = now_time();
    js.name = joint_names_;
    js.name.push_back("openarm_left_finger_joint1");
    js.position.resize(n_ + 1);
    for (int i = 0; i < n_; ++i) js.position[i] = joint_angles_cmd_(i);
    js.position[n_] = gripper_pos_cmd_;
    joint_pub_->publish(js);

    // Publish ee pose
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = js.header.stamp;
    ps.header.frame_id = base_link_.empty() ? "base_link" : base_link_;
    ps.pose.position.x = pos_quat_b_e_ik_.pos.x();
    ps.pose.position.y = pos_quat_b_e_ik_.pos.y();
    ps.pose.position.z = pos_quat_b_e_ik_.pos.z();
    ps.pose.orientation.x = pos_quat_b_e_ik_.quat.x();
    ps.pose.orientation.y = pos_quat_b_e_ik_.quat.y();
    ps.pose.orientation.z = pos_quat_b_e_ik_.quat.z();
    ps.pose.orientation.w = pos_quat_b_e_ik_.quat.w();
    ee_pose_pub_->publish(ps);
  }

  void pubTF()
  {
    // Broadcast TF from base_link_ to ee_link_ using FK result pos_quat_b_e_ik_cmd_
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now_time();
    tf_msg.header.frame_id = base_link_.empty() ? "base_link" : base_link_;
    tf_msg.child_frame_id  = ee_link_.empty()   ? "ee_link"   : ee_link_ + "_cmd";

    tf_msg.transform.translation.x = pos_quat_b_e_cmd_.pos.x();
    tf_msg.transform.translation.y = pos_quat_b_e_cmd_.pos.y();
    tf_msg.transform.translation.z = pos_quat_b_e_cmd_.pos.z();

    tf_msg.transform.rotation.x = pos_quat_b_e_cmd_.quat.x();
    tf_msg.transform.rotation.y = pos_quat_b_e_cmd_.quat.y();
    tf_msg.transform.rotation.z = pos_quat_b_e_cmd_.quat.z();
    tf_msg.transform.rotation.w = pos_quat_b_e_cmd_.quat.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

private:
  void timerCallback() 
  {
    t_ = (now_time() - start_time_).seconds();
    getPoseCommand();
    solveIK();
    solveFK();
    gripperControl();
    publishStates();
    // publishCommands();
    pubTF();
  }


  // Params / state
  std::string robot_name_, base_link_, ee_link_, rep_;
  std::vector<std::string> joint_names_;
  int num_joints_param_{0}, n_{0};
  std::vector<double> M_pos_, M_qwxyz_;

  double fs_;
  double Ts_;
  double t_;
  std::vector<double> q0_, A_, f_, phi_;
  std::chrono::duration<double> timer_period_;
  rclcpp::Time start_time_;

  MatrixXd S_;
  MatrixXd joint_limits_; // n x 4: [ll, ul, v, e]
  PosQuat M_;
  ScrewList screws_;
  PosQuat pos_quat_b_e_cmd_;
  VectorXd theta_sol_;
  VectorXd joint_angles_cmd_;
  double gripper_pos_cmd_;
  PosQuat pos_quat_b_e_ik_;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
  // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_cmd_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PoseVisualizationLeft>());
  } catch (const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
