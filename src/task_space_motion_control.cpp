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
#include <limits>

using RM = RMUtils;
using std::placeholders::_1;
using HighResClock = std::chrono::high_resolution_clock;

class TaskSpaceMotionControl : public rclcpp::Node {
public:
  TaskSpaceMotionControl()
  : rclcpp::Node("task_space_motion_control")
  {
    loadYAMLParams();
    initRobotConfig();
    initJointStates();
    initRobotState();
    initTSMCParams();
    initTimeSpec();
    initROSComponents();
    // Init logging
    RCLCPP_INFO(get_logger(), "Starting [TaskSpaceMotionControl] . . .\nConfigured with %d joints @ %.1f Hz. base=%s, ee=%s",
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
    declare_parameter<double>("fs", 100.0);}

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

  void initJointStates() {
    received_joint_states_ = false;
    // Init joint state
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

  void initRobotState() {
    // Init EE pose/twist
    pos_quat_b_e_ = RM::FKPoE(screws_, q_);
    twist_e_.setZero();
    last_log_ = std::chrono::steady_clock::now();
  }

  void initTSMCParams()
  {
    is_first_loop_ = true;
    tsmc_target_reached_ = false;

    // Desired EE pose
    pos_quat_b_e_cmd_ = RM::FKPoE(screws_, q_);
    received_cmd_ = false;

    // Pose controller params
    double pos_mult = 5.0 * std::pow(10.0, 2.0);
    double so3_mult = 5.0 * std::pow(10.0, 2.0);
    Vector3d kp_pos(1.0, 1.0, 1.0), kp_so3(1.0, 1.0, 1.0);
    kp_pos_so3_ = Matrix6d::Identity();
    kp_pos_so3_.topLeftCorner(3, 3)     = (pos_mult * kp_pos).asDiagonal();
    kp_pos_so3_.bottomRightCorner(3, 3) = (so3_mult * kp_so3).asDiagonal();
    error_norm_thresh_ << 1e-3, (1.0 * M_PI / 180.0);  // [m, rad]

    error_norm_mavg_ = Vector2d::Zero();

    // S-curve params
    scur_ = SCurveParams{1.0};

    // Error buffers
    window_size_ = 10;
    twist_e_buffer_.clear();
    error_norm_buffer_.clear();

    // Init joint velocity command
    qd_cmd_.setZero(n_);

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
    // sub: desired EE pose
    sub_pose_cmd_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose_command", rclcpp::SensorDataQoS(),
      std::bind(&TaskSpaceMotionControl::onPoseCmd, this, _1));
    
    // sub: current joint states
    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&TaskSpaceMotionControl::onJointStates, this, _1));

    // pub: joint velocity command
    pub_joint_velocity_command_ = create_publisher<sensor_msgs::msg::JointState>("/joint_velocity_command", rclcpp::SensorDataQoS());

    // control loop
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(Ts_)),
      std::bind(&TaskSpaceMotionControl::onTimer, this));
  }

  // ---------- subs / loop ----------
  void onPoseCmd(const geometry_msgs::msg::PoseStamped& msg) {
    Vector3d pos = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    Quaterniond quat = Eigen::Quaterniond(msg.pose.orientation.w,
                                          msg.pose.orientation.x,
                                          msg.pose.orientation.y,
                                          msg.pose.orientation.z);
    pos_quat_b_e_cmd_ = PosQuat(pos, quat);
    received_cmd_ = true;
  }

  void onJointStates(const sensor_msgs::msg::JointState& msg) {
    if (msg.name.empty() || msg.position.size() < msg.name.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Bad /joint_states (empty or positions < names). Ignoring.");
      return;
    }

    // Build quick lookup table: joint name -> index in message
    std::unordered_map<std::string, std::size_t> idx;
    idx.reserve(msg.name.size());
    for (std::size_t k = 0; k < msg.name.size(); ++k) idx[msg.name[k]] = k;
    // Fill q_, qd_ only for the n_ arm joints, ignore unknown/extras (e.g., gripper)
    for (int i = 0; i < n_; ++i) {
      auto it = idx.find(joint_names_[i]);
      if (it == idx.end()) continue;  // this joint not present in the msg
      std::size_t k = it->second;

      // Bounds check for position/velocity arrays
      if (k < msg.position.size()) q_(i) = msg.position[k];
      if (k < msg.velocity.size()) qd_(i) = msg.velocity[k];
    }

    // Update FK/DK for current end-effector pose/twist
    solveFKDK();

    received_joint_states_ = true;
  }


  // Helpers
  bool everyTimeInterval(const std::chrono::steady_clock::time_point& last_log, const double& t = 2.0) {
    auto now_steady = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_steady - last_log).count() >= t) return true;
    return false;
  }

  // Per-call timing helper
  static inline double elapsedUs(const HighResClock::time_point& t0,
                                const HighResClock::time_point& t1) {
      return std::chrono::duration<double, std::micro>(t1 - t0).count();
  }

  void periodicLogging(double t = 2.0) {
    if (everyTimeInterval(last_log_, t)) 
    {
      RCLCPP_INFO(get_logger(), "pos_so3_error [m, rad]: [%.4f, %.4f], w_min=%.6e",
                  error_norm_mavg_(0), error_norm_mavg_(1), w_min_);
      last_log_ = std::chrono::steady_clock::now();
    }
  }

  void solveFKDK() {
    // FK for current end-effector pose
    pos_quat_b_e_ = RM::FKPoE(screws_, q_);
    // DK for current end-effector twist
    twist_e_ = RM::Jacob(screws_, q_) * qd_;
  }
  
  void resolvedMotionRateController() 
  {
    // ===== Jacobian J(q) (6×n) and DLS pseudo-inverse mapping =====
    // J_e(θ)
    MatrixXd Je = RM::Jacob(screws_, q_); // 6 x n
    // Map from twist command to joint velocity command
    // double lambda_dls = 0.0; // DLS damping
    // double lambda_dls = 1e-2; // DLS damping
    double lambda_dls = 5e-2; // DLS damping
    if (lambda_dls > 0.0) {
        // Damped Least Squares: J⁺ = Jᵀ (J Jᵀ + λ² I_6)^{-1}
        MatrixXd A = Je * Je.transpose() + (lambda_dls * lambda_dls) * MatrixXd::Identity(6,6);
        qd_cmd_ = Je.transpose() * A.inverse() * twist_e_cmd_;
    } else {
        qd_cmd_ = Je.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(twist_e_cmd_);
    }

    // Print manipulability index and update minimum seen so far
    double w = RM::ManipulabilityIndex(Je);
    updateMinManipulability(Je, w);
    std::cout << "w = " << w << "   (log10(w) = " << RM::ManipulabilityIndexLog10(Je)
              << ")   w_min = " << w_min_ << "\n";
    // w_min = 0.00855028
  }

  void parkController() 
  {
    // ===== Jacobian J(q) (6×n) and DLS pseudo-inverse mapping =====
    // J_e(θ)
    MatrixXd Je = RM::Jacob(screws_, q_); // 6 x n
    // Map from twist command to joint velocity command

    // Pseudo-inverse with damped Least Squares: J⁺ = Jᵀ (J Jᵀ + λ² I_6)^{-1}
    double lambda_dls = 5e-2; // DLS damping
    // double lambda_dls = 0.0; // DLS damping
    MatrixXd A = Je * Je.transpose() + (lambda_dls * lambda_dls) * MatrixXd::Identity(6,6);
    MatrixXd Je_pinv = Je.transpose() * A.inverse();
    
    // Part controller (map from twist command to joint velocity command considering manipulability as secondary objective)
    // qd_cmd_ = J⁺ v + (1/λ) (I_n - J⁺ J) J_w
    double lambda = 1e-2; // secondary objective gain
    // Null space projection matrix
    MatrixXd Nproj = MatrixXd::Identity(n_, n_) - Je_pinv * Je;
    // auto t0 = HighResClock::now();
    VectorXd J_w = RM::ManipulabilityGradient(screws_, q_, lambda_dls);
    // auto t1 = HighResClock::now();
    qd_cmd_ = Je_pinv * twist_e_cmd_ + (1 / lambda) * Nproj * J_w;
    

    // Print manipulability index and update minimum
    double w = RM::ManipulabilityIndex(Je);
    updateMinManipulability(Je, w);
    std::cout << "w = " << w << "   (log10(w) = " << RM::ManipulabilityIndexLog10(Je)
              << ")   w_min = " << w_min_ << "\n";
    
    // w_min = 0.00855861 @ lambda_dls=5e-2, lambda=1e-1
    // w_min = 0.00861089 @ lambda_dls=5e-2, lambda=1e-2

    // auto time_elapsed = elapsedUs(t0, t1);
    // std::cout << "Time/Rate for ManipulabilityGradient: " << time_elapsed << " [us]"
    //           << " (" << 1e6 / time_elapsed << " [Hz])\n";
  }

  // ---- Helpers (加到 class private: 區塊) ----
  // Smoothstep 權重，滿足：m>=m_bar → 0；m<=m_bar/2 → 1；端點一階導數為 0
  inline double shapeK(double mval, double mbar) const {
    if (mval >= mbar) return 0.0;
    if (mval <= 0.5 * mbar) return 1.0;
    // t∈[0,1]: mval 從 mbar/2 → mbar 映到 1→0 的平滑三次曲線
    double t = (mval - 0.5 * mbar) / (0.5 * mbar); // t ∈ (0,1)
    // smoothstep 反向：1 - (3t^2 - 2t^3)
    return 1.0 - (3.0 * t * t - 2.0 * t * t * t);
  }

  // 取得 Jacobian 的（微）阻尼廣義逆 J^+ = J^T (JJ^T + eps^2 I)^{-1}
  inline Eigen::MatrixXd dlsPinv(const Eigen::MatrixXd& J, double eps) const {
    if (eps <= 0.0) {
      return J.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Eigen::MatrixXd::Identity(J.rows(), J.rows()));
    }
    Eigen::MatrixXd A = J * J.transpose() + (eps * eps) * Eigen::MatrixXd::Identity(J.rows(), J.rows());
    return J.transpose() * A.inverse();
  }

  // ---- 新的控制器：奇異點迴避 RMRC (第 3 節) ----
  void singularityAvoidanceController()
  {
    // 1) 目標扭速度(任務空間)：δr
    const Vector6d delta_r = twist_e_cmd_; // 你的 task-space 指令

    // 2) Jacobian 與 pseudo-inverse
    MatrixXd Je = RM::Jacob(screws_, q_);          // 6 x n
    double pinv_dls_eps_ = 1e-6;       // 只做數值穩定的很小阻尼
    MatrixXd Je_pinv = dlsPinv(Je, pinv_dls_eps_); // n x 6（eps 極小，只為數值穩定）

    // 3) manipulability 與其梯度（∂m/∂q）    (Marani Eq.(6),(10))
    double mom = RM::ManipulabilityIndex(Je);
    VectorXd dm_dq = RM::ManipulabilityGradient(screws_, q_, /*lambda_dls=*/0.0); // n x 1

    // 4) 法向量 n_m ∝ J^{+^T} (∂m/∂q)         (Marani Eq.(15))
    Vector6d nm = (Je_pinv.transpose() * dm_dq);   // 6 x 1
    double nm_norm = nm.norm();
    if (nm_norm < 1e-12) {
      // 無法穩定計算法向量，退回標準 RMRC
      qd_cmd_ = Je_pinv * delta_r;
      // 追蹤最小操控度
      updateMinManipulability(Je, mom);
      return;
    }
    nm /= nm_norm;

    // 5) 權重函數 k(m; \bar m) 與「回拉」項 k(m; \bar m/2)  (Marani Eq.(17),(19))
    double mom_thresh_ = 0.008;         // 可調下限 \bar m，越大越保守
    double k1 = shapeK(mom, mom_thresh_);          // 接近奇異時逐步壓投影
    double k2 = shapeK(mom, 0.5 * mom_thresh_);    // 進入內側時回拉至等值面

    // 6) 僅在「操控度在下降方向」時才投影 (δr ⋅ n_m > 0)  (文字說明 + Eq.(20)-(21))
    double s = delta_r.dot(nm);
    double gate = (s > 0.0) ? 1.0 : 0.0;           // 只在趨近奇異時作用

    // 校正量 δr_corr = gate * (δr⋅n_m) n_m k1  -  （負號體現在 δr_p = δr - δr_corr）
    //                 - k2 * (-n_m) ；等價於 + k2 * n_m（回拉向外）
    Vector6d delta_r_corr = gate * (s) * nm * k1 + k2 * nm;

    // 7) 投影後任務與關節速度     (Marani Eq.(22))
    Vector6d delta_r_proj = delta_r - delta_r_corr;
    qd_cmd_ = Je_pinv * delta_r_proj;

    // 8) 記錄/列印
    updateMinManipulability(Je, mom);
    std::cout << "[SA-RMRC] m=" << mom
              << "  k1=" << k1 << "  k2=" << k2
              << "  dot=" << s
              << "  w_min=" << w_min_ << "\n";
  }


  void taskSpaceMotionController() {
    // ===== Task space motion controller =====
    // Controller input (pose command and error)
    PosQuat pos_quat_m_cmd = RM::PosQuats2RelativePosQuat(pos_quat_b_e_, pos_quat_b_e_cmd_);
    pos_so3_m_cmd_ = RM::PosQuat2Posso3(pos_quat_m_cmd);

    // P-control for trajectory tracking
    Vector6d twist_cmd_raw = RM::KpPosso3(pos_so3_m_cmd_, kp_pos_so3_, tsmc_target_reached_);

    // Twist S-curve for smoothing
    Vector6d twist_e_mavg = RM::MAvg(twist_e_, twist_e_buffer_, window_size_);
    // Vector6d twist_e_mavg = Vector6d::Zero();
    Vector6d twist_cmd  = RM::SCurve(twist_cmd_raw, twist_e_mavg, scur_.lambda, k_ * Ts_, scur_.T);

    // Error thresholding
    Vector2d error_norm = Vector2d(RM::Norm(pos_so3_m_cmd_.head(3)), RM::Norm(pos_so3_m_cmd_.tail(3)));
    error_norm_mavg_ = RM::MAvg(error_norm, error_norm_buffer_, window_size_);
    auto [twist_e_cmd, tsmc_target_reached] = RM::ErrorThreshold(error_norm_mavg_, error_norm_thresh_, twist_cmd); 
    twist_e_cmd_ = twist_e_cmd;
    tsmc_target_reached_ = tsmc_target_reached;

  }

  void publishJointVelocityCommand() {
    // Publish joint velocity command
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = joint_names_;
    js.position.resize(n_);
    js.velocity.resize(n_);
    for (int i = 0; i < n_; ++i) {
      js.velocity[i] = qd_cmd_(i);
    }
    pub_joint_velocity_command_->publish(js);
  }

  void onTimer() {
    if (!received_cmd_ || !received_joint_states_) {
      if (everyTimeInterval(last_log_, 1.0)) {
        RCLCPP_WARN(get_logger(), "Waiting for /pose_command or /joint_states messages...");
        last_log_ = std::chrono::steady_clock::now();

        if(received_joint_states_) {
          RCLCPP_INFO(get_logger(), "/joint_states received.");
        }
        if(received_cmd_) {
          RCLCPP_INFO(get_logger(), "/pose_command received.");
        }
      }
      return;
    }

    if (is_first_loop_)
    {
        k_ = 0;
        is_first_loop_ = false;
    }

    // Compute twist command from pose error
    taskSpaceMotionController();

    // Map twist command to joint velocity command
    resolvedMotionRateController();
    // parkController();
    // singularityAvoidanceController();

    // Send joint velocity command to motors
    publishJointVelocityCommand();
    
    // Periodic logging
    periodicLogging(2.0);

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
  bool received_cmd_, received_joint_states_;
  PosQuat pos_quat_b_e_cmd_;
  Vector6d pos_so3_m_cmd_;
  Vector2d error_norm_mavg_;
  Matrix6d kp_pos_so3_;
  Vector6d twist_e_cmd_;
  bool tsmc_target_reached_;

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
  std::size_t window_size_;

  // timing
  double fs_, Ts_, k_, t_;
  bool is_first_loop_;
  double a_;
  double vel_limit_;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        pub_joint_velocity_command_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

  // logging
  std::chrono::steady_clock::time_point last_log_;

  // manipulability tracking
  double w_min_ { std::numeric_limits<double>::infinity() };

  // Update the lowest manipulability encountered and optionally log
  void updateMinManipulability(const MatrixXd& Je, double w_current) {
    (void)Je; // reserved for future diagnostics
    if (!std::isfinite(w_current)) return;
    if (w_current < w_min_) {
      w_min_ = w_current;
      RCLCPP_WARN(get_logger(), "[Manipulability] New minimum observed: w_min=%.6e", w_min_);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TaskSpaceMotionControl>());
  } catch (const std::exception& e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
