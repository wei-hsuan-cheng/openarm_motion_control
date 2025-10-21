#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_17.hpp"
#include "robot_kinematics_utils/robot_kinematics_utils_v1_0.hpp"

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

// MMC
#include "mmc_qp_utils/iqpsolver.hpp"
#include "mmc_qp_utils/mmc_qp_builder.hpp"
#include "mmc_qp_utils/osqp_solver.hpp"

using RM = RMUtils;
using RK = RKUtils;
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
    initMMC();
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
    q_init_.setZero(n_);
    qd_init_.setZero(n_);

    // Name-keyed params: initial_joint_position.<joint>, initial_joint_velocity.<joint>
    for (int i = 0; i < n_; ++i) {
      const std::string &name = joint_names_[i];
      // Declare per-joint params so rclcpp will accept them from YAML
      const std::string pos_key = "initial_joint_position." + name;
      const std::string vel_key = "initial_joint_velocity." + name;
      declare_parameter<double>(pos_key, std::numeric_limits<double>::quiet_NaN());
      declare_parameter<double>(vel_key, std::numeric_limits<double>::quiet_NaN());

      double vpos, vvel;
      if (get_parameter(pos_key, vpos) && std::isfinite(vpos)) q_init_(i)  = vpos;
      if (get_parameter(vel_key, vvel) && std::isfinite(vvel)) qd_init_(i) = vvel;
    }

    is_first_loop_ = true;
    last_log_ = std::chrono::steady_clock::now();
  }

  void initRobotState() {
    // Init EE pose/twist via RobotKinematics
    rk_ = std::make_unique<RKUtils>(screws_);
    rk_->UpdateRobotState(q_init_, qd_init_);
    last_log_ = std::chrono::steady_clock::now();
  }

  void initTSMCParams()
  {
    is_first_loop_ = true;
    tsmc_target_reached_ = false;

    // Desired EE pose
    pos_quat_b_e_cmd_ = rk_->pose();
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
    qd_cmd_.setZero(rk_->dof());

  }

  void initMMC() {
    solver_ = std::make_unique<OsqpSolver>();
    static_cast<OsqpSolver*>(solver_.get())->setVerbose(false);

    // Declare tunable parameters with aggressive defaults
    declare_parameter<double>("mmc.lambda_q", 1e-3);
    declare_parameter<double>("mmc.lambda_delta_min", 1e-3);
    declare_parameter<double>("mmc.lambda_delta_max", 1e+3);
    declare_parameter<double>("mmc.damper_eta", 0.8);
    declare_parameter<double>("mmc.damper_rho_i_deg", 30.0); // inner zone [deg]
    declare_parameter<double>("mmc.damper_rho_s_deg", 5.0);  // stop distance [deg]
    declare_parameter<int>("mmc.jw_sign", -1);
    declare_parameter<double>("mmc.delta_bound", 2.0);
    declare_parameter<double>("mmc.t_lin", 1.0);
    declare_parameter<double>("mmc.t_ang", 1.0);
    declare_parameter<double>("mmc.max_twist_lin", 0.6);
    declare_parameter<double>("mmc.max_twist_ang", 1.6);

    get_parameter("mmc.lambda_q", mmc_params_.lambda_q);
    get_parameter("mmc.lambda_delta_min", mmc_params_.lambda_delta_min);
    get_parameter("mmc.lambda_delta_max", mmc_params_.lambda_delta_max);
    get_parameter("mmc.damper_eta", mmc_params_.eta);
    double rho_i_deg, rho_s_deg; int jw_sign;
    get_parameter("mmc.damper_rho_i_deg", rho_i_deg);
    get_parameter("mmc.damper_rho_s_deg", rho_s_deg);
    get_parameter("mmc.jw_sign", jw_sign);
    mmc_params_.rho_i = rho_i_deg * RM::d2r;
    mmc_params_.rho_s = rho_s_deg * RM::d2r;
    mmc_params_.jw_sign = jw_sign;
    double delta_bound, t_lin, t_ang;
    get_parameter("mmc.delta_bound", delta_bound);
    get_parameter("mmc.t_lin", t_lin);
    get_parameter("mmc.t_ang", t_ang);
    mmc_params_.t_lin = t_lin;
    mmc_params_.t_ang = t_ang;
    mmc_params_.delta_min = VectorXd::Constant(6, -std::abs(delta_bound));
    mmc_params_.delta_max = VectorXd::Constant(6,  std::abs(delta_bound));

    // Cache twist limits for use in MMC_QP
    get_parameter("mmc.max_twist_lin", max_twist_lin_);
    get_parameter("mmc.max_twist_ang", max_twist_ang_);
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
    Vector3d pos = Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    Quaterniond quat = Quaterniond(msg.pose.orientation.w,
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
    
    // Fill q, qd only for the arm joints, ignore unknown/extras (e.g., gripper)
    VectorXd q = rk_->q();
    VectorXd qd = rk_->qd();

    for (int i = 0; i < rk_->dof(); ++i) {
      auto it = idx.find(joint_names_[i]);
      if (it == idx.end()) continue;  // this joint not present in the msg
      std::size_t k = it->second;

      // Bounds check for position/velocity arrays
      if (k < msg.position.size()) q(i) = msg.position[k];
      if (k < msg.velocity.size()) qd(i) = msg.velocity[k];
    }
    // Update kinematic state
    rk_->UpdateRobotState(q, qd);

    received_joint_states_ = true;
  }


  // Helpers
  // Smoothstep weight
  inline double shapeK(double mval, double mbar) const {
    // Smoothstep weight s.t. 
    // m>=m_bar -> 0；
    // m<=m_bar/2 -> 1；
    // zero first-ordered derivative at endpoints

    if (mval >= mbar) return 0.0;
    if (mval <= 0.5 * mbar) return 1.0;
    // t∈[0,1]: map from [mbar/2, mbar] to [0,1]
    double t = (mval - 0.5 * mbar) / (0.5 * mbar); // t ∈ (0,1)
    // smoothstep：1 - (3t^2 - 2t^3)
    return 1.0 - (3.0 * t * t - 2.0 * t * t * t);
  }

  // Update the lowest manipulability encountered and optionally log
  void updateMinManipulability(const MatrixXd& Je, double w_current) {
    (void)Je; // reserved for future diagnostics
    if (!std::isfinite(w_current)) return;
    if (w_current < w_min_) {
      w_min_ = w_current;
      // RCLCPP_WARN(get_logger(), "[Manipulability] New minimum observed: w_min=%.6e", w_min_);
    }
  }

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
      RCLCPP_INFO(get_logger(), "pos_so3_error [m, rad]: [%.4f, %.4f], w_curr=%.6e, w_min=%.6e",
                  error_norm_mavg_(0), error_norm_mavg_(1), rk_->manipulability(), w_min_);
      last_log_ = std::chrono::steady_clock::now();
    }
  }
  
  void taskSpaceMotionController() {
    // ===== Task space motion controller =====
    // Controller input (pose command and error)
    PosQuat pos_quat_m_cmd = RM::PosQuats2RelativePosQuat(rk_->pose(), pos_quat_b_e_cmd_);
    pos_so3_m_cmd_ = RM::PosQuat2Posso3(pos_quat_m_cmd);

    // P-control for trajectory tracking
    Vector6d twist_cmd_raw = RM::KpPosso3(pos_so3_m_cmd_, kp_pos_so3_, tsmc_target_reached_);

    // Twist S-curve for smoothing
    Vector6d twist_e_mavg = RM::MAvg(rk_->twist(), twist_e_buffer_, window_size_);
    // Vector6d twist_e_mavg = Vector6d::Zero();
    Vector6d twist_cmd  = RM::SCurve(twist_cmd_raw, twist_e_mavg, scur_.lambda, k_ * Ts_, scur_.T);

    // Error thresholding
    Vector2d error_norm = Vector2d(RM::Norm(pos_so3_m_cmd_.head(3)), RM::Norm(pos_so3_m_cmd_.tail(3)));
    error_norm_mavg_ = RM::MAvg(error_norm, error_norm_buffer_, window_size_);
    auto [twist_e_cmd, tsmc_target_reached] = RM::ErrorThreshold(error_norm_mavg_, error_norm_thresh_, twist_cmd); 
    twist_e_cmd_ = twist_e_cmd;
    tsmc_target_reached_ = tsmc_target_reached;

  }

  void RRMC69() 
  {
    // ===== Resolved-rate motion control; Whitney69 =====
    // Map from twist command to joint velocity command
    // double lambda_dls = 0.0; // DLS damping
    // double lambda_dls = 1e-2; // DLS damping
    double lambda_dls = 5e-2; // DLS damping
    qd_cmd_ = rk_->JacobPinvDLS(rk_->jacob(), lambda_dls) * twist_e_cmd_;

    // Print manipulability index and update minimum seen so far
    double w = rk_->manipulability();
    updateMinManipulability(rk_->jacob(), w);
    std::cout << "w = " << w << "   (log10(w) = " << RKUtils::ManipulabilityIndexLog10(rk_->jacob())
              << ")   w_min = " << w_min_ << "\n";
  }

  void Park99() 
  {
    // ===== Manipulability maximization via Park99 =====
    // Map from twist command to joint velocity command

    // Pseudo-inverse with damped Least Squares: J⁺ = Jᵀ (J Jᵀ + λ² I_6)^{-1}
    double lambda_dls = 5e-2; // DLS damping
    // double lambda_dls = 0.0; // DLS damping
    MatrixXd Je_pinv = rk_->JacobPinvDLS(rk_->jacob(), lambda_dls);
    
    // Part controller (map from twist command to joint velocity command considering manipulability as secondary objective)
    // qd_cmd_ = J⁺ v + (1/λ) (I_n - J⁺ J) J_w
    double lambda = 1e-2; // secondary objective gain
    // Null space projection matrix
    MatrixXd Nproj = MatrixXd::Identity(rk_->dof(), rk_->dof()) - Je_pinv * rk_->jacob();
    VectorXd J_w = rk_->ManipulabilityGradient(rk_->q(), lambda_dls);
    qd_cmd_ = Je_pinv * twist_e_cmd_ + (1 / lambda) * Nproj * J_w;

    // Print manipulability index and update minimum
    double w = rk_->manipulability();
    updateMinManipulability(rk_->jacob(), w);
    // std::cout << "w = " << w << "   (log10(w) = " << RKUtils::ManipulabilityIndexLog10(rk_->jacob())
    //           << ")   w_min = " << w_min_ << "\n";
  }

  void Marani02()
  {
    // 1) δr
    const Vector6d delta_r = twist_e_cmd_; 

    // 2) Jacobian and its pseudo-inverse
    double lambda_dls = 1e-6;       
    MatrixXd Je_pinv = rk_->JacobPinvDLS(rk_->jacob(), lambda_dls); // n x 6

    // 3) manipulability and its gradient（∂m/∂q）    (Marani Eq.(6),(10))
    double w = rk_->manipulability();
    VectorXd dm_dq = rk_->ManipulabilityGradient(rk_->q(), lambda_dls); // R^n

    // 4) Normal n_m ∝ J^{+^T} (∂m/∂q)         (Marani Eq.(15))
    Vector6d nm = (dm_dq.transpose() * Je_pinv);   // R^n
    double nm_norm = nm.norm();
    if (nm_norm < 1e-12) {
      // Standard RRMC69
      qd_cmd_ = Je_pinv * delta_r;
      updateMinManipulability(rk_->jacob(), w);
      return;
    }
    nm /= nm_norm;

    // 5) Weight function k(m; \bar m) and the "escape" term k(m; \bar m/2)  (Marani Eq.(17),(19))
    // double w_thresh_ = 1e-2;
    double w_thresh_ = 2e-2;
    double k1 = shapeK(w, w_thresh_);
    double k2 = shapeK(w, 0.5 * w_thresh_);

    // 6) only work as δr ⋅ n_m > 0  (Eq.(20)-(21))
    double s = delta_r.dot(nm);
    double gate = (s > 0.0) ? 0.0 : 1.0;

    Vector6d delta_r_corr = (-1) * gate * (s) * nm * k1 + nm * k2;

    // 7) Marani Eq.(22)
    Vector6d delta_r_proj = delta_r + delta_r_corr;
    qd_cmd_ = Je_pinv * delta_r_proj;

    // 8) Logging
    updateMinManipulability(rk_->jacob(), w);
    std::cout << "[Marani02] w=" << w
              << "  k1=" << k1 << "  k2=" << k2
              << "  dot=" << s
              << "  w_min=" << w_min_ << "\n";
  }

  void MMCQP()
  {
    const auto& J = rk_->jacob();     // 6 x n

    // Saturate commanded twist to keep QP feasible
    auto saturate = [](const Vector3d& v, double limit) -> Vector3d {
      double n = v.norm();
      if (n > limit && n > 1e-12) return (v * (limit / n)).eval();
      return v;
    };

    // const double max_lin = 0.3; // [m/s]
    // const double max_ang = M_PI / 2.0; // [rad/s]
    const double max_lin = 5.0; // [m/s]
    const double max_ang = M_PI / 1.0; // [rad/s]
    Vector6d nu; // 6
    
    nu.head<3>() = saturate(twist_e_cmd_.head<3>(), max_lin);
    nu.tail<3>() = saturate(twist_e_cmd_.tail<3>(), max_ang);

    const auto q  = rk_->q();         // n
    const int  n  = rk_->dof();

    // Manipulability gradient
    VectorXd Jw = rk_->ManipulabilityGradient(q, 1e-6); // n

    // error magnitude for scheduling lambda_delta ~ 1/e
    double pos_err = RM::Norm(pos_so3_m_cmd_.head(3));
    double rot_err = RM::Norm(pos_so3_m_cmd_.tail(3));
    double e = std::max(1e-4, pos_err + rot_err);
    double lambda_delta_hint = 1.0 * 1e3 / e;
    // double lambda_delta_hint = 1.0 * 1e0 / e;

    // Cost ratio Q_c_ratio
    double Q_c_ratio = 1.0 * 1e0;

    // Build the QP: x = [qdot(n); delta(6)]
    MMCQPBuilder builder(mmc_params_);
    QPProblem prob = builder.build(J, nu, Jw, q, joint_limits_, lambda_delta_hint, Q_c_ratio);

    // Solve
    QPResult sol = solver_->solve(prob);

    if (!sol.ok || sol.x.size() != n+6) {
      // Fallback to DLS
      double lambda_dls = 5e-2;
      qd_cmd_ = rk_->JacobPinvDLS(J, lambda_dls) * nu;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[MMCQP] QP infeasible -> Fallback to DLS.");
      return;
    }
    qd_cmd_ = sol.x.head(n);

    // // print twist command (original) and optimized result and slack values
    // std::cout << "[MMCQP] nu_cmd = " << nu.transpose() << "\n";
    // std::cout << "[MMCQP] nu_opt = " << (J * qd_cmd_).transpose() << "\n";
    // std::cout << "[MMCQP] slack = " << sol.x.tail(6).transpose() << "\n";

    updateMinManipulability(rk_->jacob(), rk_->manipulability());
  }


  void publishJointVelocityCommand() {
    // Publish joint velocity command
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = joint_names_;
    js.position.resize(rk_->dof());
    js.velocity.resize(rk_->dof());
    for (int i = 0; i < rk_->dof(); ++i) {
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
    // RRMC69();
    // Park99();
    // Marani02();

    auto t0 = HighResClock::now();
    MMCQP();
    auto t1 = HighResClock::now();
    double us = elapsedUs(t0, t1);
    RCLCPP_INFO(get_logger(), "[MMCQP] Solve time/rate: %.3f [us] / %.1f [Hz]", us, 1e6/us);


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
  VectorXd q_init_;              // (n)
  VectorXd qd_init_;             // (n)
  VectorXd qd_cmd_;         // (n)
  // Robot kinematics object
  std::unique_ptr<RKUtils> rk_;
  
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

  // --- MMC QP stuff ---
  std::unique_ptr<IQPSolver> solver_;
  MMCParams mmc_params_;
  // Twist limits used to saturate commanded twist before QP
  double max_twist_lin_{0.6};
  double max_twist_ang_{1.6};


  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        pub_joint_velocity_command_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

  // logging
  std::chrono::steady_clock::time_point last_log_;

  // manipulability tracking
  double w_min_ { std::numeric_limits<double>::infinity() };
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
