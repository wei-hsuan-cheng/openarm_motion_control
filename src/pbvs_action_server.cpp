#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// The generated header from "PBVS.action"
#include "action_interfaces/action/pbvs.hpp"

// Standard includes 
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include <tm_msgs/msg/feedback_state.hpp>
#include <std_srvs/srv/set_bool.hpp> // for arm vmode action
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// Robot math utils
#include "robot_math_utils/robot_math_utils_v1_9.hpp" 

// Eigen, etc.
#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>


using PBVS = action_interfaces::action::PBVS;
using GoalHandlePBVS  = rclcpp_action::ServerGoalHandle<PBVS>;

using RM = RMUtils;
std::string node_name = "pbvs_action_server";

class PBVSActionServer : public rclcpp::Node
{
public:
  PBVSActionServer()
  : Node(node_name),
    fs_(30.0),            
    Ts_(1.0 / fs_),
    ekf_data_ready_(false),
    arm_data_ready_(false)
  {
    // Initialise params
    fsm_state_   = PBVSFSMState::IDLE; // Initial state duing the execution
    stop_action_ = false;
    twist_frame_id_ = "flange"; // "tcp"
    k_ = 0;
    t_ = 0;

    std::cout << "[PBVS] Loaded params" << std::endl;
    /* TCP setting */
    update_tcp_params();
    /* PBVS */
    update_pbvs_controller_params();
    /* Selected target ID */
    init_selected_target_id();
    /* Ground-truth poses */
    update_gt_poses();
    /* Initialise data logging */
    update_datalogs();


    // ROS 2 components
    // Arm vmode service client
    vmode_client_ = this->create_client<std_srvs::srv::SetBool>("/forward_velocity_controller/enable");
    RCLCPP_INFO(this->get_logger(), "[VMode] Wait for /forward_velocity_controller/enable service...");
    if (!vmode_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(this->get_logger(), "[VMode] Not available after waiting");
    }
    else {
      RCLCPP_INFO(this->get_logger(), "[VMode] Service is ready!");
    }

    // Action server
    action_server__ = rclcpp_action::create_server<PBVS>(
      this,
      "pbvs", // the action name
      std::bind(&PBVSActionServer::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PBVSActionServer::handle_cancel,   this, std::placeholders::_1),
      std::bind(&PBVSActionServer::handle_accepted, this, std::placeholders::_1)
    );

    arm_fb_sub_ = this->create_subscription<tm_msgs::msg::FeedbackState>(
      "/feedback_states", 10,
      std::bind(&PBVSActionServer::arm_fb_callback_, this, std::placeholders::_1));

    ekf_pos_quat_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      "/ekf/pos_quat_b_ob", 10, 
      std::bind(&PBVSActionServer::ekf_pos_quat_callback_, this, std::placeholders::_1));

    global_fsm_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/fsm_state", 1, std::bind(&PBVSActionServer::GlobalFSMStateCallback, this, std::placeholders::_1));
    
    global_fsm_select_target_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/fsm_flag/select_target", 10, std::bind(&PBVSActionServer::GlobalFSMSelectTargetCallback, this, std::placeholders::_1));
        
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
    vision_aligned_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pbvs/vision_aligned", 1);

    RCLCPP_INFO(this->get_logger(), "PBVS Action Server started.");
  }

private:
  // Service/Action server and client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr vmode_client_;
  rclcpp_action::Server<PBVS>::SharedPtr action_server__;

  // Subs and pubs
  rclcpp::Subscription<tm_msgs::msg::FeedbackState>::SharedPtr arm_fb_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr ekf_pos_quat_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr global_fsm_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr global_fsm_select_target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_aligned_pub_;

  /* Control sampling rate */
  double fs_, Ts_;
  int k_;
  double t_;
  
  /* Rigid transform */
  PosQuat pos_quat_f_tcp_;

  /* Robot data */
  Vector6d pose_b_f_, twist_f_f_, twist_tcp_tcp_;
  PosQuat pos_quat_b_f_, pos_quat_b_tcp_, pos_quat_b_tcp_init_motion_, pos_quat_f_ob_, pos_quat_b_ob_;
  
  /* PBVS params */
  PosQuat pos_quat_tcp_ob_cmd_;
  Vector6d pos_so3_m_cmd_;
  Matrix6d kp_pos_so3_;
  std::string twist_frame_id_;

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

  /* Ground-truth poses */
  Vector6d pose_b_tcp_gt_vision_;
  PosQuat pos_quat_b_tcp_gt_vision_;

  // Buffers
  std::deque<Vector6d> twist_f_f_buffer_, twist_tcp_tcp_buffer_;
  std::deque<Vector2d> error_norm_buffer_;
  std::size_t window_size_vision_;

  // FSM
  bool stop_action_;
  bool ekf_data_ready_, arm_data_ready_;
  bool pbvs_target_reached_;
  std::mutex data_mutex_;

  enum class PBVSFSMState {
    IDLE,
    VISION_ALIGNING,
    VISION_ALIGNED
  };
  PBVSFSMState fsm_state_;

  // Selected target ID
  int selected_target_id_;

  /* Data logging */
  std::_Put_time<char> datetime_;
  std::ofstream csv_writer_vision_;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
  bool is_first_loop_vision_;

  // -------------
  // SUB + UTILS
  // -------------
  void arm_fb_callback_(const tm_msgs::msg::FeedbackState::SharedPtr msg)
  {
    // Pose
    pose_b_f_ = Vector6d(
      msg->tool_pose[0], msg->tool_pose[1], msg->tool_pose[2],
      RM::ConstrainedAngle(msg->tool_pose[3], true),
      RM::ConstrainedAngle(msg->tool_pose[4], true),
      RM::ConstrainedAngle(msg->tool_pose[5], true)
    );
    pos_quat_b_f_ = RM::R6Pose2PosQuat(pose_b_f_);
    Matrix3d R_b_f = RM::R6Pose2PosRot(pose_b_f_).rot;

    pos_quat_b_tcp_ = RM::TransformPosQuats({pos_quat_b_f_, pos_quat_f_tcp_});
    Matrix3d R_b_tcp = RM::TMat2PosRot(RM::PosQuat2TMat(pos_quat_b_tcp_)).rot;
    Vector3d p_f_tcp = RM::TMat2PosRot(RM::PosQuat2TMat(pos_quat_f_tcp_)).pos;
    Vector3d tcp_offset_b = R_b_f * p_f_tcp;

    // Twist
    Vector6d twist_b_f(
      msg->tcp_speed[0], msg->tcp_speed[1], msg->tcp_speed[2],
      msg->tcp_speed[3], msg->tcp_speed[4], msg->tcp_speed[5]
    );
    
    twist_f_f_ = RM::AdjointB2E(R_b_f, Vector3d::Zero(), twist_b_f);
    twist_tcp_tcp_ = RM::AdjointB2E(R_b_tcp, tcp_offset_b, twist_b_f);

    arm_data_ready_ = true;
  }

  void ekf_pos_quat_callback_(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pos_quat_b_ob_ = PosQuat(Vector3d(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z),
                          Quaterniond(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z));
    ekf_data_ready_ = true;
  }

  void GlobalFSMStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->data == "IDLE" || msg->data == "emergency_stop") {
      stop_action_ = true;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[PBVS] Global FSM state: %s", msg->data.c_str());
    } else {
      stop_action_ = false;
    }
  }

  void GlobalFSMSelectTargetCallback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    selected_target_id_ = msg->data;
    update_selected_target_id();
  }


  /* Arm vmode */
  void setVmode(bool enable_or_not)
  {
    // 1) Create request
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = enable_or_not;

    std::string mode_str = enable_or_not ? "ENABLED" : "DISABLED";
    RCLCPP_INFO(this->get_logger(),
                "[VMode] Requesting velocity mode => %s", mode_str.c_str());

    // 2) Send request asynchronously
    auto future_result = vmode_client_->async_send_request(
      req,
      // This is the callback once the service responds
      [this, enable_or_not](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
      {
        // Defensive check
        if (!future.valid()) {
          RCLCPP_ERROR(this->get_logger(),
                      "[VMode] Future invalid => no response from /forward_velocity_controller/enable");
          return;
        }
        auto response = future.get();
        std::string mode_str = enable_or_not ? "enabled" : "disabled";
        if (response->success) {
          RCLCPP_INFO(this->get_logger(),
                      "[VMode] Velocity controller %s (message='%s')",
                      mode_str.c_str(), response->message.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "[VMode] Failed to set velocity mode => %s (message='%s')",
                      mode_str.c_str(), response->message.c_str());
        }
      }
    );
  }


  /* Controller params */
  void update_tcp_params()
  {
    // Load pos_quat_f_tcp_ from yaml file
    // Declare default parameters (in case the YAML file doesn't provide them)
    this->declare_parameter<std::vector<double>>("pose_f_tcp_translation", {0.0, 0.0, 0.0});
    this->declare_parameter<double>("pose_f_tcp_rotation_z", 0.0);

    // Retrieve the parameters
    std::vector<double> translation;
    double rotation_z;
    this->get_parameter("pose_f_tcp_translation", translation);
    this->get_parameter("pose_f_tcp_rotation_z", rotation_z);
    pos_quat_f_tcp_ = PosQuat(Vector3d(translation[0], translation[1], translation[2]), 
                              RM::Quatz(rotation_z));

    std::cout << "pos_quat_f_tcp_: " << pos_quat_f_tcp_.pos.transpose() << ", " << pos_quat_f_tcp_.quat.w() << ", " << pos_quat_f_tcp_.quat.x() << ", " << pos_quat_f_tcp_.quat.y() << ", " << pos_quat_f_tcp_.quat.z() << std::endl;
  }

  void update_pbvs_controller_params()
  {
    
    is_first_loop_vision_ = true;
    pbvs_target_reached_ = false;

    // PBVS controller params
    // double pos_mult = 0.5 * std::pow(10.0, 0.5);
    double pos_mult = 0.25 * std::pow(10.0, 0.5);
    double so3_mult = 0.2 * std::pow(10.0, 0.5);
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
    twist_f_f_buffer_.clear();
    error_norm_buffer_.clear();

    // RCLCPP_INFO(this->get_logger(), "PBVS controller parameters updated.");
  }

  void init_selected_target_id()
  {
    // Target IDs
    // gv: 1
    // ov: 2
    // rpb: 3

    // Load pos_quat_tcp_ob_cmd_ from yaml file
    // Declare default parameters (in case the YAML file doesn't provide them)
    // gv: 1
    this->declare_parameter<std::vector<double>>("pose_tcp_gv_cmd_pos", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pose_tcp_gv_cmd_zyx_euler", {0.0, 0.0, 0.0});
    
    // ov: 2
    this->declare_parameter<std::vector<double>>("pose_tcp_ov_cmd_pos", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pose_tcp_ov_cmd_zyx_euler", {0.0, 0.0, 0.0});
    
    // rpb: 3
    this->declare_parameter<std::vector<double>>("pose_tcp_rpb_cmd_pos", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pose_tcp_rpb_cmd_zyx_euler", {0.0, 0.0, 0.0});

    // Load selected_target_id from yaml file
    // Declare default parameters (in case the YAML file doesn't provide them)
    this->declare_parameter<int>("selected_target_id", -1);
    int selected_target_id;
    this->get_parameter("selected_target_id", selected_target_id);
    selected_target_id_ = selected_target_id;

    update_selected_target_id();

  }

  void update_selected_target_id()
  {
    switch(selected_target_id_)
    {
      case 1:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Selected target (%d): gv", selected_target_id_);
          // Retrieve the parameters
          std::vector<double> pos;
          std::vector<double> zyx_euler;
          this->get_parameter("pose_tcp_gv_cmd_pos", pos);
          this->get_parameter("pose_tcp_gv_cmd_zyx_euler", zyx_euler);
          pos_quat_tcp_ob_cmd_ = RM::InvPosQuat( PosQuat( Vector3d(pos[0], pos[1], pos[2]), 
                                                 RM::zyxEuler2Quat( Vector3d(zyx_euler[0], zyx_euler[1], zyx_euler[2]) ) ) 
                                               );
          break;
        }

      case 2:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Selected target (%d): ov", selected_target_id_);
          // Retrieve the parameters
          std::vector<double> pos;
          std::vector<double> zyx_euler;
          this->get_parameter("pose_tcp_ov_cmd_pos", pos);
          this->get_parameter("pose_tcp_ov_cmd_zyx_euler", zyx_euler);
          pos_quat_tcp_ob_cmd_ = RM::InvPosQuat( PosQuat( Vector3d(pos[0], pos[1], pos[2]), 
                                                 RM::zyxEuler2Quat( Vector3d(zyx_euler[0], zyx_euler[1], zyx_euler[2]) ) ) 
                                               );
          break;
        }

      case 3:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Selected target (%d): rpb", selected_target_id_);
          // Retrieve the parameters
          std::vector<double> pos;
          std::vector<double> zyx_euler;
          this->get_parameter("pose_tcp_rpb_cmd_pos", pos);
          this->get_parameter("pose_tcp_rpb_cmd_zyx_euler", zyx_euler);
          pos_quat_tcp_ob_cmd_ = RM::InvPosQuat( PosQuat( Vector3d(pos[0], pos[1], pos[2]), 
                                                 RM::zyxEuler2Quat( Vector3d(zyx_euler[0], zyx_euler[1], zyx_euler[2]) ) ) 
                                               );
          break;
        }

      case -1:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Selected target ID not set. . .");
          break;
        }
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[PBVSActionServer] pos_quat_tcp_ob_cmd_: %f, %f, %f, %f, %f, %f, %f", 
                               pos_quat_tcp_ob_cmd_.pos(0), pos_quat_tcp_ob_cmd_.pos(1), pos_quat_tcp_ob_cmd_.pos(2), 
                               pos_quat_tcp_ob_cmd_.quat.w(), pos_quat_tcp_ob_cmd_.quat.x(), pos_quat_tcp_ob_cmd_.quat.y(), pos_quat_tcp_ob_cmd_.quat.z());


  }

  void update_gt_poses() 
  {
    // Ground-truth poses 240109
    // For PBVS
    pose_b_tcp_gt_vision_ = Vector6d(0.689339, 0.0508214, 0.522533, 1.94342, 1.52096, 1.85536); // [m, rad]

    RCLCPP_INFO(this->get_logger(), "pose_b_tcp_gt_vision_: %f, %f, %f, %f, %f, %f", pose_b_tcp_gt_vision_(0), pose_b_tcp_gt_vision_(1), pose_b_tcp_gt_vision_(2), pose_b_tcp_gt_vision_(3), pose_b_tcp_gt_vision_(4), pose_b_tcp_gt_vision_(5));
    RCLCPP_INFO(this->get_logger(), "Ground-truth poses updated.");

  }

  void update_datalogs()
  {
    const std::string& datalog_path = "./datalog/pbvs";

    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    datetime_ = std::put_time(std::localtime(&now), "%Y-%m-%d_%H-%M-%S");
    std::ostringstream datalog_filename_vision_;

    datalog_filename_vision_ << datalog_path << "/" << node_name << "_vision_" << datetime_ << ".csv";


    std::vector<std::string> header_row_vision = {
                                            "datetime", "k [idx]", "kTs [s]", "t [s]",
                                            "vx_cmd", "vy_cmd", "vz_cmd", "wx_cmd", "wy_cmd", "wz_cmd",
                                            "vx_cmd_scurve", "vy_cmd_scurve", "vz_cmd_scurve", "wx_cmd_scurve", "wy_cmd_scurve", "wz_cmd_scurve",
                                            "vx_m", "vy_m", "vz_m", "wx_m", "wy_m", "wz_m",
                                            "x_m_cmd", "y_m_cmd", "z_m_cmd", "uthx_m_cmd", "uthy_m_cmd", "uthz_m_cmd",
                                            "x_b_f_cmd", "y_b_f_cmd", "z_b_f_cmd", "Rx_b_f_cmd", "Ry_b_f_cmd", "Rz_b_f_cmd",
                                            "x_b_f", "y_b_f", "z_b_f", "Rx_b_f", "Ry_b_f", "Rz_b_f",
                                            "x_b_tcp", "y_b_tcp", "z_b_tcp", "Rx_b_tcp", "Ry_b_tcp", "Rz_b_tcp",
                                            "x_b_tcp_gt_vision", "y_b_tcp_gt_vision", "z_b_tcp_gt_vision", "Rx_b_tcp_gt_vision", "Ry_b_tcp_gt_vision", "Rz_b_tcp_gt_vision",
                                            "x_m_cmd_tcp_vision", "y_m_cmd_tcp_vision", "z_m_cmd_tcp_vision", "uthx_m_cmd_tcp_vision", "uthy_m_cmd_tcp_vision", "uthz_m_cmd_tcp_vision",
                                            "x_m_gt_tcp_vision", "y_m_gt_tcp_vision", "z_m_gt_tcp_vision", "uthx_m_gt_tcp_vision", "uthy_m_gt_tcp_vision", "uthz_m_gt_tcp_vision",
                                        };


    RM::InitDatalog(csv_writer_vision_, datalog_filename_vision_.str(), header_row_vision);
    RCLCPP_INFO(this->get_logger(), "Data logs updated. file names: %s", datalog_filename_vision_.str().c_str());
  }

  Vector6d mrad2mmdeg(const Vector6d & vec)
  {
    Vector6d vec_mm_deg;
    vec_mm_deg << vec.head(3) * 1000.0, vec.tail(3) * (180.0 / M_PI);
    return vec_mm_deg;
  }

  double getTimeSeconds() const
  {
    using namespace std::chrono;
    return duration<double>(high_resolution_clock::now() - start_time_).count() * 1e-9; // [s]
  }

  bool check_target_reached()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    bool pbvs_target_reached = false;

    // Controller input (pose command and error)
    // w.r.t. flange
    PosQuat pos_quat_f_ob_cmd = RM::TransformPosQuats({pos_quat_f_tcp_, pos_quat_tcp_ob_cmd_});
    PosQuat pos_quat_b_f_cmd = RM::TransformPosQuats({pos_quat_b_ob_, RM::InvPosQuat(pos_quat_f_ob_cmd)});
    PosQuat pos_quat_m_cmd = RM::PosQuats2RelativePosQuat(pos_quat_b_f_, pos_quat_b_f_cmd);

    // // w.r.t. tcp
    // PosQuat pos_quat_b_tcp_cmd = RM::TransformPosQuats({pos_quat_b_ob_, RM::InvPosQuat(pos_quat_tcp_ob_cmd_)});
    // PosQuat pos_quat_m_cmd = RM::PosQuats2RelativePosQuat(pos_quat_b_tcp_, pos_quat_b_tcp_cmd);

    pos_so3_m_cmd_ = RM::PosQuat2Posso3(pos_quat_m_cmd);

    // Error thresholding
    Vector2d error_norm = Vector2d(RM::Norm(pos_so3_m_cmd_.head(3)), RM::Norm(pos_so3_m_cmd_.tail(3)));
    Vector2d error_norm_mavg = RM::MAvg(error_norm, error_norm_buffer_, window_size_vision_);

    if ((error_norm_mavg.array() <= error_norm_thresh_.array()).all()) {
            pbvs_target_reached = true;
    }

    return pbvs_target_reached;
  }

  Vector6d pbvs_control_law()
  {
    
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (is_first_loop_vision_)
    {
        start_time_ = std::chrono::high_resolution_clock::now(); // Set start time on the first loop
        k_ = 0;
        is_first_loop_vision_ = false;
    }

    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    datetime_ = std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
    t_ = getTimeSeconds();

    // Controller input (pose command and error)
    // w.r.t. flange
    PosQuat pos_quat_f_ob_cmd = RM::TransformPosQuats({pos_quat_f_tcp_, pos_quat_tcp_ob_cmd_});
    PosQuat pos_quat_b_f_cmd = RM::TransformPosQuats({pos_quat_b_ob_, RM::InvPosQuat(pos_quat_f_ob_cmd)});
    PosQuat pos_quat_m_cmd = RM::PosQuats2RelativePosQuat(pos_quat_b_f_, pos_quat_b_f_cmd);

    // // w.r.t. tcp
    PosQuat pos_quat_b_tcp_cmd = RM::TransformPosQuats({pos_quat_b_ob_, RM::InvPosQuat(pos_quat_tcp_ob_cmd_)});
    // PosQuat pos_quat_m_cmd = RM::PosQuats2RelativePosQuat(pos_quat_b_tcp_, pos_quat_b_tcp_cmd);
    
    pos_so3_m_cmd_ = RM::PosQuat2Posso3(pos_quat_m_cmd);
    Vector6d pos_so3_m_cmd_tcp = RM::PosQuat2Posso3( RM::PosQuats2RelativePosQuat(pos_quat_b_tcp_, pos_quat_b_tcp_cmd) );
    Vector6d pos_so3_m_gt_tcp = RM::PosQuat2Posso3( RM::PosQuats2RelativePosQuat(pos_quat_b_tcp_, pos_quat_b_tcp_gt_vision_) );

    // P-control for trajectory tracking
    Vector6d twist_cmd_raw = RM::KpPosso3(pos_so3_m_cmd_, kp_pos_so3_, pbvs_target_reached_);

    // Twist S-curve for smoothing
    Vector6d twist_f_f_mavg = RM::MAvg(twist_f_f_, twist_f_f_buffer_, window_size_vision_);
    Vector6d twist_cmd  = RM::SCurve(twist_cmd_raw, twist_f_f_mavg, scur_.lambda, k_ * Ts_, scur_.T);

    // Error thresholding
    Vector2d error_norm = Vector2d(RM::Norm(pos_so3_m_cmd_.head(3)), RM::Norm(pos_so3_m_cmd_.tail(3)));
    Vector2d error_norm_mavg = RM::MAvg(error_norm, error_norm_buffer_, window_size_vision_);
    auto [twist_f_f_vision_cmd_, pbvs_target_reached_] = RM::ErrorThreshold(error_norm_mavg, error_norm_thresh_, twist_cmd); 

    // Data logging
    Vector6d pose_b_f_cmd = RM::PosQuat2R6Pose(pos_quat_b_f_cmd);
    Vector6d pose_b_tcp_datalog = RM::PosQuat2R6Pose(pos_quat_b_tcp_);
    std::vector<VectorXd> data_to_log = {
                                        twist_cmd_raw,
                                        twist_cmd,
                                        twist_f_f_,
                                        pos_so3_m_cmd_,
                                        pose_b_f_cmd,
                                        pose_b_f_,
                                        pose_b_tcp_datalog,
                                        pose_b_tcp_gt_vision_,
                                        pos_so3_m_cmd_tcp,
                                        pos_so3_m_gt_tcp
                                        };

    RM::Datalog(csv_writer_vision_, datetime_, k_, Ts_, t_, data_to_log);   

    k_++;
    
    // twist_f_f_vision_cmd_.setZero();
    return twist_f_f_vision_cmd_;
    
  }

  /* PBVS */
  bool vision_alignment()
  {
    Vector6d twist_cmd = pbvs_control_law();
    bool pbvs_target_reached = check_target_reached();

    // Publish twist
    sendTwistCommand(twist_cmd, twist_pub_, twist_frame_id_);

    // Return if target reached in PBVS
    return pbvs_target_reached;
  }

  void sendTwistCommand(const Vector6d & cmd, 
                        const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr & pub,
                        const std::string & frame_id = "tcp")
  {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp    = now();
    twist_msg.header.frame_id = frame_id;
    twist_msg.twist.linear.x  = cmd(0);
    twist_msg.twist.linear.y  = cmd(1);
    twist_msg.twist.linear.z  = cmd(2);
    twist_msg.twist.angular.x = cmd(3);
    twist_msg.twist.angular.y = cmd(4);
    twist_msg.twist.angular.z = cmd(5);
    pub->publish(twist_msg);
  }


  // -------------
  // GOAL callbacks
  // -------------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const PBVS::Goal> goal_msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received PBVS goal: start=%d", (int)goal_msg->start);
    if (!goal_msg->start) {
      RCLCPP_WARN(this->get_logger(), "Goal->start == false => reject");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Update parameters each time goal is accepted
    fsm_state_ = PBVSFSMState::IDLE;

    RCLCPP_INFO(this->get_logger(), "Goal accepted => start self-alignment!");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePBVS> /*gh*/)
  {
    RCLCPP_INFO(this->get_logger(), "[PBVS] Cancel request => accept");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePBVS> gh)
  {
    // Spin up a new thread
    std::thread([this, gh]() {
      this->execute(gh);
    }).detach();
  }



  // -------------
  // EXECUTION
  // -------------
  void execute(const std::shared_ptr<GoalHandlePBVS> gh)
  {
    RCLCPP_INFO(this->get_logger(), "[PBVS] Start execution (PBVS).");

    auto result   = std::make_shared<PBVS::Result>();
    auto feedback = std::make_shared<PBVS::Feedback>();

    /* Main loop */
    rclcpp::Rate loop_rate(fs_);
    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        sendTwistCommand(Vector6d::Zero(), twist_pub_, twist_frame_id_);
        RCLCPP_WARN(this->get_logger(), "[PBVS] Canceled by client!");
        result->success = false;
        result->message = "Canceled by client";
        gh->canceled(result);
        return;
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (stop_action_) {
          sendTwistCommand(Vector6d::Zero(), twist_pub_, twist_frame_id_);
          RCLCPP_WARN(this->get_logger(), "[PBVS] Stopped externally => abort");
          result->success = false;
          result->message = "Stopped externally";
          gh->abort(result);
          return;
        }
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!ekf_data_ready_ || !arm_data_ready_) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[PBVS] Waiting for data subscription. . .");
          loop_rate.sleep();
          continue;
        }
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (selected_target_id_ == -1) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[PBVS] Waiting for target ID selection. . .");
          loop_rate.sleep();
          continue;
        }
      }

      // Evaluate the current pos_so3_error
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Build feedback => pos_so3_error and time_elapsed
        feedback->pos_so3_error.clear();
        feedback->pos_so3_error.push_back(pos_so3_m_cmd_(0));
        feedback->pos_so3_error.push_back(pos_so3_m_cmd_(1));
        feedback->pos_so3_error.push_back(pos_so3_m_cmd_(2));
        feedback->pos_so3_error.push_back(pos_so3_m_cmd_(3));
        feedback->pos_so3_error.push_back(pos_so3_m_cmd_(4));
        feedback->pos_so3_error.push_back(pos_so3_m_cmd_(5));

        feedback->time_elapsed = t_;

        // Publish feedback
        gh->publish_feedback(feedback);

      }

  
      // Switch FSM states
      switch (fsm_state_)
      {
        case PBVSFSMState::IDLE:
        {
          sendTwistCommand(Vector6d::Zero(), twist_pub_, twist_frame_id_);
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[PBVS FSM] IDLE");
          if (true) {
            fsm_state_ = PBVSFSMState::VISION_ALIGNING; // Start vision alignment
          }
          break;
        }
          
        case PBVSFSMState::VISION_ALIGNING:
        {
          pbvs_target_reached_ = vision_alignment();
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[PBVS FSM] VISION_ALIGNING");
          if (pbvs_target_reached_) {
            fsm_state_ = PBVSFSMState::VISION_ALIGNED;
          }
          break;
        }
          
        case PBVSFSMState::VISION_ALIGNED:
        {
          sendTwistCommand(Vector6d::Zero(), twist_pub_, twist_frame_id_);
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[PBVS FSM] VISION_ALIGNED");

          // Switch to position control mode
          setVmode(false);
          
          RCLCPP_INFO(this->get_logger(), "[PBVS] Posso3 error within error bound => success!");
          RCLCPP_INFO(this->get_logger(), "[PBVS] Last pos_so3_error: %f, %f, %f, %f, %f, %f",
                      pos_so3_m_cmd_(0), pos_so3_m_cmd_(1), pos_so3_m_cmd_(2),
                      pos_so3_m_cmd_(3), pos_so3_m_cmd_(4), pos_so3_m_cmd_(5));
          RCLCPP_INFO(this->get_logger(), "[PBVS] Time elapsed: %f", t_);
          
          result->success = true;
          result->message = "Posso3 error is within error bound => PBVS done.";
          gh->succeed(result);

          std_msgs::msg::Bool msg_vision_aligned;
          msg_vision_aligned.data = pbvs_target_reached_;
          vision_aligned_pub_->publish(msg_vision_aligned);
          fsm_state_ = PBVSFSMState::IDLE;
          return;
        }

      }

      loop_rate.sleep();
    } // while ok

    // Shut down
    sendTwistCommand(Vector6d::Zero(), twist_pub_, twist_frame_id_);
    RCLCPP_WARN(this->get_logger(), "[PBVS] ROS shutting down => abort");
    result->success = false;
    result->message = "ROS shutdown";
    gh->abort(result);
  }

}; // end class PBVSActionServer


// Main
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PBVSActionServer>());
  rclcpp::shutdown();
  return 0;
}
