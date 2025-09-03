#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_14.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <string>

using RM = RMUtils;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("load_screw_list");

  std::cout << "\n----- Load ScrewList from YAML file -----\n";

  node->declare_parameter<std::string>("base_link", "");
  node->declare_parameter<std::string>("ee_link", "");
  node->declare_parameter<std::string>("screw_representation", "body");
  node->declare_parameter<std::vector<std::string>>("joint_names", {});
  node->declare_parameter<int>("num_joints", 0);
  node->declare_parameter<std::vector<double>>("M_position", {});        // [x,y,z]
  node->declare_parameter<std::vector<double>>("M_quaternion_wxyz", {}); // [w,x,y,z]

  std::string base_link, ee_link, rep;
  std::vector<std::string> joint_names;
  int num_joints_param = 0;
  std::vector<double> M_pos, M_qwxyz;

  node->get_parameter("base_link", base_link);
  node->get_parameter("ee_link", ee_link);
  node->get_parameter("screw_representation", rep);
  node->get_parameter("joint_names", joint_names);
  node->get_parameter("num_joints", num_joints_param);
  node->get_parameter("M_position", M_pos);
  node->get_parameter("M_quaternion_wxyz", M_qwxyz);

  std::cout << "base_link: " << base_link << "\n";
  std::cout << "ee_link  : " << ee_link   << "\n";
  std::cout << "rep(fr)  : " << rep       << " (space/body)\n";

  if (M_pos.size() != 3 || M_qwxyz.size() != 4) {
    RCLCPP_FATAL(node->get_logger(), "M_position must be length 3 and M_quaternion_wxyz must be length 4 (w,x,y,z).");
    rclcpp::shutdown();
    return 1;
  }
  PosQuat M;
  M.pos  = Vector3d(M_pos[0], M_pos[1], M_pos[2]);
  M.quat = Quaterniond(M_qwxyz[0], M_qwxyz[1], M_qwxyz[2], M_qwxyz[3]); // (w,x,y,z)

  if (joint_names.empty()) {
    RCLCPP_FATAL(node->get_logger(), "joint_names is empty.");
    rclcpp::shutdown();
    return 1;
  }
  const int n = static_cast<int>(joint_names.size());
  if (num_joints_param != 0 && num_joints_param != n) {
    RCLCPP_WARN(node->get_logger(),
      "num_joints (%d) differs from joint_names size (%d). Using %d.",
      num_joints_param, n, n);
  }

  // Compose the screw_list
  MatrixXd screw_list(6, n);
  for (int j = 0; j < n; ++j) {
    const std::string pname = "screw_list." + joint_names[j];
    node->declare_parameter<std::vector<double>>(pname, {});
    std::vector<double> arr;
    if (!node->get_parameter(pname, arr) || arr.size() != 6) {
      RCLCPP_FATAL(node->get_logger(),
                   "Parameter %s must be length-6 [v(0:2), w(3:5)].", pname.c_str());
      rclcpp::shutdown();
      return 1;
    }
    screw_list.col(j) << arr[0], arr[1], arr[2], arr[3], arr[4], arr[5];

    std::cout << pname << " -> ["
              << arr[0] << ", " << arr[1] << ", " << arr[2] << ", "
              << arr[3] << ", " << arr[4] << ", " << arr[5] << "]\n";
  }

  ScrewList screws(screw_list, M);

  std::cout << "\n-- Joints (" << n << "): ";
  for (int j = 0; j < n; ++j)
    std::cout << joint_names[j] << (j + 1 < n ? ", " : "\n");
  screws.PrintList();

  VectorXd theta = VectorXd::Zero(n);
  PosQuat pq_fk = RM::FKPoE(screws, theta);
  std::cout << "\nFK at zero config (should equal M):\n"
            << pq_fk.pos.x() << ", " << pq_fk.pos.y() << ", " << pq_fk.pos.z() << ", "
            << pq_fk.quat.w() << ", " << pq_fk.quat.x() << ", " << pq_fk.quat.y() << ", " << pq_fk.quat.z() << "\n";

  rclcpp::shutdown();
  return 0;
}
