// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <fstream>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

#include <franka_gripper/franka_gripper.h>
#include <franka_gripper/MoveAction.h>
#include <actionlib/client/simple_action_client.h>

namespace ws_embodied_ai_controllers {

class RealTimePoseController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  RealTimePoseController()
      : log_file_("error_log.txt", std::ios::out | std::ios::app),
        grasp_client("/franka_gripper/grasp", true),
        move_client("/franka_gripper/move", true),
        homing_client("/franka_gripper/homing", true) {}

  ~RealTimePoseController() { log_file_.close(); }


  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void gripperCallback(const std_msgs::Float32::ConstPtr& msg);

 private:
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Duration elapsed_time_;
  std::array<double, 16> current_pose_{};
  std::array<double, 16> new_target_pose_{};
  std::array<double, 3> current_velocity_;
  std::array<double, 3> previous_error_{0, 0, 0};
  std::array<double, 3> target_position{0, 0, 0};

  // Arrays to store PID terms
  std::array<double, 3> P_terms;
  std::array<double, 3> D_terms;

  std::array<double, 3> dx;
  std::array<double, 3> ddx{0, 0, 0};
  std::array<double, 3> dddx{0, 0, 0};

  std::array<double, 3> dx_last;
  std::array<double, 3> ddx_last;

  Eigen::Quaterniond quater_target;
  Eigen::Matrix3d rotation_matrix;
  Eigen::Matrix3d current_rotation_matrix;
  Eigen::Vector3d previous_angular_error_;
  Eigen::Vector3d angular_velocity_error;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d previous_angular_velocity;
  Eigen::Vector3d previous_angular_acceleration;

  std::mutex mutex_;
  ros::Time last_log_time_0;
  ros::Time last_log_time_1;
  ros::Time last_log_time_2;
  std::ofstream log_file_;  // 文件流对象用于记录误差

  // Define maximum jerk 4500.0/10; 9.0/2; 3.0;
  // const double dddx_max = 4500.0/10;
  // const double ddx_max = 9.0/2;
  // const double dx_max = 3.0;

  const double dddx_max = 4500.0/5;
  const double ddx_max = 9.0/2;
  const double dx_max = 3.0;

  // 定义最大值
  const double angular_jerk_max = 5000.0/5;        // 最大角跃度
  const double angular_acceleration_max = 10.0/2;  // 最大角加速度
  const double angular_velocity_max = 2.5;       // 最大角速度

  bool first_run_0 = true;
  bool first_run_1 = true;
  
  // sigma sub info
  ros::Subscriber pose_sub_ ;
  ros::Subscriber gripper_sub_;
  geometry_msgs::PoseStamped sigma_sub_info;  // 用于存储当前的位姿信息
  float gripper_angle = 0.0;  // 用于存储gripper_angle的信息

  std::array<double, 3> base_robot_position{0, 0, 0};
  std::array<double, 3> base_sigma_position{0, 0, 0};  
  Eigen::Quaterniond base_robot_quater{0, 0, 0, 0};
  Eigen::Quaterniond base_sigma_quater{0, 0, 0, 0};

  actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client;
  actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client;
      
  actionlib::SimpleActionClient<franka_gripper::HomingAction> homing_client;

};

}  // namespace ws_embodied_ai_controllers
