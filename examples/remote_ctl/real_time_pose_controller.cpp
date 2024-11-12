#include <ws_embodied_ai_controllers/real_time_pose_controller.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>  // 包含头文件以使用 std::min
#include <cmath>  // 包含头文件以使用 std::abs
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_gripper/franka_gripper.h>
#include <franka/gripper_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/HomingAction.h>

#include <actionlib/client/simple_action_client.h>
#include <franka_hw/franka_state_interface.h>

namespace ws_embodied_ai_controllers {

bool RealTimePoseController::init(hardware_interface::RobotHW* robot_hardware,
                                  ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "RealTimePoseController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("RealTimePoseController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("RealTimePoseController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("RealTimePoseController: Could not get state interface from hardware");
    return false;
  }

  ros::NodeHandle nh;

  pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/sigma7/sigma0/pose", 10, boost::bind(&RealTimePoseController::poseCallback, this, _1));
  gripper_sub_ = nh.subscribe<std_msgs::Float32>("/sigma7/sigma0/gripper_angle", 10, boost::bind(&RealTimePoseController::gripperCallback, this, _1));

    // 夹爪一定要先进行home操作，否则会出现夹爪夹不起来物体
  franka_gripper::HomingGoal homingGoal;
  homing_client.sendGoal(homingGoal);

  return true;
}

void RealTimePoseController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);

  // Initialize PID errors
  previous_error_.fill(0.0);

  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  ;
  // // 打印 current_pose_ 的所有值
  // ROS_INFO_STREAM("Current Pose:");
  // for (int i = 0; i < 16; ++i) {
  //   ROS_INFO_STREAM("current_pose_[" << i << "]: " << current_pose_[i]);
  // }

  // 提取 current_pose_ 的旋转矩阵并转换为四元数

  current_rotation_matrix << current_pose_[0], current_pose_[4], current_pose_[8], current_pose_[1],
      current_pose_[5], current_pose_[9], current_pose_[2], current_pose_[6], current_pose_[10];

  // // 打印旋转矩阵
  // ROS_INFO_STREAM("Current Rotation Matrix:");
  // ROS_INFO_STREAM(current_rotation_matrix);

  Eigen::Quaterniond current_quaternion(current_rotation_matrix);

  // // 将旋转矩阵转换成欧拉角（假设使用XYZ顺序）
  // Eigen::Vector3d euler_angles = current_rotation_matrix.eulerAngles(0, 1, 2);

  // // 打印欧拉角
  // ROS_INFO_STREAM("Current Euler Angles (XYZ order): [" << euler_angles[0] << ", "
  //                                                       << euler_angles[1] << ", "
  //                                                       << euler_angles[2] << "]");

  // // 打印当前四元数
  // ROS_INFO_STREAM("Current Quaternion:wxyz [" << current_quaternion.w() << ", "
  //                                          << current_quaternion.x() << ", "
  //                                          << current_quaternion.y() << ", "
  //                                          << current_quaternion.z() << "]");

  // ROS_INFO_STREAM(current_quaternion);

  new_target_pose_ = current_pose_;

  // target xyz
  // target_position = {0.3567, 0.3290, 0.4579};
  target_position = {new_target_pose_[12], new_target_pose_[13], new_target_pose_[14]};

  // Define the target quaternion
  quater_target = Eigen::Quaterniond(0.000464259, -0.999996, -0.000330215, -0.00191625);
  // quater_target = Eigen::Quaterniond(0.10458484 , 0.9367045 , 0.29851883 , -0.15011063);  // oriention 

  // quater_target = current_quaternion;

  // 将四元数转换为旋转矩阵
  rotation_matrix = quater_target.toRotationMatrix();

  // 调整赋值 new_target_pose_ 的旋转矩阵顺序
  new_target_pose_[0] = rotation_matrix(0, 0);
  new_target_pose_[4] = rotation_matrix(0, 1);
  new_target_pose_[8] = rotation_matrix(0, 2);
  new_target_pose_[1] = rotation_matrix(1, 0);
  new_target_pose_[5] = rotation_matrix(1, 1);
  new_target_pose_[9] = rotation_matrix(1, 2);
  new_target_pose_[2] = rotation_matrix(2, 0);
  new_target_pose_[6] = rotation_matrix(2, 1);
  new_target_pose_[10] = rotation_matrix(2, 2);

  // 将目标位置数组的值赋给new_target_pose_的平移部分
  new_target_pose_[12] = target_position[0];
  new_target_pose_[13] = target_position[1];
  new_target_pose_[14] = target_position[2];

  base_robot_position = {new_target_pose_[12], new_target_pose_[13], new_target_pose_[14]};
  base_robot_quater = current_quaternion;

  ros::Rate rate(10);  // 10 Hz

  // 等待有效的 sigma_sub_info 数据
  while (ros::ok()) {
    // 通过 ros::spinOnce() 更新回调中的 sigma_sub_info 数据
    ros::spinOnce();

    // 检查 sigma_sub_info 中的旋转数据是否有效
    if (sigma_sub_info.pose.orientation.w != 0 || sigma_sub_info.pose.orientation.x != 0 ||
        sigma_sub_info.pose.orientation.y != 0 || sigma_sub_info.pose.orientation.z != 0) {
      
      // 如果旋转数据有效，提取对应的位置信息和旋转信息作为 base
      base_sigma_position = {sigma_sub_info.pose.position.x, sigma_sub_info.pose.position.y, sigma_sub_info.pose.position.z};
      base_sigma_quater = {sigma_sub_info.pose.orientation.x, sigma_sub_info.pose.orientation.y, sigma_sub_info.pose.orientation.z, sigma_sub_info.pose.orientation.w};

      // 退出循环
      break;
    }

    // 打印等待信息
    ROS_WARN("Waiting for valid sigma_sub_info.pose data...");
    rate.sleep();
  }
}

void RealTimePoseController::update(const ros::Time& time, const ros::Duration& period) {
  elapsed_time_ += period;
  std::lock_guard<std::mutex> lock(mutex_);

  double dt = 0.001;
  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  std::array<double, 3> real_sigma_position = {sigma_sub_info.pose.position.x, sigma_sub_info.pose.position.y, sigma_sub_info.pose.position.z};
  Eigen::Quaterniond real_sigma_quater = {sigma_sub_info.pose.orientation.x, sigma_sub_info.pose.orientation.y, sigma_sub_info.pose.orientation.z, sigma_sub_info.pose.orientation.w};
  new_target_pose_[12] = base_robot_position[0] + 4.5 * (real_sigma_position[0] - base_sigma_position[0]);
  new_target_pose_[13] = base_robot_position[1] + 4.5 * (real_sigma_position[1] - base_sigma_position[1]);
  new_target_pose_[14] = base_robot_position[2] + 4.5 * (real_sigma_position[2] - base_sigma_position[2]);
  quater_target = (real_sigma_quater * base_sigma_quater.inverse()) * base_robot_quater;
  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  // PID controller parameters (to be tuned)  1  Kp / 15
  double Kp = 15/2/2;
  double Kd = Kp / 10;

  // Compute error
  std::array<double, 3> error;
  error[0] = new_target_pose_[12] - current_pose_[12];
  error[1] = new_target_pose_[13] - current_pose_[13];
  error[2] = new_target_pose_[14] - current_pose_[14];
  //-------------------------------------------------------------- time test 
  // // Check if error is within tolerance
  // double error_threshold = 0.001;  // 1 mm threshold
  // bool within_tolerance = (std::abs(error[0]) < error_threshold);

  // // Record time when error is within tolerance (only once)
  // static ros::Time start_time = ros::Time::now();
  // static bool time_recorded_ = false;

  // if (within_tolerance && !time_recorded_) {
  //   ros::Time end_time = ros::Time::now();
  //   ros::Duration time_taken = end_time - start_time;
  //   ROS_INFO_STREAM("Time taken to reach within 1 mm: " << time_taken.toSec() << " seconds");
  //   time_recorded_ = true;  // Set the flag to true after recording the time
  // }
  //-------------------------------------------------------------- 
  // Compute PID control for velocity
  for (int i = 0; i < 3; ++i) {
    // Compute PD terms
    P_terms[i] = Kp * error[i];
    D_terms[i] = Kd * (error[i] - previous_error_[i]) / dt;

    dx[i] = P_terms[i] + D_terms[i];
  }
  //-------------------------------------------------------------- limit dx ddx dddx
  Eigen::Vector3d dx_vec(dx[0], dx[1], dx[2]);
  Eigen::Vector3d dx_last_vec(dx_last[0], dx_last[1], dx_last[2]);
  Eigen::Vector3d ddx_vec = (dx_vec - dx_last_vec) / dt;
  Eigen::Vector3d ddx_last_vec(ddx_last[0], ddx_last[1], ddx_last[2]);
  Eigen::Vector3d dddx_vec = (ddx_vec - ddx_last_vec) / dt;

  // 对模长限制进行检查并缩放
  if (dddx_vec.norm() > dddx_max) {
    dddx_vec = dddx_vec.normalized() * dddx_max;
  }

  ddx_vec = ddx_last_vec + dddx_vec * dt;

  if (ddx_vec.norm() > ddx_max) {
    ddx_vec = ddx_vec.normalized() * ddx_max;
  }

  dx_vec = dx_last_vec + ddx_vec * dt;

  if (dx_vec.norm() > dx_max) {
    dx_vec = dx_vec.normalized() * dx_max;
  }

  // 更新 dx, ddx, dddx
  dx[0] = dx_vec[0];
  dx[1] = dx_vec[1];
  dx[2] = dx_vec[2];

  ddx_last[0] = ddx_vec[0];
  ddx_last[1] = ddx_vec[1];
  ddx_last[2] = ddx_vec[2];

  dx_last[0] = dx_vec[0];
  dx_last[1] = dx_vec[1];
  dx_last[2] = dx_vec[2];
  //-------------------------------------------------------------- 
  for (int i = 0; i < 3; ++i) {
    // Check for NaN or infinite values
    if (std::isnan(dx[i]) || std::isinf(dx[i])) {
      ROS_ERROR("RealTimePoseController: Computed velocity is NaN or Inf, resetting velocity");
      dx[i] = 0.0;
    }
    previous_error_[i] = error[i];
  }
  // -----------------------------------------------------------------------------------------------------------------------------------
  // Update current position based on new velocity
  for (int i = 0; i < 3; ++i) {
    current_pose_[12 + i] += dx[i] * dt;
  }
  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  double Kp_quater = 10/1.5;
  double Kd_quater = Kp_quater / 15;

  // Compute quaternion error
  current_rotation_matrix << current_pose_[0], current_pose_[4], current_pose_[8], current_pose_[1],current_pose_[5], current_pose_[9], current_pose_[2], current_pose_[6], current_pose_[10];

  Eigen::Quaterniond current_quaternion(current_rotation_matrix);
  // --------------------------------------------------------------找最短四元数
  double dot_product = quater_target.dot(current_quaternion);
  if (dot_product < 0.0) {
    quater_target.coeffs() = -quater_target.coeffs();
  }
  // --------------------------------------------------------------
  Eigen::Quaterniond quaternion_error = quater_target * current_quaternion.inverse();

  // Convert quaternion error to angle-axis representation
  Eigen::AngleAxisd angle_axis_error(quaternion_error);

  // PD control for quaternion
  angular_velocity_error = angle_axis_error.angle() * angle_axis_error.axis();

  // Initialize previous_error on first run
  if (first_run_1) {
    previous_angular_error_ = angular_velocity_error;
    first_run_1 = false;
  }

  angular_velocity = Kp_quater * angular_velocity_error + Kd_quater * (angular_velocity_error - previous_angular_error_) / dt;
  //-------------------------------------------------------------- limit angular_velocity
  // 计算 angular_acceleration 和 angular_jerk
  Eigen::Vector3d angular_acceleration = (angular_velocity - previous_angular_velocity) / dt;
  Eigen::Vector3d angular_jerk = (angular_acceleration - previous_angular_acceleration) / dt;

  // 限制 angular_jerk 的值
  if (angular_jerk.norm() > angular_jerk_max) {
    angular_jerk = angular_jerk.normalized() * angular_jerk_max;
  }

  // 计算新的 angular_acceleration
  angular_acceleration = previous_angular_acceleration + angular_jerk * dt;

  // 限制 angular_acceleration 的值
  if (angular_acceleration.norm() > angular_acceleration_max) {
    angular_acceleration = angular_acceleration.normalized() * angular_acceleration_max;
  }

  // 计算新的 angular_velocity
  angular_velocity = previous_angular_velocity + angular_acceleration * dt;

  // 限制 angular_velocity 的值
  if (angular_velocity.norm() > angular_velocity_max) {
    angular_velocity = angular_velocity.normalized() * angular_velocity_max;
  }

  // 更新上一次的值
  previous_angular_velocity = angular_velocity;
  previous_angular_acceleration = angular_acceleration;
  //--------------------------------------------------------------
  previous_angular_error_ = angular_velocity_error;

  // Update current orientation based on angular velocity
  Eigen::Vector3d angular_velocity_dt = angular_velocity * dt;
  Eigen::Quaterniond delta_quaternion(Eigen::AngleAxisd(angular_velocity_dt.norm(), angular_velocity_dt.normalized()));
  Eigen::Quaterniond new_quaternion = delta_quaternion * current_quaternion;
  new_quaternion.normalize();
  // // 打印当前四元数
  // if ((time - last_log_time_1).toSec() >= 0.5) {
  //   ROS_INFO_STREAM("delta_quaternion: [" << delta_quaternion.w() << ", "
  //                                           << delta_quaternion.x() << ", "
  //                                           << delta_quaternion.y() << ", "
  //                                           << delta_quaternion.z() << "]");
  //   last_log_time_1 = time; // Update the last log time
  // }

  // Update current_pose_ with the new orientation
  Eigen::Matrix3d new_rotation_matrix = new_quaternion.toRotationMatrix();
  current_pose_[0] = new_rotation_matrix(0, 0);
  current_pose_[4] = new_rotation_matrix(0, 1);
  current_pose_[8] = new_rotation_matrix(0, 2);
  current_pose_[1] = new_rotation_matrix(1, 0);
  current_pose_[5] = new_rotation_matrix(1, 1);
  current_pose_[9] = new_rotation_matrix(1, 2);
  current_pose_[2] = new_rotation_matrix(2, 0);
  current_pose_[6] = new_rotation_matrix(2, 1);
  current_pose_[10] = new_rotation_matrix(2, 2);
  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  // 实例化夹爪配置
  franka_gripper::MoveGoal goal;
  goal.width = abs(gripper_angle) * 0.16;       
  // 设置速度、力、容差
  goal.speed = 0.1 * 10;
  // goal.force = 60;
  // goal.epsilon.inner = 0.005;
  // goal.epsilon.outer = 0.005;
  
  // ROS_INFO_STREAM("goal: " << goal);  
  
  // 发送抓取命令
  move_client.sendGoal(goal);
  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  // ROS_INFO("Error: [%f, %f, %f]", error[0], error[1], error[2]);
  // // Print the required variables every 0.5 seconds
  if ((time - last_log_time_0).toSec() >= 0.1) {
    // ROS_INFO("Error: [%f, %f, %f]", error[0], error[1], error[2]);
    // ROS_INFO_STREAM("quater_target: [" << quater_target);
    // ROS_INFO_STREAM("angular_velocity_error: [" << angular_velocity_error);
    // ROS_INFO_STREAM("real_sigma_quater: [" << real_sigma_quater);
    // ROS_INFO_STREAM("base_sigma_quater: [" << base_sigma_quater);
    // ROS_INFO_STREAM("base_robot_quater: [" << base_robot_quater);
    // ROS_INFO_STREAM("base_robot_position: [" << base_robot_position[0] << base_robot_position[1] << base_robot_position[2]);
    last_log_time_0 = time;
  }
  // -----------------------------------------------------------------------------------------------------------------------------------
  // Set the new pose command
  cartesian_pose_handle_->setCommand(current_pose_);
}

void RealTimePoseController::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  sigma_sub_info = *msg;
  // ROS_INFO("Position: x=%f, y=%f, z=%f", sigma_sub_info.pose.position.x, sigma_sub_info.pose.position.y, sigma_sub_info.pose.position.z); 
  // ROS_INFO("Orientation: x=%f,y=%f, z=%f, w=%f", sigma_sub_info.pose.orientation.x, sigma_sub_info.pose.orientation.y, sigma_sub_info.pose.orientation.z, sigma_sub_info.pose.orientation.w);
}

void RealTimePoseController::gripperCallback(const std_msgs::Float32::ConstPtr& msg) {
  gripper_angle = msg->data;
  // ROS_INFO_STREAM("gripper_angle: " << gripper_angle);
}

}  // namespace ws_embodied_ai_controllers

PLUGINLIB_EXPORT_CLASS(ws_embodied_ai_controllers::RealTimePoseController,
                       controller_interface::ControllerBase)
