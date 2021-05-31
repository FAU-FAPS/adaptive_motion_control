/*
 * Copyright 2021 Institute for Factory Automation and Production Systems (FAPS)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "robot_middleware/connection_manager/simple_socket_manager.h"
#include "robot_middleware/eigen_typedefs.h"
#include "robot_middleware/velocity_control_settings.h"

#include "motion_control_msgs/VelocityCommand.h"
#include "moveit/robot_state/robot_state.h"
#include "ros/ros.h"
#include "simple_message/robot_status.h"
#include "simple_message/simple_message.h"
#include "simple_message/typed_message.h"

#include <array>
#include <condition_variable>
#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robot_middleware
{

enum class MotionControlMode
{
  IDLE,
  LOCKED,
  MOTION_COMMAND_RELAYING,
  JOINT_TRAJECTORY_STREAMING,
  JOGGING,
  POSE_TRACKING
};

class RobotDriver
{
public:
  static const std::size_t MAX_NUM_JOINTS = 10;
  static const std::map<MotionControlMode, const char*> MOTION_CTRL_MODE_NAMES;

  RobotDriver(const std::string& robot_ip, const moveit::core::RobotModelConstPtr& robot_model);

  virtual ~RobotDriver();

  bool init();

  bool init(int motion_port, int state_port);

  bool connect();

  bool isConnected();

  bool isMotionPossible();

  int getDefaultMotionPort()
  {
    return default_motion_port_;
  }

  int getDefaultStatePort()
  {
    return default_state_port_;
  }

  std::size_t getNumJoints()
  {
    return num_joints_;
  }

  moveit::core::RobotState getRobotState()
  {
    std::lock_guard lock(state_mtx_);
    moveit::core::RobotState tmp = robot_state_;
    return tmp;
  }

  industrial::robot_status::RobotStatus getRobotStatus()
  {
    std::lock_guard lock(status_mtx_);
    industrial::robot_status::RobotStatus tmp = robot_status_;
    return tmp;
  }

  const connection_manager::SimpleSocketManagerPtr& getMotionClientManager()
  {
    return motion_client_manager_;
  }

  const connection_manager::SimpleSocketManagerPtr& getStateClientManager()
  {
    return state_client_manager_;
  }

  const VelocityControlSettings& getVelocityControlSettings()
  {
    return vel_ctrl_settings_;
  }

  bool getTransform(const std::string& tcp_frame, const std::string& reference_frame, Eigen::Isometry3d& trsf);

  bool setMotionControlMode(MotionControlMode mode);

  void setJointPositions(const std::array<double, MAX_NUM_JOINTS>& position, ros::Time& timestamp);

  void setJointPositionsAndVelocities(const std::array<double, MAX_NUM_JOINTS>& position,
                                      const std::array<double, MAX_NUM_JOINTS>& velocity, ros::Time& timestamp);

  void setRobotStatus(industrial::robot_status::RobotStatus& status, ros::Time& timestamp);

  bool waitForStateUpdate(ros::Time& last_update_time, int timeout_ms);

  bool waitForFeedback(const std::string& tcp_frame, const std::string& reference_frame, ros::Time& last_feedback_time,
                       int timeout_ms, Eigen::Isometry3d& trsf);

  bool startJogging(uint8_t type);

  bool startPoseTracking(uint8_t type);

  bool stopMotionControl();

  virtual bool sendVelocityCommand(const motion_control_msgs::VelocityCommand& vel_cmd);

protected:
  void computeCartesianLimitsScale(const eigen_typedefs::Vector& cart_vel, double& linear_scale, double& angular_scale);

  void computeJointLimitsScale(const eigen_typedefs::Vector& joint_pos, const eigen_typedefs::Vector& joint_vel,
                               double& scale);

  bool enforceLimits(motion_control_msgs::VelocityCommand& vel_cmd);

  bool getTransform_internal(const std::string& tcp_frame, const std::string& reference_frame, Eigen::Isometry3d& trsf);

  bool sendRequest(industrial::typed_message::TypedMessage& typed_msg);

  bool sendRequest(industrial::simple_message::SimpleMessage& msg);

  virtual bool startVelocityControl(uint8_t type, MotionControlMode mode);

  virtual bool sendVelocityCommand_internal(const motion_control_msgs::VelocityCommand& vel_cmd) = 0;

  virtual bool sendVelocityConfig_internal(uint8_t type) = 0;

  bool waitForStateUpdate_internal(std::unique_lock<std::mutex>& lock, ros::Time& last_update_time, int timeout_ms);

  const std::string LOGNAME = "robot_driver";

  std::string robot_ip_;
  connection_manager::SimpleSocketManagerPtr motion_client_manager_;
  connection_manager::SimpleSocketManagerPtr state_client_manager_;
  int default_motion_port_;
  int default_state_port_;

  ros::NodeHandle nh_;
  ros::Time state_timestamp_;
  ros::Time status_timestamp_;
  ros::Publisher raw_vel_cmd_publisher_;
  ros::Publisher filt_vel_cmd_publisher_;
  bool publish_vel_cmd_;

  moveit::core::RobotState robot_state_;
  industrial::robot_status::RobotStatus robot_status_;
  VelocityControlSettings vel_ctrl_settings_;
  MotionControlMode motion_ctrl_mode_;

  std::size_t num_joints_;
  std::string planning_group_;
  const moveit::core::JointModelGroup* joint_model_group_;
  std::vector<std::string> joint_names_;
  std::vector<moveit::core::VariableBounds> joint_limits_;
  std::map<std::string, moveit::core::VariableBounds&> joint_limits_map_;

  std::condition_variable state_update_cv_;
  std::mutex motion_ctrl_mtx_;
  std::mutex state_mtx_;
  std::mutex status_mtx_;
  std::thread::id motion_ctrl_owner_;
};

using RobotDriverPtr = std::shared_ptr<RobotDriver>;

}  // namespace robot_middleware
