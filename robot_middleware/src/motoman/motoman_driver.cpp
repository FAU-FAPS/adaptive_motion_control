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

#include "robot_middleware/motoman/motoman_driver.h"

#include "robot_middleware/motoman/simple_message/velocity_command_message.h"
#include "robot_middleware/motoman/simple_message/velocity_config_message.h"

#include <algorithm>

using namespace industrial::shared_types;
using namespace motoman::simple_message;
using namespace moveit::core;

namespace motoman
{

MotomanDriver::MotomanDriver(const std::string& robot_ip, const RobotModelConstPtr& robot_model)
  : RobotDriver(robot_ip, robot_model)
{
  // Motoman specific default port numbers
  default_motion_port_ = 50240;
  default_state_port_ = 50241;
}

MotomanDriver::~MotomanDriver()
{
}

bool MotomanDriver::sendVelocityCommand_internal(const motion_control_msgs::VelocityCommand& vel_cmd)
{
  // convert command type
  VelocityCommandType vel_cmd_type = toMotomanVelocityCommandType(vel_cmd.type);
  if (vel_cmd_type == VelocityCommandType::UNDEFINED)
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to convert velocity command type (undefined)");
    return false;
  }

  VelocityCommandMessage vel_cmd_msg;
  vel_cmd_msg.data_.type_ = static_cast<shared_int>(vel_cmd_type);

  int max_num_joints = std::min(VelocityCommand::MAX_NUM_JOINTS, vel_cmd.cmd.size());
  for (int i = 0; i < max_num_joints; ++i)
    vel_cmd_msg.data_.vector_[i] = (shared_real)vel_cmd.cmd[i];

  return sendRequest(vel_cmd_msg);
}

bool MotomanDriver::sendVelocityConfig_internal(uint8_t type)
{
  // convert command type
  VelocityCommandType vel_cmd_type = toMotomanVelocityCommandType(type);
  if (vel_cmd_type == VelocityCommandType::UNDEFINED)
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to convert velocity command type (undefined)");
    return false;
  }

  VelocityConfigMessage vel_cfg_msg;
  VelocityConfig& cfg = vel_cfg_msg.data_;
  cfg.cmd_type_ = static_cast<shared_int>(vel_cmd_type);
  cfg.max_linear_velocity_ = vel_ctrl_settings_.cartesian_limits.max_linear_velocity;

  // set maximum angular velocity depending on the command type and parameters (default: cartesian)
  if (vel_cmd_type == VelocityCommandType::ANGLE)
    cfg.max_angular_velocity_ =
        vel_ctrl_settings_.joint_limits.has_velocity_limit ? vel_ctrl_settings_.joint_limits.max_velocity : 99999.0;
  else
    cfg.max_angular_velocity_ = vel_ctrl_settings_.cartesian_limits.max_angular_velocity;

  // do not limit acceleration
  cfg.max_linear_acceleration_ = 99999.0;
  cfg.max_angular_acceleration_ = 99999.0;

  ROS_DEBUG_NAMED(LOGNAME,
                  "Sending velocity config message\n"
                  "  cmd_type:                  %d\n"
                  "  user_coord_num:            %d\n"
                  "  tool_file_num:             %d\n"
                  "  filter_size:               %d\n"
                  "  max_linear_velocity:       %f\n"
                  "  max_angular_velocity:      %f\n"
                  "  max_linear_acceleration:   %f\n"
                  "  max_angular_acceleration:  %f\n",
                  cfg.cmd_type_, cfg.user_coord_num_, cfg.tool_file_num_, cfg.filter_size_, cfg.max_linear_velocity_,
                  cfg.max_angular_velocity_, cfg.max_linear_acceleration_, cfg.max_angular_acceleration_);

  return sendRequest(vel_cfg_msg);
}

VelocityCommandType MotomanDriver::toMotomanVelocityCommandType(uint8_t type)
{
  auto rtn = VelocityCommandType::UNDEFINED;

  switch (type)
  {
    case motion_control_msgs::VelocityCommand::JOINT:
      rtn = VelocityCommandType::ANGLE;
      break;

    case motion_control_msgs::VelocityCommand::BASE_FRAME:
      rtn = VelocityCommandType::BASE_FRAME;
      break;

    case motion_control_msgs::VelocityCommand::TOOL_FRAME:
      rtn = VelocityCommandType::TOOL_FRAME;
      break;

    default:
      break;
  }

  return rtn;
}
}  // namespace motoman
