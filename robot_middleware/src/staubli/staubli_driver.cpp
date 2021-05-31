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

#include "robot_middleware/staubli/staubli_driver.h"

#include "robot_middleware/staubli/simple_message/velocity_command_message.h"
#include "robot_middleware/staubli/simple_message/velocity_config_message.h"

#include <algorithm>

using namespace industrial::shared_types;
using namespace moveit::core;
using namespace staubli::simple_message;

namespace staubli
{

StaubliDriver::StaubliDriver(const std::string& robot_ip, const RobotModelConstPtr& robot_model)
  : RobotDriver(robot_ip, robot_model)
{
}

StaubliDriver::~StaubliDriver()
{
}

bool StaubliDriver::sendVelocityCommand_internal(const motion_control_msgs::VelocityCommand& vel_cmd)
{
  // convert command type
  VelocityCommandType vel_cmd_type = toStaubliVelocityCommandType(vel_cmd.type);
  if (vel_cmd_type == VelocityCommandType::INVALID)
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to convert velocity command type (invalid)");
    return false;
  }

  VelocityCommandMessage vel_cmd_msg;
  vel_cmd_msg.data_.type_ = static_cast<shared_int>(vel_cmd_type);

  int max_num_joints = std::min(VelocityCommand::MAX_NUM_JOINTS, vel_cmd.cmd.size());
  for (int i = 0; i < max_num_joints; ++i)
    vel_cmd_msg.data_.vector_[i] = (shared_real)vel_cmd.cmd[i];

  return sendRequest(vel_cmd_msg);
}

bool StaubliDriver::sendVelocityConfig_internal(uint8_t type)
{
  VelocityConfigMessage vel_cfg_msg;
  VelocityConfig& cfg = vel_cfg_msg.data_;

  // convert command type
  VelocityCommandType vel_cmd_type = toStaubliVelocityCommandType(type);
  if (vel_cmd_type == VelocityCommandType::INVALID)
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to convert velocity command type (invalid)");
    return false;
  }

  cfg.cmd_type_ = static_cast<shared_int>(vel_cmd_type);
  cfg.accel_ = 1.0;  // no acceleration limit (100% of nominal acceleration)

  // determine the maximum nominal velocity
  double max_nominal_velocity = 0.0;
  for (const VariableBounds& b : joint_limits_)
    max_nominal_velocity = std::max(max_nominal_velocity, b.max_velocity_);

  // determine the actual maximum velocity to use
  double max_actual_velocity = vel_ctrl_settings_.joint_limits.has_velocity_limit ?
                                   std::min(vel_ctrl_settings_.joint_limits.max_velocity, max_nominal_velocity) :
                                   max_nominal_velocity;

  cfg.vel_ = max_actual_velocity / max_nominal_velocity;  // determine % of nominal velocity for actual maximum velocity
  cfg.tvel_ = vel_ctrl_settings_.cartesian_limits.max_linear_velocity;
  cfg.rvel_ = vel_ctrl_settings_.cartesian_limits.max_angular_velocity;

  ROS_DEBUG_NAMED(LOGNAME,
                  "Sending velocity config message\n"
                  "  cmd_type:  %d\n"
                  "  frame_ref: [%f, %f, %f, %f, %f, %f]\n"
                  "  tool_ref:  [%f, %f, %f, %f, %f, %f]\n"
                  "  accel:     %f\n"
                  "  vel:       %f\n"
                  "  tvel:      %f\n"
                  "  rvel:      %f\n",
                  cfg.cmd_type_, cfg.frame_ref_[0], cfg.frame_ref_[1], cfg.frame_ref_[2], cfg.frame_ref_[3],
                  cfg.frame_ref_[4], cfg.frame_ref_[5], cfg.tool_ref_[0], cfg.tool_ref_[1], cfg.tool_ref_[2],
                  cfg.tool_ref_[3], cfg.tool_ref_[4], cfg.tool_ref_[5], cfg.accel_, cfg.vel_, cfg.tvel_, cfg.rvel_);

  return sendRequest(vel_cfg_msg);
}

VelocityCommandType StaubliDriver::toStaubliVelocityCommandType(uint8_t type)
{
  auto rtn = VelocityCommandType::INVALID;

  switch (type)
  {
    case motion_control_msgs::VelocityCommand::JOINT:
      rtn = VelocityCommandType::JOINT;
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

}  // namespace staubli
