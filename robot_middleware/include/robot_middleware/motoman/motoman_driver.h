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

#include "robot_middleware/motoman/simple_message/velocity_command_type.h"
#include "robot_middleware/robot_driver.h"

#include "motion_control_msgs/VelocityCommand.h"
#include "moveit/robot_model/robot_model.h"
#include "ros/ros.h"

#include <string>

namespace motoman
{

class MotomanDriver : public robot_middleware::RobotDriver
{
public:
  explicit MotomanDriver(const std::string& robot_ip, const moveit::core::RobotModelConstPtr& robot_model);

  ~MotomanDriver();

  bool sendVelocityCommand_internal(const motion_control_msgs::VelocityCommand& vel_cmd) override;

  bool sendVelocityConfig_internal(uint8_t type) override;

private:
  motoman::simple_message::VelocityCommandType toMotomanVelocityCommandType(uint8_t type);
};

}  // namespace motoman
