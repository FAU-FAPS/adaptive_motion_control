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

#include "robot_middleware/velocity_control_settings.h"

#include "ros/ros.h"

namespace robot_middleware
{

VelocityControlSettings::VelocityControlSettings(const std::string& log_name) : LOGNAME(log_name)
{
}

VelocityControlSettings::~VelocityControlSettings()
{
}

bool VelocityControlSettings::initParam(const ros::NodeHandle& parent, const std::string& ns)
{
  ros::NodeHandle nh(parent, ns);
  std::size_t error = 0;

  error += !rosparam_shortcuts::get(LOGNAME, nh, "base_frame", this->base_frame);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "tool_frame", this->tool_frame);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "control_loop_frequency", this->control_loop_frequency);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "cartesian_limits/max_linear_velocity",
                                    this->cartesian_limits.max_linear_velocity);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "cartesian_limits/max_angular_velocity",
                                    this->cartesian_limits.max_angular_velocity);
  error +=
      !rosparam_shortcuts::get(LOGNAME, nh, "joint_limits/has_velocity_limit", this->joint_limits.has_velocity_limit);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "joint_limits/max_velocity", this->joint_limits.max_velocity);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "joint_limits/stop_tolerance", this->joint_limits.stop_tolerance);

  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  return true;
}

}  // namespace robot_middleware
