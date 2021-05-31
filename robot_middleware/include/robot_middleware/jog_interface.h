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

#include "robot_middleware/robot_driver.h"
#include "robot_middleware/velocity_control_settings.h"

#include "control_msgs/JointJog.h"
#include "geometry_msgs/TwistStamped.h"
#include "motion_control_msgs/VelocityCommand.h"
#include "ros/ros.h"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robot_middleware
{

enum class JogInterfaceState
{
  IDLE,
  CARTESIAN_JOGGING,
  JOINT_JOGGING
};

class JogInterface
{
public:
  explicit JogInterface();

  ~JogInterface();

  void init(const RobotDriverPtr& driver);

  void start();

  void stop();

  void reset();

private:
  void twistCb(const geometry_msgs::TwistStampedConstPtr& cmd);

  void jointJogCb(const control_msgs::JointJogConstPtr& cmd);

  bool inError(ros::Time latest_cmd_timestamp);

  bool setCommand(const geometry_msgs::TwistStampedConstPtr& cmd);

  bool setCommand(const control_msgs::JointJogConstPtr& cmd);

  bool setState(JogInterfaceState new_state);

  void loop();

  bool controlLoop();

  const std::string LOGNAME = "jog_interface";
  const double DEFAULT_RECEIVE_TIMEOUT = 0.1;  // seconds

  RobotDriverPtr driver_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_twist_;
  ros::Subscriber sub_joint_jog_;
  geometry_msgs::Twist twist_cmd_;
  control_msgs::JointJog joint_jog_cmd_;
  motion_control_msgs::VelocityCommand vel_cmd_;
  ros::Time cmd_timestamp_;

  JogInterfaceState state_;
  double control_loop_hz_;
  double receive_timeout_;
  bool keep_running_;
  bool in_error_;

  std::thread control_task_;
  std::condition_variable command_received_cv_;
  std::mutex state_mtx_;
};

}  // namespace robot_middleware
