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

#include "Eigen/Dense"
#include "angles/angles.h"
#include "control_toolbox/pid.h"
#include "geometry_msgs/PoseStamped.h"
#include "motion_control_msgs/VelocityCommand.h"
#include "ros/ros.h"
#include "tf2_eigen/tf2_eigen.h"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robot_middleware
{

class PoseTrackingController
{
public:
  PoseTrackingController();

  ~PoseTrackingController();

  void init(const RobotDriverPtr& driver);

  void start();

  void stop();

  void reset();

private:
  bool initPid(const std::string& prefix, control_toolbox::Pid& pid);

  void poseGoalCb(const geometry_msgs::PoseStampedConstPtr& goal);

  bool hasReachedGoal(const Eigen::Vector3d& linear_error, const Eigen::AngleAxisd& angular_error,
                      ros::Time goal_timestamp);

  void computeCommand(const Eigen::Vector3d& linear_error, const Eigen::AngleAxisd& angular_error,
                      motion_control_msgs::VelocityCommand& vel_cmd);

  void loop();

  bool controlLoop();

  const std::string LOGNAME = "pose_tracking";
  const std::string DEFAULT_TCP_FRAME = "tool0";
  const double DEFAULT_GOAL_TOLERANCE_LINEAR = 0.0001;                      // 0.1 mm
  const double DEFAULT_GOAL_TOLERANCE_ANGULAR = angles::from_degrees(0.1);  // 0.1 deg
  const double DEFAULT_GOAL_SETTLE_TIME = 0.1;                              // 100 ms
  const double DEFAULT_RECEIVE_TIMEOUT = 1.0;                               // 1.0 second

  RobotDriverPtr driver_;

  ros::NodeHandle nh_;
  ros::Duration controller_period_;
  ros::Time goal_timestamp_;
  ros::Time last_goal_timestamp_;
  ros::Subscriber sub_;
  geometry_msgs::PoseStamped goal_;

  control_toolbox::Pid pid_linear_x_;
  control_toolbox::Pid pid_linear_y_;
  control_toolbox::Pid pid_linear_z_;
  control_toolbox::Pid pid_angular_;

  std::string tcp_frame_;
  double goal_tolerance_linear_;
  double goal_tolerance_angular_;
  double goal_settle_time_;
  double receive_timeout_;
  bool keep_running_;
  bool in_error_;

  std::thread control_task_;
  std::condition_variable goal_received_;
  std::mutex goal_mtx_;
};

}  // namespace robot_middleware
