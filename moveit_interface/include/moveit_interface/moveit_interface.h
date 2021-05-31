/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Institute for Factory Automation and Production Systems (FAPS)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <ros/ros.h>
#include <motion_control_msgs/GetMotionPlan.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/Trigger.h>

class MoveItInterface
{
public:
  explicit MoveItInterface(const ros::NodeHandle& nh);

  ~MoveItInterface();

  void run();

private:
  bool getMotionPlan(motion_control_msgs::GetMotionPlan::Request& req,
                     motion_control_msgs::GetMotionPlan::Response& res);

  bool executeMotionPlan(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  const int DEFAULT_PLANNING_TIME = 5;  // in seconds
  const double DEFAULT_VELOCITY_SCALING = 0.1;
  const double DEFAULT_ACCELERATION_SCALING = 0.1;
  ros::NodeHandle nh_;
  ros::ServiceServer plan_service_;
  ros::ServiceServer execute_service_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::PlanPtr plan_;
};
