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

#include "moveit_interface/moveit_interface.h"

#include <exception>
#include <memory>
#include <string>

using namespace moveit::planning_interface;

MoveItInterface::MoveItInterface(const ros::NodeHandle& nh) : nh_(nh)
{
  ROS_INFO("Initializing the MoveIt interface");
  std::string planning_group;

  if (!ros::param::get("~planning_group", planning_group))
    throw std::runtime_error("Missing required parameter 'planning_group'");

  move_group_ = std::make_shared<MoveGroupInterface>(planning_group);
  plan_ = std::make_shared<MoveGroupInterface::Plan>();

  // print info
  ROS_INFO("Planning group: %s", move_group_->getName().c_str());
  ROS_INFO("Planning frame: %s", move_group_->getPlanningFrame().c_str());
  ROS_INFO("Pose reference frame: %s", move_group_->getPoseReferenceFrame().c_str());
  ROS_INFO("End effector link: %s", move_group_->getEndEffectorLink().c_str());

  // init service servers for planning and execution
  plan_service_ = nh_.advertiseService("plan", &MoveItInterface::getMotionPlan, this);
  execute_service_ = nh_.advertiseService("execute", &MoveItInterface::executeMotionPlan, this);
}

MoveItInterface::~MoveItInterface()
{
}

void MoveItInterface::run()
{
  // wait for /joint_states topic
  ROS_INFO("Waiting for a message from the topic /joint_states");
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  ROS_INFO("Ready to receive motion requests");
  ros::waitForShutdown();
}

bool MoveItInterface::getMotionPlan(motion_control_msgs::GetMotionPlan::Request& req,
                                    motion_control_msgs::GetMotionPlan::Response& res)
{
  ROS_INFO("Received 'plan' request.");

  // planner id
  if (req.planner_id.empty())
    move_group_->setPlannerId(move_group_->getDefaultPlannerId());
  else
    move_group_->setPlannerId(req.planner_id);

  // planning time
  if (req.allowed_planning_time <= 0)
    move_group_->setPlanningTime(DEFAULT_PLANNING_TIME);
  else
    move_group_->setPlanningTime(req.allowed_planning_time);

  // velocity scaling
  double velocity_scaling =
      (req.max_velocity_scaling_factor <= 0) ? DEFAULT_VELOCITY_SCALING : req.max_velocity_scaling_factor;
  move_group_->setMaxVelocityScalingFactor(velocity_scaling);

  // acceleration scaling
  double acceleration_scaling =
      (req.max_acceleration_scaling_factor <= 0) ? DEFAULT_ACCELERATION_SCALING : req.max_acceleration_scaling_factor;
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);

  // pose reference frame
  if (req.goal.header.frame_id.empty())
    move_group_->setPoseReferenceFrame(move_group_->getPlanningFrame());
  else
    move_group_->setPoseReferenceFrame(req.goal.header.frame_id);

  // pose goal
  move_group_->setPoseTarget(req.goal.pose);

  // clang-format off
  ROS_INFO_STREAM("Planning parameters:\n"
                  "  Planner ID:           " << move_group_->getPlannerId() << "\n"
                  "  Planning time:        " << move_group_->getPlanningTime() << "\n"
                  "  Velocity scaling:     " << velocity_scaling << "\n"
                  "  Acceleration scaling: " << acceleration_scaling << "\n"
                  "  Pose reference frame: " << move_group_->getPoseReferenceFrame() << "\n"
                  "  Pose goal:            " << "P.xyz = [" << req.goal.pose.position.x << ", "
                                                            << req.goal.pose.position.y << ", "
                                                            << req.goal.pose.position.z << "], "
                                             << "Q.xyzw = [" << req.goal.pose.orientation.x << ", "
                                                             << req.goal.pose.orientation.y << ", "
                                                             << req.goal.pose.orientation.z << ", "
                                                             << req.goal.pose.orientation.w << "]");
  // clang-format on

  // plan
  plan_ = std::make_shared<MoveGroupInterface::Plan>();
  MoveItErrorCode error_code = move_group_->plan(*plan_);

  if (error_code == MoveItErrorCode::SUCCESS)
  {
    ROS_INFO("Motion planning succeeded. Execution can be triggered now.");
    res.success = true;
    res.trajectory = plan_->trajectory_.joint_trajectory;
  }
  else
  {
    ROS_ERROR("Failed to find a motion plan.");
    res.success = false;
  }

  res.error_code = error_code;

  return true;
}

bool MoveItInterface::executeMotionPlan(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Received 'execute' request.");
  MoveItErrorCode error_code = move_group_->execute(*plan_);

  if (error_code == MoveItErrorCode::SUCCESS)
  {
    res.success = true;
    res.message = "Motion plan was executed successfully.";
    ROS_INFO_STREAM(res.message);
  }
  else
  {
    res.success = false;
    res.message = "Motion plan could not be executed. MoveIt error code: " + std::to_string(error_code.val);
    ROS_ERROR_STREAM(res.message);
  }

  return true;
}
