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

#include "robot_middleware/jog_interface.h"

#include "tf2_eigen/tf2_eigen.h"

#include <algorithm>
#include <cstring>
#include <vector>

namespace robot_middleware
{

JogInterface::JogInterface()
  : driver_(nullptr)
  , nh_(LOGNAME)
  , cmd_timestamp_(0)
  , state_(JogInterfaceState::IDLE)
  , control_loop_hz_(0)
  , keep_running_(false)
  , in_error_(false)
{
  if (!nh_.param("receive_timeout", receive_timeout_, DEFAULT_RECEIVE_TIMEOUT))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter '%s' not found. Using default value: %f",
                   nh_.resolveName("receive_timeout").c_str(), DEFAULT_RECEIVE_TIMEOUT);
  }

  ROS_DEBUG_NAMED(LOGNAME, "Initialized jog interface 'receive_timeout': %f", receive_timeout_);
}

JogInterface::~JogInterface()
{
  stop();
}

void JogInterface::init(const RobotDriverPtr& driver)
{
  driver_ = driver;
  control_loop_hz_ = driver_->getVelocityControlSettings().control_loop_frequency;
  reset();
}

void JogInterface::start()
{
  keep_running_ = true;
  control_task_ = std::thread(&JogInterface::loop, this);
  sub_twist_ = nh_.subscribe<geometry_msgs::TwistStamped>("vel_cmd", 1, &JogInterface::twistCb, this);
  sub_joint_jog_ = nh_.subscribe<control_msgs::JointJog>("joint_jog_cmd", 1, &JogInterface::jointJogCb, this);
}

void JogInterface::stop()
{
  if (keep_running_)
  {
    sub_twist_.shutdown();
    sub_joint_jog_.shutdown();
    keep_running_ = false;
    command_received_cv_.notify_one();  // wake up thread if necessary
    if (control_task_.joinable())
      control_task_.join();
    reset();
  }
}

void JogInterface::reset()
{
  state_ = JogInterfaceState::IDLE;
  std::memset(vel_cmd_.cmd.data(), 0, sizeof(vel_cmd_.cmd));
  vel_cmd_.type = motion_control_msgs::VelocityCommand::UNDEFINED;
}

void JogInterface::twistCb(const geometry_msgs::TwistStampedConstPtr& cmd)
{
  ros::Time timestamp = ros::Time::now();
  std::lock_guard lock(state_mtx_);
  bool in_error = inError(timestamp);
  cmd_timestamp_ = timestamp;

  if (!in_error && setState(JogInterfaceState::CARTESIAN_JOGGING))
  {
    if (!setCommand(cmd))
    {
      reset();
      return;
    }

    command_received_cv_.notify_one();
  }
}

void JogInterface::jointJogCb(const control_msgs::JointJogConstPtr& cmd)
{
  ros::Time timestamp = ros::Time::now();
  std::lock_guard lock(state_mtx_);
  bool in_error = inError(timestamp);
  cmd_timestamp_ = timestamp;

  if (!in_error && setState(JogInterfaceState::JOINT_JOGGING))
  {
    if (!setCommand(cmd))
    {
      reset();
      return;
    }

    command_received_cv_.notify_one();
  }
}

bool JogInterface::inError(ros::Time latest_cmd_timestamp)
{
  // check error state and recover after receive timeout
  if (in_error_ && (latest_cmd_timestamp - cmd_timestamp_).toSec() > receive_timeout_)
    in_error_ = false;

  if (in_error_)
  {
    ROS_DEBUG_THROTTLE_NAMED(3, LOGNAME,
                             "Ignoring current jog command stream when the jog interface was in error state. "
                             "Jogging will be available again after a receive timeout of %.2f seconds",
                             receive_timeout_);
  }

  return in_error_;
}

bool JogInterface::setCommand(const geometry_msgs::TwistStampedConstPtr& cmd)
{
  // check frame id
  if (cmd->header.frame_id.empty())
  {
    // no error, just warn once
    ROS_WARN_ONCE_NAMED(
        LOGNAME,
        "No reference frame specified in twist command. Using the base frame '%s' as reference frame by default.",
        driver_->getVelocityControlSettings().base_frame.c_str());
    vel_cmd_.type = motion_control_msgs::VelocityCommand::BASE_FRAME;
  }
  else if (cmd->header.frame_id == driver_->getVelocityControlSettings().base_frame)
  {
    vel_cmd_.type = motion_control_msgs::VelocityCommand::BASE_FRAME;
  }
  else if (cmd->header.frame_id == driver_->getVelocityControlSettings().tool_frame)
  {
    vel_cmd_.type = motion_control_msgs::VelocityCommand::TOOL_FRAME;
  }
  else
  {
    auto robot_state = driver_->getRobotState();
    if (!robot_state.knowsFrameTransform(cmd->header.frame_id))
    {
      ROS_ERROR_THROTTLE_NAMED(1, LOGNAME, "Unknown frame '%s' in jog command", cmd->header.frame_id.c_str());

      return false;
    }

    auto base_link = robot_state.getLinkModel(driver_->getVelocityControlSettings().base_frame);
    auto tool_link = robot_state.getLinkModel(driver_->getVelocityControlSettings().tool_frame);
    auto reference_link = robot_state.getLinkModel(cmd->header.frame_id);

    // check for fixed transform to the base frame
    auto fixed_trsf_map_base = base_link->getAssociatedFixedTransforms();
    if (fixed_trsf_map_base.find(reference_link) != fixed_trsf_map_base.end())
    {
      ROS_DEBUG_NAMED(LOGNAME,
                      "Reference frame '%s' has a fixed transform to the base frame. Transforming twist command.",
                      cmd->header.frame_id.c_str());

      // transform twist into the base frame
      Eigen::Isometry3d trsf = fixed_trsf_map_base[reference_link];
      Eigen::Matrix<double, 6, 1> twist_in;
      Eigen::Matrix<double, 6, 1> twist_out;
      tf2::fromMsg(cmd->twist, twist_in);
      twist_out.segment<3>(0) = trsf.linear() * twist_in.segment<3>(0);
      twist_out.segment<3>(3) = trsf.linear() * twist_in.segment<3>(3);

      // set the actual velocity command
      std::copy(twist_out.data(), twist_out.data() + twist_out.size(), vel_cmd_.cmd.data());
      vel_cmd_.type = motion_control_msgs::VelocityCommand::BASE_FRAME;

      return true;
    }

    // check for fixed transform to the tool frame
    auto fixed_trsf_map_tool = tool_link->getAssociatedFixedTransforms();
    if (fixed_trsf_map_tool.find(reference_link) != fixed_trsf_map_tool.end())
    {
      ROS_DEBUG_NAMED(LOGNAME,
                      "Reference frame '%s' has a fixed transform to the tool frame. Transforming twist command.",
                      cmd->header.frame_id.c_str());

      // transform twist into tool frame
      Eigen::Isometry3d tool_to_ref_trsf = fixed_trsf_map_tool[reference_link];
      Eigen::Matrix<double, 6, 1> twist_in;
      Eigen::Matrix<double, 6, 1> twist_out;
      tf2::fromMsg(cmd->twist, twist_in);
      twist_out.segment<3>(0) = tool_to_ref_trsf.linear() * twist_in.segment<3>(0);
      twist_out.segment<3>(3) = tool_to_ref_trsf.linear() * twist_in.segment<3>(3);

      // additional translation of the tool frame caused by rotation around the reference frame
      twist_out.segment<3>(0) += tool_to_ref_trsf.translation().cross(twist_out.segment<3>(3));

      // set the actual velocity command
      std::copy(twist_out.data(), twist_out.data() + twist_out.size(), vel_cmd_.cmd.data());
      vel_cmd_.type = motion_control_msgs::VelocityCommand::TOOL_FRAME;

      return true;
    }

    // error otherwise
    ROS_ERROR_THROTTLE_NAMED(1, LOGNAME,
                             "The given reference frame '%s' can not be used for jogging. It should be the base frame "
                             "'%s', the tool frame '%s' or a fixed transform to one of these.",
                             cmd->header.frame_id.c_str(), driver_->getVelocityControlSettings().base_frame.c_str(),
                             driver_->getVelocityControlSettings().tool_frame.c_str());

    return false;
  }

  vel_cmd_.cmd[0] = cmd->twist.linear.x;
  vel_cmd_.cmd[1] = cmd->twist.linear.y;
  vel_cmd_.cmd[2] = cmd->twist.linear.z;
  vel_cmd_.cmd[3] = cmd->twist.angular.x;
  vel_cmd_.cmd[4] = cmd->twist.angular.y;
  vel_cmd_.cmd[5] = cmd->twist.angular.z;

  return true;
}

bool JogInterface::setCommand(const control_msgs::JointJogConstPtr& cmd)
{
  vel_cmd_.type = motion_control_msgs::VelocityCommand::JOINT;
  int max_axes = std::min(vel_cmd_.cmd.size(), cmd->velocities.size());
  for (int i = 0; i < max_axes; i++)
    vel_cmd_.cmd[i] = cmd->velocities[i];

  return true;
}

bool JogInterface::setState(JogInterfaceState new_state)
{
  if (state_ == new_state)
    return true;

  if (state_ == JogInterfaceState::IDLE)
    state_ = new_state;
  else
    ROS_WARN_THROTTLE_NAMED(2, LOGNAME, "State transition (%d->%d) not allowed", (int)state_, (int)new_state);

  return (state_ == new_state);
}

void JogInterface::loop()
{
  ROS_INFO_NAMED(LOGNAME, "Ready to receive jog commands");

  while (ros::ok() && keep_running_)
  {
    // sleep until new command received
    {
      std::unique_lock<std::mutex> lock(state_mtx_);
      reset();
      ROS_DEBUG_NAMED(LOGNAME, "Waiting for jog command...");
      command_received_cv_.wait(lock);

      // check if everything is ok!
      if (!ros::ok() || !keep_running_)
        break;

      // wait again, if the state did not change
      if (state_ == JogInterfaceState::IDLE)
        continue;

      // scoped lock gets unlocked so the command data can be updated
    }

    // try to start velocity control and enter the control loop
    in_error_ = !driver_->startJogging(vel_cmd_.type);
    if (!in_error_)
    {
      in_error_ = !controlLoop();

      // stop
      std::memset(vel_cmd_.cmd.data(), 0, sizeof(vel_cmd_.cmd));
      in_error_ = !driver_->sendVelocityCommand(vel_cmd_);

      // set the driver in idle state
      driver_->stopMotionControl();
    }
  }
}

bool JogInterface::controlLoop()
{
  motion_control_msgs::VelocityCommand vel_cmd;
  ROS_DEBUG_NAMED(LOGNAME, "Entering the control loop");

  ros::Time state_update_timestamp = ros::Time::now();
  int state_wait_timeout = 100;  // in ms

  // ros::Rate rate(control_loop_hz_);
  while (ros::ok() && keep_running_)
  {
    if (!driver_->waitForStateUpdate(state_update_timestamp, state_wait_timeout))
    {
      ROS_WARN_NAMED(LOGNAME, "No state update for %d ms", state_wait_timeout);
      return false;
    }

    {
      std::unique_lock<std::mutex> lock(state_mtx_);

      if (receive_timeout_ > 0 && (ros::Time::now() - cmd_timestamp_).toSec() >= receive_timeout_)
      {
        ROS_DEBUG_NAMED(LOGNAME, "No jog command received until timeout. Exiting control loop.");
        break;
      }

      vel_cmd = vel_cmd_;

      // scoped lock gets unlocked so the command data can be updated
    }

    if (!driver_->sendVelocityCommand(vel_cmd))
      return false;

    // rate.sleep();
  }

  return true;
}

}  // namespace robot_middleware
