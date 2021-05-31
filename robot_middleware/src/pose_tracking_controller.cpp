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

#include "robot_middleware/pose_tracking_controller.h"

#include <cassert>
#include <cstring>

#ifndef NDEBUG
#include <cstdio>
#endif

namespace robot_middleware
{

PoseTrackingController::PoseTrackingController()
  : driver_(nullptr)
  , nh_(LOGNAME)
  , controller_period_(0.0)
  , goal_timestamp_(0.0)
  , last_goal_timestamp_(0.0)
  , tcp_frame_(DEFAULT_TCP_FRAME)
  , goal_tolerance_linear_(DEFAULT_GOAL_TOLERANCE_LINEAR)
  , goal_tolerance_angular_(DEFAULT_GOAL_TOLERANCE_ANGULAR)
  , goal_settle_time_(DEFAULT_GOAL_SETTLE_TIME)
  , receive_timeout_(DEFAULT_RECEIVE_TIMEOUT)
  , keep_running_(false)
  , in_error_(false)
{
}

PoseTrackingController::~PoseTrackingController()
{
  stop();
}

void PoseTrackingController::init(const RobotDriverPtr& driver)
{
  driver_ = driver;

  // set cycle time of the controller
  assert(driver_->getVelocityControlSettings().control_loop_frequency > 0);
  controller_period_ = ros::Duration(1.0 / driver_->getVelocityControlSettings().control_loop_frequency);

  // init pid controllers
  std::size_t error = 0;
  error += !initPid("pid_linear_x", pid_linear_x_);
  error += !initPid("pid_linear_y", pid_linear_y_);
  error += !initPid("pid_linear_z", pid_linear_z_);
  error += !initPid("pid_angular", pid_angular_);

  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  // other parameters
  if (!nh_.param("tcp_frame", tcp_frame_, DEFAULT_TCP_FRAME))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter '%s' not found. Using default value: %s", nh_.resolveName("tcp_frame").c_str(),
                   DEFAULT_TCP_FRAME.c_str());
  }

  if (!nh_.param("goal_tolerance_linear", goal_tolerance_linear_, DEFAULT_GOAL_TOLERANCE_LINEAR))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter '%s' not found. Using default value: %f",
                   nh_.resolveName("goal_tolerance_linear").c_str(), DEFAULT_GOAL_TOLERANCE_LINEAR);
  }

  if (!nh_.param("goal_tolerance_angular", goal_tolerance_angular_, DEFAULT_GOAL_TOLERANCE_ANGULAR))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter '%s' not found. Using default value: %f",
                   nh_.resolveName("goal_tolerance_angular").c_str(), DEFAULT_GOAL_TOLERANCE_ANGULAR);
  }

  if (!nh_.param("goal_settle_time", goal_settle_time_, DEFAULT_GOAL_SETTLE_TIME))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter '%s' not found. Using default value: %f",
                   nh_.resolveName("goal_settle_time").c_str(), DEFAULT_GOAL_SETTLE_TIME);
  }

  if (!nh_.param("receive_timeout", receive_timeout_, DEFAULT_RECEIVE_TIMEOUT))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter '%s' not found. Using default value: %f",
                   nh_.resolveName("receive_timeout").c_str(), DEFAULT_RECEIVE_TIMEOUT);
  }

  ROS_DEBUG_NAMED(LOGNAME, "Initialized pose tracking 'tcp_frame':              %s", tcp_frame_.c_str());
  ROS_DEBUG_NAMED(LOGNAME, "Initialized pose tracking 'goal_tolerance_linear':  %f", goal_tolerance_linear_);
  ROS_DEBUG_NAMED(LOGNAME, "Initialized pose tracking 'goal_tolerance_angular': %f", goal_tolerance_angular_);
  ROS_DEBUG_NAMED(LOGNAME, "Initialized pose tracking 'goal_settle_time':       %f", goal_settle_time_);
  ROS_DEBUG_NAMED(LOGNAME, "Initialized pose tracking 'receive_timeout':        %f", receive_timeout_);

  // TODO: sanity checks for parameters?
}

bool PoseTrackingController::initPid(const std::string& prefix, control_toolbox::Pid& pid)
{
  if (!pid.initParam(ros::names::append(nh_.getNamespace(), prefix)))
    return false;

  ROS_INFO_NAMED(LOGNAME, "Initialized PID controller '%s'", prefix.c_str());

#ifndef NDEBUG
  double p, i, d, i_max, i_min;
  bool antiwindup;
  pid.getGains(p, i, d, i_max, i_min, antiwindup);
  printf("  %s:\n"
         "    p:          %.3f\n"
         "    i:          %.3f\n"
         "    d:          %.3f\n"
         "    i_max:      %.3f\n"
         "    i_min:      %.3f\n"
         "    antiwindup: %d\n",
         prefix.c_str(), p, i, d, i_max, i_min, antiwindup);
#endif

  return true;
}

void PoseTrackingController::start()
{
  keep_running_ = true;
  control_task_ = std::thread(&PoseTrackingController::loop, this);
  sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("goal", 1, &PoseTrackingController::poseGoalCb, this);
}

void PoseTrackingController::stop()
{
  if (keep_running_)
  {
    sub_.shutdown();
    keep_running_ = false;
    goal_received_.notify_one();  // wake up thread if necessary
    if (control_task_.joinable())
      control_task_.join();
    reset();
  }
}

void PoseTrackingController::reset()
{
  pid_linear_x_.reset();
  pid_linear_y_.reset();
  pid_linear_z_.reset();
  pid_angular_.reset();
}

void PoseTrackingController::poseGoalCb(const geometry_msgs::PoseStampedConstPtr& goal)
{
  ros::Time timestamp = ros::Time::now();
  std::lock_guard lock(goal_mtx_);

  // check error state and recover after receive timeout
  if (in_error_ && (timestamp - goal_timestamp_).toSec() > receive_timeout_)
    in_error_ = false;

  goal_timestamp_ = timestamp;

  if (in_error_)
  {
    ROS_DEBUG_THROTTLE_NAMED(3, LOGNAME,
                             "Ignoring current pose goal stream when pose tracking was in error state. "
                             "Pose tracking will be available again after a receive timeout of %.2f seconds",
                             receive_timeout_);
  }
  else
  {
    // store pose goal and notify loop thread
    goal_ = *goal;
    goal_received_.notify_one();
  }
}

bool PoseTrackingController::hasReachedGoal(const Eigen::Vector3d& linear_error, const Eigen::AngleAxisd& angular_error,
                                            ros::Time goal_timestamp)
{
  static ros::Time goal_enter_timestamp;
  static bool goal_entered;

  double linear_error_abs = std::abs(linear_error.norm());
  double angular_error_abs = std::abs(angular_error.angle());

  // forcing the controller to stay active on new goals even if goal reached
  goal_enter_timestamp = std::max(goal_enter_timestamp, goal_timestamp);

  if (linear_error_abs <= goal_tolerance_linear_ && angular_error_abs <= goal_tolerance_angular_)
  {
    if (goal_entered)
    {
      if (ros::Time::now() - goal_enter_timestamp >= ros::Duration(goal_settle_time_))
        return true;
    }
    else
    {
      goal_entered = true;
      goal_enter_timestamp = ros::Time::now();
    }
  }
  else
  {
    goal_entered = false;
  }

  return false;
}

void PoseTrackingController::computeCommand(const Eigen::Vector3d& linear_error, const Eigen::AngleAxisd& angular_error,
                                            motion_control_msgs::VelocityCommand& vel_cmd)
{
  auto& cmd = vel_cmd.cmd;

  cmd[0] = pid_linear_x_.computeCommand(linear_error.x(), controller_period_);
  cmd[1] = pid_linear_y_.computeCommand(linear_error.y(), controller_period_);
  cmd[2] = pid_linear_z_.computeCommand(linear_error.z(), controller_period_);

  double angle_cmd = pid_angular_.computeCommand(angular_error.angle(), controller_period_);
  cmd[3] = angle_cmd * angular_error.axis().x();
  cmd[4] = angle_cmd * angular_error.axis().y();
  cmd[5] = angle_cmd * angular_error.axis().z();
}

void PoseTrackingController::loop()
{
  ROS_INFO_NAMED(LOGNAME, "Ready to receive pose goals");

  while (ros::ok() && keep_running_)
  {
    ROS_DEBUG_NAMED(LOGNAME, "Waiting for new pose goal...");

    // sleep until new goal received
    {
      std::unique_lock lock(goal_mtx_);
      reset();
      goal_received_.wait(lock);

      // check if everything is ok!
      if (!ros::ok() || !keep_running_)
        break;

      // wait again, if it was a spurious wakeup
      if (goal_timestamp_ == last_goal_timestamp_)
        continue;

      last_goal_timestamp_ = goal_timestamp_;

      // scoped lock gets unlocked so the goal can be updated in the callback
    }

    // try to start pose tracking mode and enter the control loop
    in_error_ = !driver_->startPoseTracking(motion_control_msgs::VelocityCommand::BASE_FRAME);
    if (!in_error_)
    {
      in_error_ = !controlLoop();

      // set the driver in idle state
      driver_->stopMotionControl();
    }
  }
}

bool PoseTrackingController::controlLoop()
{
  // flag to exit control loop when the goal is reached within tolerance and settle time
  bool reached_goal = false;

  // goal data
  Eigen::Isometry3d goal_trsf;
  Eigen::Vector3d goal_position;
  Eigen::Quaterniond goal_orientation;
  ros::Time goal_timestamp;
  std::string reference_frame;

  // feedback data
  Eigen::Isometry3d feedback_trsf;
  Eigen::Vector3d feedback_position;
  Eigen::Quaterniond feedback_orientation;
  ros::Time last_feedback_time = ros::Time::now();

  // velocity command
  motion_control_msgs::VelocityCommand vel_cmd;
  vel_cmd.type = motion_control_msgs::VelocityCommand::BASE_FRAME;

  ROS_DEBUG_NAMED(LOGNAME, "Entering control loop");

  while (ros::ok() && keep_running_ && !reached_goal)
  {
    // store local copy of the goal data
    {
      std::lock_guard lock(goal_mtx_);
      tf2::fromMsg(goal_.pose, goal_trsf);
      goal_timestamp = goal_timestamp_;
      reference_frame = goal_.header.frame_id;
    }

    // check if a reference frame is given or set default frame (base frame)
    const std::string& base_frame = driver_->getVelocityControlSettings().base_frame;
    const std::string& actual_reference_frame = reference_frame.empty() ? base_frame : reference_frame;
    if (reference_frame.empty())
    {
      ROS_WARN_ONCE_NAMED(
          LOGNAME, "No reference frame specified in goal pose. Using the base frame '%s' as reference frame by default",
          actual_reference_frame.c_str());
    }

    // transform goal into the base frame if necessary
    if (actual_reference_frame != base_frame)
    {
      robot_state::RobotState state = driver_->getRobotState();

      // check if the reference frame exists in the robot model
      if (!state.knowsFrameTransform(actual_reference_frame))
      {
        ROS_ERROR_NAMED(LOGNAME, "The reference frame '%s' of goal pose does not exist in the robot model",
                        actual_reference_frame.c_str());
        return false;
      }

      Eigen::Isometry3d reference_trsf =
          state.getGlobalLinkTransform(base_frame).inverse() * state.getGlobalLinkTransform(actual_reference_frame);

      // goal pose as is (w.r.t. to a reference frame)
      Eigen::Isometry3d goal_trsf_ref = goal_trsf;
      // goal pose transformed into the base frame (actually used)
      goal_trsf = reference_trsf * goal_trsf_ref;

#ifndef NDEBUG
      Eigen::Vector3d p = goal_trsf_ref.translation();
      Eigen::Quaterniond q = Eigen::Quaterniond(goal_trsf_ref.linear());
      printf("Goal (w.r.t. '%s'):\n"
             "  P.xyz  = [%.5f, %.5f, %.5f]\n"
             "  Q.xyzw = [%.5f, %.5f, %.5f, %.5f]\n"
             "---\n",
             actual_reference_frame.c_str(), p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());

      p = goal_trsf.translation();
      q = Eigen::Quaterniond(goal_trsf.linear());
      printf("Goal (w.r.t. '%s'):\n"
             "  P.xyz  = [%.5f, %.5f, %.5f]\n"
             "  Q.xyzw = [%.5f, %.5f, %.5f, %.5f]\n"
             "---\n",
             base_frame.c_str(), p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
#endif
    }

    // check if new goal arrived within timeout
    if (receive_timeout_ > 0 && (ros::Time::now() - goal_timestamp_).toSec() >= receive_timeout_)
    {
      ROS_DEBUG_NAMED(LOGNAME, "No goal received until timeout. Exiting control loop.");
      break;
    }

    // get the most recent feedback from the robot (syncing with the robots interpolation clock)
    // TODO: decrease timeout for feedback and get from ROS parameter(?)
    if (!driver_->waitForFeedback(tcp_frame_, base_frame, last_feedback_time, 100, feedback_trsf))
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed to get feedback from the robot. Exiting control loop.");
      return false;
    }

    // decompose transforms into translation and rotation
    goal_position = goal_trsf.translation();
    goal_orientation = Eigen::Quaterniond(goal_trsf.linear());
    feedback_position = feedback_trsf.translation();
    feedback_orientation = Eigen::Quaterniond(feedback_trsf.linear());

    // compute error
    Eigen::Vector3d linear_error = goal_position - feedback_position;
    Eigen::AngleAxisd angular_error = Eigen::AngleAxisd(goal_orientation * feedback_orientation.inverse());

    // check if goal reached
    if (hasReachedGoal(linear_error, angular_error, goal_timestamp))
    {
      // set flag to exit loop and send zero vector to stop the robot
      reached_goal = true;
      std::memset(vel_cmd.cmd.data(), 0, sizeof(vel_cmd.cmd));
    }
    else
    {
      computeCommand(linear_error, angular_error, vel_cmd);
    }

    // send command to robot
    if (!driver_->sendVelocityCommand(vel_cmd))
      return false;

#ifndef NDEBUG
    Eigen::Vector3d angular_error_vec = angular_error.axis() * angular_error.angle();
    const auto& cmd = vel_cmd.cmd;
    // clang-format off
    printf("Controller state:\n"
           "  goal:     Vector3 = [%.5f, %.5f, %.5f], Q.xyzw = [%.5f, %.5f, %.5f, %.5f]\n"
           "  feedback: Vector3 = [%.5f, %.5f, %.5f], Q.xyzw = [%.5f, %.5f, %.5f, %.5f]\n"
           "  error:    Vector6 = [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f]\n"
           "  command:  Vector6 = [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f]\n"
           "---\n",
           goal_position.x(), goal_position.y(), goal_position.z(),
           goal_orientation.x(), goal_orientation.y(), goal_orientation.z(), goal_orientation.w(),
           feedback_position.x(), feedback_position.y(), feedback_position.z(),
           feedback_orientation.x(), feedback_orientation.y(), feedback_orientation.z(), feedback_orientation.w(),
           linear_error.x(), linear_error.y(), linear_error.z(),
           angular_error_vec.x(), angular_error_vec.y(), angular_error_vec.z(),
           cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);
    // clang-format on
#endif
  }

  return true;
}

}  // namespace robot_middleware
