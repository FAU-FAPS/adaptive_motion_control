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

#include "robot_middleware/robot_middleware.h"

namespace robot_middleware
{

RobotMiddleware::RobotMiddleware(const ros::NodeHandle& nh) : nh_(nh), driver_(nullptr)
{
}

RobotMiddleware::~RobotMiddleware()
{
}

bool RobotMiddleware::init()
{
  // get the robot IP from private parameter
  std::string robot_ip;
  if (!ros::param::get("~robot_ip", robot_ip))
  {
    ROS_ERROR("Failed to get the '~robot_ip' parameter");
    return false;
  }

  // check if driver type is given as a parameter
  std::string driver_type;
  ros::param::get("~driver_type", driver_type);
  if (driver_type.empty())
  {
    // if not, try to guess driver type from robot name
    const std::string& robot_name = robot_model_loader_.getModel()->getName();
    driver_type = robot_name.substr(0, robot_name.find('_'));
    ROS_WARN(
        "Missing or empty parameter '~driver_type'. Trying to guess the driver type from the robot name: '%s' -> '%s'",
        robot_name.c_str(), driver_type.c_str());
  }

  // select the appropriate robot driver
  if (driver_type == "motoman")
  {
    ROS_INFO("Using the Motoman driver");
    driver_ = std::make_shared<motoman::MotomanDriver>(robot_ip, robot_model_loader_.getModel());
  }
  else if (driver_type == "staubli")
  {
    ROS_INFO("Using the Staubli driver");
    driver_ = std::make_shared<staubli::StaubliDriver>(robot_ip, robot_model_loader_.getModel());
  }
  else
  {
    ROS_ERROR("Failed to select an appropriate robot driver for given driver type '%s'. "
              "Only 'motoman' and 'staubli' are supported (case-sensitive).",
              driver_type.c_str());

    return false;
  }

  // init the driver
  if (!driver_->init())
  {
    ROS_ERROR("Failed to initialize the robot driver");
    return false;
  }

  // init the robot server proxy
  if (!server_proxy_.init(driver_->getDefaultMotionPort(), driver_->getDefaultStatePort()))
  {
    ROS_ERROR("Failed to initialize the robot server proxy");
    return false;
  }

  // init the relay handlers
  state_relay_handler_.init(/* driver:  */ driver_,
                            /* in:      */ driver_->getStateClientManager(),
                            /* out:     */ server_proxy_.getStateServerManager());
  motion_relay_handler_.init(/* driver:  */ driver_,
                             /* in:      */ server_proxy_.getMotionServerManager(),
                             /* out:     */ driver_->getMotionClientManager());

  // init the controllers
  jog_interface_.init(driver_);
  pose_tracking_controller_.init(driver_);

  ROS_INFO("Initialized the middleware");

  return true;
}

void RobotMiddleware::run()
{
  // start message relay handlers
  motion_relay_handler_.start();
  state_relay_handler_.start();

  // start controllers
  jog_interface_.start();
  pose_tracking_controller_.start();

  ROS_INFO("All interfaces are ready and running. Starting the connection tasks.");

  // start async connection tasks
  driver_->connect();
  server_proxy_.connect();

  ros::spin();
}

}  // namespace robot_middleware
