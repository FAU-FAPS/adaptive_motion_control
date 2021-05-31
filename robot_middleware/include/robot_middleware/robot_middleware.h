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

#include "robot_middleware/jog_interface.h"
#include "robot_middleware/message_relay_handler/motion_relay_handler.h"
#include "robot_middleware/message_relay_handler/state_relay_handler.h"
#include "robot_middleware/motoman/motoman_driver.h"
#include "robot_middleware/pose_tracking_controller.h"
#include "robot_middleware/robot_driver.h"
#include "robot_middleware/robot_server_proxy.h"
#include "robot_middleware/staubli/staubli_driver.h"
#include "robot_middleware/velocity_control_settings.h"

#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "ros/ros.h"
#include "simple_message/socket/tcp_server.h"

#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace robot_middleware
{

class RobotMiddleware
{
public:
  explicit RobotMiddleware(const ros::NodeHandle& nh);

  ~RobotMiddleware();

  bool init();

  void run();

private:
  ros::NodeHandle nh_;
  RobotDriverPtr driver_;
  RobotServerProxy server_proxy_;
  JogInterface jog_interface_;
  PoseTrackingController pose_tracking_controller_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  robot_middleware::message_relay_handler::StateRelayHandler state_relay_handler_;
  robot_middleware::message_relay_handler::MotionRelayHandler motion_relay_handler_;
};

}  // namespace robot_middleware
