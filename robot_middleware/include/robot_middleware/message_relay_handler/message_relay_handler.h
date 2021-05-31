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

#include "robot_middleware/connection_manager/simple_socket_manager.h"
#include "robot_middleware/robot_driver.h"

#include "ros/ros.h"
#include "simple_message/simple_message.h"

#include <string>
#include <thread>

namespace robot_middleware
{
namespace message_relay_handler
{

class MessageRelayHandler
{
public:
  explicit MessageRelayHandler(const std::string& name);

  ~MessageRelayHandler();

  virtual void init(const RobotDriverPtr& driver,
                    const robot_middleware::connection_manager::SimpleSocketManagerPtr& in,
                    const robot_middleware::connection_manager::SimpleSocketManagerPtr& out);

  const char* getName();

  void spin();

  void spinOnce();

  void start();

  void stop();

protected:
  virtual void onReceiveTimeout();

  virtual void onReceiveFail();

  virtual bool handleMessage(industrial::simple_message::SimpleMessage& msg, ros::Time& timestamp);

  bool sendReply(const robot_middleware::connection_manager::SimpleSocketManagerPtr& conn, int msg_type,
                 industrial::simple_message::ReplyType reply_type);

  const std::string LOGNAME = "message_relay_handler";
  const double DEFAULT_RECEIVE_TIMEOUT = 1.0;  // seconds

  const std::string name_;
  robot_middleware::RobotDriverPtr driver_;
  robot_middleware::connection_manager::SimpleSocketManagerPtr in_conn_mngr_;
  robot_middleware::connection_manager::SimpleSocketManagerPtr out_conn_mngr_;
  ros::NodeHandle nh_;
  int receive_timeout_;  // ms
  std::thread spinner_task_;
};

}  // namespace message_relay_handler
}  // namespace robot_middleware
