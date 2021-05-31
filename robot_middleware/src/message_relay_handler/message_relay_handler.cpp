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

#include "robot_middleware/message_relay_handler/message_relay_handler.h"

#include <cstdio>

using namespace industrial::simple_message;
using namespace robot_middleware::connection_manager;

namespace robot_middleware
{
namespace message_relay_handler
{

MessageRelayHandler::MessageRelayHandler(const std::string& name)
  : name_(name), driver_(nullptr), in_conn_mngr_(nullptr), out_conn_mngr_(nullptr), nh_(name)
{
  double receive_timeout_seconds;

  if (!nh_.param("receive_timeout", receive_timeout_seconds, DEFAULT_RECEIVE_TIMEOUT))
  {
    ROS_WARN_NAMED(LOGNAME, "Parameter '%s' not found. Using default value: %f",
                   nh_.resolveName("receive_timeout").c_str(), DEFAULT_RECEIVE_TIMEOUT);
  }

  receive_timeout_ = receive_timeout_seconds * 1e3;
  ROS_DEBUG_NAMED(LOGNAME, "Initialized %s 'receive_timeout': %d ms", getName(), receive_timeout_);
}

MessageRelayHandler::~MessageRelayHandler()
{
  stop();
}

void MessageRelayHandler::init(const RobotDriverPtr& driver, const SimpleSocketManagerPtr& in,
                               const SimpleSocketManagerPtr& out)
{
  driver_ = driver;
  in_conn_mngr_ = in;
  out_conn_mngr_ = out;
}

const char* MessageRelayHandler::getName()
{
  return name_.c_str();
}

void MessageRelayHandler::onReceiveTimeout()
{
  ROS_DEBUG_NAMED(LOGNAME, "[%s] Receive timeout", getName());
}

void MessageRelayHandler::onReceiveFail()
{
  ROS_ERROR_NAMED(LOGNAME, "[%s] Receive error", getName());
}

bool MessageRelayHandler::handleMessage(SimpleMessage& msg, ros::Time& timestamp)
{
  ROS_DEBUG_NAMED(LOGNAME, "[%s] Received message (msgType: %d)", getName(), msg.getMessageType());
  return true;
}

void MessageRelayHandler::spin()
{
  ROS_DEBUG_NAMED(LOGNAME, "[%s] Start spinning", getName());

  while (ros::ok())
  {
    // wait until _in_ connection is ready
    while (ros::ok() && !in_conn_mngr_->isConnected())
      ros::Duration(0.1).sleep();

    spinOnce();
  }
}

void MessageRelayHandler::spinOnce()
{
  // check connection
  if (!in_conn_mngr_->isConnected())
    return;

  // handle receive timeout
  if (!in_conn_mngr_->isReadyReceive(receive_timeout_))
  {
    onReceiveTimeout();
    return;
  }

  // receive message
  SimpleMessage msg;
  if (!in_conn_mngr_->receiveMsg(msg))
  {
    onReceiveFail();
    return;
  }

  // handle message
  ros::Time timestamp = ros::Time::now();
  bool rtn = handleMessage(msg, timestamp);

  // relay message if 'confirmed' by handler callback
  if (rtn && out_conn_mngr_->isConnected())
  {
    rtn = out_conn_mngr_->sendMsg(msg);
    if (rtn)
    {
      ROS_DEBUG_NAMED(LOGNAME, "[%s] Relayed message", getName());
    }
    else
    {
      ROS_ERROR_NAMED(LOGNAME, "[%s] Could not relay message to OUT connection (%s)", getName(),
                      out_conn_mngr_->getName());
    }
  }
  else
  {
    ROS_DEBUG_NAMED(LOGNAME, "[%s] Not relaying message", getName());
    rtn = false;
  }

  // relay reply message from _out_ to _in_ connection if required
  if (msg.getCommType() == CommType::SERVICE_REQUEST)
  {
    // receive reply from _out_ connection
    if (rtn)
    {
      SimpleMessage reply;
      rtn = out_conn_mngr_->receiveMsg(reply);
      if (rtn)
      {
        ROS_DEBUG_NAMED(LOGNAME, "[%s] Received reply message from OUT connection (%s)", getName(),
                        out_conn_mngr_->getName());

        // send reply to _in_ connection
        if (in_conn_mngr_->sendMsg(reply))
        {
          ROS_DEBUG_NAMED(LOGNAME, "[%s] Relayed reply message to IN connection (%s)", getName(),
                          in_conn_mngr_->getName());
        }
        else
        {
          ROS_ERROR_NAMED(LOGNAME, "[%s] Could not relay reply message to IN connection (%s)", getName(),
                          in_conn_mngr_->getName());
        }
      }
      else
      {
        ROS_ERROR_NAMED(LOGNAME, "[%s] Could not receive reply message from OUT connection (%s)", getName(),
                        out_conn_mngr_->getName());
      }
    }

    // send 'FAILURE' reply if receiving reply from _out_ connection failed
    if (!rtn)
    {
      ROS_DEBUG_NAMED(LOGNAME, "[%s] Sending reply message 'FAILURE' to IN connection (%s)", getName(),
                      in_conn_mngr_->getName());

      sendReply(in_conn_mngr_, msg.getMessageType(), ReplyType::FAILURE);
    }
  }
}

void MessageRelayHandler::start()
{
  spinner_task_ = std::thread(&MessageRelayHandler::spin, this);
}

void MessageRelayHandler::stop()
{
  if (spinner_task_.joinable())
  {
#ifndef NDEBUG
    printf("DEBUG: Waiting for '%s' thread to join...\n", getName());
#endif
    spinner_task_.join();
  }
}

bool MessageRelayHandler::sendReply(const SimpleSocketManagerPtr& conn, int msg_type, ReplyType reply_type)
{
  SimpleMessage reply;
  reply.init(msg_type, CommType::SERVICE_REPLY, reply_type);
  return conn->sendMsg(reply);
}

}  // namespace message_relay_handler
}  // namespace robot_middleware
