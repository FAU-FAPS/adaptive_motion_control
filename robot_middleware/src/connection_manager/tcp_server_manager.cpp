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

#include "robot_middleware/connection_manager/tcp_server_manager.h"

#include <cstdio>

using namespace industrial::tcp_server;

namespace robot_middleware
{
namespace connection_manager
{

TcpServerManager::TcpServerManager(const std::string& name, int port) : SimpleSocketManager(name, port)
{
}

TcpServerManager::~TcpServerManager()
{
#ifndef NDEBUG
  printf("DEBUG: Destructing TcpServerManager '%s'\n", getName());
#endif
}

bool TcpServerManager::init()
{
  auto tcp_server = std::make_unique<TcpServer>();
  if (!tcp_server->init(port_))
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to initialize connection '%s' (0.0.0.0:%d)", getName(), port_);
    return false;
  }

  conn_ = std::move(tcp_server);
  ROS_INFO_NAMED(LOGNAME, "Initialized connection '%s' (0.0.0.0:%d)", getName(), port_);

  return true;
}

bool TcpServerManager::connect()
{
  if (conn_->isConnected())
  {
    ROS_WARN_NAMED(LOGNAME, "[%s] Attempted to connect while in connected state", getName());
    return true;
  }

  if (connected_once_)
  {
    ROS_INFO_NAMED(LOGNAME, "[%s] Trying to re-establish connection", getName());
    // no init required for TcpServer!
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "[%s] Trying to connect", getName());
  }

  while (ros::ok())
  {
    // TcpServer::makeConnect blocks until client accepted!
    bool rtn = conn_->makeConnect();

#ifndef NDEBUG
    printf("DEBUG: makeConnect() returned %d for '%s'\n", rtn, getName());
#endif

    if (isConnected())
    {
      connected_once_ = true;
      ROS_INFO_NAMED(LOGNAME, "[%s] Connected", getName());
      return true;
    }
  }

  return false;
}

}  // namespace connection_manager
}  // namespace robot_middleware
