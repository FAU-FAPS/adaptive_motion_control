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

#include "robot_middleware/staubli/simple_message/velocity_command_message.h"

using namespace industrial::byte_array;
using namespace industrial::simple_message;

namespace staubli
{
namespace simple_message
{

VelocityCommandMessage::VelocityCommandMessage()
{
  this->init();
}

VelocityCommandMessage::~VelocityCommandMessage()
{
}

void VelocityCommandMessage::init()
{
  this->setMessageType(1641);
  this->data_.init();
}

void VelocityCommandMessage::init(const VelocityCommand& data)
{
  this->data_ = data;
}

bool VelocityCommandMessage::init(SimpleMessage& msg)
{
  this->init();
  ByteArray data = msg.getData();
  if (!data.unload(this->data_))
  {
    LOG_ERROR("Failed to unload velocity command message data");
    return false;
  }
  return true;
}

bool VelocityCommandMessage::load(ByteArray* buffer)
{
  LOG_COMM("Executing velocity command message load");

  if (!buffer->load(this->data_))
  {
    LOG_ERROR("Failed to load velocity command data");
    return false;
  }

  return true;
}

bool VelocityCommandMessage::unload(ByteArray* buffer)
{
  LOG_COMM("Executing velocity command message unload");

  if (!buffer->unload(this->data_))
  {
    LOG_ERROR("Failed to unload velocity command data");
    return false;
  }

  return true;
}

}  // namespace simple_message
}  // namespace staubli
