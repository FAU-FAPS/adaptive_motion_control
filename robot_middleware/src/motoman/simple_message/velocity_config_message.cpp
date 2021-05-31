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

#include "robot_middleware/motoman/simple_message/velocity_config_message.h"

using namespace industrial::byte_array;
using namespace industrial::simple_message;

namespace motoman
{
namespace simple_message
{

VelocityConfigMessage::VelocityConfigMessage()
{
  this->init();
}

VelocityConfigMessage::~VelocityConfigMessage()
{
}

void VelocityConfigMessage::init()
{
  this->setMessageType(2040);
  this->data_.init();
}

void VelocityConfigMessage::init(const VelocityConfig& data)
{
  this->data_ = data;
}

bool VelocityConfigMessage::init(SimpleMessage& msg)
{
  this->init();
  ByteArray data = msg.getData();

  if (!data.unload(this->data_))
  {
    LOG_ERROR("Failed to unload velocity config message data");
    return false;
  }

  return true;
}

bool VelocityConfigMessage::load(ByteArray* buffer)
{
  LOG_COMM("Executing velocity config message load");

  if (!buffer->load(this->data_))
  {
    LOG_ERROR("Failed to load velocity config message data");
    return false;
  }

  return true;
}

bool VelocityConfigMessage::unload(ByteArray* buffer)
{
  LOG_COMM("Executing velocity config message unload");

  if (!buffer->unload(this->data_))
  {
    LOG_ERROR("Failed to unload velocity config message data");
    return false;
  }

  return false;
}

}  // namespace simple_message
}  // namespace motoman
