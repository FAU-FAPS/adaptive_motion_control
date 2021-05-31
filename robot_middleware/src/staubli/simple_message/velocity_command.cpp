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

#include "robot_middleware/staubli/simple_message/velocity_command.h"

#include <cstring>

using namespace industrial::byte_array;
using namespace industrial::shared_types;

namespace staubli
{
namespace simple_message
{

VelocityCommand::VelocityCommand()
{
  this->init();
}

VelocityCommand::~VelocityCommand()
{
}

void VelocityCommand::init()
{
  this->sequence_ = 0;
  this->type_ = 0;
  std::memset(this->vector_, 0, sizeof(this->vector_));
}

bool VelocityCommand::load(ByteArray* buffer)
{
  LOG_COMM("Executing velocity command load");

  if (!buffer->load(this->sequence_))
  {
    LOG_ERROR("Failed to load velocity command sequence");
    return false;
  }

  for (const shared_real& value : this->vector_)
  {
    if (!buffer->load(value))
    {
      LOG_ERROR("Failed to load velocity command vector");
      return false;
    }
  }

  if (!buffer->load(this->type_))
  {
    LOG_ERROR("Failed to load velocity command type");
    return false;
  }

  return true;
}

bool VelocityCommand::unload(ByteArray* buffer)
{
  LOG_COMM("Executing velocity command unload");

  if (!buffer->unload(this->type_))
  {
    LOG_ERROR("Failed to unload velocity command type");
    return false;
  }

  for (int i = MAX_NUM_JOINTS - 1; i >= 0; i--)
  {
    if (!buffer->unload(this->vector_[i]))
    {
      LOG_ERROR("Failed to unload velocity command vector");
      return false;
    }
  }

  if (!buffer->unload(this->sequence_))
  {
    LOG_ERROR("Failed to unload velocity command sequence");
    return false;
  }

  return true;
}

}  // namespace simple_message
}  // namespace staubli
