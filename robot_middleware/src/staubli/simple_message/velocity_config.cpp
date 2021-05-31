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

#include "robot_middleware/staubli/simple_message/velocity_config.h"

#include <cstring>

using namespace industrial::byte_array;
using namespace industrial::shared_types;

namespace staubli
{
namespace simple_message
{

VelocityConfig::VelocityConfig()
{
  this->init();
}

VelocityConfig::~VelocityConfig()
{
}

void VelocityConfig::init()
{
  this->cmd_type_ = 0;
  std::memset(this->frame_ref_, 0, sizeof(this->frame_ref_));
  std::memset(this->tool_ref_, 0, sizeof(this->tool_ref_));
  this->accel_ = 0.0;
  this->vel_ = 0.0;
  this->tvel_ = 0.0;
  this->rvel_ = 0.0;
}

bool VelocityConfig::load(ByteArray* buffer)
{
  LOG_COMM("Executing velocity config load");

  if (!buffer->load(this->cmd_type_))
  {
    LOG_ERROR("Failed to load velocity config cmd_type");
    return false;
  }

  for (const shared_real& value : this->frame_ref_)
  {
    if (!buffer->load(value))
    {
      LOG_ERROR("Failed to load velocity config frame_ref");
      return false;
    }
  }

  for (const shared_real& value : this->tool_ref_)
  {
    if (!buffer->load(value))
    {
      LOG_ERROR("Failed to load velocity config tool_ref");
      return false;
    }
  }

  if (!buffer->load(accel_))
  {
    LOG_ERROR("Failed to load velocity config accel");
    return false;
  }

  if (!buffer->load(vel_))
  {
    LOG_ERROR("Failed to load velocity config vel");
    return false;
  }

  if (!buffer->load(this->tvel_))
  {
    LOG_ERROR("Failed to load velocity config tvel");
    return false;
  }

  if (!buffer->load(this->rvel_))
  {
    LOG_ERROR("Failed to load velocity config rvel");
    return false;
  }

  return true;
}

bool VelocityConfig::unload(ByteArray* buffer)
{
  LOG_COMM("Executing velocity config unload");

  if (!buffer->unload(this->rvel_))
  {
    LOG_ERROR("Failed to unload velocity config rvel");
    return false;
  }

  if (!buffer->unload(this->tvel_))
  {
    LOG_ERROR("Failed to unload velocity config tvel");
    return false;
  }

  if (!buffer->unload(this->vel_))
  {
    LOG_ERROR("Failed to unload velocity config vel");
    return false;
  }

  if (!buffer->unload(this->accel_))
  {
    LOG_ERROR("Failed to unload velocity config accel");
    return false;
  }

  for (int i = 5; i >= 0; i--)
  {
    if (!buffer->unload(this->tool_ref_[i]))
    {
      LOG_ERROR("Failed to unload velocity config tool_ref");
      return false;
    }
  }

  for (int i = 5; i >= 0; i--)
  {
    if (!buffer->unload(this->frame_ref_[i]))
    {
      LOG_ERROR("Failed to unload velocity config frame_ref");
      return false;
    }
  }

  if (!buffer->unload(this->cmd_type_))
  {
    LOG_ERROR("Failed to unload velocity config cmd_type");
    return false;
  }

  return true;
}

}  // namespace simple_message
}  // namespace staubli
