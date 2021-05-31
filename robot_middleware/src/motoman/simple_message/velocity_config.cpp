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

#include "robot_middleware/motoman/simple_message/velocity_config.h"

using namespace industrial::byte_array;
using namespace industrial::shared_types;

namespace motoman
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
  this->user_coord_num_ = 0;
  this->tool_file_num_ = 0;
  this->filter_size_ = 0;
  this->max_linear_velocity_ = 0.0;
  this->max_angular_velocity_ = 0.0;
  this->max_linear_acceleration_ = 0.0;
  this->max_angular_acceleration_ = 0.0;
}

bool VelocityConfig::load(ByteArray* buffer)
{
  LOG_COMM("Executing velocity config load");

  if (!buffer->load(this->cmd_type_))
  {
    LOG_ERROR("Failed to load velocity config cmd_type");
    return false;
  }
  if (!buffer->load(this->user_coord_num_))
  {
    LOG_ERROR("Failed to load velocity config user_coord_num");
    return false;
  }
  if (!buffer->load(this->tool_file_num_))
  {
    LOG_ERROR("Failed to load velocity config tool_file_num");
    return false;
  }
  if (!buffer->load(this->filter_size_))
  {
    LOG_ERROR("Failed to load velocity config filter_size");
    return false;
  }
  if (!buffer->load(this->max_linear_velocity_))
  {
    LOG_ERROR("Failed to load velocity config max_linear_velocity");
    return false;
  }
  if (!buffer->load(this->max_angular_velocity_))
  {
    LOG_ERROR("Failed to load velocity config max_angular_velocity");
    return false;
  }
  if (!buffer->load(this->max_linear_acceleration_))
  {
    LOG_ERROR("Failed to load velocity config max_linear_acceleration");
    return false;
  }
  if (!buffer->load(this->max_angular_acceleration_))
  {
    LOG_ERROR("Failed to load velocity config max_linear_acceleration");
    return false;
  }

  return true;
}

bool VelocityConfig::unload(ByteArray* buffer)
{
  LOG_COMM("Executing velocity config unload");

  if (!buffer->unload(this->max_angular_acceleration_))
  {
    LOG_ERROR("Failed to unload velocity config max_angular_acceleration");
    return false;
  }
  if (!buffer->unload(this->max_linear_acceleration_))
  {
    LOG_ERROR("Failed to unload velocity config max_linear_acceleration");
    return false;
  }
  if (!buffer->unload(this->max_angular_velocity_))
  {
    LOG_ERROR("Failed to unload velocity config max_angular_velocity");
    return false;
  }
  if (!buffer->unload(this->max_linear_velocity_))
  {
    LOG_ERROR("Failed to unload velocity config max_linear_velocity");
    return false;
  }
  if (!buffer->unload(this->filter_size_))
  {
    LOG_ERROR("Failed to unload velocity config filter_size");
    return false;
  }
  if (!buffer->unload(this->tool_file_num_))
  {
    LOG_ERROR("Failed to unload velocity config tool_file_num");
    return false;
  }
  if (!buffer->unload(this->user_coord_num_))
  {
    LOG_ERROR("Failed to unload velocity config user_coord_num");
    return false;
  }
  if (!buffer->unload(this->cmd_type_))
  {
    LOG_ERROR("Failed to unload velocity config cmd_type");
    return false;
  }

  return true;
}

}  // namespace simple_message
}  // namespace motoman
