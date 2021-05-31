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

#include "robot_middleware/motoman/simple_message/velocity_config.h"

#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#include "simple_message/simple_message.h"
#include "simple_message/typed_message.h"

namespace motoman
{
namespace simple_message
{

class VelocityConfigMessage : public industrial::typed_message::TypedMessage
{
public:
  VelocityConfigMessage(void);
  ~VelocityConfigMessage(void);

  void init(void);
  void init(const VelocityConfig& data);
  bool init(industrial::simple_message::SimpleMessage& msg);

  unsigned int byteLength()
  {
    return this->data_.byteLength();
  }

  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);

  VelocityConfig data_;
};

}  // namespace simple_message
}  // namespace motoman
