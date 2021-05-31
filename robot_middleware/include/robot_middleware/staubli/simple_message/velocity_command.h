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

#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#include "simple_message/shared_types.h"
#include "simple_message/simple_serialize.h"

namespace staubli
{
namespace simple_message
{

class VelocityCommand : public industrial::simple_serialize::SimpleSerialize
{
public:
  VelocityCommand();

  ~VelocityCommand();

  void init(void);

  /**
   * Overrides - SimpleSerialize
   */
  unsigned int byteLength()
  {
    return 2 * sizeof(industrial::shared_types::shared_int) +
           MAX_NUM_JOINTS * sizeof(industrial::shared_types::shared_real);
  }

  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);

  static const std::size_t MAX_NUM_JOINTS = 6;

  industrial::shared_types::shared_int sequence_;
  industrial::shared_types::shared_real vector_[MAX_NUM_JOINTS];
  industrial::shared_types::shared_int type_;
};

}  // namespace simple_message
}  // namespace staubli
