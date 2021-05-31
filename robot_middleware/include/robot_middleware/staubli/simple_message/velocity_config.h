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

class VelocityConfig : public industrial::simple_serialize::SimpleSerialize
{
public:
  VelocityConfig();

  ~VelocityConfig();

  void init(void);

  /**
   * Overrides - SimpleSerialize
   */
  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int) + (4 + 2 * 6) * sizeof(industrial::shared_types::shared_real);
  }

  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);

  industrial::shared_types::shared_int cmd_type_;
  industrial::shared_types::shared_real frame_ref_[6];
  industrial::shared_types::shared_real tool_ref_[6];
  industrial::shared_types::shared_real accel_;  // maximum joint acceleration in % of the nominal acceleration [0,1]
  industrial::shared_types::shared_real vel_;    // maximum joint velocity in % of the nominal velocity [0,1]
  industrial::shared_types::shared_real tvel_;   // maximum linear velocity of the tool-center-point in m/s
  industrial::shared_types::shared_real rvel_;   // maximum angular velocity of the tool-center-point in rad/s
};

}  // namespace simple_message
}  // namespace staubli
