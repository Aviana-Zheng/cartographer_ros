/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/time_conversion.h"

#include "cartographer/common/time.h"
#include "ros/ros.h"

namespace cartographer_ros {

// kUtsEpochOffsetFromUnixEpochInSeconds  719162ll * 24ll * 60ll * 60ll   单位s
// 719162 是0001年1月1日到1970年1月1日所经历的天数
::ros::Time ToRos(::cartographer::common::Time time) {
  // 千万分之一秒,0.1us.    10000000   再化为ns，  *100
  int64 uts_timestamp = ::cartographer::common::ToUniversal(time);
  int64 ns_since_unix_epoch =
      (uts_timestamp -
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
           10000000ll) *
      100ll;
  ::ros::Time ros_time;
  ros_time.fromNSec(ns_since_unix_epoch);
  return ros_time;
}

// TODO(pedrofernandez): Write test.
// ICU库 https://unicode-org.github.io/icu/userguide/datetime/timezone/
::cartographer::common::Time FromRos(const ::ros::Time& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ::cartographer::common::FromUniversal(
      (time.sec +
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (time.nsec + 50) / 100);  // + 50 to get the rounding correct.
}

}  // namespace cartographer_ros
