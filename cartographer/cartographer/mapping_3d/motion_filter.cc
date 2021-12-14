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

#include "cartographer/mapping_3d/motion_filter.h"

#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {

proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::MotionFilterOptions options;
  options.set_max_time_seconds(
      parameter_dictionary->GetDouble("max_time_seconds"));
  options.set_max_distance_meters(
      parameter_dictionary->GetDouble("max_distance_meters"));
  options.set_max_angle_radians(
      parameter_dictionary->GetDouble("max_angle_radians"));
  return options;
}

MotionFilter::MotionFilter(const proto::MotionFilterOptions& options)
    : options_(options) {}

bool MotionFilter::IsSimilar(const common::Time time,
                             const transform::Rigid3d& pose) {
  LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
      << "Motion filter reduced the number of scans to "
      << 100. * num_different_ / num_total_ << "%.";
  // 首先对num_total_进行了累加
  // 这是一个记录所有的进入更新函数的扫描数的总数的变量
  ++num_total_;
  // 对当前位姿是否可更新进行判断
  /*
  首先判断num_total_是否大于1，
  因为num_total_等于1时表明地图还没有更新过，那么当前扫描当然是要对地图进行更新了;

  其次是判断当前时间与上一次更新时间之间的时间差，
  cartographer默认是每隔一段时间就要对子图进行强制更新的，这与hectorSLAM不同，
  如果你在使用cartographer中觉得这个功能不好，可以自行隐藏这一段代码；

  最后就是对当前位姿与上一更新时刻的位姿进行比较，
  当位姿差满足要求是，也是要进行更新的。

  参数的设置要根据机器人本体运行的速度和雷达的性能来的，大部分情况下，默认参数都能很好的工作。
  */
  if (num_total_ > 1 &&
      time - last_time_ <= common::FromSeconds(options_.max_time_seconds()) &&
      (pose.translation() - last_pose_.translation()).norm() <=
          options_.max_distance_meters() &&
      // 包括正负
      transform::GetAngle(pose.inverse() * last_pose_) <=
          options_.max_angle_radians()) {
    //LOG(INFO) << transform::GetAngle(pose.inverse() * last_pose_) << std::endl;
    return true;
  }
  //LOG(INFO) << transform::GetAngle(pose.inverse()) << std::endl;
  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  return false;
}

}  // namespace mapping_3d
}  // namespace cartographer
