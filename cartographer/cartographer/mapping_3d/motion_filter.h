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

#ifndef CARTOGRAPHER_MAPPING_3D_MOTION_FILTER_H_
#define CARTOGRAPHER_MAPPING_3D_MOTION_FILTER_H_

#include <limits>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping_3d/proto/motion_filter_options.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_3d {

/*
使用场景:
在local_trajectory_build_2d中，AddRangeData函数调用AddAccumulatedRangeData函数来
扫描匹配并插入扫描数据，AddAccumulatedRangeData函数在扫描匹配完成后，
会调用InsertIntoSubmap函数将扫描数据插入子图，而InsertIntoSubmap函数第一步就会
调用motion_filter_.IsSimilar(time, pose_estimate)来判断当前扫描是否能插入子图中
*/
proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// Takes poses as input and filters them to get fewer poses.
class MotionFilter {
 public:
  // 完成对参数的配置
  explicit MotionFilter(const proto::MotionFilterOptions& options);

  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  // 如果累积运动（线性、旋转或时间）高于阈值，则返回 false。 否则累积相对运动并返回 true。
  // 判断当前位姿与上一时刻更新时的位姿是否有足够的空间和时间距离
  bool IsSimilar(common::Time time, const transform::Rigid3d& pose);

 private:
  const proto::MotionFilterOptions options_; // MotionFilter相关的参数配置
  int num_total_ = 0;  // 当前轨迹中进入判断的所有扫描数
  int num_different_ = 0;   // 参与更新的扫描数
  common::Time last_time_;  // 上一次更新的时间
  transform::Rigid3d last_pose_;   // 上一次更新的位姿
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_MOTION_FILTER_H_
