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

#include "cartographer/transform/transform_interpolation_buffer.h"

#include <algorithm>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace transform {

void TransformInterpolationBuffer::Push(const common::Time time,
                                        const transform::Rigid3d& transform) {
  if (deque_.size() > 0) {
    CHECK_GE(time, latest_time()) << "New transform is older than latest.";
  }
  deque_.push_back(TimestampedTransform{time, transform});
}

bool TransformInterpolationBuffer::Has(const common::Time time) const {
  if (deque_.empty()) {
    return false;
  }
  return earliest_time() <= time && time <= latest_time();
}

transform::Rigid3d TransformInterpolationBuffer::Lookup(
    const common::Time time) const {
  CHECK(Has(time)) << "Missing transform for: " << time;
  // 这两个函数内部使用了二分查找，所以必须用在有序的区间上
  // std::lower_bound() 是在区间内找到第一个大于等于 value 的值的位置并返回，如果没找到就返回 end() 位置。
  // 而 std::upper_bound() 是找到第一个大于 value 值的位置并返回，如果找不到同样返回 end() 位置。
  auto start =
      std::lower_bound(deque_.begin(), deque_.end(), time,
                       [](const TimestampedTransform& timestamped_transform,
                          const common::Time time) {
                         return timestamped_transform.time < time;
                       });
  // 判断time是否位于端点处
  auto end = start;
  if (end->time == time) {
    return end->transform;
  }
  --start;
  if (start->time == time) {
    return start->transform;
  }
  // 如果time不在端点处，则进行插值
  const double duration = common::ToSeconds(end->time - start->time);
  const double factor = common::ToSeconds(time - start->time) / duration;
  const Eigen::Vector3d origin =
      start->transform.translation() +
      (end->transform.translation() - start->transform.translation()) * factor;
  // 可以使用被称为球面线性插值（Slerp Algorithm）的方法对四元数进行插值运算，
  // 从而解决了平滑旋转的插值问题
  const Eigen::Quaterniond rotation =
      Eigen::Quaterniond(start->transform.rotation())
          .slerp(factor, Eigen::Quaterniond(end->transform.rotation()));
  return transform::Rigid3d(origin, rotation);
}

common::Time TransformInterpolationBuffer::earliest_time() const {
  CHECK(!empty()) << "Empty buffer.";
  return deque_.front().time;
}

common::Time TransformInterpolationBuffer::latest_time() const {
  CHECK(!empty()) << "Empty buffer.";
  return deque_.back().time;
}

bool TransformInterpolationBuffer::empty() const { return deque_.empty(); }

std::unique_ptr<TransformInterpolationBuffer>
TransformInterpolationBuffer::FromTrajectory(
    const mapping::proto::Trajectory& trajectory) {
  auto interpolation_buffer =
      common::make_unique<TransformInterpolationBuffer>();
  for (const mapping::proto::Trajectory::Node& node : trajectory.node()) {
    interpolation_buffer->Push(common::FromUniversal(node.timestamp()),
                               transform::ToRigid3(node.pose()));
  }
  return interpolation_buffer;
}

}  // namespace transform
}  // namespace cartographer
