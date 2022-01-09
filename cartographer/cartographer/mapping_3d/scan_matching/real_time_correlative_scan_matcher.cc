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

#include "cartographer/mapping_3d/scan_matching/real_time_correlative_scan_matcher.h"

#include <cmath>

#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

RealTimeCorrelativeScanMatcher::RealTimeCorrelativeScanMatcher(
    const mapping_2d::scan_matching::proto::
        RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

// 入口函数
float RealTimeCorrelativeScanMatcher::Match(
    const transform::Rigid3d& initial_pose_estimate,   //机器人初始位置
    const sensor::PointCloud& point_cloud,   //雷达点云
    const HybridGrid& hybrid_grid,   //概率栅格图
    transform::Rigid3d* pose_estimate   /*求得的最佳位姿*/  ) const {
  CHECK_NOTNULL(pose_estimate);
  float best_score = -1.f;
  // 生成所有候选解，并遍历所有的候选解(位姿增量)
  for (const transform::Rigid3f& transform : GenerateExhaustiveSearchTransforms(
           hybrid_grid.resolution(), point_cloud)) {
    // 生成候选解（需要乘以位姿）
    const transform::Rigid3f candidate =
        initial_pose_estimate.cast<float>() * transform;
    // 针对该候选解打分
    const float score = ScoreCandidate(
        hybrid_grid, sensor::TransformPointCloud(point_cloud, candidate),
        transform);
    // 最高分迭代
    if (score > best_score) {
      best_score = score;
      *pose_estimate = candidate.cast<double>();
    }
  }
  return best_score;
}

// 生成候选解增量
std::vector<transform::Rigid3f>
RealTimeCorrelativeScanMatcher::GenerateExhaustiveSearchTransforms(
    const float resolution, const sensor::PointCloud& point_cloud) const {
  std::vector<transform::Rigid3f> result;
  // 获取线搜索框搜索范围, 线性搜索步长=线性搜索距离（x,y,z方向）÷ 分辨率（每个格子的长度）
  const int linear_window_size =
      common::RoundToInt(options_.linear_search_window() / resolution);
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  // 得到最大的测距
  float max_scan_range = 3.f * resolution;
  for (const Eigen::Vector3f& point : point_cloud) {
    const float range = point.norm();
    max_scan_range = std::max(range, max_scan_range);
  }
  const float kSafetyMargin = 1.f - 1e-3f;
  // 采用余弦定理计算角度步长，计算方法和2D相同
  const float angular_step_size =
      kSafetyMargin * std::acos(1.f - common::Pow2(resolution) /
                                          (2.f * common::Pow2(max_scan_range)));
  // 计算角度搜索框搜索范围
  const int angular_window_size =
      common::RoundToInt(options_.angular_search_window() / angular_step_size);
  for (int z = -linear_window_size; z <= linear_window_size; ++z) {
    for (int y = -linear_window_size; y <= linear_window_size; ++y) {
      for (int x = -linear_window_size; x <= linear_window_size; ++x) {
        for (int rz = -angular_window_size; rz <= angular_window_size; ++rz) {
          for (int ry = -angular_window_size; ry <= angular_window_size; ++ry) {
            for (int rx = -angular_window_size; rx <= angular_window_size;
                 ++rx) {
              const Eigen::Vector3f angle_axis(rx * angular_step_size,
                                               ry * angular_step_size,
                                               rz * angular_step_size);
              // 生成候选解增量
              result.emplace_back(
                  Eigen::Vector3f(x * resolution, y * resolution,
                                  z * resolution),
                  transform::AngleAxisVectorToRotationQuaternion(angle_axis));
            }
          }
        }
      }
    }
  }
  return result;
}

// 打分
float RealTimeCorrelativeScanMatcher::ScoreCandidate(
    const HybridGrid& hybrid_grid,
    const sensor::PointCloud& transformed_point_cloud,
    const transform::Rigid3f& transform) const {
  float score = 0.f;
  // 获得每个point在概率栅格图hybrid_grid上面的概率score, 求和，
  for (const Eigen::Vector3f& point : transformed_point_cloud) {
    score += hybrid_grid.GetProbability(hybrid_grid.GetCellIndex(point));
  }
  // 得到平均分
  score /= static_cast<float>(transformed_point_cloud.size());
  const float angle = transform::GetAngle(transform);
  // 求得的分值score,要乘以对应的权重。候选位置的平移量translation，旋转量angle越大，
  // 说明越偏离初始值，可靠性越差，对应的权重就越小。
  score *=
      std::exp(-common::Pow2(transform.translation().norm() *
                                 options_.translation_delta_cost_weight() +
                             angle * options_.rotation_delta_cost_weight()));
  CHECK_GT(score, 0.f);
  return score;
}

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer
