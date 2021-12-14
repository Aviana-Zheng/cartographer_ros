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

#include "cartographer/mapping_2d/scan_matching/real_time_correlative_scan_matcher.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

proto::RealTimeCorrelativeScanMatcherOptions
CreateRealTimeCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::RealTimeCorrelativeScanMatcherOptions options;
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_translation_delta_cost_weight(
      parameter_dictionary->GetDouble("translation_delta_cost_weight"));
  options.set_rotation_delta_cost_weight(
      parameter_dictionary->GetDouble("rotation_delta_cost_weight"));
  CHECK_GE(options.translation_delta_cost_weight(), 0.);
  CHECK_GE(options.rotation_delta_cost_weight(), 0.);
  return options;
}

RealTimeCorrelativeScanMatcher::RealTimeCorrelativeScanMatcher(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

// 生成所有候选位置
std::vector<Candidate>
RealTimeCorrelativeScanMatcher::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
  int num_candidates = 0;
  // 遍历所有的点云，计算候选的点数总量
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    // 获取x方向offset数量
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);
    // 获取y方向offset数量
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);
    // x*y为offset总量
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  std::vector<Candidate> candidates;
  candidates.reserve(num_candidates);
  // 遍历所有的点云
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    // 生成candidates，一个offset（一个新的位姿）对应一个点云，
    // 所以有点云数量*x方向offset数量*y方向offset数量个可能解
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

// 匹配主函数
double RealTimeCorrelativeScanMatcher::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud,
    const ProbabilityGrid& probability_grid,
    transform::Rigid2d* pose_estimate) const {
  // 存放匹配结果的pose_estimate不能为空指针
  CHECK_NOTNULL(pose_estimate);
  // 点云旋转
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  // 构建搜索框参数
  const SearchParameters search_parameters(
      options_.linear_search_window(), options_.angular_search_window(),
      rotated_point_cloud, probability_grid.limits().resolution());
  
  // 切片旋转,按照搜索角度/步长，以初始值为基础，生成一些列的旋转后的点云，按顺序存放在vector里
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  // 将旋转点云，从旋转点云坐标中，转化到地图坐标中去。根据坐标，获得 cell index
  const std::vector<DiscreteScan> discrete_scans = DiscretizeScans(
      probability_grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  // 生成所有候选位置
  std::vector<Candidate> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);
  // 对每个候选位置进行打分
  ScoreCandidates(probability_grid, discrete_scans, search_parameters,
                  &candidates);

  // 找到score最高的候选位置，就为所求位置。
  const Candidate& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  *pose_estimate = transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
  return best_candidate.score;
}

// 运用暴力匹配的方法,对每个候选位置进行打分
void RealTimeCorrelativeScanMatcher::ScoreCandidates(
    const ProbabilityGrid& probability_grid,
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate>* const candidates) const {
  
  // 遍历所有的candidates可能解
  for (Candidate& candidate : *candidates) {
    candidate.score = 0.f;
    // 遍历点云中所有的点
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      // 计算累积占据概率
      const float probability =
          probability_grid.GetProbability(proposed_xy_index);
      candidate.score += probability;
    }
    // 计算平均概率
    candidate.score /=
        static_cast<float>(discrete_scans[candidate.scan_index].size());
    // std::hypot：计算两点之间的距离
    // 最终的score=平均score*exp^(-x^2):x=距离*权重+角度*权重
    candidate.score *=   //权重处理
        std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) *
                                   options_.translation_delta_cost_weight() +
                               std::abs(candidate.orientation) *
                                   options_.rotation_delta_cost_weight()));
    // C++中的hypot()函数返回所传递的参数平方和的平方根,h = sqrt(x2+y2)
    CHECK_GT(candidate.score, 0.f);
  }
}

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer
