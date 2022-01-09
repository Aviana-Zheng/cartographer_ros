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

#include "cartographer/mapping_3d/scan_matching/fast_correlative_scan_matcher.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping_3d/scan_matching/precomputation_grid.h"
#include "cartographer/mapping_3d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

proto::FastCorrelativeScanMatcherOptions
CreateFastCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::FastCorrelativeScanMatcherOptions options;
  options.set_branch_and_bound_depth(
      parameter_dictionary->GetInt("branch_and_bound_depth"));
  options.set_full_resolution_depth(
      parameter_dictionary->GetInt("full_resolution_depth"));
  options.set_rotational_histogram_size(
      parameter_dictionary->GetInt("rotational_histogram_size"));
  options.set_min_rotational_score(
      parameter_dictionary->GetDouble("min_rotational_score"));
  options.set_linear_xy_search_window(
      parameter_dictionary->GetDouble("linear_xy_search_window"));
  options.set_linear_z_search_window(
      parameter_dictionary->GetDouble("linear_z_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  return options;
}

// 0是原始栅格，在0的基础上，对于depth 1, 其附近(包含自己)8个栅格的概率值，均和第一个栅格比大小，取两者最大值；
// 在depth  1的基础上， 对于depth 2, 其附近(包含自己)8个栅格的概率值，均和第一个栅格比大小，取两者最大值；
// 到full_resolution_depth时候， 需要将分辨率扩大两倍，即现在的一个栅格体积是上一个depth的8倍
// 扩大后的栅格其附近(包含自己)8个栅格的概率值，均和第一个栅格比大小，取两者最大值；
// 依次类推
class PrecomputationGridStack {
 public:
  PrecomputationGridStack(
      const HybridGrid& hybrid_grid,
      const proto::FastCorrelativeScanMatcherOptions& options) {
    CHECK_GE(options.branch_and_bound_depth(), 1);
    CHECK_GE(options.full_resolution_depth(), 1);
    precomputation_grids_.reserve(options.branch_and_bound_depth());
    precomputation_grids_.push_back(ConvertToPrecomputationGrid(hybrid_grid));
    Eigen::Array3i last_width = Eigen::Array3i::Ones();
    for (int depth = 1; depth != options.branch_and_bound_depth(); ++depth) {
      const bool half_resolution = depth >= options.full_resolution_depth();
      const Eigen::Array3i next_width = ((1 << depth) * Eigen::Array3i::Ones());
      // 每个分辨率栅格中的体素
      const int full_voxels_per_high_resolution_voxel =
          1 << std::max(0, depth - options.full_resolution_depth());
      // Eigen::Array3i每个元素都和这个标量相加
      // 计算每一个滑窗的范围
      const Eigen::Array3i shift =
          (next_width - last_width +
           (full_voxels_per_high_resolution_voxel - 1)) /
          full_voxels_per_high_resolution_voxel;
      precomputation_grids_.push_back(
          PrecomputeGrid(precomputation_grids_.back(), half_resolution, shift));
      last_width = next_width;
    }
  }

  const PrecomputationGrid& Get(int depth) const {
    return precomputation_grids_.at(depth);
  }

  int max_depth() const { return precomputation_grids_.size() - 1; }

 private:
  std::vector<PrecomputationGrid> precomputation_grids_;
};

FastCorrelativeScanMatcher::FastCorrelativeScanMatcher(
    const HybridGrid& hybrid_grid,
    const std::vector<mapping::TrajectoryNode>& nodes,
    const proto::FastCorrelativeScanMatcherOptions& options)
    : options_(options),
      resolution_(hybrid_grid.resolution()),
      width_in_voxels_(hybrid_grid.grid_size()),
      precomputation_grid_stack_(
          common::make_unique<PrecomputationGridStack>(hybrid_grid, options)),
      rotational_scan_matcher_(nodes, options_.rotational_histogram_size()) {}

FastCorrelativeScanMatcher::~FastCorrelativeScanMatcher() {}

bool FastCorrelativeScanMatcher::Match(
    const transform::Rigid3d& initial_pose_estimate,
    const sensor::PointCloud& coarse_point_cloud,
    const sensor::PointCloud& fine_point_cloud, const float min_score,
    float* const score, transform::Rigid3d* const pose_estimate) const {
  const SearchParameters search_parameters{
      common::RoundToInt(options_.linear_xy_search_window() / resolution_),
      common::RoundToInt(options_.linear_z_search_window() / resolution_),
      options_.angular_search_window()};
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   coarse_point_cloud, fine_point_cloud,
                                   min_score, score, pose_estimate);
}

bool FastCorrelativeScanMatcher::MatchFullSubmap(
    const Eigen::Quaterniond& gravity_alignment,
    const sensor::PointCloud& coarse_point_cloud,
    const sensor::PointCloud& fine_point_cloud, const float min_score,
    float* const score, transform::Rigid3d* const pose_estimate) const {
  // 作者首先使用IMU确定了重力方向，将两个匹配的3D scan 的概率网格的Z轴和重力方向对齐
  const transform::Rigid3d initial_pose_estimate(Eigen::Vector3d::Zero(),
                                                 gravity_alignment);
  float max_point_distance = 0.f;
  for (const Eigen::Vector3f& point : coarse_point_cloud) {
    max_point_distance = std::max(max_point_distance, point.norm());
  }
  // 栅格总长度的一半加上雷达点云的最远探测距离
  const int linear_window_size =
      (width_in_voxels_ + 1) / 2 +
      common::RoundToInt(max_point_distance / resolution_ + 0.5f);
  const SearchParameters search_parameters{linear_window_size,
                                           linear_window_size, M_PI};
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   coarse_point_cloud, fine_point_cloud,
                                   min_score, score, pose_estimate);
}

bool FastCorrelativeScanMatcher::MatchWithSearchParameters(
    const FastCorrelativeScanMatcher::SearchParameters& search_parameters,
    const transform::Rigid3d& initial_pose_estimate,
    const sensor::PointCloud& coarse_point_cloud,
    const sensor::PointCloud& fine_point_cloud, const float min_score,
    float* const score, transform::Rigid3d* const pose_estimate) const {
  CHECK_NOTNULL(score);
  CHECK_NOTNULL(pose_estimate);

  // 离散化点云，这里通过直方图匹配滤掉了一部分候选scan，而且每个离散Scan3D都是多分辨率下的网格索引；
  const std::vector<DiscreteScan> discrete_scans = GenerateDiscreteScans(
      search_parameters, coarse_point_cloud, fine_point_cloud,
      initial_pose_estimate.cast<float>());

  // 对离散scan在最低分辨率概率网格上进行离散化（xyz方向上），生成候选集合，并计算每个候选集合的平均
  // 得分，按照得分进行排序，返回经过排序的候选集合；
  const std::vector<Candidate> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(search_parameters, discrete_scans);

  //接下来进行分支界定，获得最后的姿态结果；
  const Candidate best_candidate = BranchAndBound(
      search_parameters, discrete_scans, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score);
  if (best_candidate.score > min_score) {
    *score = best_candidate.score;  // 最终得分；
    *pose_estimate =
        (transform::Rigid3f(
             resolution_ * best_candidate.offset.matrix().cast<float>(), 
             Eigen::Quaternionf::Identity()) *
         discrete_scans[best_candidate.scan_index].pose)
            .cast<double>();
    return true;
  }
  return false;
}

DiscreteScan FastCorrelativeScanMatcher::DiscretizeScan(
    const FastCorrelativeScanMatcher::SearchParameters& search_parameters,
    const sensor::PointCloud& point_cloud,
    const transform::Rigid3f& pose) const {
  // Contains a vector of discretized scans for each 'depth'.
  std::vector<std::vector<Eigen::Array3i>> cell_indices_per_depth;

  const PrecomputationGrid& original_grid = precomputation_grid_stack_->Get(0);
  std::vector<Eigen::Array3i> full_resolution_cell_indices;
  for (const Eigen::Vector3f& point :
       sensor::TransformPointCloud(point_cloud, pose)) {
    full_resolution_cell_indices.push_back(original_grid.GetCellIndex(point));
  }
  const int full_resolution_depth = std::min(options_.full_resolution_depth(),
                                             options_.branch_and_bound_depth());
  CHECK_GE(full_resolution_depth, 1);
  // full_resolution_depth个网格对应的点云索引是一样的
  for (int i = 0; i != full_resolution_depth; ++i) {
    cell_indices_per_depth.push_back(full_resolution_cell_indices);
  }
  const int low_resolution_depth =
      options_.branch_and_bound_depth() - full_resolution_depth;
  CHECK_GE(low_resolution_depth, 0);
  const Eigen::Array3i search_window_start(
      -search_parameters.linear_xy_window_size,
      -search_parameters.linear_xy_window_size,
      -search_parameters.linear_z_window_size);
  // 点云分辨率改变了
  for (int i = 0; i != low_resolution_depth; ++i) {
    const int reduction_exponent = i + 1;
    const Eigen::Array3i low_resolution_search_window_start(
        search_window_start[0] >> reduction_exponent,
        search_window_start[1] >> reduction_exponent,
        search_window_start[2] >> reduction_exponent);
    cell_indices_per_depth.emplace_back();
    for (const Eigen::Array3i& cell_index : full_resolution_cell_indices) {
      const Eigen::Array3i cell_at_start = cell_index + search_window_start;
      const Eigen::Array3i low_resolution_cell_at_start(
          cell_at_start[0] >> reduction_exponent,
          cell_at_start[1] >> reduction_exponent,
          cell_at_start[2] >> reduction_exponent);
      cell_indices_per_depth.back().push_back(
          low_resolution_cell_at_start - low_resolution_search_window_start);
    }
  }
  return DiscreteScan{pose, cell_indices_per_depth};
}

// 生成离散scans，其中计算角分辨率的方法论文中已给出，其推导方法如下，
// 假设当 r << dmax 时，三角形双边*似相等，由余弦公式可得：
std::vector<DiscreteScan> FastCorrelativeScanMatcher::GenerateDiscreteScans(
    const FastCorrelativeScanMatcher::SearchParameters& search_parameters,
    const sensor::PointCloud& coarse_point_cloud,
    const sensor::PointCloud& fine_point_cloud,
    const transform::Rigid3f& initial_pose) const {
  std::vector<DiscreteScan> result;
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  // 最高分辨率的三倍，这是最小值，防止dmax太小
  float max_scan_range = 3.f * resolution_;
  for (const Eigen::Vector3f& point : coarse_point_cloud) {
    // 点(x,y,z)的模值，正常情况下，必然会大于max_scan_range，即求dmax；
    const float range = point.norm();
    max_scan_range = std::max(range, max_scan_range);
  }

  // 1 - 0.01 = 0.99; 保证角度扰动比dmax计算的小一点；
  const float kSafetyMargin = 1.f - 1e-2f;
  // 使用余弦定理计算出角度step；
  const float angular_step_size =
      kSafetyMargin * std::acos(1.f - common::Pow2(resolution_) /
                                          (2.f * common::Pow2(max_scan_range)));
  const int angular_window_size = common::RoundToInt(
      search_parameters.angular_search_window / angular_step_size);
  // TODO(whess): Should there be a small search window for rotations around
  // x and y?
  // 以现在所在的初始值，绕Z轴正负方向分别旋转angular_step_size角度，共旋转angular_window_size次
  // 这里只有绕z轴旋转的角度；
  std::vector<float> angles;
  for (int rz = -angular_window_size; rz <= angular_window_size; ++rz) {
    angles.push_back(rz * angular_step_size);
  }
  // 计算两个统计直方图的相似度，将当前的统计直方图按照Z轴的角度不断旋转，进而求出相似度；
  const std::vector<float> scores = rotational_scan_matcher_.Match(
      sensor::TransformPointCloud(fine_point_cloud, initial_pose), angles);
  
  for (size_t i = 0; i != angles.size(); ++i) {
    if (scores[i] < options_.min_rotational_score()) {
      continue;
    }
    // 如果某个角度使得匹配结果大于阈值，说明这是一个候选位姿，
    // 因此生成位姿（位移+旋转），用以生成离散scan；
    const Eigen::Vector3f angle_axis(0.f, 0.f, angles[i]);
    // It's important to apply the 'angle_axis' rotation between the translation
    // and rotation of the 'initial_pose', so that the rotation is around the
    // origin of the range data, and yaw is in map frame.
    const transform::Rigid3f pose(
        initial_pose.translation(),
        transform::AngleAxisVectorToRotationQuaternion(angle_axis) *
            initial_pose.rotation());
    // 将点云在该pose下进行离散化
    result.push_back(
        DiscretizeScan(search_parameters, coarse_point_cloud, pose));
  }
  return result;
}

// 对离散scan在最低分辨率概率网格上进行离散化（xyz方向上），生成候选集合，
std::vector<Candidate>
FastCorrelativeScanMatcher::GenerateLowestResolutionCandidates(
    const FastCorrelativeScanMatcher::SearchParameters& search_parameters,
    const int num_discrete_scans  /* 角度切片数量 */) const {
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  const int num_lowest_resolution_linear_xy_candidates =
      (2 * search_parameters.linear_xy_window_size + linear_step_size) /
      linear_step_size;
  const int num_lowest_resolution_linear_z_candidates =
      (2 * search_parameters.linear_z_window_size + linear_step_size) /
      linear_step_size;
  // 候选位移数量
  const int num_candidates =
      num_discrete_scans *
      common::Power(num_lowest_resolution_linear_xy_candidates, 2) *
      num_lowest_resolution_linear_z_candidates;
  std::vector<Candidate> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != num_discrete_scans; ++scan_index) {
    for (int z = -search_parameters.linear_z_window_size;
         z <= search_parameters.linear_z_window_size; z += linear_step_size) {
      for (int y = -search_parameters.linear_xy_window_size;
           y <= search_parameters.linear_xy_window_size;
           y += linear_step_size) {
        for (int x = -search_parameters.linear_xy_window_size;
             x <= search_parameters.linear_xy_window_size;
             x += linear_step_size) {
          candidates.emplace_back(scan_index, Eigen::Array3i(x, y, z));
        }
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

void FastCorrelativeScanMatcher::ScoreCandidates(
    const int depth, const std::vector<DiscreteScan>& discrete_scans,
    std::vector<Candidate>* const candidates) const {
  const int reduction_exponent =
      std::max(0, depth - options_.full_resolution_depth() + 1);
  // 候选位姿
  for (Candidate& candidate : *candidates) {
    int sum = 0;
    const DiscreteScan& discrete_scan = discrete_scans[candidate.scan_index];
    const Eigen::Array3i offset(candidate.offset[0] >> reduction_exponent,
                                candidate.offset[1] >> reduction_exponent,
                                candidate.offset[2] >> reduction_exponent);
    CHECK_LT(depth, discrete_scan.cell_indices_per_depth.size());
    for (const Eigen::Array3i& cell_index :
         discrete_scan.cell_indices_per_depth[depth]) {
      // 低分辨率栅格上点云索引
      const Eigen::Array3i proposed_cell_index = cell_index + offset;
      sum += precomputation_grid_stack_->Get(depth).value(proposed_cell_index);
    }
    candidate.score = PrecomputationGrid::ToProbability(
        sum /
        static_cast<float>(discrete_scan.cell_indices_per_depth[depth].size()));
  }
  std::sort(candidates->begin(), candidates->end(), std::greater<Candidate>());
}

std::vector<Candidate>
FastCorrelativeScanMatcher::ComputeLowestResolutionCandidates(
    const FastCorrelativeScanMatcher::SearchParameters& search_parameters,
    const std::vector<DiscreteScan>& discrete_scans) const {
  std::vector<Candidate> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters,
                                         discrete_scans.size());
  ScoreCandidates(precomputation_grid_stack_->max_depth(), discrete_scans,
                  &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

Candidate FastCorrelativeScanMatcher::BranchAndBound(
    const FastCorrelativeScanMatcher::SearchParameters& search_parameters,
    const std::vector<DiscreteScan>& discrete_scans,
    const std::vector<Candidate>& candidates, // 最低分辨率栅格生成的候选位姿
    const int candidate_depth,
    float min_score) const {
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }

  Candidate best_high_resolution_candidate(0, Eigen::Array3i::Zero());
  best_high_resolution_candidate.score = min_score;
  for (const Candidate& candidate : candidates) {
    // 分支定界程序出口
    if (candidate.score <= min_score) {
      break;
    }
    std::vector<Candidate> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    for (int z : {0, half_width}) {
      if (candidate.offset.z() + z > search_parameters.linear_z_window_size) {
        break;
      }
      for (int y : {0, half_width}) {
        if (candidate.offset.y() + y >
            search_parameters.linear_xy_window_size) {
          break;
        }
        for (int x : {0, half_width}) {
          if (candidate.offset.x() + x >
              search_parameters.linear_xy_window_size) {
            break;
          }
          higher_resolution_candidates.emplace_back(
              candidate.scan_index, candidate.offset + Eigen::Array3i(x, y, z));
        }
      }
    }
    ScoreCandidates(candidate_depth - 1, discrete_scans,
                    &higher_resolution_candidates);
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(search_parameters, discrete_scans,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer
