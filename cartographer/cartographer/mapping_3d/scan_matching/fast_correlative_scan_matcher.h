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

// This is an implementation of a 3D branch-and-bound algorithm similar to
// mapping_2d::FastCorrelativeScanMatcher.

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/mapping_3d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h"
#include "cartographer/mapping_3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

/*
    这个方法在3D匹配中主要使用在回环检测中，使用的同样是分枝定界的方法寻找最优值。

    在计算过程中，作者首先使用IMU确定了重力方向，将两个匹配的3D scan 的概率网格的Z轴和重力方向对齐，
    这样匹配过程中就可以仅仅考虑偏航（yaw）方向的搜索窗口尺度问题；
    然后作者在Z轴方向上做了切片处理，沿Z轴垂直的xoy*面做切片，然后统计每个切片内的点的分布情况，
    最后将所有切片内点的分布情况映射成一个直方图，最终的scan匹配度通过两个直方图的余弦距离衡量，
    下面稍微详细的分析一下，其中的关键是如何统计切片内的点的分布情况，以及如何映射成一个直方图，
    它需要确保两个相似的场景映射成的直方图类似，而扫描两个相似场景的位姿（位移+旋转）却不一定相似。
 */
proto::FastCorrelativeScanMatcherOptions
CreateFastCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* parameter_dictionary);

class PrecomputationGridStack;

struct DiscreteScan {
  transform::Rigid3f pose;
  // Contains a vector of discretized scans for each 'depth'.
  std::vector<std::vector<Eigen::Array3i>> cell_indices_per_depth;
};

struct Candidate {
  // scan_index表示角度切片的角度值 offset 是位移值
  Candidate(const int scan_index, const Eigen::Array3i& offset)
      : scan_index(scan_index), offset(offset) {}

  // Index into the discrete scans vectors.
  int scan_index;

  // Linear offset from the initial pose in cell indices. For lower resolution
  // candidates this is the lowest offset of the 2^depth x 2^depth x 2^depth
  // block of possibilities.
  Eigen::Array3i offset;

  // Score, higher is better.
  float score = 0.f;

  bool operator<(const Candidate& other) const { return score < other.score; }
  bool operator>(const Candidate& other) const { return score > other.score; }
};

class FastCorrelativeScanMatcher {
 public:
  FastCorrelativeScanMatcher(
      const HybridGrid& hybrid_grid,
      const std::vector<mapping::TrajectoryNode>& nodes,
      const proto::FastCorrelativeScanMatcherOptions& options);
  ~FastCorrelativeScanMatcher();

  FastCorrelativeScanMatcher(const FastCorrelativeScanMatcher&) = delete;
  FastCorrelativeScanMatcher& operator=(const FastCorrelativeScanMatcher&) =
      delete;

  // Aligns 'coarse_point_cloud' within the 'hybrid_grid' given an
  // 'initial_pose_estimate'. If a score above 'min_score' (excluding equality)
  // is possible, true is returned, and 'score' and 'pose_estimate' are updated
  // with the result. 'fine_point_cloud' is used to compute the rotational scan
  // matcher score.
  bool Match(const transform::Rigid3d& initial_pose_estimate,
             const sensor::PointCloud& coarse_point_cloud,
             const sensor::PointCloud& fine_point_cloud, float min_score,
             float* score, transform::Rigid3d* pose_estimate) const;

  // Aligns 'coarse_point_cloud' within the 'hybrid_grid' given a rotation which
  // is expected to be approximately gravity aligned. If a score above
  // 'min_score' (excluding equality) is possible, true is returned, and 'score'
  // and 'pose_estimate' are updated with the result. 'fine_point_cloud' is used
  // to compute the rotational scan matcher score.
  bool MatchFullSubmap(const Eigen::Quaterniond& gravity_alignment,
                       const sensor::PointCloud& coarse_point_cloud,
                       const sensor::PointCloud& fine_point_cloud,
                       float min_score, float* score,
                       transform::Rigid3d* pose_estimate) const;

 private:
  struct SearchParameters {
    const int linear_xy_window_size;     // voxels
    const int linear_z_window_size;      // voxels
    const double angular_search_window;  // radians
  };

  bool MatchWithSearchParameters(
      const SearchParameters& search_parameters,
      const transform::Rigid3d& initial_pose_estimate,
      const sensor::PointCloud& coarse_point_cloud,
      const sensor::PointCloud& fine_point_cloud, float min_score, float* score,
      transform::Rigid3d* pose_estimate) const;
  DiscreteScan DiscretizeScan(const SearchParameters& search_parameters,
                              const sensor::PointCloud& point_cloud,
                              const transform::Rigid3f& pose) const;
  std::vector<DiscreteScan> GenerateDiscreteScans(
      const SearchParameters& search_parameters,
      const sensor::PointCloud& coarse_point_cloud,
      const sensor::PointCloud& fine_point_cloud,
      const transform::Rigid3f& initial_pose) const;
  std::vector<Candidate> GenerateLowestResolutionCandidates(
      const SearchParameters& search_parameters, int num_discrete_scans) const;
  void ScoreCandidates(int depth,
                       const std::vector<DiscreteScan>& discrete_scans,
                       std::vector<Candidate>* const candidates) const;
  std::vector<Candidate> ComputeLowestResolutionCandidates(
      const SearchParameters& search_parameters,
      const std::vector<DiscreteScan>& discrete_scans) const;
  Candidate BranchAndBound(const SearchParameters& search_parameters,
                           const std::vector<DiscreteScan>& discrete_scans,
                           const std::vector<Candidate>& candidates,
                           int candidate_depth, float min_score) const;

  const proto::FastCorrelativeScanMatcherOptions options_;
  const float resolution_;
  const int width_in_voxels_;
  std::unique_ptr<PrecomputationGridStack> precomputation_grid_stack_;
  RotationalScanMatcher rotational_scan_matcher_;
};

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_
