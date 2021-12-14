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

// This is an implementation of the algorithm described in "Real-Time
// Correlative Scan Matching" by Olson.
//
// It is similar to the RealTimeCorrelativeScanMatcher but has a different
// trade-off: Scan matching is faster because more effort is put into the
// precomputation done for a given map. However, this map is immutable after
// construction.

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/scan_matching/correlative_scan_matcher.h"
#include "cartographer/mapping_2d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

proto::FastCorrelativeScanMatcherOptions
CreateFastCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// A precomputed grid that contains in each cell (x0, y0) the maximum
// probability in the width x width area defined by x0 <= x < x0 + width and
// y0 <= y < y0.
/* 一个预先计算的网格，在每个单元格 (x0, y0) 中包含由 x0 <= x < x0 + width 和 y0 <= y < y0
 * 定义的宽度 x 宽度区域中的最大概率。 
 */
// 来存储预算图的数据类型,只是一个容器而已
// Cartographer仍然采取了一种空间换时间的方法来进一步的提高搜索效率
// 实际的预算图是在PrecomputationGrid2D对象构建的时候计算的
class PrecomputationGrid {
 public:
  PrecomputationGrid(const ProbabilityGrid& probability_grid,
                     const CellLimits& limits, int width,  // width滑窗的宽度
                     std::vector<float>* reusable_intermediate_grid);

  // Returns a value between 0 and 255 to represent probabilities between
  // kMinProbability and kMaxProbability.
  int GetValue(const Eigen::Array2i& xy_index) const {
    // intermediate前width - 1个值无意义
    // intermediate取的是栅格右侧(width-1)个栅格和此栅格的最大值
    const Eigen::Array2i local_xy_index = xy_index - offset_;
    // The static_cast<unsigned> is for performance to check with 2 comparisons
    // xy_index.x() < offset_.x() || xy_index.y() < offset_.y() ||
    // local_xy_index.x() >= wide_limits_.num_x_cells ||
    // local_xy_index.y() >= wide_limits_.num_y_cells
    // instead of using 4 comparisons.
    if (static_cast<unsigned>(local_xy_index.x()) >=
            static_cast<unsigned>(wide_limits_.num_x_cells) ||
        static_cast<unsigned>(local_xy_index.y()) >=
            static_cast<unsigned>(wide_limits_.num_y_cells)) {
      return 0;
    }
    const int stride = wide_limits_.num_x_cells;
    return cells_[local_xy_index.x() + local_xy_index.y() * stride];
  }

  // Maps values from [0, 255] to [kMinProbability, kMaxProbability].
  static float ToProbability(float value) {
    return mapping::kMinProbability +
           value *
               ((mapping::kMaxProbability - mapping::kMinProbability) / 255.f);
  }

 private:
  uint8 ComputeCellValue(float probability) const;

  // Offset of the precomputation grid in relation to the 'probability_grid'
  // including the additional 'width' - 1 cells.
  // intermediate前width - 1个值无意义
  // intermediate取的是栅格右侧(width-1)个栅格和此栅格的最大值
  // offset_(-width + 1, -width + 1),
  const Eigen::Array2i offset_;

  // Size of the precomputation grid.
  const CellLimits wide_limits_;

  // Probabilites mapped to 0 to 255.
  std::vector<uint8> cells_;
};

class PrecomputationGridStack;

// An implementation of "Real-Time Correlative Scan Matching" by Olson.
class FastCorrelativeScanMatcher {
 public:
  // 构造函数，它有两个输入参数:
  // grid是子图的占用栅格
  // options则记录了各种配置项。 在它的成员构造列表中，完成了三个成员的构造。
  FastCorrelativeScanMatcher(
      const ProbabilityGrid& probability_grid,
      const proto::FastCorrelativeScanMatcherOptions& options);
  ~FastCorrelativeScanMatcher();

  FastCorrelativeScanMatcher(const FastCorrelativeScanMatcher&) = delete;
  FastCorrelativeScanMatcher& operator=(const FastCorrelativeScanMatcher&) =
      delete;

  // Aligns 'point_cloud' within the 'probability_grid' given an
  // 'initial_pose_estimate'. If a score above 'min_score' (excluding equality)
  // is possible, true is returned, and 'score' and 'pose_estimate' are updated
  // with the result.
  /* 5个输入参数， 
   * 其中initial_pose_estimate描述了初始的位姿估计；
   * point_cloud则是将要考察的路径节点下的激光点云数据；
   * min_score是一个搜索节点的最小得分， 也就是前文中提到的score_threshold；
   * 指针score和pose_estimate是两个输出参数，
   * 用于成功匹配后返回匹配度和位姿估计。
   */
  bool Match(const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, float min_score,
             float* score, transform::Rigid2d* pose_estimate) const;

  // Aligns 'point_cloud' within the full 'probability_grid', i.e., not
  // restricted to the configured search window. If a score above 'min_score'
  // (excluding equality) is possible, true is returned, and 'score' and
  // 'pose_estimate' are updated with the result.
  // 用于全地图匹配的接口函数
  /* 4个输入参数， 
   * point_cloud则是将要考察的路径节点下的激光点云数据；
   * min_score是一个搜索节点的最小得分， 也就是前文中提到的score_threshold；
   * 指针score和pose_estimate是两个输出参数，
   * 用于成功匹配后返回匹配度和位姿估计。
   * 全地图匹配也是调用函数MatchWithSearchParameters来实际完成扫描匹配的。
   * 所不同的是，它需要提供一个以子图中心为搜索起点，覆盖整个子图的搜索窗口
   */
  bool MatchFullSubmap(const sensor::PointCloud& point_cloud, float min_score,
                       float* score, transform::Rigid2d* pose_estimate) const;

 private:
  // The actual implementation of the scan matcher, called by Match() and
  // MatchFullSubmap() with appropriate 'initial_pose_estimate' and
  // 'search_parameters'.
  bool MatchWithSearchParameters(
      SearchParameters search_parameters,
      const transform::Rigid2d& initial_pose_estimate,
      const sensor::PointCloud& point_cloud, float min_score, float* score,
      transform::Rigid2d* pose_estimate) const;
  std::vector<Candidate> ComputeLowestResolutionCandidates(
      const std::vector<DiscreteScan>& discrete_scans,
      const SearchParameters& search_parameters) const;
  std::vector<Candidate> GenerateLowestResolutionCandidates(
      const SearchParameters& search_parameters) const;
  void ScoreCandidates(const PrecomputationGrid& precomputation_grid,
                       const std::vector<DiscreteScan>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate>* const candidates) const;
  Candidate BranchAndBound(const std::vector<DiscreteScan>& discrete_scans,
                           const SearchParameters& search_parameters,
                           const std::vector<Candidate>& candidates,
                           int candidate_depth, float min_score) const;

  // 关于闭环检测的扫描匹配器的各种配置
  const proto::FastCorrelativeScanMatcherOptions options_;
  // 子图的地图作用范围，我们已经在分析占用栅格的数据结构的时候， 简单了解了该数据结构的字段和作用。
  MapLimits limits_;
  // 预算图的存储结构，用于查询不同分支尺寸下的搜索节点上界。
  std::unique_ptr<PrecomputationGridStack> precomputation_grid_stack_;
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_
