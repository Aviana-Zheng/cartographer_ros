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

#include "cartographer/mapping_2d/scan_matching/fast_correlative_scan_matcher.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

namespace {

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved(检索).
// All of it in (amortized) O(1).  所有这些都在（摊销）O(1) 中。 
class SlidingWindowMaximum {
 public:
  // 添加值，会将小于填入值的其他值删掉，并将这个值放到最后。
  void AddValue(const float value) {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }
  
  // 删除值，如果第一个值等于要删除的这个值，则将这个值删掉。
  void RemoveValue(const float value) {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  // 获取最大值，因为是按照顺序存储的，第一个值是最大的。
  float GetMaximum() const {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurence, and so on.
  std::deque<float> non_ascending_maxima_;
};

}  // namespace

proto::FastCorrelativeScanMatcherOptions
CreateFastCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::FastCorrelativeScanMatcherOptions options;
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_branch_and_bound_depth(
      parameter_dictionary->GetInt("branch_and_bound_depth"));
  return options;
}

PrecomputationGrid::PrecomputationGrid(  // width  滑窗的宽度
    const ProbabilityGrid& probability_grid, const CellLimits& limits,
    const int width, std::vector<float>* reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells) {
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);
  // 列数
  const int stride = wide_limits_.num_x_cells;
  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + width.
  // 首先，我们计算在 x0 <= x < x0 + width定义的跨度中实现的每个 (x0, y) 的最大概率。 
  std::vector<float>& intermediate = *reusable_intermediate_grid;
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);

  // 实现滑窗从网格最左侧开始滑入，每次运动一个栅格，并最后从右侧滑出。
  // 实现了将width个栅格合一的操作.
  // 之后再将y++，实现了整个原始网格的遍历
  for (int y = 0; y != limits.num_y_cells; ++y) {
    SlidingWindowMaximum current_values;
    // 获取 grid 的x坐标的索引: 首先获取 (0, y)
    current_values.AddValue(
        probability_grid.GetProbability(Eigen::Array2i(0, y)));
    // 步骤一 滑窗开始滑入网格, intermediate前width - 1个值无意义，
    // 即intermediate取的是栅格右侧(width-1)个栅格和此栅格的最大值
    // 首先将滑窗中最大的值放入 intermediate的对应位置处
    // 之后向滑窗的最右侧填入一个值, 这部分只添加值
    for (int x = -width + 1; x != 0; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      if (x + width < limits.num_x_cells) {
        current_values.AddValue(
            probability_grid.GetProbability(Eigen::Array2i(x + width, y)));
      }
    }
    // 步骤二  滑窗已经完全滑入网格中
    // 首先将滑窗中最大的值放入 intermediate的对应位置处 
    // 之后先删除滑窗的最左侧处的值，之后再向滑窗的最右侧填入一个值
    for (int x = 0; x < limits.num_x_cells - width; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          probability_grid.GetProbability(Eigen::Array2i(x, y)));
      current_values.AddValue(
          probability_grid.GetProbability(Eigen::Array2i(x + width, y)));
    }
    // 步骤三  滑窗滑出网格
    // 此时，首先将滑窗中最大的值放入 intermediate的对应位置处，
    // 之后逐步删除滑窗的最左侧处的值。
    // 此时会导致 intermediate 的宽度比 grid 宽 width-1 个栅格
    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          probability_grid.GetProbability(Eigen::Array2i(x, y)));
    }
    // 理论上，滑窗走完地图的一行之后应该是空的，经过 只入，一出一入，只出，3个步骤
    current_values.CheckIsEmpty();
    // 之后再将y++，实现了整个原始网格的遍历
  }
  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.
  /*
   * 首先，x的遍历范围为 intermediate 的宽，比 grid 的网格数多width-1。
   * 之后，进行Y方向的遍历，y方向的滑窗遍历不再去读取原始grid了，直接去读取 intermediate ，
   * 所以他是可重用的。
   * 由于使用滑窗的方式，所以 cells的 高会比 intermediate 的高 多 width - 1。
   */
  // 滑窗竖着走一遍，只不过这次添加到滑窗里的值是 intermediate
  // 所以intermediate 是 可重用的：reusable_intermediate_grid
  for (int x = 0; x != wide_limits_.num_x_cells; ++x) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(intermediate[x]);
    // 逐步的将 滑窗的值 放入 cells 变量，也就是多分辨率网格的存储的真实位置。
    for (int y = -width + 1; y != 0; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      if (y + width < limits.num_y_cells) {
        current_values.AddValue(intermediate[x + (y + width) * stride]);
      }
    }
    for (int y = 0; y < limits.num_y_cells - width; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + width) * stride]);
    }
    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y) {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }
    current_values.CheckIsEmpty();
  }
}

uint8 PrecomputationGrid::ComputeCellValue(const float probability) const {
  const int cell_value = common::RoundToInt(
      (probability - mapping::kMinProbability) *
      (255.f / (mapping::kMaxProbability - mapping::kMinProbability)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

class PrecomputationGridStack {
 public:
  PrecomputationGridStack(
      const ProbabilityGrid& probability_grid,
      const proto::FastCorrelativeScanMatcherOptions& options) {
    CHECK_GE(options.branch_and_bound_depth(), 1);
    // 根据配置文件中 branch_and_bound_depth 的值，默认为7，确定 最大的分辨率，
    // 也就是64个栅格合成一个格子。
    const int max_width = 1 << (options.branch_and_bound_depth() - 1);
    // 7个不同分辨率的地图
    precomputation_grids_.reserve(options.branch_and_bound_depth());
    std::vector<float> reusable_intermediate_grid;
    const CellLimits limits = probability_grid.limits().cell_limits();
    // 经过滑窗后产生的栅格地图会变宽，比原地图多了width-1个格子
    reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                       limits.num_y_cells);
    // emplace_back会生成一个临时的变量，会调用PrecomputationGrid2D的构造函数
    // 分辨率逐渐变大，i=0时就是默认分辨率（0.05），i=6时，width=64,也就是64个格子合成一个值
    for (int i = 0; i != options.branch_and_bound_depth(); ++i) {
      const int width = 1 << i;
      precomputation_grids_.emplace_back(probability_grid, limits, width,
                                         &reusable_intermediate_grid);
    }
  }

  const PrecomputationGrid& Get(int index) {
    return precomputation_grids_[index];
  }

  int max_depth() const { return precomputation_grids_.size() - 1; }

 private:
  std::vector<PrecomputationGrid> precomputation_grids_;
};

FastCorrelativeScanMatcher::FastCorrelativeScanMatcher(
    const ProbabilityGrid& probability_grid,
    const proto::FastCorrelativeScanMatcherOptions& options)
    : options_(options),
      limits_(probability_grid.limits()),
      precomputation_grid_stack_(
          new PrecomputationGridStack(probability_grid, options)) {}

FastCorrelativeScanMatcher::~FastCorrelativeScanMatcher() {}

/* 5个输入参数， 
  * 其中initial_pose_estimate描述了初始的位姿估计；
  * point_cloud则是将要考察的路径节点下的激光点云数据；
  * min_score是一个搜索节点的最小得分， 也就是前文中提到的score_threshold；
  * 指针score和pose_estimate是两个输出参数，
  * 用于成功匹配后返回匹配度和位姿估计。
  */
bool FastCorrelativeScanMatcher::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  const SearchParameters search_parameters(options_.linear_search_window(),
                                           options_.angular_search_window(),
                                           point_cloud, limits_.resolution());
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

// 用于全地图匹配的接口函数
/* 4个输入参数， 
  * point_cloud则是将要考察的路径节点下的激光点云数据；
  * min_score是一个搜索节点的最小得分， 也就是前文中提到的score_threshold；
  * 指针score和pose_estimate是两个输出参数，
  * 用于成功匹配后返回匹配度和位姿估计。
  * 全地图匹配也是调用函数MatchWithSearchParameters来实际完成扫描匹配的。
  * 所不同的是，它需要提供一个以子图中心为搜索起点，覆盖整个子图的搜索窗口
  */
bool FastCorrelativeScanMatcher::MatchFullSubmap(
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // Compute a search window around the center of the submap that includes it
  // fully.
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // Linear search window, 1e6 cells/direction.
      M_PI,  // Angular search window, 180 degrees in both directions.
      point_cloud, limits_.resolution());
  const transform::Rigid2d center = transform::Rigid2d::Translation(
      limits_.max() - 0.5 * limits_.resolution() *
                          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                                          limits_.cell_limits().num_x_cells));
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate);
}

// 完成扫描匹配，进而实现闭环检测
bool FastCorrelativeScanMatcher::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // 检查一下指针score和pose_estimate非空 在后续的计算过程中将要修改这两个指针所指对象的数据内容
  CHECK_NOTNULL(score);
  CHECK_NOTNULL(pose_estimate);

  // 获取初始位姿估计的方向角，将激光点云中的点都绕Z轴转动相应的角度得到rotated_point_cloud
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  
  // 调用定义在correlative_scan_matcher_2d.cc中的函数GenerateRotatedScans获得搜索窗口下
  // 机器人朝向各个方向角时的点云数据。
  // 容器rotated_scans中保存了姿态为ε0+(0,0,δθjθ),jθ ∈ Z,−wθ≤jθ≤wθ时所有的点云数据， 
  // 其中ε0就是这里的初始位姿估计initial_pose_estimate，δθ则是角度搜索步长，
  // 它由对象search_parameters描述
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  /* 通过同样定义在correlative_scan_matcher_2d.cc中的函数DiscretizeScans， 
   * 完成对旋转后的点云数据离散化的操作，即将浮点类型的点云数据转换成整型的栅格单元索引。 
   */
  const std::vector<DiscreteScan> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  // 尽可能的缩小搜索窗口的大小，以减小搜索空间，提高搜索效率
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());

  // 首先通过函数ComputeLowestResolutionCandidates完成对搜索空间的第一次分割，
  // 得到初始子空间节点集合{C0}。 该函数在最低分辨率的栅格地图上查表得到各个搜索节点c∈{C0}的上界，
  // 并降序排列。
  const std::vector<Candidate> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
  // 最后调用函数BranchAndBound完成分支定界搜索，搜索的结果将被保存在best_candidate中
  const Candidate best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score);
  // 检查最优解的值，如果大于指定阈值min_score就认为匹配成功，
  // 修改输入参数指针score和pose_estimate所指的对象。否则认为不匹配，不存在闭环，直接返回。
  if (best_candidate.score > min_score) {
    *score = best_candidate.score;
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    return true;
  }
  return false;
}

// 以离散化之后的各搜索方向上的点云数据和搜索配置为输入
std::vector<Candidate>
FastCorrelativeScanMatcher::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters) const {
  // 调用函数GenerateLowestResolutionCandidates完成对搜索空间的初始分割
  std::vector<Candidate> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);
  // 通过函数ScoreCandidates计算各个候选点的评分并排序
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

// 根据搜索配置来完成初始分割
std::vector<Candidate>
FastCorrelativeScanMatcher::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  // 首先根据预算图的金字塔高度计算初始分割的粒度2^h0
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  int num_candidates = 0;
  // 遍历所有搜索方向，累计各个方向下空间的分割数量，
  // 得到num_candidates描述了搜索空间初始分割的子空间数量。
  // num_scans = 2 * num_angular_perturbations + 1;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    // 笛卡尔乘积，组合数量
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  std::vector<Candidate> candidates;
  candidates.reserve(num_candidates);
  // 一个三层的嵌套for循环中，构建各个候选点。最后检查一下候选点数量，通过后返回
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

/*
 * 有四个参数
 * precomputation_grid是将要查询的预算图
 * discrete_scans则是离散化的各搜索角度下的激光点云，
 * search_parameters则是搜索配置， 
 * candidates则是候选点集合将在本函数中
 * 计算得分并排序
 */
void FastCorrelativeScanMatcher::ScoreCandidates(
    const PrecomputationGrid& precomputation_grid,
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate>* const candidates) const {
  for (Candidate& candidate : *candidates) {
    int sum = 0;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      sum += precomputation_grid.GetValue(proposed_xy_index);
    }
    candidate.score = PrecomputationGrid::ToProbability(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
  }
  std::sort(candidates->begin(), candidates->end(), std::greater<Candidate>());
}

Candidate FastCorrelativeScanMatcher::BranchAndBound(
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate>& candidates, const int candidate_depth,
    float min_score) const {
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }

  Candidate best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;
  for (const Candidate& candidate : candidates) {
    if (candidate.score <= min_score) {
      // 跳出循环
      break;
    }
    std::vector<Candidate> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    for (int x_offset : {0, half_width}) {
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;  //不能超出x边界
      }
      for (int y_offset : {0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;  //不能超出y边界
        }
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer
