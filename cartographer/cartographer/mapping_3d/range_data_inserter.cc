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

#include "cartographer/mapping_3d/range_data_inserter.h"

#include "Eigen/Core"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {

namespace {

void InsertMissesIntoGrid(const std::vector<uint16>& miss_table,
                          const Eigen::Vector3f& origin,
                          const sensor::PointCloud& returns,
                          HybridGrid* hybrid_grid,
                          const int num_free_space_voxels) {
  // 计算起点cell的index
  const Eigen::Array3i origin_cell = hybrid_grid->GetCellIndex(origin);
  for (const Eigen::Vector3f& hit : returns) {
    // 计算终点cell的index
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);

    // 计算起点和终点cell的向量
    const Eigen::Array3i delta = hit_cell - origin_cell;
    // 计算起点到终点之间的最大值，cwiseAbs()是求元素的绝对值,maxCoeff()是求元素中的最大值。
    const int num_samples = delta.cwiseAbs().maxCoeff();
    CHECK_LT(num_samples, 1 << 15);
    // 'num_samples' is the number of samples(样本) we equi-distantly(等距) place on the
    // line between 'origin' and 'hit'. (including a fractional(小数) part for sub-
    // voxels(子体素)) It is chosen so that between two samples we change from one voxel
    // to the next on the fastest changing dimension. 我们在变化最快的维度上从一个体素更改为下一个
    //
    // Only the last 'num_free_space_voxels' are updated for performance.
    // 仅更新最后的“num_free_space_voxels”以提高性能。 
    // 沿着向量，寻找miss的cell，并更新cell的miss概率
    for (int position = std::max(0, num_samples - num_free_space_voxels);
         position < num_samples; ++position) {
      const Eigen::Array3i miss_cell =
          origin_cell + delta * position / num_samples;
      hybrid_grid->ApplyLookupTable(miss_cell, miss_table);
    }
  }
}

}  // namespace

proto::RangeDataInserterOptions CreateRangeDataInserterOptions(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::RangeDataInserterOptions options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_num_free_space_voxels(
      parameter_dictionary->GetInt("num_free_space_voxels"));
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

// 在构造函数里同样计算了两个表hit_table和miss_table，与2D不一样的是，
// 该表里存放的是点的hit概率，而2D算的是free概率
RangeDataInserter::RangeDataInserter(
    const proto::RangeDataInserterOptions& options)
    : options_(options),
      hit_table_(mapping::ComputeLookupTableToApplyOdds(
          mapping::Odds(options_.hit_probability()))),
      miss_table_(mapping::ComputeLookupTableToApplyOdds(
          mapping::Odds(options_.miss_probability()))) {}

// 插入点云数据
void RangeDataInserter::Insert(const sensor::RangeData& range_data,
                               HybridGrid* hybrid_grid) const {
  CHECK_NOTNULL(hybrid_grid)->StartUpdate();

  for (const Eigen::Vector3f& hit : range_data.returns) {
    // 找到Index
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);
    // 更新cell的hit概率
    hybrid_grid->ApplyLookupTable(hit_cell, hit_table_);
  }

  // By not starting a new update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  // 通过在插入命中后不开始新的更新，我们赋予命中优先级（即不会因为同一单元格中的未命中而忽略任何命中）。
  // 相当于2D的ray_casting，计算miss光线经过的cell，并更新cell的miss概率 
  InsertMissesIntoGrid(miss_table_, range_data.origin, range_data.returns,
                       hybrid_grid, options_.num_free_space_voxels());
}

}  // namespace mapping_3d
}  // namespace cartographer
