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

#include "cartographer/mapping_3d/scan_matching/precomputation_grid.h"

#include <algorithm>

#include "Eigen/Core"
#include "cartographer/common/math.h"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

namespace {

// C++11 defines that integer division(除法) rounds(取整) towards zero. For index math, we
// actually need it to round towards negative infinity(负无穷大). Luckily bit shifts have
// that property.
inline int DivideByTwoRoundingTowardsNegativeInfinity(const int value) {
  return value >> 1;
}

// Computes the half resolution index corresponding to the full resolution
// 'cell_index'.
Eigen::Array3i CellIndexAtHalfResolution(const Eigen::Array3i& cell_index) {
  return Eigen::Array3i(
      DivideByTwoRoundingTowardsNegativeInfinity(cell_index[0]),
      DivideByTwoRoundingTowardsNegativeInfinity(cell_index[1]),
      DivideByTwoRoundingTowardsNegativeInfinity(cell_index[2]));
}

}  // namespace

// 将概率值由 15 bit [1, 32767] range 转换为  8 bit  [0, 255]
PrecomputationGrid ConvertToPrecomputationGrid(const HybridGrid& hybrid_grid) {
  PrecomputationGrid result(hybrid_grid.resolution());
  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    const int cell_value = common::RoundToInt(
        (mapping::ValueToProbability(it.GetValue()) -
         mapping::kMinProbability) *
        (255.f / (mapping::kMaxProbability - mapping::kMinProbability)));
    CHECK_GE(cell_value, 0);
    CHECK_LE(cell_value, 255);
    *result.mutable_value(it.GetCellIndex()) = cell_value;
  }
  return result;
}

PrecomputationGrid PrecomputeGrid(const PrecomputationGrid& grid,
                                  const bool half_resolution,
                                  const Eigen::Array3i& shift) {
  PrecomputationGrid result(grid.resolution());
  // 其附近(包含自己)8个栅格的概率值，均和第一个栅格比大小，取两者最大值，类似滑窗搜索
  for (auto it = PrecomputationGrid::Iterator(grid); !it.Done(); it.Next()) {
    for (int i = 0; i != 8; ++i) {
      // We use this value to update 8 values in the resulting grid, at
      // position (x - {0, 'shift'}, y - {0, 'shift'}, z - {0, 'shift'}).
      // If 'shift' is 2 ** (depth - 1), where depth 0 is the original grid,
      // this results in precomputation grids analogous(类似于) to the 2D case.
      // shift * PrecomputationGrid::GetOctant(i)  2个Eigen::Array3i，对应的元素相乘
      const Eigen::Array3i cell_index =
          it.GetCellIndex() - shift * PrecomputationGrid::GetOctant(i);
      auto* const cell_value = result.mutable_value(
          half_resolution ? CellIndexAtHalfResolution(cell_index) : cell_index);
      // 取该滑窗内的最大值
      *cell_value = std::max(it.GetValue(), *cell_value);
    }
  }
  return result;
}

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer
