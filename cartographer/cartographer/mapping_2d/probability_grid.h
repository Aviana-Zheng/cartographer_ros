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

#ifndef CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/proto/probability_grid.pb.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

// Represents a 2D grid of probabilities.
class ProbabilityGrid {
 public:
  explicit ProbabilityGrid(const MapLimits& limits)
      : limits_(limits),
        cells_(limits_.cell_limits().num_x_cells *
                   limits_.cell_limits().num_y_cells,
               mapping::kUnknownProbabilityValue),
        max_x_(0),
        max_y_(0),
        min_x_(limits_.cell_limits().num_x_cells - 1),
        min_y_(limits_.cell_limits().num_y_cells - 1) {}

  explicit ProbabilityGrid(const proto::ProbabilityGrid& proto)
      : limits_(proto.limits()),
        cells_(),
        update_indices_(proto.update_indices().begin(),
                        proto.update_indices().end()),
        max_x_(proto.max_x()),
        max_y_(proto.max_y()),
        min_x_(proto.min_x()),
        min_y_(proto.min_y()) {
    cells_.reserve(proto.cells_size());
    for (const auto cell : proto.cells()) {
      CHECK_LE(cell, std::numeric_limits<uint16>::max());
      cells_.push_back(cell);
    }
  }

  // Returns the limits of this ProbabilityGrid.
  const MapLimits& limits() const { return limits_; }

  // Starts the next update sequence.
  void StartUpdate() {
    while (!update_indices_.empty()) {
      DCHECK_GE(cells_[update_indices_.back()], mapping::kUpdateMarker);
      cells_[update_indices_.back()] -= mapping::kUpdateMarker;
      update_indices_.pop_back();
    }
  }

  // Sets the probability of the cell at 'xy_index' to the given 'probability'.
  // Only allowed if the cell was unknown before.
  // 设置对应index的概率值， 在cell单元存储的则是概率对应的free 
  void SetProbability(const Eigen::Array2i& xy_index, const float probability) {
    // 取cell_index在栅格地图中对应索引的单元的地址
    // ToFlatIndex, 二维坐标转换为1维索引
    // mutable_correspondence_cost_cells() 返回的grid中grid地图
    uint16& cell = cells_[GetIndexOfCell(xy_index)];
    CHECK_EQ(cell, mapping::kUnknownProbabilityValue);
    // 将概率值转换对应的整型数，然后存入对应的索引的单元中
    cell = mapping::ProbabilityToValue(probability);
    // 返回的是有效概率值的矩形框，每更新一个cell， 则更新矩形框box， 
    // 采用eigen库extend，自动更新有效框
    UpdateBounds(xy_index);
  }

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'xy_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // StartUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  bool ApplyLookupTable(const Eigen::Array2i& xy_index,
                        const std::vector<uint16>& table) {
    DCHECK_EQ(table.size(), mapping::kUpdateMarker);
    const int cell_index = GetIndexOfCell(xy_index);
    uint16& cell = cells_[cell_index];
    //LOG(INFO) << cell << std::endl;
    if (cell >= mapping::kUpdateMarker) {
      return false;
    }
    update_indices_.push_back(cell_index);
    // table[cell] cell对应的是上一时刻的odds值
    cell = table[cell];
    //LOG(INFO) << cell << std::endl;
    DCHECK_GE(cell, mapping::kUpdateMarker);
    UpdateBounds(xy_index);
    return true;
  }

  // Returns the probability of the cell with 'xy_index'.
  float GetProbability(const Eigen::Array2i& xy_index) const {
    // 返回对应的index的占有概率值
    if (limits_.Contains(xy_index)) {
      return mapping::ValueToProbability(cells_[GetIndexOfCell(xy_index)]);
    }
    return mapping::kMinProbability;
  }

  // Returns the probability of the cell containing the point ('x', 'y').
  float GetProbability(const double x, const double y) const {
    return GetProbability(limits_.GetXYIndexOfCellContainingPoint(x, y));
  }

  // Returns true if the probability at the specified index is known.
  bool IsKnown(const Eigen::Array2i& xy_index) const {
    return limits_.Contains(xy_index) && cells_[GetIndexOfCell(xy_index)] !=
                                             mapping::kUnknownProbabilityValue;
  }

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const {
    *offset = Eigen::Array2i(min_x_, min_y_);
    *limits = CellLimits(std::max(max_x_, min_x_) - min_x_ + 1,
                         std::max(max_y_, min_y_) - min_y_ + 1);
  }

  // Grows the map as necessary to include 'x' and 'y'. This changes the meaning
  // of these coordinates going forward. This method must be called immediately
  // after 'StartUpdate', before any calls to 'ApplyLookupTable'.
  // 更新地图大小，同时将原grid中数据按照位置放入新放大的grid中
  void GrowLimits(const double x, const double y) {
    CHECK(update_indices_.empty());
    //如果当前的存在point不在范围内，即需要更新，采用迭代方法放大地图边界，
    while (!limits_.Contains(limits_.GetXYIndexOfCellContainingPoint(x, y))) {
      //获取原来的地图大小的中心坐标，即栅格索引
      const int x_offset = limits_.cell_limits().num_x_cells / 2;
      const int y_offset = limits_.cell_limits().num_y_cells / 2;
      // 地图扩大两倍
      // grid最大值加上原来的一半， 地图总大小放大一倍。 
      // 地图原点对应真实世界位置改变
      // 旧的栅格地图在新的栅格地图中的位置是offset，相当于做了一个平移变换
      const MapLimits new_limits(
          limits_.resolution(),
          limits_.max() +
              limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
          CellLimits(2 * limits_.cell_limits().num_x_cells,
                     2 * limits_.cell_limits().num_y_cells));
      // 行数，用于转换1维索引
      const int stride = new_limits.cell_limits().num_x_cells;
      // 新的offset stride是地图中x的格数，y_offset对应真实世界中的x轴
      const int offset = x_offset + stride * y_offset;
      // 新大小
      const int new_size = new_limits.cell_limits().num_x_cells *
                           new_limits.cell_limits().num_y_cells;
      // 更新grid概率，即将原来的概率赋值在新的grid中
      std::vector<uint16> new_cells(new_size,
                                    mapping::kUnknownProbabilityValue);
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
          // 旧的栅格地图在新的栅格地图中的位置是offset，相当于做了一个平移变换
          new_cells[offset + j + i * stride] =
              cells_[j + i * limits_.cell_limits().num_x_cells];
        }
      }
      cells_ = new_cells;
      limits_ = new_limits;
      min_x_ += x_offset;
      min_y_ += y_offset;
      max_x_ += x_offset;
      max_y_ += y_offset;
    }
  }

  proto::ProbabilityGrid ToProto() const {
    proto::ProbabilityGrid result;
    *result.mutable_limits() = cartographer::mapping_2d::ToProto(limits_);
    result.mutable_cells()->Reserve(cells_.size());
    for (const auto cell : cells_) {
      result.mutable_cells()->Add(cell);
    }
    result.mutable_update_indices()->Reserve(update_indices_.size());
    for (const auto update : update_indices_) {
      result.mutable_update_indices()->Add(update);
    }
    result.set_max_x(max_x_);
    result.set_max_y(max_y_);
    result.set_min_x(min_x_);
    result.set_min_y(min_y_);
    return result;
  }

 private:
  // Returns the index of the cell at 'xy_index'.
  int GetIndexOfCell(const Eigen::Array2i& xy_index) const {
    CHECK(limits_.Contains(xy_index)) << xy_index;
    return limits_.cell_limits().num_x_cells * xy_index.y() + xy_index.x();
  }

  void UpdateBounds(const Eigen::Array2i& xy_index) {
    min_x_ = std::min(min_x_, xy_index.x());
    min_y_ = std::min(min_y_, xy_index.y());
    max_x_ = std::max(max_x_, xy_index.x());
    max_y_ = std::max(max_y_, xy_index.y());
  }

  MapLimits limits_;   // 地图大小边界，包括x和y最大值， 分辨率， x和yu方向栅格数
  //grid 地图存储值，采用uint16类型，
  std::vector<uint16> cells_;  // Highest bit is update marker.
  // 记录已经更新过的索引
  std::vector<int> update_indices_;

  // Minimum and maximum cell coordinates of known cells to efficiently compute
  // cropping limits.
  // 对应实际意义的最大和最小边界
  int max_x_;
  int max_y_;
  int min_x_;
  int min_y_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
