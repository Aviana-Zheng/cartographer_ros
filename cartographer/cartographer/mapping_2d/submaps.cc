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

#include "cartographer/mapping_2d/submaps.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

ProbabilityGrid ComputeCroppedProbabilityGrid(
    const ProbabilityGrid& probability_grid) {
  Eigen::Array2i offset;
  CellLimits limits;
  probability_grid.ComputeCroppedLimits(&offset, &limits);
  const double resolution = probability_grid.limits().resolution();
  // 相当于做了一个平移变换，将submaps地图中的最小值平移至原点
  const Eigen::Vector2d max =
      probability_grid.limits().max() -
      resolution * Eigen::Vector2d(offset.y(), offset.x());
  ProbabilityGrid cropped_grid(MapLimits(resolution, max, limits));
  cropped_grid.StartUpdate();
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(limits)) {
    if (probability_grid.IsKnown(xy_index + offset)) {
      cropped_grid.SetProbability(
          xy_index, probability_grid.GetProbability(xy_index + offset));
    }
  }
  return cropped_grid;
}

proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SubmapsOptions options;
  options.set_resolution(parameter_dictionary->GetDouble("resolution"));
  options.set_half_length(parameter_dictionary->GetDouble("half_length"));
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());
  CHECK_GT(options.num_range_data(), 0);
  return options;
}

Submap::Submap(const MapLimits& limits, const Eigen::Vector2f& origin)
    : mapping::Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      probability_grid_(limits) {}

void Submap::ToResponseProto(
    const transform::Rigid3d&,
    mapping::proto::SubmapQuery::Response* const response) const {
  response->set_submap_version(num_range_data());

  Eigen::Array2i offset;
  CellLimits limits;
  probability_grid_.ComputeCroppedLimits(&offset, &limits);

  string cells;
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(limits)) {
    if (probability_grid_.IsKnown(xy_index + offset)) {
      // We would like to add 'delta' but this is not possible using a value and
      // alpha. We use premultiplied alpha, so when 'delta' is positive we can
      // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
      // zero, and use 'alpha' to subtract. This is only correct when the pixel
      // is currently white, so walls will look too gray. This should be hard to
      // detect visually for the user, though.
      /*
      我们想添加“delta”，但使用值和 alpha 是不可能的。 我们使用预乘 alpha，
      因此当 'delta' 为正时，我们可以通过将 'alpha' 设置为零来添加它。 
      如果它是负数，我们将 'value' 设置为零，并使用 'alpha' 进行减法。 
      这仅在像素当前为白色时才正确，因此墙壁看起来太灰。 但是，这对于用户来说应该很难在视觉上检测到。 
      */
      const int delta =
          128 - mapping::ProbabilityToLogOddsInteger(
                    probability_grid_.GetProbability(xy_index + offset));
      const uint8 alpha = delta > 0 ? 0 : -delta;
      const uint8 value = delta > 0 ? delta : 0;
      cells.push_back(value);
      cells.push_back((value || alpha) ? alpha : 1);
    } else {
      constexpr uint8 kUnknownLogOdds = 0;
      cells.push_back(static_cast<uint8>(kUnknownLogOdds));  // value
      cells.push_back(0);                                    // alpha
    }
  }
  common::FastGzipString(cells, response->mutable_cells());

  response->set_width(limits.num_x_cells);
  response->set_height(limits.num_y_cells);
  const double resolution = probability_grid_.limits().resolution();
  response->set_resolution(resolution);
  const double max_x =
      probability_grid_.limits().max().x() - resolution * offset.y();
  const double max_y =
      probability_grid_.limits().max().y() - resolution * offset.x();
  *response->mutable_slice_pose() = transform::ToProto(
      local_pose().inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.)));
}

void Submap::InsertRangeData(const sensor::RangeData& range_data,
                             const RangeDataInserter& range_data_inserter) {
  CHECK(!finished_);
  range_data_inserter.Insert(range_data, &probability_grid_);
  ++num_range_data_;
}

void Submap::Finish() {
  CHECK(!finished_);
  // 计算裁剪栅格地图
  probability_grid_ = ComputeCroppedProbabilityGrid(probability_grid_);
  finished_ = true;
}

ActiveSubmaps::ActiveSubmaps(const proto::SubmapsOptions& options)
    : options_(options),
      range_data_inserter_(options.range_data_inserter_options()) {
  // We always want to have at least one likelihood field which we can return,
  // and will create it at the origin in absence of a better choice.
  AddSubmap(Eigen::Vector2f::Zero());
}

void ActiveSubmaps::InsertRangeData(const sensor::RangeData& range_data) {
  for (auto& submap : submaps_) {
    //两个submap全部插入新的range_data，表明old的submap2d包含了所有new submap2d内容
    submap->InsertRangeData(range_data, range_data_inserter_);
  }
  // 当range data 数量达到配置的值时，将新建个submap
  if (submaps_.back()->num_range_data() == options_.num_range_data()) {
    AddSubmap(range_data.origin.head<2>());
  }
}

std::vector<std::shared_ptr<Submap>> ActiveSubmaps::submaps() const {
  return submaps_;
}

int ActiveSubmaps::matching_index() const { return matching_submap_index_; }

void ActiveSubmaps::FinishSubmap() {
  Submap* submap = submaps_.front().get();
  submap->Finish();
  ++matching_submap_index_;
  // erase函数可以用于删除vector容器中的一个或者一段元素
  submaps_.erase(submaps_.begin());
}

// 添加一个新的submap
void ActiveSubmaps::AddSubmap(const Eigen::Vector2f& origin) {
  // 如果已经存在两个submap，即old和new均存在
  // 则剔除old的submap2d
  if (submaps_.size() > 1) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    // 新建时，如果submap的个数为2,则需要先将第一个submap设置成finish，并从active submap中删掉
    FinishSubmap();
  }
  // 新建一个active submap
  const int num_cells_per_dimension =
      common::RoundToInt(2. * options_.half_length() / options_.resolution()) +
      1;
  submaps_.push_back(common::make_unique<Submap>(
      MapLimits(options_.resolution(),
                // 以origin为中点的地图
                origin.cast<double>() +
                    options_.half_length() * Eigen::Vector2d::Ones(),
                CellLimits(num_cells_per_dimension, num_cells_per_dimension)),
      origin));
  LOG(INFO) << "Added submap " << matching_submap_index_ + submaps_.size();
}

}  // namespace mapping_2d
}  // namespace cartographer
