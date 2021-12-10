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

#ifndef CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
#define CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/proto/map_limits.pb.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.
class MapLimits {
 public:
  MapLimits(const double resolution, const Eigen::Vector2d& max,
            const CellLimits& cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits) {
    CHECK_GT(resolution_, 0.);
    CHECK_GT(cell_limits.num_x_cells, 0.);
    CHECK_GT(cell_limits.num_y_cells, 0.);
  }

  explicit MapLimits(const proto::MapLimits& map_limits)
      : resolution_(map_limits.resolution()),
        max_(transform::ToEigen(map_limits.max())),
        cell_limits_(map_limits.cell_limits()) {}

  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  const Eigen::Vector2d& max() const { return max_; }

  // Returns the limits of the grid in number of cells.
  const CellLimits& cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the point ('x', 'y') which may be
  // outside the map, i.e., negative or too large indices that will return
  // false for Contains().
  // 世界坐标是正常的我们认识的坐标，左下角为原点，向右为x，向上为y。通过实际长度point找到cell
  // 我们给一个真实长度，世界坐标点point (x,y)，如何得到网格坐标点cell (x,y)
  // 其中max_表示的是地图最大范围
  Eigen::Array2i GetXYIndexOfCellContainingPoint(const double x,
                                                 const double y) const {
    // 标准矩阵: 矩阵在数学中的定义是 m行n列的数阵
    // 而在图形学中，有行主序和列主序两种排列方式，行主序和标准矩阵是一样的排列方式，
    // 列主序则是竖着排列 即行主序的转置
    // 看到左上角为栅格地图的原点，即栅格（0,0），且栅格地图是行主序
    // 如何理解呢？可以将栅格地图理解为左上角为原点（0,0），向右为x轴，向下为y轴
    // 而对于point点来说，坐标系是正常的如下图所示：
    // 		          	    x^
    //  		               |
    //  	          	     |
    //  		  		         |
    //  		   		         |
    //  		               |
    //   <—————————————————0 
    //   y                  
    // 而对于栅格坐标系为：
    //  0——————————————————> x
    //  |
    //  |
    //  |
    //  |
    //  |
    //  y 
    //   	           x 
    // 所以需要用最大范围减去点的坐标，除以分辨率
    // 只有max_比point大二分之一分辨率的时候代表了栅格地图的原点，
    // 而max表示了地图的最大范围，因此我推断：
    // 返回的这个点 是 栅格的中心点，因此，栅格点(grid_point)是一个格子的中心
    // Index values are row major and the top left has Eigen::Array2i::Zero()
    // and contains (centered_max_x, centered_max_y). We need to flip and
    // rotate.
    return Eigen::Array2i(
        common::RoundToInt((max_.y() - y) / resolution_ - 0.5),
        common::RoundToInt((max_.x() - x) / resolution_ - 0.5));
  }

  // Returns true of the ProbabilityGrid contains 'xy_index'.
  bool Contains(const Eigen::Array2i& xy_index) const {
    return (Eigen::Array2i(0, 0) <= xy_index).all() &&
           (xy_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

 private:
  double resolution_;
  Eigen::Vector2d max_;
  CellLimits cell_limits_;
};

inline proto::MapLimits ToProto(const MapLimits& map_limits) {
  proto::MapLimits result;
  result.set_resolution(map_limits.resolution());
  *result.mutable_max() = transform::ToProto(map_limits.max());
  *result.mutable_cell_limits() = ToProto(map_limits.cell_limits());
  return result;
}

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
