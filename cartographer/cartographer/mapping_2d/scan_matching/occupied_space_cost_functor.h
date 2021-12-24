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

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTOR_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTOR_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

// Computes the cost of inserting occupied space described by the point cloud
// into the map. The cost increases with the amount of free space that would be
// replaced by occupied space.
// 类OccupiedSpaceCostFunction2D是真正的扫描匹配的主角
class OccupiedSpaceCostFunctor {
 public:
  // Creates an OccupiedSpaceCostFunctor using the specified map, resolution
  // level, and point cloud.
  // scaling_factor:
  // 权重options_.occupied_space_weight() / std::sqrt(static_cast<double>(point_cloud.size()))
  OccupiedSpaceCostFunctor(const double scaling_factor,   //权重
                           const sensor::PointCloud& point_cloud,
                           const ProbabilityGrid& probability_grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        probability_grid_(probability_grid) {}

  OccupiedSpaceCostFunctor(const OccupiedSpaceCostFunctor&) = delete;
  OccupiedSpaceCostFunctor& operator=(const OccupiedSpaceCostFunctor&) = delete;

  // 对运算符"()"的重载
  // 在函数的一开始，先把迭代查询的输入参数pose转换为坐标变换Tε用临时变量transform记录
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    const GridArrayAdapter adapter(probability_grid_);
    // 然后使用Ceres库原生提供的双三次插值迭代器。
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapLimits& limits = probability_grid_.limits();

    /*
     * 接着根据cartographer论文中式(CS)针对每个hit点计算对应的残差代价。通过hit点坐标与transform的相乘
     * 得到其在地图坐标系下的坐标Tεhk。 在第14行通过刚刚构建的迭代器，和地图坐标，获取在hit点
     * 出现的概率。该函数调用有三个参数，前两个参数用来描述x,y轴索引， 第三个参数用于记录插值后
     * 的结果。这里的xy索引计算的比较奇怪，它通过GridArrayAdapter类中成员函数GetValue获取栅格
     * 数据，这里不再细述。 此外由于占用栅格中原本存储的就是栅格击中的概率， 所以这里查询出来
     * 的概率就是(1−Msmooth(Tεhk))
     */
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].x())),
                                         (T(point_cloud_[i].y())), T(1.));
      // hit点坐标与transform的相乘得到其在地图坐标系下的坐标
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      interpolator.Evaluate(  //先行(y)后列(x)
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              T(kPadding),   // 地图中的坐标
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              T(kPadding),
          &residual[i]);
      residual[i] = scaling_factor_ * (1. - residual[i]);
    }
    return true;
  }

 private:
  // kPadding用来判断插值点是否在地图内
  static constexpr int kPadding = INT_MAX / 4;
  class GridArrayAdapter {
   public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const ProbabilityGrid& probability_grid)
        : probability_grid_(probability_grid) {}

    void GetValue(const int row, const int column, double* const value) const {
      // 先判断是否在地图内，超出地图边界则认为没有被击中，概率为mapping::kMinProbability
      // row < kPadding || column < kPadding ，hit点比地图对应的最大值还大
      // row >= NumRows() - kPadding || column >= NumCols() - kPadding 
      // hit点比地图对应的最小点还小
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = mapping::kMinProbability;
      } else {
        *value = static_cast<double>(probability_grid_.GetProbability(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    // 边界y
    int NumRows() const {
      return probability_grid_.limits().cell_limits().num_y_cells +
             2 * kPadding;
    }

    // 边界x
    int NumCols() const {
      return probability_grid_.limits().cell_limits().num_x_cells +
             2 * kPadding;
    }

   private:
    const ProbabilityGrid& probability_grid_;
  };

  const double scaling_factor_;  //权重
  const sensor::PointCloud& point_cloud_;
  const ProbabilityGrid& probability_grid_;
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTOR_H_
