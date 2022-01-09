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

#include "cartographer/mapping_2d/scan_matching/correlative_scan_matcher.h"

#include <cmath>

#include "cartographer/common/math.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const sensor::PointCloud& point_cloud,
                                   const double resolution)
    : resolution(resolution) {
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  // 找到最远的点
  float max_scan_range = 3.f * resolution;
  for (const Eigen::Vector3f& point : point_cloud) {
    const float range = point.head<2>().norm();
    max_scan_range = std::max(range, max_scan_range);
  }
  // 计算角度增长step,1 - 0.001 = 0.999; 保证角度扰动比dmax计算的小一点；
  const double kSafetyMargin = 1. - 1e-3;
  // 角分辨率的推导 一个分辨率栅格对应的角度,计算角分辨率的方法在论文中有体现，公式(7)
  angular_perturbation_step_size =
      kSafetyMargin * std::acos(1. - common::Pow2(resolution) /
                                         (2. * common::Pow2(max_scan_range)));
  // 单向角度搜索次数, 计算单向角度有多少个增长的step
  num_angular_perturbations = //  std::ceil   -->向上取整数：
      std::ceil(angular_search_window / angular_perturbation_step_size);
  // Angular search window, 180 degrees in both directions.
  // num_scans切片数目
  // 因为在根据角度生成点云时，是从-angular_search_window到+angular_search_window生成的
  // 所以切片数目需要乘以2 再+1
  num_scans = 2 * num_angular_perturbations + 1;

  // 计算线性搜索框有多少个step，step的size是resolution
  const int num_linear_perturbations =   //  std::ceil   -->向上取整数：
      std::ceil(linear_search_window / resolution);
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

SearchParameters::SearchParameters(const int num_linear_perturbations,
                                   const int num_angular_perturbations,
                                   const double angular_perturbation_step_size,
                                   const double resolution)
    : num_angular_perturbations(num_angular_perturbations),
      angular_perturbation_step_size(angular_perturbation_step_size),
      resolution(resolution),
      num_scans(2 * num_angular_perturbations + 1) {
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

void SearchParameters::ShrinkToFit(const std::vector<DiscreteScan>& scans,
                                   const CellLimits& cell_limits) {
  CHECK_EQ(scans.size(), num_scans);
  CHECK_EQ(linear_bounds.size(), num_scans);
  for (int i = 0; i != num_scans; ++i) {
    Eigen::Array2i min_bound = Eigen::Array2i::Zero();
    Eigen::Array2i max_bound = Eigen::Array2i::Zero();
    for (const Eigen::Array2i& xy_index : scans[i]) {
      // 两个array相应元素的最小值
      min_bound = min_bound.min(-xy_index);
      max_bound = max_bound.max(Eigen::Array2i(cell_limits.num_x_cells - 1,
                                               cell_limits.num_y_cells - 1) -
                                xy_index);
    }
    linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
    linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
    linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
    linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
  }
}

/*
 * 根据搜索参数把激光数据进行多次旋转 相当于在进行compute 2d slice的时候进行最外层的角度循环
 * 这里的激光进行转化之后，虽然角度和世界坐标系重合，但是原点依然是不重合的
 */
// 切片旋转,按照搜索角度/步长，以初始值为基础，生成一些列的旋转后的点云，按顺序存放在vector里
std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters) {
  std::vector<sensor::PointCloud> rotated_scans;
  rotated_scans.reserve(search_parameters.num_scans);   // num_scans切片数目

  // 搜索的角度的起点 -offset
  double delta_theta = -search_parameters.num_angular_perturbations *
                       search_parameters.angular_perturbation_step_size;
  // 按照-angular->+angular区间，生成2 * num_angular_perturbations + 1个点云
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       ++scan_index,   // 加上角度搜索步长
           delta_theta += search_parameters.angular_perturbation_step_size) {
    rotated_scans.push_back(sensor::TransformPointCloud(
        point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf(
                         delta_theta, Eigen::Vector3f::UnitZ()))));
  }
  return rotated_scans;
}

/*
 * 把一系列的激光数据scans转换到世界坐标系中的坐标(initial_translation)。
 * 因为在进行rotated scans生成的时候，激光数据的角度已经和世界坐标系重合了。
 * 因此这里进行转换的时候只需要进行平移就可以了。
 * 然后把世界坐标系中的物理坐标离散化，转换为地图坐标系的坐标
 * 这个函数返回一些列地图坐标系中的激光扫描数据
 * 经过这个函数之后，各个激光数据角度都和世界坐标系对齐了。
 * 原点都和世界坐标系重合了。
 */
// 将旋转点云，从旋转点云坐标中，转化到地图坐标中去。根据坐标，获得 cell index
std::vector<DiscreteScan> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation) {
  std::vector<DiscreteScan> discrete_scans;
  discrete_scans.reserve(scans.size());
  // 遍历每帧中的点云
  for (const sensor::PointCloud& scan : scans) {
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());
    // 点云中的每个点加上平移，转换到世界坐标系，并得到在地图中对应的id
    for (const Eigen::Vector3f& point : scan) {
      // 从旋转点云坐标中，转换到世界坐标系中
      const Eigen::Vector2f translated_point =
          Eigen::Affine2f(initial_translation) * point.head<2>();
      // 获得该点，对应的栅格索引
      discrete_scans.back().push_back(
          map_limits.GetXYIndexOfCellContainingPoint(translated_point.x(),
                                                     translated_point.y()));
    }
  }
  return discrete_scans;
}

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer


/*

Function	function
abs      	绝对值
sqrt	    平方根
min(.)	   两个array相应元素的最小值

int main()
{
  ArrayXf a = ArrayXf::Random(5);
  a *= 2;
  cout << "a =" << endl 
       << a << endl;
  cout << "a.abs() =" << endl 
       << a.abs() << endl;
  cout << "a.abs().sqrt() =" << endl 
       << a.abs().sqrt() << endl;
  cout << "a.min(a.abs().sqrt()) =" << endl 
       << a.min(a.abs().sqrt()) << endl;
}


a =
-1.99499
0.254341
-1.22678
 1.23496
0.340037

a.abs() =
 1.99499
0.254341
 1.22678
 1.23496
0.340037

a.abs().sqrt() =
 1.41244
0.504323
  1.1076
 1.11129
0.583127

a.min(a.abs().sqrt()) =
-1.99499
0.254341
-1.22678
 1.11129
0.340037

*/