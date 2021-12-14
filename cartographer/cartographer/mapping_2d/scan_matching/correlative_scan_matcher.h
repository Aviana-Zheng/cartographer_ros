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

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_H_
// correlative 相关的
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

typedef std::vector<Eigen::Array2i> DiscreteScan;  // 定义的整型容器

// Describes the search space.
// 描述了窗口大小、分辨率等搜索信息
struct SearchParameters {
  // Linear search window in pixel offsets; bounds are inclusive.
  // 像素偏移中的线性搜索窗口； 边界包括在内 
  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  // Linear search window, 1e6 cells/direction.
  // Angular search window, 180 degrees in both directions.
  SearchParameters(double linear_search_window, double angular_search_window,
                   const sensor::PointCloud& point_cloud, double resolution);

  // For testing.
  SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                   double angular_perturbation_step_size, double resolution);

  // Tightens the search window as much as possible.
  // 尽可能收紧搜索窗口。    以减小搜索空间，提高搜索效率
  void ShrinkToFit(const std::vector<DiscreteScan>& scans,
                   const CellLimits& cell_limits);

  // perturbations 扰动
  int num_angular_perturbations;   //单向角度搜索次数
  double angular_perturbation_step_size;  // 角度搜索步长
  double resolution;
  int num_scans;  // num_scans  角度切片数目
  std::vector<LinearBounds> linear_bounds;  // Per rotated scans. 每轮扫描 
};

// Generates a collection of rotated scans.
// 获得搜索窗口下机器人朝向各个方向角时的点云数据 
std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
// 将旋转后的扫描转换并离散化为整数向量指数。 
//  完成对旋转后的点云数据离散化的操作，即将浮点类型的点云数据转换成整型的栅格单元索引
std::vector<DiscreteScan> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation);

// A possible solution.
struct Candidate {
  Candidate(const int init_scan_index, const int init_x_index_offset,
            const int init_y_index_offset,
            const SearchParameters& search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        // 得到真实的xy   offset
        x(-y_index_offset * search_parameters.resolution),
        y(-x_index_offset * search_parameters.resolution),
        // scan_index包含了角度信息，计算方式如下
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {}

  // Index into the rotated scans vector.
  int scan_index = 0; // 角度的搜索索引

  // Linear offset from the initial pose.
  int x_index_offset = 0;   // x轴的搜索索引
  int y_index_offset = 0;   // y轴的搜索索引

  // Pose of this Candidate relative to the initial pose.
  double x = 0.;   // 相对于初始位姿的x偏移量
  double y = 0.;    	// 相对于初始位姿的y偏移量
  double orientation = 0.;   // 相对于初始位姿的角度偏移量

  // Score, higher is better.
  float score = 0.f;   // 候选点的评分，越高越好

  // 排序方法
  // 定义了两个比较操作符的重载用于方便比较候选点的优劣
  bool operator<(const Candidate& other) const { return score < other.score; }
  bool operator>(const Candidate& other) const { return score > other.score; }
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_H_
