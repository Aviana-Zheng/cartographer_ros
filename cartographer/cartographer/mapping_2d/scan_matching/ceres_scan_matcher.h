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

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/scan_matching/proto/ceres_scan_matcher_options.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

proto::CeresScanMatcherOptions CreateCeresScanMatcherOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// Align scans with an existing map using Ceres.
class CeresScanMatcher {
 public:
  // 构造函数  记录下配置项
  // 通过函数CreateCeresSolverOptions和配置项ceres_solver_options装载Ceres优化库的配置
  explicit CeresScanMatcher(const proto::CeresScanMatcherOptions& options);
  virtual ~CeresScanMatcher();

  // 屏蔽了拷贝构造和拷贝赋值
  CeresScanMatcher(const CeresScanMatcher&) = delete;
  CeresScanMatcher& operator=(const CeresScanMatcher&) = delete;

  // Aligns 'point_cloud' within the 'probability_grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  /*
   * 有6个参数，
   * previous_pose:用于约束位姿估计的xy坐标，上一个姿态
   * initial_pose_estimate:给定机器人的初始位姿估计
   * point_cloud:点云数据
   * probability_grid:栅格地图
   * pose_estimate:返回优化后的位姿估计 
   * summary:返回优化迭代信息。
   * 在给定机器人的初始位姿估计initial_pose_estimate的情况下，
   * 尽可能的将扫描的点云数据point_cloud放置到栅格地图grid上， 
   * 同时返回优化后的位姿估计pose_estimate和优化迭代信息summary。
   * 而参数previous_pose主要用于约束位姿估计的xy坐标，
   * 在Local SLAM的业务主线调用该函数的时候传递的是位姿估计器的估计值，
   * Cartographer认为优化后的机器人位置应当与该估计值的偏差不大。
   */
  void Match(const transform::Rigid2d& previous_pose,
             const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud,
             const ProbabilityGrid& probability_grid,
             transform::Rigid2d* pose_estimate,
             ceres::Solver::Summary* summary) const;

 private:
  const proto::CeresScanMatcherOptions options_; // 用于记录扫描匹配器的相关配置
  ceres::Solver::Options ceres_solver_options_;  // 优化过程的配置
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
