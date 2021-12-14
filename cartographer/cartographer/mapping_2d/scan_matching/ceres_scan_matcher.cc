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

#include "cartographer/mapping_2d/scan_matching/ceres_scan_matcher.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/scan_matching/occupied_space_cost_functor.h"
#include "cartographer/mapping_2d/scan_matching/rotation_delta_cost_functor.h"
#include "cartographer/mapping_2d/scan_matching/translation_delta_cost_functor.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

proto::CeresScanMatcherOptions CreateCeresScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions options;
  options.set_occupied_space_weight(
      parameter_dictionary->GetDouble("occupied_space_weight"));
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

CeresScanMatcher::CeresScanMatcher(
    const proto::CeresScanMatcherOptions& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

CeresScanMatcher::~CeresScanMatcher() {}

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
void CeresScanMatcher::Match(const transform::Rigid2d& previous_pose,
                             const transform::Rigid2d& initial_pose_estimate,
                             const sensor::PointCloud& point_cloud,
                             const ProbabilityGrid& probability_grid,
                             transform::Rigid2d* const pose_estimate,
                             ceres::Solver::Summary* const summary) const {
  // 用一个double数组记录下初始位姿，作为Ceres优化迭代的初值
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  // 构建ceres::Problem对象
  ceres::Problem problem;
  // 检查权重
  CHECK_GT(options_.occupied_space_weight(), 0.);
  // 通过接口AddResidualBlock添加残差项  残差主要有三个方面的来源
  // 配置文件中为这三类来源定义了权重
  // 1、占用栅格与扫描数据的匹配度
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunctor, ceres::DYNAMIC,
                                      3>(
          new OccupiedSpaceCostFunctor(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, probability_grid),
          point_cloud.size()),
      nullptr, ceres_pose_estimate);
  // 检查权重
  CHECK_GT(options_.translation_weight(), 0.);
  // 2、优化后的位置相对于target_translation的距离
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor, 2, 3>(
          new TranslationDeltaCostFunctor(options_.translation_weight(),
                                          previous_pose)),
      nullptr, ceres_pose_estimate);
  // 检查权重
  CHECK_GT(options_.rotation_weight(), 0.);
  // 3、旋转角度相对于迭代初值的偏差
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor, 1, 3>(
          new RotationDeltaCostFunctor(options_.rotation_weight(),
                                       ceres_pose_estimate[2])),
      nullptr, ceres_pose_estimate);
  
  // 求解并更新位姿估计
  ceres::Solve(ceres_solver_options_, &problem, summary);

  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer
