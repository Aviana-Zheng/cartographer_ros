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

#include "cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h"

#include <string>
#include <utility>
#include <vector>

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping_3d/ceres_pose.h"
#include "cartographer/mapping_3d/scan_matching/occupied_space_cost_functor.h"
#include "cartographer/mapping_3d/scan_matching/rotation_delta_cost_functor.h"
#include "cartographer/mapping_3d/scan_matching/translation_delta_cost_functor.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {
namespace {

/*
optimization_problem.cc
struct ConstantYawQuaternionPlus {
   template <typename T>
   bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
     const T delta_norm =
         ceres::sqrt(common::Pow2(delta[0]) + common::Pow2(delta[1]));
     const T sin_delta_over_delta =
         delta_norm < 1e-6 ? T(1.) : ceres::sin(delta_norm) / delta_norm;
     T q_delta[4];
     q_delta[0] = delta_norm < 1e-6 ? T(1.) : ceres::cos(delta_norm);
     q_delta[1] = sin_delta_over_delta * delta[0];
     q_delta[2] = sin_delta_over_delta * delta[1];
     q_delta[3] = T(0.);
     // We apply the 'delta' which is interpreted as an angle-axis rotation
     // vector in the xy-plane of the submap frame. This way we can align to
     // gravity because rotations around the z-axis in the submap frame do not
     // change gravity alignment, while disallowing random rotations of the map
     // that have nothing to do with gravity alignment (i.e. we disallow steps
     // just changing "yaw" of the complete map).
     ceres::QuaternionProduct(x, q_delta, x_plus_delta);
     return true;
   }
 };
 */
struct YawOnlyQuaternionPlus {
  template <typename T>
  // https://github.com/cartographer-project/cartographer/issues/1770
  // delta represents the change in yaw.
  // 此处的'delta'不是angle-axis？ 此处取了近似？ delta 约等于 sin(delta)
  bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
    const T clamped_delta = common::Clamp(delta[0], T(-0.5), T(0.5));
    T q_delta[4];
    q_delta[0] = ceres::sqrt(1. - clamped_delta * clamped_delta);
    q_delta[1] = T(0.);
    q_delta[2] = T(0.);
    q_delta[3] = clamped_delta;
    ceres::QuaternionProduct(q_delta, x, x_plus_delta);
    return true;
  }
};

}  // namespace

proto::CeresScanMatcherOptions CreateCeresScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions options;
  for (int i = 0;; ++i) {
    const string lua_identifier = "occupied_space_weight_" + std::to_string(i);
    if (!parameter_dictionary->HasKey(lua_identifier)) {
      break;
    }
    options.add_occupied_space_weight(
        parameter_dictionary->GetDouble(lua_identifier));
  }
  // 存储一些权重参数，即优化时的信息矩阵；
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  options.set_only_optimize_yaw(
      parameter_dictionary->GetBool("only_optimize_yaw"));
  // 优化参数，包括最大迭代次数、线程、单调性等；
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

void CeresScanMatcher::Match(const transform::Rigid3d& previous_pose,
                             const transform::Rigid3d& initial_pose_estimate,   //由scan matching或其它传感器提供的初始值；
                             const std::vector<PointCloudAndHybridGridPointers>&
                                 point_clouds_and_hybrid_grids,   // 混合的概率网格；
                             transform::Rigid3d* const pose_estimate,
                             ceres::Solver::Summary* const summary) {
  // 首先声明一个ceres最小二乘问题；
  ceres::Problem problem;
  // http://ceres-solver.org/nnls_modeling.html#autodifflocalparameterization
  CeresPose ceres_pose(
      initial_pose_estimate, nullptr /* translation_parameterization */,
      options_.only_optimize_yaw()
          ? std::unique_ptr<ceres::LocalParameterization>(  // 只优化Yaw角
                common::make_unique<ceres::AutoDiffLocalParameterization<
                    YawOnlyQuaternionPlus, 4, 1>>())
          : std::unique_ptr<ceres::LocalParameterization>(   // 优化所有的角度
                common::make_unique<ceres::QuaternionParameterization>()),
      &problem);
  
  // 这里点云所在的混合概率网格只有两层吗？因为选项中权重设置只有两个参数哇！！！
  // 而且第一个为低分辨率的权重，第二个为高分辨率的权重，因此第一个权重应该远小于第二个权重；
  CHECK_EQ(options_.occupied_space_weight_size(),
           point_clouds_and_hybrid_grids.size());
  // 针对高低分辨率地图，分别添加点的匹配残差, 这是概率网格的残差构造；
  for (size_t i = 0; i != point_clouds_and_hybrid_grids.size(); ++i) {
    CHECK_GT(options_.occupied_space_weight(i), 0.);
    const sensor::PointCloud& point_cloud =
        *point_clouds_and_hybrid_grids[i].first;
    const HybridGrid& hybrid_grid = *point_clouds_and_hybrid_grids[i].second;
    problem.AddResidualBlock(
        // 高低分辨率地图的匹配权重来自设置参数，并且同一分辨率地图的每个点的权重一样，
        // 因为残差是平方的，所以要开根号呀
        // 构建自动求导的代价函数，即点云关于旋转和*移的残差项；
        new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunctor,
                                        ceres::DYNAMIC, 3, 4>(
            new OccupiedSpaceCostFunctor(
                options_.occupied_space_weight(i) /
                    // weight1 / sqrt(size)，这是残差的一个系数，因为残差需要进行*方，因此开根号可以理解；
                    std::sqrt(static_cast<double>(point_cloud.size())),
                point_cloud, hybrid_grid),
            point_cloud.size()),
        nullptr /* loss function */, ceres_pose.translation(), ceres_pose.rotation());
        // loss function 为nullptr时，表示使用残差的*方项；
  }
  // *移单独构造了一个残差函数，并且一块进行优化
  CHECK_GT(options_.translation_weight(), 0.);
  // 添加平移残差
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor, 3, 3>(
          new TranslationDeltaCostFunctor(options_.translation_weight(),
                                          previous_pose)),
      nullptr /* loss function */, ceres_pose.translation());
  
  // 单独给*移和旋转构造一个残差函数！
  CHECK_GT(options_.rotation_weight(), 0.);
  // 添加旋转残差
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor, 3, 4>(
          new RotationDeltaCostFunctor(options_.rotation_weight(),
                                       initial_pose_estimate.rotation())),
      nullptr /* loss function */, ceres_pose.rotation());

  // 求解该问题；
  ceres::Solve(ceres_solver_options_, &problem, summary);

  *pose_estimate = ceres_pose.ToRigid();
}

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer
