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

#include "cartographer/mapping_2d/sparse_pose_graph/optimization_problem.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping_2d/sparse_pose_graph/spa_cost_function.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {
namespace sparse_pose_graph {

namespace {

// Converts a pose into the 3 optimization variable format used for Ceres:
// translation in x and y, followed by the rotation angle representing the
// orientation.
std::array<double, 3> FromPose(const transform::Rigid2d& pose) {
  return {{pose.translation().x(), pose.translation().y(),
           pose.normalized_angle()}};
}

// Converts a pose as represented for Ceres back to an transform::Rigid2d pose.
transform::Rigid2d ToPose(const std::array<double, 3>& values) {
  return transform::Rigid2d({values[0], values[1]}, values[2]);
}

}  // namespace

OptimizationProblem::OptimizationProblem(
    const mapping::sparse_pose_graph::proto::OptimizationProblemOptions&
        options)
    : options_(options) {}

OptimizationProblem::~OptimizationProblem() {}

// 根据trajectory_id分类vector，deque按照时间关系记录的IMU数据
void OptimizationProblem::AddImuData(const int trajectory_id,
                                     const common::Time time,
                                     const Eigen::Vector3d& linear_acceleration,
                                     const Eigen::Vector3d& angular_velocity) {
  CHECK_GE(trajectory_id, 0);
  imu_data_.resize(
      std::max(imu_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  imu_data_[trajectory_id].push_back(
      mapping_3d::ImuData{time, linear_acceleration, angular_velocity});
}
// 根据trajectory_id分类vector, vector按照时间关系加入node信息
void OptimizationProblem::AddTrajectoryNode(
    const int trajectory_id, const common::Time time,
    const transform::Rigid2d& initial_point_cloud_pose,
    const transform::Rigid2d& point_cloud_pose) {
  CHECK_GE(trajectory_id, 0);
  node_data_.resize(
      std::max(node_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  node_data_[trajectory_id].push_back(
      NodeData{time, initial_point_cloud_pose, point_cloud_pose});
}

// 根据trajectory_id分类vector, vector按照时间关系加入submap信息
void OptimizationProblem::AddSubmap(const int trajectory_id,
                                    const transform::Rigid2d& submap_pose) {
  CHECK_GE(trajectory_id, 0);
  submap_data_.resize(
      std::max(submap_data_.size(), static_cast<size_t>(trajectory_id) + 1));
  submap_data_[trajectory_id].push_back(SubmapData{submap_pose});
}

// 设置ceres最大迭代次数
void OptimizationProblem::SetMaxNumIterations(const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}

/** 
 * @brief 后端优化核心函数，通过Ceres优化库，调整子图和路径节点的世界位姿
 * @param constraints 位姿图的约束
 */
void OptimizationProblem::Solve(const std::vector<Constraint>& constraints) {
  // 路径节点为空就没有必要进行优化
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }

  // 创建优化问题对象
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  // Set the starting point.
  // TODO(hrapp): Move ceres data into SubmapData.
  // 创建一些临时变量，用于描述优化问题
  std::vector<std::deque<std::array<double, 3>>> C_submaps(submap_data_.size());
  std::vector<std::deque<std::array<double, 3>>> C_nodes(node_data_.size());
  
  // 遍历所有子图
  // 提取所有Submap位姿信息，并保存在C_submaps中　，按照轨迹trajectory_id顺序及时间顺序排列
  for (size_t trajectory_id = 0; trajectory_id != submap_data_.size();
       ++trajectory_id) {
    for (size_t submap_index = 0;
         submap_index != submap_data_[trajectory_id].size(); ++submap_index) {
      
      // 如果是第一幅子图，就通过SetParameterBlockConstant将对应的参数设定为常数，
      // 这样在Ceres在迭代求解的过程中将不会改变这些参数。
      if (trajectory_id == 0 && submap_index == 0) {
        // Fix the pose of the first submap of the first trajectory.
        C_submaps[trajectory_id].push_back(
            FromPose(transform::Rigid2d::Identity()));
        // 将子图全局位姿作为优化参数告知对象problem。
        // AddResidualBlock检查到有此指针，则不再增加
        problem.AddParameterBlock(C_submaps[trajectory_id].back().data(), 3);
        // 设定对应的参数模块在优化过程中保持不变
        problem.SetParameterBlockConstant(
            C_submaps[trajectory_id].back().data());
      } else {
        C_submaps[trajectory_id].push_back(
            FromPose(submap_data_[trajectory_id][submap_index].pose));
        problem.AddParameterBlock(C_submaps[trajectory_id].back().data(), 3);
      }
    }
  }
  
  // 遍历所有路径节点
  // 提取所有Node位姿信息，并保存在C_nodes中，按照轨迹trajectory_id顺序及时间顺序排列
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    C_nodes[trajectory_id].resize(node_data_[trajectory_id].size());
    
    for (size_t node_index = 0; node_index != node_data_[trajectory_id].size();
         ++node_index) {
      C_nodes[trajectory_id][node_index] =
          FromPose(node_data_[trajectory_id][node_index].point_cloud_pose);
      // 将节点的全局位姿作为优化参数告知对象problem。
      // 一次性增加所有待优化参数，AddResidualBlock检查到有此指针，则不再增加
      problem.AddParameterBlock(C_nodes[trajectory_id][node_index].data(), 3);
    }
  }

  // Add cost functions for intra- and inter-submap constraints.
  // 遍历所有的约束，描述优化问题的残差块。
  // 函数CreateAutoDiffSpaCostFunction用于提供对应约束的SPA代价计算。
  // 如果是通过闭环检测构建的约束，则为之提供一个Huber的核函数，
  // 用于降低错误的闭环检测对最终的优化结果带来的负面影响。
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<SpaCostFunction, 3 /* residuals */,
                                                  3 /* start pose variables */, 
                                                  3 /* end pose variables */>(
            new SpaCostFunction(constraint.pose)),
        // Only loop closure constraints should have a loss function.
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr,
        C_submaps.at(constraint.submap_id.trajectory_id)
            .at(constraint.submap_id.submap_index)
            .data(),
        C_nodes.at(constraint.node_id.trajectory_id)
            .at(constraint.node_id.node_index)
            .data());
  }

  // Add penalties for changes between consecutive scans.
  // 遍历所有的路径节点，为相邻节点之间添加约束
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    for (size_t node_index = 1; node_index < node_data_[trajectory_id].size();
         ++node_index) {
      // 根据Local SLAM以及里程计等局部定位信息建立相邻的路径节点initial_point_cloud_pose之间的位姿变换关系。
      // 并将之描述为残差项提供给problem对象。
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<SpaCostFunction, 3 /* residuals */,
                                                  3 /* start pose variables */, 
                                                  3 /* end pose variables */>(
              new SpaCostFunction(Constraint::Pose{
                  transform::Embed3D(node_data_[trajectory_id][node_index - 1]
                                         .initial_point_cloud_pose.inverse() *
                                     node_data_[trajectory_id][node_index]
                                         .initial_point_cloud_pose),
                  options_.consecutive_scan_translation_penalty_factor(),
                  options_.consecutive_scan_rotation_penalty_factor()})),
          nullptr /* loss function */,
          C_nodes[trajectory_id][node_index - 1].data(),
          C_nodes[trajectory_id][node_index].data());
    }
  }

  // Solve.
  // 在描述完优化参数和残差计算方法之后，就可以通过ceres求解优化问题。对象summary将记录整个求解过程。
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);

  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
  }

  // Store the result.
  // 最后记录优化的结果
  for (size_t trajectory_id = 0; trajectory_id != submap_data_.size();
       ++trajectory_id) {
    for (size_t submap_index = 0;
         submap_index != submap_data_[trajectory_id].size(); ++submap_index) {
      submap_data_[trajectory_id][submap_index].pose =
          ToPose(C_submaps[trajectory_id][submap_index]);
    }
  }
  for (size_t trajectory_id = 0; trajectory_id != node_data_.size();
       ++trajectory_id) {
    for (size_t node_index = 0; node_index != node_data_[trajectory_id].size();
         ++node_index) {
      node_data_[trajectory_id][node_index].point_cloud_pose =
          ToPose(C_nodes[trajectory_id][node_index]);
    }
  }
}

const std::vector<std::vector<NodeData>>& OptimizationProblem::node_data()
    const {
  return node_data_;
}

const std::vector<std::vector<SubmapData>>& OptimizationProblem::submap_data()
    const {
  return submap_data_;
}

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer
