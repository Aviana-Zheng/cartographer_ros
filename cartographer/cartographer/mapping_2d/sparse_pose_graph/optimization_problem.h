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

#ifndef CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_
#define CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_

#include <array>
#include <deque>
#include <map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/sparse_pose_graph/proto/optimization_problem_options.pb.h"
#include "cartographer/mapping_3d/imu_integration.h"

namespace cartographer {
namespace mapping_2d {
namespace sparse_pose_graph {

struct NodeData {
  common::Time time;
  transform::Rigid2d initial_point_cloud_pose; // 路径节点与子图相差的姿态
  transform::Rigid2d point_cloud_pose;   // 全局位姿
};

struct SubmapData {
  transform::Rigid2d pose; // global_pose 全局位姿
};

// Implements the SPA loop closure method.
class OptimizationProblem {
 public:
  using Constraint = mapping::SparsePoseGraph::Constraint;

  // 初始化，传递全局配置
  explicit OptimizationProblem(
      const mapping::sparse_pose_graph::proto::OptimizationProblemOptions&
          options);
  ~OptimizationProblem();

  OptimizationProblem(const OptimizationProblem&) = delete;
  OptimizationProblem& operator=(const OptimizationProblem&) = delete;

  // 根据trajectory_id分类vector，deque按照时间关系记录的IMU数据
  void AddImuData(int trajectory_id, common::Time time,
                  const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity);
  // 根据trajectory_id分类vector, vector按照时间关系加入node信息
  void AddTrajectoryNode(int trajectory_id, common::Time time,
                         const transform::Rigid2d& initial_point_cloud_pose,
                         const transform::Rigid2d& point_cloud_pose);
  // 根据trajectory_id分类vector, vector按照时间关系加入submap信息
  void AddSubmap(int trajectory_id, const transform::Rigid2d& submap_pose);

  // 设置ceres最大迭代次数
  void SetMaxNumIterations(int32 max_num_iterations);

  // Computes the optimized poses.
  /** 
   * @brief 后端优化核心函数，通过Ceres优化库，调整子图和路径节点的世界位姿
   * @param constraints 位姿图的约束
   */
  void Solve(const std::vector<Constraint>& constraints);

  const std::vector<std::vector<NodeData>>& node_data() const;
  const std::vector<std::vector<SubmapData>>& submap_data() const;

 private:
  mapping::sparse_pose_graph::proto::OptimizationProblemOptions options_;
  // 根据trajectory_id分类vector，deque按照时间关系记录的IMU数据
  std::vector<std::deque<mapping_3d::ImuData>> imu_data_;
  // 根据trajectory_id分类vector, vector按照时间关系加入node信息
  std::vector<std::vector<NodeData>> node_data_;
  // 根据trajectory_id分类vector, vector按照时间关系加入submap信息
  std::vector<std::vector<SubmapData>> submap_data_; 
};

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_OPTIMIZATION_PROBLEM_H_
