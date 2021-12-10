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

#include "cartographer/mapping/map_builder.h"

#include <cmath>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/collated_trajectory_builder.h"
#include "cartographer/mapping_2d/global_trajectory_builder.h"
#include "cartographer/mapping_3d/global_trajectory_builder.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

//backpack_3d.lua/revo_lds.lua/taurob_tracker.lua
/*
include "map_builder.lua"
include "trajectory_builder.lua"
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = false,
  use_laser_scan = false,
  use_multi_echo_laser_scan = false,
  num_point_clouds = 2,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
}
TRAJECTORY_BUILDER_3D.scans_per_accumulation = 160
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7
MAP_BUILDER.sparse_pose_graph.optimization_problem.huber_scale = 5e2
MAP_BUILDER.sparse_pose_graph.optimize_every_n_scans = 320
MAP_BUILDER.sparse_pose_graph.constraint_builder.sampling_ratio = 0.03
MAP_BUILDER.sparse_pose_graph.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- Reuse the coarser 3D voxel filter to speed up the computation of loop closure
-- constraints.
MAP_BUILDER.sparse_pose_graph.constraint_builder.adaptive_voxel_filter = TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter
MAP_BUILDER.sparse_pose_graph.constraint_builder.min_score = 0.62
return options
*/
proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::MapBuilderOptions options;
  options.set_use_trajectory_builder_2d(
      parameter_dictionary->GetBool("use_trajectory_builder_2d"));
  options.set_use_trajectory_builder_3d(
      parameter_dictionary->GetBool("use_trajectory_builder_3d"));
  options.set_num_background_threads(
      parameter_dictionary->GetNonNegativeInt("num_background_threads"));
  *options.mutable_sparse_pose_graph_options() = CreateSparsePoseGraphOptions(
      parameter_dictionary->GetDictionary("sparse_pose_graph").get());
  CHECK_NE(options.use_trajectory_builder_2d(),
           options.use_trajectory_builder_3d());
  return options;
}

MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
  if (options.use_trajectory_builder_2d()) {
    sparse_pose_graph_2d_ = common::make_unique<mapping_2d::SparsePoseGraph>(
        options_.sparse_pose_graph_options(), &thread_pool_);
    sparse_pose_graph_ = sparse_pose_graph_2d_.get();
  }
  if (options.use_trajectory_builder_3d()) {
    sparse_pose_graph_3d_ = common::make_unique<mapping_3d::SparsePoseGraph>(
        options_.sparse_pose_graph_options(), &thread_pool_);
    sparse_pose_graph_ = sparse_pose_graph_3d_.get();
  }
}

MapBuilder::~MapBuilder() {}

// 根据传感器id和options新建一个轨迹线，返回轨迹线的索引
int MapBuilder::AddTrajectoryBuilder(
    const std::unordered_set<string>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options) {
  const int trajectory_id = trajectory_builders_.size();
  if (options_.use_trajectory_builder_3d()) {
    CHECK(trajectory_options.has_trajectory_builder_3d_options());
    trajectory_builders_.push_back(
      // class CollatedTrajectoryBuilder : public TrajectoryBuilder
        common::make_unique<CollatedTrajectoryBuilder>(
            &sensor_collator_, trajectory_id, expected_sensor_ids,
            common::make_unique<mapping_3d::GlobalTrajectoryBuilder>(
                trajectory_options.trajectory_builder_3d_options(),
                trajectory_id, sparse_pose_graph_3d_.get())));
  } else {
    CHECK(trajectory_options.has_trajectory_builder_2d_options());
    trajectory_builders_.push_back(
        common::make_unique<CollatedTrajectoryBuilder>(
            &sensor_collator_, trajectory_id, expected_sensor_ids,
            common::make_unique<mapping_2d::GlobalTrajectoryBuilder>(
                trajectory_options.trajectory_builder_2d_options(),
                trajectory_id, sparse_pose_graph_2d_.get())));
  }
  return trajectory_id;
}

// 根据轨迹id返回指向该轨迹的TrajectoryBuilder对象指针。
TrajectoryBuilder* MapBuilder::GetTrajectoryBuilder(
    const int trajectory_id) const {
  // const int trajectory_id = trajectory_builders_.size();
  // 根据传感器id和options新建一个轨迹线，返回轨迹线的索引
  // vector的第几个就是第几条轨迹线
  return trajectory_builders_.at(trajectory_id).get();
}

// 标记该轨迹已完成data采集，后续不再接收data
void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_.FinishTrajectory(trajectory_id);
}

// 阻塞的轨迹，常见于该条轨迹上的传感器迟迟不提交data。
int MapBuilder::GetBlockingTrajectoryId() const {
  return sensor_collator_.GetBlockingTrajectoryId();
}

// 把轨迹id和子图索引对应的submap，序列化到文件
string MapBuilder::SubmapToProto(const mapping::SubmapId& submap_id,
                                 proto::SubmapQuery::Response* const response) {
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }

  const int num_submaps =
      sparse_pose_graph_->num_submaps(submap_id.trajectory_id);
  if (submap_id.submap_index < 0 || submap_id.submap_index >= num_submaps) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but there are only " + std::to_string(num_submaps) +
           " submaps in this trajectory.";
  }

  const auto submap_data = sparse_pose_graph_->GetSubmapData(submap_id);
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it has been trimmed.";
  }
  submap_data.submap->ToResponseProto(submap_data.pose, response);
  return "";
}

// 在建图的轨迹数量
int MapBuilder::num_trajectory_builders() const {
  return trajectory_builders_.size();
}

SparsePoseGraph* MapBuilder::sparse_pose_graph() { return sparse_pose_graph_; }

}  // namespace mapping
}  // namespace cartographer
