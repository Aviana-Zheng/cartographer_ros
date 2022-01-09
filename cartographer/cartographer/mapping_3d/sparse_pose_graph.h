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

#ifndef CARTOGRAPHER_MAPPING_3D_SPARSE_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_3D_SPARSE_POSE_GRAPH_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/trajectory_connectivity.h"
#include "cartographer/mapping_3d/sparse_pose_graph/constraint_builder.h"
#include "cartographer/mapping_3d/sparse_pose_graph/optimization_problem.h"
#include "cartographer/mapping_3d/submaps.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping_3d {

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping in 3D:
// Each scan has been matched against one or more submaps (adding a constraint
// for each match), both poses of scans and of submaps are to be optimized.
// All constraints are between a submap i and a scan j.
class SparsePoseGraph : public mapping::SparsePoseGraph {
 public:
  SparsePoseGraph(const mapping::proto::SparsePoseGraphOptions& options,
                  common::ThreadPool* thread_pool);
  ~SparsePoseGraph() override;

  SparsePoseGraph(const SparsePoseGraph&) = delete;
  SparsePoseGraph& operator=(const SparsePoseGraph&) = delete;

  // Adds a new 'range_data_in_tracking' observation at 'time', and a 'pose'
  // that will later be optimized. The 'pose' was determined by scan matching
  // against 'insertion_submaps.front()' and the scan was inserted into the
  // 'insertion_submaps'. If 'insertion_submaps.front().finished()' is 'true',
  // this submap was inserted into for the last time.
  void AddScan(
      common::Time time, const sensor::RangeData& range_data_in_tracking,
      const transform::Rigid3d& pose, int trajectory_id,
      const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
      EXCLUDES(mutex_);

  // Adds new IMU data to be used in the optimization.
  void AddImuData(int trajectory_id, common::Time time,
                  const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity);

  void AddTrimmer(std::unique_ptr<mapping::PoseGraphTrimmer> trimmer) override {
    LOG(FATAL) << "Not yet implemented for 3D.";
  }
  void RunFinalOptimization() override;
  std::vector<std::vector<int>> GetConnectedTrajectories() override;
  int num_submaps(int trajectory_id) EXCLUDES(mutex_) override;
  mapping::SparsePoseGraph::SubmapData GetSubmapData(
      const mapping::SubmapId& submap_id) EXCLUDES(mutex_) override;
  std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>>
  GetAllSubmapData() EXCLUDES(mutex_) override;
  transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id)
      EXCLUDES(mutex_) override;
  std::vector<std::vector<mapping::TrajectoryNode>> GetTrajectoryNodes()
      override EXCLUDES(mutex_);
  std::vector<Constraint> constraints() override EXCLUDES(mutex_);

 private:
  // The current state of the submap in the background threads. When this
  // transitions to kFinished, all scans are tried to match against this submap.
  // Likewise, all new scans are matched against submaps which are finished.
  // 子图的状态主要是给后台的线程提供的
  /* 
   * 一开始子图的状态都是kActive的，当它切换到kFinished的状态下后就会与所有的节点
   * 进行一次扫描匹配操作。此外新增的节点也会与所有kFinished状态的子图进行扫描匹配。 
   * 这一操作我们可以理解为是进行闭环检测，通过遍历与所有的kFinished状态的子图，或者节点，
   * 应当可以找到发生闭环的地点并建立一个约束来描述
   */
  enum class SubmapState { kActive, kFinished, kTrimmed };
  struct SubmapData {
    // 记录了具体的子图对象
    std::shared_ptr<const Submap> submap;

    // IDs of the scans that were inserted into this map together with
    // constraints for them. They are not to be matched again when this submap
    // becomes 'finished'.
    // 记录了所有直接插入submap的节点
    std::set<mapping::NodeId> node_ids;

    // 记录了子图的状态
    SubmapState state = SubmapState::kActive;
  };

  // Handles a new work item.
  void AddWorkItem(std::function<void()> work_item) REQUIRES(mutex_);

  // Grows the optimization problem to have an entry for every element of
  // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
  std::vector<mapping::SubmapId> GrowSubmapTransformsAsNeeded(
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
      REQUIRES(mutex_);

  // Adds constraints for a scan, and starts scan matching in the background.
  void ComputeConstraintsForScan(
      int trajectory_id,
      std::vector<std::shared_ptr<const Submap>> insertion_submaps,
      bool newly_finished_submap, const transform::Rigid3d& pose)
      REQUIRES(mutex_);

  // Computes constraints for a scan and submap pair.
  void ComputeConstraint(const mapping::NodeId& node_id,
                         const mapping::SubmapId& submap_id) REQUIRES(mutex_);

  // Adds constraints for older scans whenever a new submap is finished.
  void ComputeConstraintsForOldScans(const mapping::SubmapId& submap_id)
      REQUIRES(mutex_);

  // Registers the callback to run the optimization once all constraints have
  // been computed, that will also do all work that queue up in 'scan_queue_'.
  void HandleScanQueue() REQUIRES(mutex_);

  // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
  // all computations have finished.
  void WaitForAllComputations() EXCLUDES(mutex_);

  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  void RunOptimization() EXCLUDES(mutex_);

  // Computes the local to global frame transform based on the given optimized
  // 'submap_transforms'.
  transform::Rigid3d ComputeLocalToGlobalTransform(
      const std::vector<std::vector<sparse_pose_graph::SubmapData>>&
          submap_transforms,
      int trajectory_id) const REQUIRES(mutex_);

  mapping::SparsePoseGraph::SubmapData GetSubmapDataUnderLock(
      const mapping::SubmapId& submap_id) REQUIRES(mutex_);

  // 位姿图的各种配置
  const mapping::proto::SparsePoseGraphOptions options_;
  // 用于多线程运行时保护重要数据资源的互斥量
  common::Mutex mutex_;

  // If it exists, further scans must be added to this queue, and will be
  // considered later.  智能指针形式的工作队列，用于记录将要完成的任务
  std::unique_ptr<std::deque<std::function<void()>>> scan_queue_
      GUARDED_BY(mutex_);

  // How our various trajectories are related.描述不同轨迹之间的连接状态
  mapping::TrajectoryConnectivity trajectory_connectivity_ GUARDED_BY(mutex_);

  // We globally localize a fraction of the scans from each trajectory.
  // 以trajectory_id为索引的字典，用于对各个轨迹上的部分节点进行全局定位的采样器
  std::unordered_map<int, std::unique_ptr<common::FixedRatioSampler>>
      global_localization_samplers_ GUARDED_BY(mutex_);

  // Number of scans added since last loop closure.
  // 一个计数器，记录了自从上次闭环检测之后新增的节点数量，用以判断是否开始后端优化
  int num_scans_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;

  // Whether the optimization has to be run before more data is added.
  // 标识当前是否正在进行闭环检测。
  bool run_loop_closure_ GUARDED_BY(mutex_) = false;

  // Current optimization problem.
  // 描述当前优化问题的对象，应该是PoseGraph2D的核心。
  sparse_pose_graph::OptimizationProblem optimization_problem_;
  // 约束构造器，用于异步的计算约束。
  sparse_pose_graph::ConstraintBuilder constraint_builder_ GUARDED_BY(mutex_);
  // 记录了位姿图中的所有约束。
  std::vector<Constraint> constraints_ GUARDED_BY(mutex_);

  // Submaps get assigned an ID and state as soon as they are seen, even
  // before they take part in the background computations.
  // 该容器记录了所有的子图数据及其内部节点，其中NestedVectorsById是对std::map的一个封装，
  // SubmapData除了描述了子图的数据之外还记录了所有内部的节点。
  mapping::NestedVectorsById<SubmapData, mapping::SubmapId> submap_data_
      GUARDED_BY(mutex_);

  // Connectivity structure of our trajectories by IDs.
  std::vector<std::vector<int>> connected_components_;
  // Trajectory ID to connected component ID.
  std::map<int, size_t> reverse_connected_components_;

  // Data that are currently being shown.记录轨迹节点的容器
  mapping::NestedVectorsById<mapping::TrajectoryNode, mapping::NodeId>
      trajectory_nodes_ GUARDED_BY(mutex_);
  // 添加的节点数量，一个节点一次闭环检测
  int num_trajectory_nodes_ GUARDED_BY(mutex_) = 0;

  // Current submap transforms used for displaying data.
  std::vector<std::vector<sparse_pose_graph::SubmapData>>
      optimized_submap_transforms_ GUARDED_BY(mutex_);
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SPARSE_POSE_GRAPH_H_
