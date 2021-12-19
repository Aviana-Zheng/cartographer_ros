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

#ifndef CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_
#define CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/sparse_pose_graph/proto/constraint_builder_options.pb.h"
#include "cartographer/mapping/trajectory_connectivity.h"
#include "cartographer/mapping_2d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_2d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_3d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/voxel_filter.h"

namespace cartographer {
namespace mapping_2d {
namespace sparse_pose_graph {

// Returns (map <- submap) where 'submap' is a coordinate system at the origin
// of the Submap.
transform::Rigid2d ComputeSubmapPose(const Submap& submap);

/* Asynchronously(异步的) computes constraints.
 * 进行闭环检测并构建约束
 * 
 * Intermingle(混搭) an arbitrary(任意的) number of calls to MaybeAddConstraint() or
 * MaybeAddGlobalConstraint, then call WhenDone(). After all computations are
 * done the 'callback' will be called with the result and another
 * MaybeAdd(Global)Constraint()/WhenDone() cycle can follow.
 * 我们可以在任意调用MaybeAddConstraint、MaybeAddGlobalConstraint、NotifyEndOfNode之后，
 * 调用一次WhenDone接口，来注册回调函数结束一轮操作。 如此不断的重复这个过程就可以持续地进行
 * 闭环检测，添加必要的约束。
 * 
 * This class is thread-safe.
 */
class ConstraintBuilder {
 public:
  using Constraint = mapping::SparsePoseGraph::Constraint;
  using Result = std::vector<Constraint>;
 
  /*
   * 构造函数，它有两个输入参数，分别记录了配置项和线程池对象。 这个配置项options
   * 早在Cartographer的ROS入口中就已经从配置文件中加载了。 
   * 而线程池对象thread_pool则指向地图构建器对象map_builder的一个成员变量。 
   * 在构造函数成员构造列表中，直接将这两个对象赋予了成员变量options_和thread_pool_。
   * 同时根据参数配置完成了采样器对象sampler_、滤波器对象adaptive_voxel_filter_
   * 和基于Ceres的扫描匹配器ceres_scan_matcher_的初始构造。
   */
  ConstraintBuilder(
      const mapping::sparse_pose_graph::proto::ConstraintBuilderOptions&
          options,
      common::ThreadPool* thread_pool);
  /*
   * 析构函数，该函数实际没有什么具体的操作，只检查了各种状态，
   * 保证释放对象的时候没有任务在后台运行。
   */
  ~ConstraintBuilder();
  
  // 屏蔽拷贝构造函数和拷贝赋值操作符
  ConstraintBuilder(const ConstraintBuilder&) = delete;
  ConstraintBuilder& operator=(const ConstraintBuilder&) = delete;

  // Schedules(时刻表，计划) exploring a new constraint between 'submap' identified by
  // 'submap_id', and the 'point_cloud' for 'node_id'. The 'initial_pose' is
  // relative(相对) to the 'submap'.
  //
  // The pointees of 'submap' and 'point_cloud' must stay valid until all
  // computations are finished.
  // 检查子图和路径节点之间是否存在可能的约束
  /*
   * 有5个输入参数，
   * 其中submap_id和node_id分别是子图和路径节点的索引。 
   * 指针submap和point_cloud分别指向了考察的子图对象和路径节点中记录了激光点云数据，
   * 需要注意的是这两个对象的生命周期应当能够覆盖后端优化的计算过程。 
   * initial_relative_pose记录了路径节点相对于子图的初始位姿，提供了优化迭代的一个初值。
   */
  void MaybeAddConstraint(const mapping::SubmapId& submap_id,
                          const Submap* submap, const mapping::NodeId& node_id,
                          const sensor::PointCloud* point_cloud,
                          const transform::Rigid2d& initial_relative_pose);

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id' and the 'point_cloud' for 'node_id'. This performs full-submap
  // matching.
  //
  // The 'trajectory_connectivity' is updated if the full-submap match succeeds.
  //
  // The pointees of 'submap' and 'point_cloud' must stay valid until all
  // computations are finished.
  // 计算子图和路径节点之间是否存在可能的约束,所不同的是，该接口只有四个输入参数，
  // 没有提供初始相对位姿， 而且它的扫描匹配是在整个子图上进行的
  void MaybeAddGlobalConstraint(
      const mapping::SubmapId& submap_id, const Submap* submap,
      const mapping::NodeId& node_id, const sensor::PointCloud* point_cloud,
      mapping::TrajectoryConnectivity* trajectory_connectivity);

  // Must be called after all computations related to one node have been added.
  /*
   * 闭环检测是在每次向后端添加路径节点的时候触发的。 除了要建立路径节点与新旧子图之间的内部约束之外，
   * 还要与所有处于kFinished状态的子图进行一次匹配计算可能存在的约束，
   * 如果旧子图切换到kFinished状态还需要将之与所有路径节点进行匹配。完成了这些操作之后， 
   * 它就会调用接口NotifyEndOfNode， 通知对象constraint_builder_完成了一个路径节点的插入工作
   */
  void NotifyEndOfScan();

  // Registers the 'callback' to be called with the results, after all
  // computations triggered by MaybeAddConstraint() have finished.
  void WhenDone(std::function<void(const Result&)> callback);

  // Returns the number of consecutive finished scans.
  int GetNumFinishedScans();

  // Delete data related to 'submap_id'.
  void DeleteScanMatcher(const mapping::SubmapId& submap_id);

 private:
  // 为每个子图都构建一个SubmapScanMatcher类型的扫描匹配器。
  // 这个扫描匹配器是定义在类ConstraintBuilder2D内部的一个结构体
  // 它有2个字段，分别记录子图的占用栅格、扫描匹配器内核
  struct SubmapScanMatcher {
    const ProbabilityGrid* probability_grid; // 子图的占用栅格
    std::unique_ptr<scan_matching::FastCorrelativeScanMatcher>
        fast_correlative_scan_matcher;   // 扫描匹配器内核
  };

  // Either schedules the 'work_item', or if needed, schedules the scan matcher
  // construction and queues the 'work_item'.
  // 通过函数ScheduleSubmapScanMatcherConstructionAndQueueWorkItem构建了一个扫描匹配器
  // 这个函数并没有完成扫描匹配器的构建，而是通过lambda表达式和线程池推后实现的
  void ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
      const mapping::SubmapId& submap_id, const ProbabilityGrid* submap,
      std::function<void()> work_item) REQUIRES(mutex_);

  // Constructs the scan matcher for a 'submap', then schedules its work items.
  // 为每个子图都构建一个SubmapScanMatcher类型的扫描匹配器
  void ConstructSubmapScanMatcher(const mapping::SubmapId& submap_id,
                                  const ProbabilityGrid* submap)
      EXCLUDES(mutex_);

  // Returns the scan matcher for a submap, which has to exist.
  const SubmapScanMatcher* GetSubmapScanMatcher(
      const mapping::SubmapId& submap_id) EXCLUDES(mutex_);

  // Runs in a background thread and does computations for an additional
  // constraint, assuming 'submap' and 'point_cloud' do not change anymore.
  // If 'match_full_submap' is true, and global localization succeeds, will
  // connect 'node_id.trajectory_id' and 'submap_id.trajectory_id' in
  // 'trajectory_connectivity'.
  // As output, it may create a new Constraint in 'constraint'.
  // 在后台线程中完成子图与路径节点之间可能的约束计算
  /*
   * submap_id、submap、node_id、point_cloud分别是待考察的子图和路径节点的索引和数据内容。
   * 布尔变量match_full_submap用于指示是否完成遍历子图。 
   * trajectory_connectivity  联通区域
   * initial_relative_pose描述了路径节点相对于子图的初始位姿，
   * constraint则用于输出约束结果。
   */
  void ComputeConstraint(
      const mapping::SubmapId& submap_id, const Submap* submap,
      const mapping::NodeId& node_id, bool match_full_submap,
      mapping::TrajectoryConnectivity* trajectory_connectivity,
      const sensor::PointCloud* point_cloud,
      const transform::Rigid2d& initial_relative_pose,
      std::unique_ptr<Constraint>* constraint) EXCLUDES(mutex_);

  // Decrements the 'pending_computations_' count. If all computations are done,
  // runs the 'when_done_' callback and resets the state.
  // 当一轮MaybeAdd-WhenDone任务结束后，用来调用WhenDone接口注册的回调函数的
  void FinishComputation(int computation_index) EXCLUDES(mutex_);
  
  // 关于约束构建器的各种配置
  const mapping::sparse_pose_graph::proto::ConstraintBuilderOptions options_;
  // 线程池，用于并行地完成闭环检测
  common::ThreadPool* thread_pool_;
  // 保护公共资源的互斥量
  common::Mutex mutex_;

  // 'callback' set by WhenDone().
  // 通过接口WhenDone注册的回调函数对象
  std::unique_ptr<std::function<void(const Result&)>> when_done_
      GUARDED_BY(mutex_);

  // Index of the scan in reaction to which computations are currently
  // added. This is always the highest scan index seen so far, even when older
  // scans are matched against a new submap.
  // 当前添加计算的扫描索引。 这始终是迄今为止看到的最高扫描索引，即使旧扫描与新子图匹配。 
  int current_computation_ GUARDED_BY(mutex_) = 0;

  // For each added scan, maps to the number of pending(待办的) computations that were
  // added for it.
  // 对于每个添加的扫描，映射到为其添加的挂起计算的数量。 
  std::map<int, int> pending_computations_ GUARDED_BY(mutex_);

  // Constraints currently being computed in the background. A deque is used to
  // keep pointers valid when adding more entries.
  // 用于保存后台计算的约束的双端队列
  std::deque<std::unique_ptr<Constraint>> constraints_ GUARDED_BY(mutex_);

  // Map of already constructed scan matchers by 'submap_id'.
  // 记录各个子图的扫描匹配器的容器，以SubmapId为索引。
  std::map<mapping::SubmapId, SubmapScanMatcher> submap_scan_matchers_
      GUARDED_BY(mutex_);

  // Map by 'submap_id' of scan matchers under construction, and the work
  // to do once construction is done.
  // 通过正在构建的扫描匹配器的“submap_id”映射，以及构建完成后要做的工作。 
  std::map<mapping::SubmapId, std::vector<std::function<void()>>>
      submap_queued_work_items_ GUARDED_BY(mutex_);
  
  // 采样器
  common::FixedRatioSampler sampler_;
  // 点云滤波器
  const sensor::AdaptiveVoxelFilter adaptive_voxel_filter_;
  // 基于Ceres库的扫描匹配器
  scan_matching::CeresScanMatcher ceres_scan_matcher_;

  // Histogram of scan matcher scores.
  // 扫描匹配得分的直方图
  common::Histogram score_histogram_ GUARDED_BY(mutex_);
};

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_
