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

#include "cartographer/mapping_2d/sparse_pose_graph/constraint_builder.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping_2d/scan_matching/proto/ceres_scan_matcher_options.pb.h"
#include "cartographer/mapping_2d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {
namespace sparse_pose_graph {

transform::Rigid2d ComputeSubmapPose(const Submap& submap) {
  return transform::Project2D(submap.local_pose());
}

/*
 * 构造函数，它有两个输入参数，分别记录了配置项和线程池对象。 这个配置项options
 * 早在Cartographer的ROS入口中就已经从配置文件中加载了。 
 * 而线程池对象thread_pool则指向地图构建器对象map_builder的一个成员变量。 
 * 在构造函数成员构造列表中，直接将这两个对象赋予了成员变量options_和thread_pool_。
 * 同时根据参数配置完成了采样器对象sampler_、滤波器对象adaptive_voxel_filter_
 * 和基于Ceres的扫描匹配器ceres_scan_matcher_的初始构造。
 */
ConstraintBuilder::ConstraintBuilder(
    const mapping::sparse_pose_graph::proto::ConstraintBuilderOptions& options,
    common::ThreadPool* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      sampler_(options.sampling_ratio()),
      adaptive_voxel_filter_(options.adaptive_voxel_filter_options()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) {}

/*
 * 析构函数，该函数实际没有什么具体的操作，只检查了各种状态，
 * 保证释放对象的时候没有任务在后台运行。
 */
ConstraintBuilder::~ConstraintBuilder() {
  common::MutexLocker locker(&mutex_);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(pending_computations_.size(), 0);
  CHECK_EQ(submap_queued_work_items_.size(), 0);
  CHECK(when_done_ == nullptr);
}

/*
 * 有5个输入参数，
 * 其中submap_id和node_id分别是子图和路径节点的索引。 
 * 指针submap和point_cloud分别指向了考察的子图对象和路径节点中记录了激光点云数据，
 * 需要注意的是这两个对象的生命周期应当能够覆盖后端优化的计算过程。 
 * initial_relative_pose记录了路径节点相对于子图的初始位姿，提供了优化迭代的一个初值。
 */
void ConstraintBuilder::MaybeAddConstraint(
    const mapping::SubmapId& submap_id, const Submap* const submap,
    const mapping::NodeId& node_id, const sensor::PointCloud* const point_cloud,
    const transform::Rigid2d& initial_relative_pose) {
  // 如果初始的相对位姿显示，路径节点与子图相差很远，就直接返回不在两者之间建立约束
  // 这个距离阈值可以通过配置项max_constraint_distance来设定。这样做有两个好处， 
  // 其一可以一定程度上降低约束的数量，减少全局图优化的计算量，提高系统的运行效率; 
  // 其二，如果两者相差太远，很可能会受到累积误差的影响，导致添加错误的约束，
  // 给最终的全局优化带来负面的影响， 这样直接抛弃一些不太合理的约束，看似遗漏了很多信息，
  // 但对于优化结果而言可能是一件好事。
  if (initial_relative_pose.translation().norm() >
      options_.max_constraint_distance()) {
    return;
  }
  // 设定固定时间添加一个约束，时间太短则不添加，采样频率0.3
  if (sampler_.Pulse()) {
    // 通过互斥量对象mutex_加锁
    common::MutexLocker locker(&mutex_);
    // 向容器constraints_中添加新的约束
    constraints_.emplace_back();
    auto* const constraint = &constraints_.back();
    ++pending_computations_[current_computation_];
    const int current_computation = current_computation_;
    // 通过函数ScheduleSubmapScanMatcherConstructionAndQueueWorkItem构建了一个扫描匹配器
    // 这个函数并没有完成扫描匹配器的构建，而是通过lambda表达式和线程池推后实现的
    // lambda表达式中调用的函数ComputeConstraint具体完成了约束的计算。
    // 如果A函数调用了B函数，B函数排除了某个能力EXCLUDES(mutex_)，A函数就不能递归排除该功能
    // work_item就是lambda表达式
    ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
        submap_id, &submap->probability_grid(), [=]() EXCLUDES(mutex_) {
          ComputeConstraint(submap_id, submap, node_id,
                            false,   /* match_full_submap */
                            nullptr, /* trajectory_connectivity */
                            point_cloud, initial_relative_pose, constraint);
          FinishComputation(current_computation);
        });
  }
}

// 计算子图和路径节点之间是否存在可能的约束,所不同的是，该接口只有5个输入参数，
// 没有提供初始相对位姿， 而且它的扫描匹配是在整个子图上进行的
void ConstraintBuilder::MaybeAddGlobalConstraint(
    const mapping::SubmapId& submap_id, const Submap* const submap,
    const mapping::NodeId& node_id, const sensor::PointCloud* const point_cloud,
    mapping::TrajectoryConnectivity* const trajectory_connectivity) {
  common::MutexLocker locker(&mutex_);
  constraints_.emplace_back();
  auto* const constraint = &constraints_.back();
  ++pending_computations_[current_computation_];
  const int current_computation = current_computation_;
  ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
      submap_id, &submap->probability_grid(), [=]() EXCLUDES(mutex_) {
        ComputeConstraint(submap_id, submap, node_id,
                          true, /* match_full_submap */
                          trajectory_connectivity, point_cloud,
                          transform::Rigid2d::Identity(), constraint);
        FinishComputation(current_computation);
      });
}

/*
 * 闭环检测是在每次向后端添加路径节点的时候触发的。 除了要建立路径节点与新旧子图之间的内部约束之外，
 * 还要与所有处于kFinished状态的子图进行一次匹配计算可能存在的约束，
 * 如果旧子图切换到kFinished状态还需要将之与所有路径节点进行匹配。完成了这些操作之后， 
 * 它就会调用接口NotifyEndOfNode， 通知对象constraint_builder_完成了一个路径节点的插入工作
 */
void ConstraintBuilder::NotifyEndOfScan() {
  common::MutexLocker locker(&mutex_);
  ++current_computation_;
}

// 只有一个输入参数，记录了当所有的闭环检测任务结束之后的回调函数
void ConstraintBuilder::WhenDone(
    const std::function<void(const ConstraintBuilder::Result&)> callback) {
  common::MutexLocker locker(&mutex_);
  CHECK(when_done_ == nullptr);
  // 指针when_done_记录下回调函数
  when_done_ =
      common::make_unique<std::function<void(const Result&)>>(callback);
  ++pending_computations_[current_computation_];
  const int current_computation = current_computation_;
  // 添加到线程池的调度队列中
  thread_pool_->Schedule(
      [this, current_computation] { FinishComputation(current_computation); });
}

// 通过函数ScheduleSubmapScanMatcherConstructionAndQueueWorkItem构建了一个扫描匹配器
// 这个函数并没有完成扫描匹配器的构建，而是通过lambda表达式和线程池推后实现的
// 如果已有扫描匹配器，work_item直接加入线程池，否则，先创建，work_item再加入线程池
void ConstraintBuilder::ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
    const mapping::SubmapId& submap_id, const ProbabilityGrid* const submap,
    const std::function<void()> work_item) {
  if (submap_scan_matchers_[submap_id].fast_correlative_scan_matcher !=
      nullptr) {
    thread_pool_->Schedule(work_item);
  } else {
    // 将计算约束的任务添加到线程池的调度队列中，并将其设置为完成轨迹节点约束计算任务的依赖，
    // 保证在完成了所有计算约束的任务之后才会执行constraint_task的计算任务。
    submap_queued_work_items_[submap_id].push_back(work_item);
    if (submap_queued_work_items_[submap_id].size() == 1) {
      // 将构建扫描匹配器的任务放置到线程池的调度队列中
      thread_pool_->Schedule(
          [=]() { ConstructSubmapScanMatcher(submap_id, submap); });
    }
  }
}

// 为每个子图都构建一个SubmapScanMatcher类型的扫描匹配器
// 先创建扫描匹配器，work_item再加入线程池
void ConstraintBuilder::ConstructSubmapScanMatcher(
    const mapping::SubmapId& submap_id, const ProbabilityGrid* const submap) {
  // 成功创建的匹配器将以submap_id为索引被保存在容器submap_scan_matchers_中
  auto submap_scan_matcher =
      common::make_unique<scan_matching::FastCorrelativeScanMatcher>(
          *submap, options_.fast_correlative_scan_matcher_options());
  common::MutexLocker locker(&mutex_);
  submap_scan_matchers_[submap_id] = {submap, std::move(submap_scan_matcher)};
  for (const std::function<void()>& work_item :
       submap_queued_work_items_[submap_id]) {
    thread_pool_->Schedule(work_item);
  }
  submap_queued_work_items_.erase(submap_id);
}

const ConstraintBuilder::SubmapScanMatcher*
ConstraintBuilder::GetSubmapScanMatcher(const mapping::SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  const SubmapScanMatcher* submap_scan_matcher =
      &submap_scan_matchers_[submap_id];
  CHECK(submap_scan_matcher->fast_correlative_scan_matcher != nullptr);
  return submap_scan_matcher;
}

// 在后台线程中完成子图与路径节点之间可能的约束计算
/*
 * submap_id、submap、node_id、point_cloud分别是待考察的子图和路径节点的索引和数据内容。
 * 布尔变量match_full_submap用于指示是否完成遍历子图。 
 * trajectory_connectivity  联通区域
 * initial_relative_pose描述了路径节点相对于子图的初始位姿，
 * constraint则用于输出约束结果。
 */
void ConstraintBuilder::ComputeConstraint(
    const mapping::SubmapId& submap_id, const Submap* const submap,
    const mapping::NodeId& node_id, bool match_full_submap,
    mapping::TrajectoryConnectivity* trajectory_connectivity,
    const sensor::PointCloud* const point_cloud,
    const transform::Rigid2d& initial_relative_pose,
    std::unique_ptr<ConstraintBuilder::Constraint>* constraint) {
  // initial_pose用于描述在世界坐标系下路径节点与子图之间的相对位置关系
  const transform::Rigid2d initial_pose =
      ComputeSubmapPose(*submap) * initial_relative_pose;
  const SubmapScanMatcher* const submap_scan_matcher =
      GetSubmapScanMatcher(submap_id);
  const sensor::PointCloud filtered_point_cloud =
      adaptive_voxel_filter_.Filter(*point_cloud);

  // The 'constraint_transform' (i <- j) is computed from:
  // - a 'filtered_point_cloud' in j,
  // - the initial guess 'initial_pose' for (map <- j),
  // - the result 'pose_estimate' of Match() (map <- j).
  // - the ComputeSubmapPose() (map <- i)
  // score和pose_estimate用于记录扫描匹配之后的得分和位姿估计
  float score = 0.;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

  // 根据输入参数match_full_submap选择不同的扫描匹配方式，如果没有匹配成功则直接退出。
  if (match_full_submap) {
    if (submap_scan_matcher->fast_correlative_scan_matcher->MatchFullSubmap(
            filtered_point_cloud, options_.global_localization_min_score(),
            &score, &pose_estimate)) {
      CHECK_GT(score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
      trajectory_connectivity->Connect(node_id.trajectory_id,
                                       submap_id.trajectory_id);
    } else {
      return;
    }
  } else {
    if (submap_scan_matcher->fast_correlative_scan_matcher->Match(
            initial_pose, filtered_point_cloud, options_.min_score(), &score,
            &pose_estimate)) {
      // We've reported a successful local match.
      CHECK_GT(score, options_.min_score());
    } else {
      return;
    }
  }
  {
    // 在一个局部的locker的保护下，将新获得的约束得分统计到一个直方图中
    common::MutexLocker locker(&mutex_);
    score_histogram_.Add(score);
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  // 最后使用Ceres扫描匹配进一步的对刚刚构建的约束进行优化，
  // 并将这种新建的约束标记为INTER_SUBMAP类型。
  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.Match(pose_estimate, pose_estimate, filtered_point_cloud,
                            *submap_scan_matcher->probability_grid,
                            &pose_estimate, &unused_summary);

  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;
  constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    options_.loop_closure_translation_weight(),
                                    options_.loop_closure_rotation_weight()},
                                   Constraint::INTER_SUBMAP});

  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with " << filtered_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const transform::Rigid2d difference =
          initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(difference.normalized_angle());
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}

// 当一轮MaybeAdd-WhenDone任务结束后，用来调用WhenDone接口注册的回调函数的
// current_computation  :   computation_index
void ConstraintBuilder::FinishComputation(const int computation_index) {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    common::MutexLocker locker(&mutex_);
    if (--pending_computations_[computation_index] == 0) {
      pending_computations_.erase(computation_index);
    }
    if (pending_computations_.empty()) {
      CHECK_EQ(submap_queued_work_items_.size(), 0);
      if (when_done_ != nullptr) {
        for (const std::unique_ptr<Constraint>& constraint : constraints_) {
          if (constraint != nullptr) {
            // 将MaybeAdd过程中得到的约束放到result对象中
            result.push_back(*constraint);
          }
        }
        if (options_.log_matches()) {
          LOG(INFO) << constraints_.size() << " computations resulted in "
                    << result.size() << " additional constraints.";
          LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
        }
        // 清空约束列表constraints_
        constraints_.clear();
        // 用临时变量callback记录下回调函数对象
        callback = std::move(when_done_);
        // 释放when_done_指针
        when_done_.reset();
      }
    }
  }
  // 将result作为参数传递给回调函数
  if (callback != nullptr) {
    (*callback)(result);
  }
}

int ConstraintBuilder::GetNumFinishedScans() {
  common::MutexLocker locker(&mutex_);
  if (pending_computations_.empty()) {
    return current_computation_;
  }
  // C ++ map begin()函数用于返回引用map容器第一个元素的迭代器。
  return pending_computations_.begin()->first;
}

void ConstraintBuilder::DeleteScanMatcher(const mapping::SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  CHECK(pending_computations_.empty());
  submap_scan_matchers_.erase(submap_id);
}

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer
