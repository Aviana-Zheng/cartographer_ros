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

#include "cartographer/mapping_3d/sparse_pose_graph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
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
#include "cartographer/mapping/sparse_pose_graph/proto/constraint_builder_options.pb.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/voxel_filter.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {

SparsePoseGraph::SparsePoseGraph(
    // options是从配置文件中装载的关于位姿图的配置项，
    const mapping::proto::SparsePoseGraphOptions& options,
    // thread_pool则是一个线程池
    common::ThreadPool* thread_pool)
    : options_(options),
      optimization_problem_(options_.optimization_problem_options(),
                            sparse_pose_graph::OptimizationProblem::FixZ::kNo),
      constraint_builder_(options_.constraint_builder_options(), thread_pool) {}

SparsePoseGraph::~SparsePoseGraph() {
  WaitForAllComputations();
  common::MutexLocker locker(&mutex_);
  CHECK(scan_queue_ == nullptr);
}

// 后端优化的数据准备,将子图的初始位姿提供给后端优化器  trajectory_id是运行轨迹的索引
// insertion_submaps则是从Local SLAM一路传递过来的新旧子图
// 输出是一个vector容器，它将记录insertion_submaps中各个子图分配的SubmapId
std::vector<mapping::SubmapId> SparsePoseGraph::GrowSubmapTransformsAsNeeded(
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap>>& insertion_submaps) {
  // 先检查insertion_submaps非空
  CHECK(!insertion_submaps.empty());
  // 获取后端优化器的子图位姿信息记录到临时对象submap_data中,返回std::vector<std::vector<SubmapData>>
  const auto& submap_data = optimization_problem_.submap_data();
  // 根据Local SLAM中子图的维护方式，如果输入参数insertion_submaps中只有一个子图，
  // 意味着重新开始了一条新的轨迹。
  if (insertion_submaps.size() == 1) {
    // If we don't already have an entry for the first submap, add one.
    if (static_cast<size_t>(trajectory_id) >= submap_data.size() ||
        submap_data[trajectory_id].empty()) {
      // 通过后端优化器的接口AddSubmap创建一条新的轨迹，并将子图的全局位姿信息喂给优化器
      optimization_problem_.AddSubmap(trajectory_id,
                                      insertion_submaps[0]->local_pose());
    }
    // 为子图赋予唯一的SubmapId
    const mapping::SubmapId submap_id{
        trajectory_id, static_cast<int>(submap_data[trajectory_id].size()) - 1};
    // SparsePoseGraph::AddScan已经将submap_data_更新
    CHECK(submap_data_.at(submap_id).submap == insertion_submaps.front());
    return {submap_id};
  }
  // 说明trajectory_id下已经至少有了一个子图
  // 此时输入的insertion_submaps中应当有两个子图，也就是Local SLAM中维护的新旧子图
  CHECK_EQ(2, insertion_submaps.size());
  // 用局部变量last_submap_id记录下后端优化器中最新子图的索引
  const mapping::SubmapId last_submap_id{
      trajectory_id,
      static_cast<int>(submap_data.at(trajectory_id).size() - 1)};

  // 根据last_submap_id检查一下后端优化器中最新的子图是否与insertion_submaps中的旧图是同一个对象。
  if (submap_data_.at(last_submap_id).submap == insertion_submaps.front()) {
    // In this case, 'last_submap_id' is the ID of 'insertions_submaps.front()'
    // and 'insertions_submaps.back()' is new.
    // 若是，则说明新图是Local SLAM新建的子图后端尚未记录。 此时需要将新图的全局位姿提供给后端优化器，
    // 并分配一个SubmapId。然后将新旧子图的SubmapId放到容器中一并返回。
    const auto& first_submap_pose =
        submap_data.at(trajectory_id).at(last_submap_id.submap_index).pose;
    optimization_problem_.AddSubmap(
        trajectory_id, first_submap_pose *
                           insertion_submaps[0]->local_pose().inverse() *
                           insertion_submaps[1]->local_pose());
    return {last_submap_id,
            mapping::SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
  }
  // 最后只剩下一种情况，Local SLAM并没有再新建子图了，此时后端中记录了所有的子图，
  // 只需要将新旧子图对应的SubmapId返回即可。
  CHECK(submap_data_.at(last_submap_id).submap == insertion_submaps.back());
  const mapping::SubmapId front_submap_id{trajectory_id,
                                          last_submap_id.submap_index - 1};
  CHECK(submap_data_.at(front_submap_id).submap == insertion_submaps.front());
  return {front_submap_id, last_submap_id};
}

// 添加点云数据 输入更新子图时的点云信息以及相对位姿
//  trajectory_id记录了轨迹索引，insertion_submaps则是更新的子图
void SparsePoseGraph::AddScan(
    common::Time time, const sensor::RangeData& range_data_in_tracking,
    const transform::Rigid3d& pose, const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap>>& insertion_submaps) {
  // 先将局部位姿转换成为世界坐标系下的位姿  T_newmap_local * T_local_scan = T_newmap_scan
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * pose);
  // 先对信号量加锁，以保证多线程的安全
  common::MutexLocker locker(&mutex_);
  // 根据输入的数据和刚刚生成的全局位姿构建一个TrajectoryNode的对象， 
  // 并将之添加到节点的容器trajectory_nodes_中
  trajectory_nodes_.Append(
      trajectory_id,
      mapping::TrajectoryNode{
          std::make_shared<const mapping::TrajectoryNode::Data>(
              mapping::TrajectoryNode::Data{
                  time, sensor::RangeData{Eigen::Vector3f::Zero(), {}, {}},
                  sensor::Compress(range_data_in_tracking),
                  transform::Rigid3d::Identity()}),
          optimized_pose});
  // 为新添加的节点分配一个NodeId
  ++num_trajectory_nodes_;
  // 维护各个轨迹之间的连接关系
  trajectory_connectivity_.Add(trajectory_id);

  // Test if the 'insertion_submap.back()' is one we never saw before.
  // 根据Local SLAM中子图的维护方式和子图更新数据的传递过程，
  // 我们可以认为输入参数insertion_submaps.back()中记录了最新的子图。
  if (trajectory_id >= submap_data_.num_trajectories() ||   // 先判断容器submap_data_中是否
      submap_data_.num_indices(trajectory_id) == 0 ||      // 有轨迹索引为trajectory_id的子图
      submap_data_            // 再判定insertion_submaps.back()中的子图是否是新生成的
              .at(mapping::SubmapId{
                  trajectory_id, submap_data_.num_indices(trajectory_id) - 1})
              .submap != insertion_submaps.back()) {
    // 若是则将之添加到容器submap_data_中， 同时分配一个SubmapId
    const mapping::SubmapId submap_id =
        submap_data_.Append(trajectory_id, SubmapData());
    submap_data_.at(submap_id).submap = insertion_submaps.back();
  }

  // Make sure we have a sampler for this trajectory.
  if (!global_localization_samplers_[trajectory_id]) {
    global_localization_samplers_[trajectory_id] =
        common::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
  }

  // We have to check this here, because it might have changed by the time we
  // execute the lambda.
  // 通过insertion_submaps.front()来查询旧图的更新状态
  const bool newly_finished_submap = insertion_submaps.front()->finished();
  // 通过lambda表达式和函数AddWorkItem注册一个为新增节点添加约束的任务ComputeConstraintsForScan。
  // 根据Cartographer的思想， 在该任务下应当会将新增的节点与所有已经处于kFinished状态的子图
  // 进行一次匹配建立可能存在的闭环约束。 此外，当有新的子图进入kFinished状态时，
  // 还会将之与所有的节点进行一次匹配
  AddWorkItem([=]() REQUIRES(mutex_) {
    ComputeConstraintsForScan(trajectory_id, insertion_submaps,
                              newly_finished_submap, pose);
  });
}

// 如果工作队列(scan_queue_)存在就将任务(work_item)放到队列中，如果不存在就直接执行
void SparsePoseGraph::AddWorkItem(std::function<void()> work_item) {
  if (scan_queue_ == nullptr) {
    work_item();
  } else {
    scan_queue_->push_back(work_item);
  }
}

// 后端优化器除了要处理子图和路径节点的世界坐标关系之外，它还考虑了里程计、IMU等传感器的数据。
// AddImuData通过后端优化器的接口将IMU传感器的数据喂给对象optimization_problem_
void SparsePoseGraph::AddImuData(const int trajectory_id, common::Time time,
                                 const Eigen::Vector3d& linear_acceleration,
                                 const Eigen::Vector3d& angular_velocity) {
  common::MutexLocker locker(&mutex_);
  AddWorkItem([=]() REQUIRES(mutex_) {
    optimization_problem_.AddImuData(trajectory_id, time, linear_acceleration,
                                     angular_velocity);
  });
}

/* 
 * Cartographer在后台通过一个线程池并行的进行闭环检测，计算约束。 
 * Global SLAM的核心PoseGraph2D通过MaybeAdd-WhenDone调用循环来组织后台的闭环检测。 
 * 所谓的MaybeAdd-WhenDone调用循环是指，在任意调用MaybeAddConstraint、
 * MaybeAddGlobalConstraint、NotifyEndOfScan之后，调用一次WhenDone，如此循环往复。
 */
void SparsePoseGraph::ComputeConstraint(const mapping::NodeId& node_id,
                                        const mapping::SubmapId& submap_id) {
  CHECK(submap_data_.at(submap_id).state == SubmapState::kFinished);

  const transform::Rigid3d inverse_submap_pose =
      optimization_problem_.submap_data()
          .at(submap_id.trajectory_id)
          .at(submap_id.submap_index)
          .pose.inverse();

  // T_submap_newmap * T_newmap_scan = T_submap_scan  优化后的
  const transform::Rigid3d initial_relative_pose =
      inverse_submap_pose * optimization_problem_.node_data()
                                .at(node_id.trajectory_id)
                                .at(node_id.node_index)
                                .point_cloud_pose;

  std::vector<mapping::TrajectoryNode> submap_nodes;
  for (const mapping::NodeId& submap_node_id :
       submap_data_.at(submap_id).node_ids) {
    submap_nodes.push_back(mapping::TrajectoryNode{
        trajectory_nodes_.at(submap_node_id).constant_data,
        inverse_submap_pose * trajectory_nodes_.at(submap_node_id).pose});  // 优化前的位姿
  }

  // Only globally match against submaps not in this trajectory.
  if (node_id.trajectory_id != submap_id.trajectory_id &&
      global_localization_samplers_[node_id.trajectory_id]->Pulse()) {
    // In this situation, 'initial_relative_pose' is:
    //
    // submap <- global map 2 <- global map 1 <- tracking
    //               (agreeing on gravity)
    //
    // Since they possibly came from two disconnected trajectories, the only
    // guaranteed(保证) connection between the tracking(scan) and the submap frames is
    // an agreement on the direction of gravity. Therefore, excluding(排除) yaw,
    // 'initial_relative_pose.rotation()' is a good estimate of the relative(相对)
    // orientation(方向) of the point cloud in the submap frame. Finding the correct
    // yaw component(分量) will be handled by the matching procedure(程序) in the
    // FastCorrelativeScanMatcher, and the given yaw is essentially(基本) ignored.
    // std::shared_ptr::get 获取指针。
    constraint_builder_.MaybeAddGlobalConstraint(
        submap_id, submap_data_.at(submap_id).submap.get(), node_id,
        &trajectory_nodes_.at(node_id).constant_data->range_data_3d.returns,
        submap_nodes, initial_relative_pose.rotation(),
        &trajectory_connectivity_);
  } else {
    const bool scan_and_submap_trajectories_connected =
        reverse_connected_components_.count(node_id.trajectory_id) > 0 &&
        reverse_connected_components_.count(submap_id.trajectory_id) > 0 &&
        reverse_connected_components_.at(node_id.trajectory_id) ==
            reverse_connected_components_.at(submap_id.trajectory_id);
    if (node_id.trajectory_id == submap_id.trajectory_id ||
        scan_and_submap_trajectories_connected) {
      // std::shared_ptr::get 获取指针。
      constraint_builder_.MaybeAddConstraint(
          submap_id, submap_data_.at(submap_id).submap.get(), node_id,
          &trajectory_nodes_.at(node_id).constant_data->range_data_3d.returns,
          submap_nodes, initial_relative_pose);
    }
  }
}

void SparsePoseGraph::ComputeConstraintsForOldScans(
    const mapping::SubmapId& submap_id) {
  const auto& submap_data = submap_data_.at(submap_id);
  const auto& node_data = optimization_problem_.node_data();
  for (size_t trajectory_id = 0; trajectory_id != node_data.size();
       ++trajectory_id) {
    for (size_t node_index = 0; node_index != node_data[trajectory_id].size();
         ++node_index) {
      const mapping::NodeId node_id{static_cast<int>(trajectory_id),
                                    static_cast<int>(node_index)};
      if (submap_data.node_ids.count(node_id) == 0) {
        ComputeConstraint(node_id, submap_id);
      }
    }
  }
}

/* 计算路径节点与子图之间的约束关系，触发工作队列的构建和运行   trajectory_id是轨迹编号
 * insertion_submaps则是从Local SLAM一路传递过来的新旧子图， 
 * newly_finished_submap表示旧图是否结束更新了, pose 是T_oldmap_scan
 * 通过约束构建器计算约束检查可能存在的闭环之外，它将子图和路径节点的初始位姿提供给后端优化器了
 * 提供路径节点只是通过调用优化器的接口AddTrejectoryNode实现的， 
 * 子图的提供则是通过PoseGraph2D的成员函数GrowSubmapTransformsAsNeeded完成的。
 */
void SparsePoseGraph::ComputeConstraintsForScan(
    const int trajectory_id,
    std::vector<std::shared_ptr<const Submap>> insertion_submaps,
    const bool newly_finished_submap, const transform::Rigid3d& pose) {

  // 通过函数GrowSubmapTransformsAsNeeded获取新旧子图的索引。 实际上这个函数还是蛮长的，
  // 它除了获取ID之外还检查了新子图是否第一次被后端看见，若是则为之计算全局位姿
  // 并喂给后端优化器optimization_problem_。
  const std::vector<mapping::SubmapId> submap_ids =
      GrowSubmapTransformsAsNeeded(trajectory_id, insertion_submaps);
  CHECK_EQ(submap_ids.size(), insertion_submaps.size());
  const mapping::SubmapId matching_id = submap_ids.front();
  // 计算节点世界坐标系下的位姿ε_j^s。
  // T_newmap_submap * T_submap_oldmap * T_oldmap_scan = T_newmap_scan
  const transform::Rigid3d optimized_pose =
      optimization_problem_.submap_data()
          .at(matching_id.trajectory_id)
          .at(matching_id.submap_index)
          .pose *
      insertion_submaps.front()->local_pose().inverse() * pose;
  const mapping::NodeId node_id{
      matching_id.trajectory_id,
      static_cast<size_t>(matching_id.trajectory_id) <
              optimization_problem_.node_data().size()
          ? static_cast<int>(optimization_problem_.node_data()
                                 .at(matching_id.trajectory_id)
                                 .size())
          : 0};
  // 获取节点数据
  const auto& scan_data = trajectory_nodes_.at(node_id).constant_data;
  // 调用优化器的接口AddTrejectoryNode实现提供路径节点
  optimization_problem_.AddTrajectoryNode(matching_id.trajectory_id,
                                          scan_data->time, optimized_pose);
  // 为新增的节点和新旧子图之间添加INTRA_SUBMAP类型的约束
  for (size_t i = 0; i < insertion_submaps.size(); ++i) {
    const mapping::SubmapId submap_id = submap_ids[i];
    CHECK(submap_data_.at(submap_id).state == SubmapState::kActive);
    submap_data_.at(submap_id).node_ids.emplace(node_id);
    // 计算节点相对于子图的局部位姿ε_ij
    // T_submap_oldmap * T_oldmap_scan = T_submap_scan
    const transform::Rigid3d constraint_transform =
        insertion_submaps[i]->local_pose().inverse() * pose;
    constraints_.push_back(
        Constraint{submap_id,
                   node_id,
                   {constraint_transform, options_.matcher_translation_weight(),
                    options_.matcher_rotation_weight()},
                   Constraint::INTRA_SUBMAP});
  }

  // 遍历所有已经处于kFinished状态的子图，建立它们与新增节点之间可能的约束。
  for (int trajectory_id = 0; trajectory_id < submap_data_.num_trajectories();
       ++trajectory_id) {
    for (int submap_index = 0;
         submap_index < submap_data_.num_indices(trajectory_id);
         ++submap_index) {
      const mapping::SubmapId submap_id{trajectory_id, submap_index};
      if (submap_data_.at(submap_id).state == SubmapState::kFinished) {
        CHECK_EQ(submap_data_.at(submap_id).node_ids.count(node_id), 0);
        ComputeConstraint(node_id, submap_id);
      }
    }
  }

  // 旧图切换到kFinished状态，则遍历所有已经进行过优化的节点，建立它们与旧图之间可能的约束。
  if (newly_finished_submap) {
    const mapping::SubmapId finished_submap_id = submap_ids.front();
    SubmapData& finished_submap_data = submap_data_.at(finished_submap_id);
    CHECK(finished_submap_data.state == SubmapState::kActive);
    finished_submap_data.state = SubmapState::kFinished;
    // We have a new completed submap, so we look into adding constraints for
    // old scans.
    ComputeConstraintsForOldScans(finished_submap_id);
  }

  // 通知约束构建器新增节点的操作结束
  constraint_builder_.NotifyEndOfScan();
  // 增加计数器num_nodes_since_last_loop_closure_
  ++num_scans_since_last_loop_closure_;
  // HandleScanQueue对于工作队列的运转有着重要的作用
  // 当累积了一定数量的新节点后就会触发闭环检测
  if (options_.optimize_every_n_scans() > 0 &&
      num_scans_since_last_loop_closure_ > options_.optimize_every_n_scans()) {
    CHECK(!run_loop_closure_);
    run_loop_closure_ = true;
    // If there is a 'scan_queue_' already, some other thread will take care.
    // 判定工作队列是否存在，如果不存在就创建一个对象
    if (scan_queue_ == nullptr) {
      scan_queue_ = common::make_unique<std::deque<std::function<void()>>>();
      HandleScanQueue();
    }
  }
}

// HandleScanQueue对于工作队列的运转有着重要的作用
void SparsePoseGraph::HandleScanQueue() {
  // 通过约束构建器的WhenDone接口注册了一个回调函数HandleWorkQueue
  constraint_builder_.WhenDone(
      [this](const sparse_pose_graph::ConstraintBuilder::Result& result) {
        {
          common::MutexLocker locker(&mutex_);
          constraints_.insert(constraints_.end(), result.begin(), result.end());
        }
        RunOptimization();

        common::MutexLocker locker(&mutex_);
        num_scans_since_last_loop_closure_ = 0;
        run_loop_closure_ = false;
        // 在一个循环中处理掉work_queue_中所有等待的任务，这些任务主要是添加节点、添加传感器数据到位姿图中
        while (!run_loop_closure_) {
          if (scan_queue_->empty()) {
            LOG(INFO) << "We caught up. Hooray!";
            scan_queue_.reset();
            return;
          }
          scan_queue_->front()();
          scan_queue_->pop_front();
        }
        // We have to optimize again.
        // 有时还没有完全处理完队列中的所有任务，就因为状态run_loop_closure_再次为true开启新的闭环检测而退出
        // 此时需要重新注册一下工作队列。
        HandleScanQueue();
      });
}

void SparsePoseGraph::WaitForAllComputations() {
  bool notification = false;
  common::MutexLocker locker(&mutex_);
  const int num_finished_scans_at_start =
      constraint_builder_.GetNumFinishedScans();
  while (!locker.AwaitWithTimeout(
      [this]() REQUIRES(mutex_) {
        return constraint_builder_.GetNumFinishedScans() ==
               num_trajectory_nodes_;
      },
      common::FromSeconds(1.))) {
    std::ostringstream progress_info;
    progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                  << 100. *
                         (constraint_builder_.GetNumFinishedScans() -
                          num_finished_scans_at_start) /
                         (num_trajectory_nodes_ - num_finished_scans_at_start)
                  << "%...";
    std::cout << "\r\x1b[K" << progress_info.str() << std::flush;
  }
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;
  constraint_builder_.WhenDone(
      [this, &notification](
          const sparse_pose_graph::ConstraintBuilder::Result& result) {
        common::MutexLocker locker(&mutex_);
        constraints_.insert(constraints_.end(), result.begin(), result.end());
        notification = true;
      });
  locker.Await([&notification]() { return notification; });
}

void SparsePoseGraph::RunFinalOptimization() {
  WaitForAllComputations();
  optimization_problem_.SetMaxNumIterations(
      options_.max_num_final_iterations());
  RunOptimization();
  optimization_problem_.SetMaxNumIterations(
      options_.optimization_problem_options()
          .ceres_solver_options()
          .max_num_iterations());
}

void SparsePoseGraph::RunOptimization() {
  // 先检查一下是否给后端优化器喂过数据
  if (optimization_problem_.submap_data().empty()) {
    return;
  }
  /*
   * 通过后端优化器的接口Solve进行SPA优化。根据注释的说法，程序运行到这里的时候，
   * 实际上没有其他线程访问对象optimization_problem_, constraints_, 这2个对象。
   * 又因为这个Solve接口实在太耗时了，所以没有在该函数之前加锁，以防止阻塞其他任务
   */
  optimization_problem_.Solve(constraints_);
  common::MutexLocker locker(&mutex_);

  /*
   * 完成上面框图中Global SLAM的第三个任务，对在后端进行SPA优化过程中新增的节点的位姿进行调整，
   * 以适应优化后的世界地图和运动轨迹。 获取后端优化器中子图和路径节点的数据，
   * 用临时对象node_data记录之，并遍历所有的轨迹
   */
  const auto& node_data = optimization_problem_.node_data();
  // 遍历所有的节点，用优化后的位姿来更新轨迹点的世界坐标
  for (int trajectory_id = 0;
       trajectory_id != static_cast<int>(node_data.size()); ++trajectory_id) {
    int node_index = 0;
    const int num_nodes = trajectory_nodes_.num_indices(trajectory_id);
    for (; node_index != static_cast<int>(node_data[trajectory_id].size());
         ++node_index) {
      const mapping::NodeId node_id{trajectory_id, node_index};
      trajectory_nodes_.at(node_id).pose =
          node_data[trajectory_id][node_index].point_cloud_pose;
    }
    // Extrapolate all point cloud poses that were added later.
    // 然后计算SPA优化前后的世界坐标变换关系，并将之左乘在后来新增的路径节点的全局位姿上，
    // 得到修正后的轨迹
    // SPA优化后,得到对全局坐标系的修正转换
    //  T_newglobal_submap * T_submap_local = T_newglobal_local
    const auto local_to_new_global = ComputeLocalToGlobalTransform(
        optimization_problem_.submap_data(), trajectory_id);
    // SPA优化前,得到对全局坐标系的修正转换
    // T_oldglobal_submap * T_submap_local = T_oldglobal_local
    const auto local_to_old_global = ComputeLocalToGlobalTransform(
        optimized_submap_transforms_, trajectory_id);
    // 综合考虑优化前后，全局坐标系的修正
    //  T_newglobal_local * T_local_oldglobal = T_newglobal_oldglobal
    const transform::Rigid3d old_global_to_new_global =
        local_to_new_global * local_to_old_global.inverse();
    // 最后，更新节点位姿，并用成员变量记录下当前的节点位姿。
    for (; node_index < num_nodes; ++node_index) {
      const mapping::NodeId node_id{trajectory_id, node_index};
      trajectory_nodes_.at(node_id).pose =
          old_global_to_new_global * trajectory_nodes_.at(node_id).pose;
    }
  }
  optimized_submap_transforms_ = optimization_problem_.submap_data();
  connected_components_ = trajectory_connectivity_.ConnectedComponents();
  reverse_connected_components_.clear();
  for (size_t i = 0; i != connected_components_.size(); ++i) {
    for (const int trajectory_id : connected_components_[i]) {
      reverse_connected_components_.emplace(trajectory_id, i);
    }
  }
}

std::vector<std::vector<mapping::TrajectoryNode>>
SparsePoseGraph::GetTrajectoryNodes() {
  common::MutexLocker locker(&mutex_);
  return trajectory_nodes_.data();
}

std::vector<SparsePoseGraph::Constraint> SparsePoseGraph::constraints() {
  common::MutexLocker locker(&mutex_);
  return constraints_;
}

// 根据最新一次优化之后的子图位姿生成局部坐标系到世界坐标系的坐标变换关系
transform::Rigid3d SparsePoseGraph::GetLocalToGlobalTransform(
    const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  return ComputeLocalToGlobalTransform(optimized_submap_transforms_,
                                       trajectory_id);
}

std::vector<std::vector<int>> SparsePoseGraph::GetConnectedTrajectories() {
  common::MutexLocker locker(&mutex_);
  return connected_components_;
}

int SparsePoseGraph::num_submaps(const int trajectory_id) {
  common::MutexLocker locker(&mutex_);
  if (trajectory_id >= submap_data_.num_trajectories()) {
    return 0;
  }
  return submap_data_.num_indices(trajectory_id);
}

mapping::SparsePoseGraph::SubmapData SparsePoseGraph::GetSubmapData(
    const mapping::SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>>
SparsePoseGraph::GetAllSubmapData() {
  common::MutexLocker locker(&mutex_);
  std::vector<std::vector<mapping::SparsePoseGraph::SubmapData>>
      all_submap_data(submap_data_.num_trajectories());
  for (int trajectory_id = 0; trajectory_id < submap_data_.num_trajectories();
       ++trajectory_id) {
    all_submap_data[trajectory_id].reserve(
        submap_data_.num_indices(trajectory_id));
    for (int submap_index = 0;
         submap_index < submap_data_.num_indices(trajectory_id);
         ++submap_index) {
      all_submap_data[trajectory_id].emplace_back(GetSubmapDataUnderLock(
          mapping::SubmapId{trajectory_id, submap_index}));
    }
  }
  return all_submap_data;
}

// 根据最新一次优化之后的子图位姿生成局部坐标系到世界坐标系的坐标变换关系
transform::Rigid3d SparsePoseGraph::ComputeLocalToGlobalTransform(
    const std::vector<std::vector<sparse_pose_graph::SubmapData>>&
        submap_transforms,
    const int trajectory_id) const {
  // 没有该轨迹或者submap_transforms为空，返回单位阵
  if (trajectory_id >= static_cast<int>(submap_transforms.size()) ||
      submap_transforms.at(trajectory_id).empty()) {
    return transform::Rigid3d::Identity();
  }
  // 最新一次优化之后的子图位姿
  const mapping::SubmapId last_optimized_submap_id{
      trajectory_id,
      static_cast<int>(submap_transforms.at(trajectory_id).size() - 1)};
  // Accessing 'local_pose' in Submap is okay, since the member is const.
  // 最新一次优化之后的子图位姿   T_newglobal_submap * T_submap_local = T_newglobal_local
  return submap_transforms.at(trajectory_id).back().pose *
         submap_data_.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

mapping::SparsePoseGraph::SubmapData SparsePoseGraph::GetSubmapDataUnderLock(
    const mapping::SubmapId& submap_id) {
  auto submap = submap_data_.at(submap_id).submap;
  // We already have an optimized pose.
  if (submap_id.trajectory_id <
          static_cast<int>(optimized_submap_transforms_.size()) &&
      submap_id.submap_index < static_cast<int>(optimized_submap_transforms_
                                                    .at(submap_id.trajectory_id)
                                                    .size())) {
    return {submap, optimized_submap_transforms_.at(submap_id.trajectory_id)
                        .at(submap_id.submap_index)
                        .pose};
  }
  // We have to extrapolate(外推). T_newmap_oldmap * T_oldmap_submap = T_newmap_submap
  return {submap, ComputeLocalToGlobalTransform(optimized_submap_transforms_,
                                                submap_id.trajectory_id) *
                      submap->local_pose()};
}

}  // namespace mapping_3d
}  // namespace cartographer

/*
 * lambda表达式是C++11标准之后引入的特性，与其他语言中提到的匿名函数差不多，可以理解为一个没有名称的内联函数。
 * 一个lambda表达式具有如下的语法形式:
 * [capture list] (parameter list) -> return type { function body }
 * 这里的lambda表达式描述的是一个void()类型的函数，它没有返回值也没有参数列表，
 * 所以上述语法中的"-> return type"部分就不存在。 
 * [capture list]中获取的是在函数体中用到的一些变量。捕获this指针，是为了能够访问成员变量trimmers_，
 * 而trimmer_ptr则是从输入参数中获取的修饰器对象指针。 
 * 这个表达式的作用就是将传参的修饰器指针放入容器trimmers_中。
 */