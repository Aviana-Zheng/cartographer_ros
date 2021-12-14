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

#ifndef CARTOGRAPHER_MAPPING_2D_LOCAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_2D_LOCAL_TRAJECTORY_BUILDER_H_

#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/odometry_state_tracker.h"
#include "cartographer/mapping_2d/proto/local_trajectory_builder_options.pb.h"
#include "cartographer/mapping_2d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_2d/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/mapping_3d/motion_filter.h"
#include "cartographer/sensor/configuration.h"
#include "cartographer/sensor/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_2d {

// Wires up the local SLAM stack (i.e. UKF, scan matching, etc.) without loop
// closure.
class LocalTrajectoryBuilder {
 public:
  // InsertionResult就是用来保存插入Local Slam的一个节点的数据结构
  // 如果扫描匹配的结果被运动滤波器过滤了，那么字段insertion_result中记录的是一个空指针"nullptr"
  struct InsertionResult {
    common::Time time;  // 时间戳
    // 更新后的submap列表（仅最新的submap）
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
    // tracking 坐标系下的位姿(Local SLAM坐标系下的位姿), 
    // 需要乘以tracking frame in map的转换(Local SLAM坐标系到Global SLAM坐标系的转换)
    // 才能转到全局坐标系
    transform::Rigid3d tracking_to_tracking_2d;
    // tracking 坐标系下 本帧的点云
    sensor::RangeData range_data_in_tracking_2d;
    transform::Rigid2d pose_estimate_2d;
  };

  explicit LocalTrajectoryBuilder(
      const proto::LocalTrajectoryBuilderOptions& options);
  ~LocalTrajectoryBuilder();
  
  // 此外为了防止一些意外的情况发生，该类屏蔽了拷贝构造函数和拷贝赋值运算符。
  LocalTrajectoryBuilder(const LocalTrajectoryBuilder&) = delete;
  LocalTrajectoryBuilder& operator=(const LocalTrajectoryBuilder&) = delete;

  const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate& pose_estimate()
      const;
  // 添加激光传感器的扫描数据
  std::unique_ptr<InsertionResult> AddHorizontalRangeData(
      common::Time, const sensor::RangeData& range_data);
  // 添加IMU数据
  void AddImuData(common::Time time, const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity);
  // 添加里程计数据
  void AddOdometerData(common::Time time, const transform::Rigid3d& pose);

 private:
  sensor::RangeData TransformAndFilterRangeData(
      const transform::Rigid3f& tracking_to_tracking_2d,
      const sensor::RangeData& range_data) const;

  // Scan matches 'range_data_in_tracking_2d' and fill in the 'pose_observation'
  // with the result.
  void ScanMatch(common::Time time, const transform::Rigid3d& pose_prediction,
                 const transform::Rigid3d& tracking_to_tracking_2d,
                 const sensor::RangeData& range_data_in_tracking_2d,
                 transform::Rigid3d* pose_observation);

  // Lazily constructs an ImuTracker.
  // 初始化位姿估计器
  void InitializeImuTracker(common::Time time);

  // Updates the current estimate to reflect the given 'time'.
  // 某个时间的位姿预测
  void Predict(common::Time time);

  const proto::LocalTrajectoryBuilderOptions options_;  // 轨迹跟踪器的配置选项
  // 当前正在维护的子图
  ActiveSubmaps active_submaps_;  
  // 上一次预测的位姿值
  mapping::GlobalTrajectoryBuilderInterface::PoseEstimate last_pose_estimate_;

  // Current 'pose_estimate_' and 'velocity_estimate_' at 'time_'.
  // 上一次预测位姿pose_estimate_  velocity_estimate_的时间
  common::Time time_ = common::Time::min();
  // 当前预测的位姿值
  transform::Rigid3d pose_estimate_ = transform::Rigid3d::Identity();
  // 当前预测的速度值
  Eigen::Vector2d velocity_estimate_ = Eigen::Vector2d::Zero();
  // 上一次点云匹配的时间
  common::Time last_scan_match_time_ = common::Time::min();
  // This is the difference between the model (constant velocity, IMU)
  // prediction 'pose_estimate_' and the odometry prediction. To get the
  // odometry prediction, right-multiply this to 'pose_estimate_'.
  // 里程计全局姿态的修正值
  transform::Rigid3d odometry_correction_ = transform::Rigid3d::Identity();

  mapping_3d::MotionFilter motion_filter_;    // 运动滤波器，对位姿相关的数据进行降采样
  scan_matching::RealTimeCorrelativeScanMatcher
      real_time_correlative_scan_matcher_;   // 实时相关性分析的扫描匹配器
  // 使用Ceres库将扫描数据放置到地图中的扫描匹配器
  scan_matching::CeresScanMatcher ceres_scan_matcher_;   
  
  // 2个位姿估计器，用一段时间内的位姿数据估计线速度和角速度，进而预测运动
  std::unique_ptr<mapping::ImuTracker> imu_tracker_;
  mapping::OdometryStateTracker odometry_state_tracker_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_LOCAL_TRAJECTORY_BUILDER_H_
