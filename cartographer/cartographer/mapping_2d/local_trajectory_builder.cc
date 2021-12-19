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

#include "cartographer/mapping_2d/local_trajectory_builder.h"

#include <limits>

#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping_2d {

// 从构造函数的构造列表中几乎完成了所有对象的构建，但是留下了位姿估计器对象imu_tracker_
// 在InitializeImuTracker中完成构造
LocalTrajectoryBuilder::LocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      odometry_state_tracker_(options_.num_odometry_states()) {}

LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {}

sensor::RangeData LocalTrajectoryBuilder::TransformAndFilterRangeData(
    const transform::Rigid3f& tracking_to_tracking_2d,
    const sensor::RangeData& range_data) const {
  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  sensor::RangeData returns_and_misses{range_data.origin, {}, {}};
  // 遍历点云hit点
  for (const Eigen::Vector3f& hit : range_data.returns) {
    const float range = (hit - range_data.origin).norm(); // norm()模长
    // 判断点云是否超过最小距离，未超过不进行处理
    if (range >= options_.min_range()) {
      // 判断点云小于最大距离，认为hit，放入returns_and_misses的returns
      if (range <= options_.max_range()) {
        returns_and_misses.returns.push_back(hit);
      } else {  
        // 判断点云超过最大距离，则认为是丢失，放入returns_and_misses的misses,
        returns_and_misses.misses.push_back(
            range_data.origin + options_.missing_data_ray_length() *
                                    (hit - range_data.origin).normalized());
                                    // normalized()单位化
      }
    }
  }
  // 修剪点云，并将点云数据结构中的坐标从tracking坐标系转换到submap中(仅仅使两者在同一平面，没有yaw)
  const sensor::RangeData cropped = sensor::CropRangeData(
      sensor::TransformRangeData(returns_and_misses, tracking_to_tracking_2d),
      options_.min_z(), options_.max_z());
  // 对点云进行滤波并返回
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFiltered(cropped.returns, options_.voxel_filter_size()),
      sensor::VoxelFiltered(cropped.misses, options_.voxel_filter_size())};
}

// 获取根据点云计算的位姿pose_estimate_
void LocalTrajectoryBuilder::ScanMatch(
    common::Time time, const transform::Rigid3d& pose_prediction, // 里程计预测的state位姿
    const transform::Rigid3d& tracking_to_tracking_2d,  // T_submap_scannow，不包含yaw
    const sensor::RangeData& range_data_in_tracking_2d,
    transform::Rigid3d* pose_observation) {
  
  //帧节匹配的流程分为4个步骤：

  // 1、得到一个地图  active_submaps_维持的两个submap前面一个作为匹配地图
  std::shared_ptr<const Submap> matching_submap =
      active_submaps_.submaps().front();

  // 2、得到一个预估位姿。 pose_prediction_2d表示预估的yaw角     
  // 预测的位姿pose_prediction是3d的，因此必须把它旋转到2d平面
  // tracking_to_tracking_2d 是使pose_prediction不包含yaw的旋转部分
  // 因为这里是2d-slam所以要把预测的位姿旋转到2d平面
  transform::Rigid2d pose_prediction_2d =
      transform::Project2D(pose_prediction * tracking_to_tracking_2d.inverse());
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  // csm用滤波器提供的初始值进行优化，然后提供一个更好的初始值给ceres-scan-match
  transform::Rigid2d initial_ceres_pose = pose_prediction_2d;

  // 定义一个滤波器
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.adaptive_voxel_filter_options());
  // 对激光雷达数据进行滤波 & 转换成点云数据  这里的点云数据是在平面机器人坐标系中
  const sensor::PointCloud filtered_point_cloud_in_tracking_2d =
      adaptive_voxel_filter.Filter(range_data_in_tracking_2d.returns);
  
  // 配置文件中是否需要用csm来优化ceres-scan-match的初始解
  // 默认未打开
  if (options_.use_online_correlative_scan_matching()) {
    // 3、进行csm匹配，得到一个初始解：initial_ceres_pose，它又分为4步骤。
    // 通过csm和滤波器过后的2d平面的　激光雷达数据来进行位姿优化
    // 传入预测的初始位姿＼激光雷达数据＼栅格地图
    // 返回一个更好的值initial_ceres_pose
    real_time_correlative_scan_matcher_.Match(
        pose_prediction_2d, filtered_point_cloud_in_tracking_2d,
        matching_submap->probability_grid(), &initial_ceres_pose);
    // CSM先算出一个初始解，叫做initial_ceres_pose,再把这个解作为基于优化的初始解。
  }

  // 4、再调用ceres优化的方法进行一次匹配。
  // 最终通过ceres_scan_match来得到最终的位姿
  // 这里得到的位姿是tracking_2d坐标系到map坐标系的转换
  transform::Rigid2d tracking_2d_to_map;
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(pose_prediction_2d, initial_ceres_pose,
                            filtered_point_cloud_in_tracking_2d,
                            matching_submap->probability_grid(),
                            &tracking_2d_to_map, &summary);

  *pose_observation =
      transform::Embed3D(tracking_2d_to_map) * tracking_to_tracking_2d;
}

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::AddHorizontalRangeData(
    const common::Time time, const sensor::RangeData& range_data) {
  // Initialize IMU tracker now if we do not ever use an IMU.
  if (!options_.use_imu_data()) {
    InitializeImuTracker(time);
  }

  if (imu_tracker_ == nullptr) {
    // Until we've initialized the IMU tracker with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "ImuTracker not yet initialized.";
    return nullptr;
  }
  
  // 在接收到range_data数据的时刻，进行位姿预测
  // time_ = time
  Predict(time);
  // 里程计修正的state位姿
  const transform::Rigid3d odometry_prediction =
      pose_estimate_ * odometry_correction_;
  // Predict(time)利用imu速度、角速度数据预测的位姿
  const transform::Rigid3d model_prediction = pose_estimate_;
  // TODO(whess): Prefer IMU over odom orientation if configured?
  const transform::Rigid3d& pose_prediction = odometry_prediction;

  // Computes the rotation without yaw, as defined by GetYaw().
  // tracking_to_tracking_2d 是使pose_prediction不包含yaw的旋转部分
  const transform::Rigid3d tracking_to_tracking_2d =
      transform::Rigid3d::Rotation(
          Eigen::Quaterniond(Eigen::AngleAxisd(
              -transform::GetYaw(pose_prediction), Eigen::Vector3d::UnitZ())) *
          pose_prediction.rotation());

  const sensor::RangeData range_data_in_tracking_2d =
      TransformAndFilterRangeData(tracking_to_tracking_2d.cast<float>(),
                                  range_data);

  // 没有hit点，则返回空指针
  if (range_data_in_tracking_2d.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // 获取根据点云计算的位姿pose_estimate_
  ScanMatch(time, pose_prediction, tracking_to_tracking_2d,
            range_data_in_tracking_2d, &pose_estimate_);
  
  odometry_correction_ = transform::Rigid3d::Identity();
  if (!odometry_state_tracker_.empty()) {
    // We add an odometry state, so that the correction from the scan matching
    // is not removed by the next odometry data we get.
    odometry_state_tracker_.AddOdometryState(
        {time, odometry_state_tracker_.newest().odometer_pose,
         odometry_state_tracker_.newest().state_pose *
             odometry_prediction.inverse() * pose_estimate_});
  }

  // Improve the velocity estimate.
  // 判断当前收到的点云时间比上一次收到的点云时间大
  if (last_scan_match_time_ > common::Time::min() &&
      time > last_scan_match_time_) {
    const double delta_t = common::ToSeconds(time - last_scan_match_time_);
    // 更新速度预测
    velocity_estimate_ += (pose_estimate_.translation().head<2>() -
                           model_prediction.translation().head<2>()) /
                          delta_t;
  }
  // Predict(time);----> time_ = time,记录添加点云数据的上一个时刻
  last_scan_match_time_ = time_;

  // Remove the untracked z-component which floats around 0 in the UKF.
  const auto translation = pose_estimate_.translation();
  pose_estimate_ = transform::Rigid3d(
      transform::Rigid3d::Vector(translation.x(), translation.y(), 0.),
      pose_estimate_.rotation());

  // tracking_2d平面在map中的位姿
  const transform::Rigid3d tracking_2d_to_map =
      pose_estimate_ * tracking_to_tracking_2d.inverse();
  // 记录time时刻的PoseEstimate，包含时间、T_map_nowscan姿态、在map坐标系下的点云
  last_pose_estimate_ = {
      time, pose_estimate_,
      sensor::TransformPointCloud(range_data_in_tracking_2d.returns,
                                  tracking_2d_to_map.cast<float>())};

  // 平面位移x,y、yaw
  const transform::Rigid2d pose_estimate_2d =
      transform::Project2D(tracking_2d_to_map);
  
  // 如果两帧scan的运动相似，被motion_filter_过滤掉，返回空指针
  if (motion_filter_.IsSimilar(time, transform::Embed3D(pose_estimate_2d))) {
    return nullptr;
  }

  // 创建子地图
  std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  for (std::shared_ptr<Submap> submap : active_submaps_.submaps()) {
    insertion_submaps.push_back(submap);
  }
  active_submaps_.InsertRangeData(
      TransformRangeData(range_data_in_tracking_2d,
                         transform::Embed3D(pose_estimate_2d.cast<float>())));

  return common::make_unique<InsertionResult>(InsertionResult{
      time, std::move(insertion_submaps), tracking_to_tracking_2d,
      range_data_in_tracking_2d, pose_estimate_2d});
}

const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate&
LocalTrajectoryBuilder::pose_estimate() const {
  return last_pose_estimate_;
}

// 添加IMU数据
void LocalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  // 首先检查配置项是否要求使用IMU数据
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";

  // InitializeImuTracker完成位姿估计器的初始化工作
  InitializeImuTracker(time);
  // 在接收到IMU数据的时刻，进行位姿预测
  Predict(time);
  // 更新IMU观测值
  imu_tracker_->AddImuLinearAccelerationObservation(linear_acceleration);
  imu_tracker_->AddImuAngularVelocityObservation(angular_velocity);
}

// 添加里程计数据
void LocalTrajectoryBuilder::AddOdometerData(
    const common::Time time, const transform::Rigid3d& odometer_pose) {
  // 检查对象imu_tracker_是否为空指针来判定位姿估计器是否已经完成初始化工作
  if (imu_tracker_ == nullptr) {
    // Until we've initialized the IMU tracker we do not want to call Predict().
    LOG(INFO) << "ImuTracker not yet initialized.";
    return;
  }

  // 在接收到里程计数据的时刻，进行位姿预测
  Predict(time);
  // 若通过则直接将里程计的数据喂给odometry_state_tracker_对象
  if (!odometry_state_tracker_.empty()) {
    const auto& previous_odometry_state = odometry_state_tracker_.newest();
    // T_last_start * T_start_now = T_last_now
    const transform::Rigid3d delta =
        previous_odometry_state.odometer_pose.inverse() * odometer_pose;
    // T_map_start * T_last_now = T_map_now
    const transform::Rigid3d new_pose =
        previous_odometry_state.state_pose * delta;
    odometry_correction_ = pose_estimate_.inverse() * new_pose;
  }
  // 有点多余  pose_estimate_ * odometry_correction_ = 
  // pose_estimate_ * pose_estimate_.inverse() * new_pose = new_pose
  // 更新odometry_state_tracker_值
  odometry_state_tracker_.AddOdometryState(
      {time, odometer_pose, pose_estimate_ * odometry_correction_});
}

// 从构造函数的构造列表中几乎完成了所有对象的构建，但是留下了位姿估计器对象imu_tracker_
void LocalTrajectoryBuilder::InitializeImuTracker(const common::Time time) {
  // 检查对象imu_tracker_是否是一个空指针
  // 如果不是意味着已经创建了一个位姿估计器对象，直接返回
  // 否则创建一个imu_tracker_类型的位姿估计器，并在最后添加一个imu_gravity_time_constant。
  // 完成imu_tracker_的初始化
  if (imu_tracker_ == nullptr) {
    imu_tracker_ = common::make_unique<mapping::ImuTracker>(
        options_.imu_gravity_time_constant(), time);
  }
}

// 某个时间的位姿预测
void LocalTrajectoryBuilder::Predict(const common::Time time) {
  CHECK(imu_tracker_ != nullptr);
  CHECK_LE(time_, time);
  const double last_yaw = transform::GetYaw(imu_tracker_->orientation());
  imu_tracker_->Advance(time);
  if (time_ > common::Time::min()) {
    const double delta_t = common::ToSeconds(time - time_);
    // Constant velocity model.
    const Eigen::Vector3d translation =
        pose_estimate_.translation() +
        delta_t *
            Eigen::Vector3d(velocity_estimate_.x(), velocity_estimate_.y(), 0.);
    // Use the current IMU tracker roll and pitch for gravity alignment, and
    // apply its change in yaw.
    const Eigen::Quaterniond rotation = // T_map_now
        Eigen::AngleAxisd( // T_map_start
            transform::GetYaw(pose_estimate_.rotation()) - last_yaw,
            Eigen::Vector3d::UnitZ()) *  // T_start_now
        imu_tracker_->orientation();
    pose_estimate_ = transform::Rigid3d(translation, rotation);
  }
  time_ = time;
}

}  // namespace mapping_2d
}  // namespace cartographer
