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

#include "cartographer/mapping_3d/optimizing_local_trajectory_builder.h"

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping_3d/proto/optimizing_local_trajectory_builder_options.pb.h"
#include "cartographer/mapping_3d/rotation_cost_function.h"
#include "cartographer/mapping_3d/scan_matching/occupied_space_cost_functor.h"
#include "cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h"
#include "cartographer/mapping_3d/scan_matching/translation_delta_cost_functor.h"
#include "cartographer/mapping_3d/translation_cost_function.h"
#include "cartographer/transform/transform.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_3d {

namespace {

// Computes the cost of differences between two velocities. For the constant
// velocity model the residuals are just the vector difference.
class VelocityDeltaCostFunctor {
 public:
  explicit VelocityDeltaCostFunctor(const double scaling_factor)
      : scaling_factor_(scaling_factor) {}

  VelocityDeltaCostFunctor(const VelocityDeltaCostFunctor&) = delete;
  VelocityDeltaCostFunctor& operator=(const VelocityDeltaCostFunctor&) = delete;

  template <typename T>
  bool operator()(const T* const old_velocity, const T* const new_velocity,
                  T* residual) const {
    residual[0] = scaling_factor_ * (new_velocity[0] - old_velocity[0]);
    residual[1] = scaling_factor_ * (new_velocity[1] - old_velocity[1]);
    residual[2] = scaling_factor_ * (new_velocity[2] - old_velocity[2]);
    return true;
  }

 private:
  const double scaling_factor_;
};

class RelativeTranslationAndYawCostFunction {
 public:
  RelativeTranslationAndYawCostFunction(const double translation_scaling_factor,
                                        const double rotation_scaling_factor,
                                        const transform::Rigid3d& delta)
      : translation_scaling_factor_(translation_scaling_factor),
        rotation_scaling_factor_(rotation_scaling_factor),
        delta_(delta) {}

  RelativeTranslationAndYawCostFunction(
      const RelativeTranslationAndYawCostFunction&) = delete;
  RelativeTranslationAndYawCostFunction& operator=(
      const RelativeTranslationAndYawCostFunction&) = delete;

  template <typename T>
  bool operator()(const T* const start_translation,
                  const T* const start_rotation, const T* const end_translation,
                  const T* const end_rotation, T* residual) const {
    const transform::Rigid3<T> start(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(start_translation),
        Eigen::Quaternion<T>(start_rotation[0], start_rotation[1],
                             start_rotation[2], start_rotation[3]));
    const transform::Rigid3<T> end(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(end_translation),
        Eigen::Quaternion<T>(end_rotation[0], end_rotation[1], end_rotation[2],
                             end_rotation[3]));

    const transform::Rigid3<T> delta = end.inverse() * start;
    const transform::Rigid3<T> error = delta.inverse() * delta_.cast<T>();
    residual[0] = translation_scaling_factor_ * error.translation().x();
    residual[1] = translation_scaling_factor_ * error.translation().y();
    residual[2] = translation_scaling_factor_ * error.translation().z();
    residual[3] = rotation_scaling_factor_ * transform::GetYaw(error);
    return true;
  }

 private:
  const double translation_scaling_factor_;
  const double rotation_scaling_factor_;
  const transform::Rigid3d delta_;
};

}  // namespace

OptimizingLocalTrajectoryBuilder::OptimizingLocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options)
    : options_(options),
      ceres_solver_options_(common::CreateCeresSolverOptions(
          options.ceres_scan_matcher_options().ceres_solver_options())),
      active_submaps_(options.submaps_options()),
      num_accumulated_(0),
      motion_filter_(options.motion_filter_options()) {}

OptimizingLocalTrajectoryBuilder::~OptimizingLocalTrajectoryBuilder() {}

void OptimizingLocalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  imu_data_.push_back(ImuData{
      time, linear_acceleration, angular_velocity,
  });
  RemoveObsoleteSensorData();
}

void OptimizingLocalTrajectoryBuilder::AddOdometerData(
    const common::Time time, const transform::Rigid3d& pose) {
  odometer_data_.push_back(OdometerData{time, pose});
  RemoveObsoleteSensorData();
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::InsertionResult>
OptimizingLocalTrajectoryBuilder::AddRangefinderData(
    const common::Time time, const Eigen::Vector3f& origin,
    const sensor::PointCloud& ranges) {
  CHECK_GT(ranges.size(), 0);

  // TODO(hrapp): Handle misses.
  // TODO(hrapp): Where are NaNs in range_data_in_tracking coming from?
  sensor::PointCloud point_cloud;
  // ??????????????????hit???
  for (const Eigen::Vector3f& hit : ranges) {
    const Eigen::Vector3f delta = hit - origin;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        point_cloud.push_back(hit);
      }
    }
  }

  auto high_resolution_options =
      options_.high_resolution_adaptive_voxel_filter_options();
  // ????????????????????????????????????????????????????????????  
  // min_num_points = 150,  scans_per_accumulation = 1
  high_resolution_options.set_min_num_points(
      high_resolution_options.min_num_points() /
      options_.scans_per_accumulation());
  // ??????
  sensor::AdaptiveVoxelFilter high_resolution_adaptive_voxel_filter(
      high_resolution_options);
  const sensor::PointCloud high_resolution_filtered_points =
      high_resolution_adaptive_voxel_filter.Filter(point_cloud);

  auto low_resolution_options =
      options_.low_resolution_adaptive_voxel_filter_options();
  low_resolution_options.set_min_num_points(
      low_resolution_options.min_num_points() /
      options_.scans_per_accumulation());
  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      low_resolution_options);
  const sensor::PointCloud low_resolution_filtered_points =
      low_resolution_adaptive_voxel_filter.Filter(point_cloud);

  if (batches_.empty()) {
    // First rangefinder data ever. Initialize to the origin.
    batches_.push_back(
        Batch{time, point_cloud, high_resolution_filtered_points,
              low_resolution_filtered_points,
              State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                    Eigen::Vector3d::Zero())});
  } else {
    const Batch& last_batch = batches_.back();
    batches_.push_back(Batch{
        time, point_cloud, high_resolution_filtered_points,
        low_resolution_filtered_points,
        PredictState(last_batch.state, last_batch.time, time),  // ??????IMU????????????state
    });
  }
  ++num_accumulated_;

  RemoveObsoleteSensorData();
  return MaybeOptimize(time);
}

// ??????????????????????????????
void OptimizingLocalTrajectoryBuilder::RemoveObsoleteSensorData() {
  if (imu_data_.empty()) {
    batches_.clear();
    return;
  }

  while (batches_.size() >
         static_cast<size_t>(options_.scans_per_accumulation())) {
    batches_.pop_front();
  }

  // ????????????
  while (imu_data_.size() > 1 &&
         (batches_.empty() || imu_data_[1].time <= batches_.front().time)) {
    imu_data_.pop_front();
  }

  // ????????????
  while (
      odometer_data_.size() > 1 &&
      (batches_.empty() || odometer_data_[1].time <= batches_.front().time)) {
    odometer_data_.pop_front();
  }
}

// batches???????????????
void OptimizingLocalTrajectoryBuilder::TransformStates(
    const transform::Rigid3d& transform) {
  for (Batch& batch : batches_) {
    const transform::Rigid3d new_pose = transform * batch.state.ToRigid();
    const auto& velocity = batch.state.velocity;
    const Eigen::Vector3d new_velocity =
        transform.rotation() *
        Eigen::Vector3d(velocity[0], velocity[1], velocity[2]);
    batch.state =
        State(new_pose.translation(), new_pose.rotation(), new_velocity);
  }
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::InsertionResult>
OptimizingLocalTrajectoryBuilder::MaybeOptimize(const common::Time time) {
  // TODO(hrapp): Make the number of optimizations configurable.
  if (num_accumulated_ < options_.scans_per_accumulation() &&
      num_accumulated_ % 10 != 0) {
    return nullptr;
  }

  ceres::Problem problem;
  std::shared_ptr<const Submap> matching_submap =
      active_submaps_.submaps().front();
  // We transform the states in 'batches_' in place to be in the submap frame as
  // expected by the OccupiedSpaceCostFunctor. This is reverted after solving
  // the optimization problem. ???????????????????????????????????????
  TransformStates(matching_submap->local_pose().inverse());
  for (size_t i = 0; i < batches_.size(); ++i) {
    Batch& batch = batches_[i];
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<scan_matching::OccupiedSpaceCostFunctor,
                                        ceres::DYNAMIC, 3, 4>(
            new scan_matching::OccupiedSpaceCostFunctor(
                options_.optimizing_local_trajectory_builder_options()
                        .high_resolution_grid_weight() /
                    std::sqrt(static_cast<double>(
                        batch.high_resolution_filtered_points.size())),
                batch.high_resolution_filtered_points,
                matching_submap->high_resolution_hybrid_grid()),
            batch.high_resolution_filtered_points.size()),
        nullptr, batch.state.translation.data(), batch.state.rotation.data());
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<scan_matching::OccupiedSpaceCostFunctor,
                                        ceres::DYNAMIC, 3, 4>(
            new scan_matching::OccupiedSpaceCostFunctor(
                options_.optimizing_local_trajectory_builder_options()
                        .low_resolution_grid_weight() /
                    std::sqrt(static_cast<double>(
                        batch.low_resolution_filtered_points.size())),
                batch.low_resolution_filtered_points,
                matching_submap->low_resolution_hybrid_grid()),
            batch.low_resolution_filtered_points.size()),
        nullptr, batch.state.translation.data(), batch.state.rotation.data());

    if (i == 0) {
      problem.SetParameterBlockConstant(batch.state.translation.data());
      problem.SetParameterBlockConstant(batch.state.rotation.data());
      problem.AddParameterBlock(batch.state.velocity.data(), 3);
      problem.SetParameterBlockConstant(batch.state.velocity.data());
    } else {
      problem.SetParameterization(batch.state.rotation.data(),
                                  new ceres::QuaternionParameterization());
    }
  }

  auto it = imu_data_.cbegin();
  for (size_t i = 1; i < batches_.size(); ++i) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<VelocityDeltaCostFunctor, 3, 3, 3>(
            new VelocityDeltaCostFunctor(
                options_.optimizing_local_trajectory_builder_options()
                    .velocity_weight())),
        nullptr, batches_[i - 1].state.velocity.data(),
        batches_[i].state.velocity.data());

    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<TranslationCostFunction, 3, 3, 3, 3>(
            new TranslationCostFunction(
                options_.optimizing_local_trajectory_builder_options()
                    .translation_weight(),
                common::ToSeconds(batches_[i].time - batches_[i - 1].time))),
        nullptr, batches_[i - 1].state.translation.data(),
        batches_[i].state.translation.data(),
        batches_[i - 1].state.velocity.data());

    const IntegrateImuResult<double> result =
        IntegrateImu(imu_data_, batches_[i - 1].time, batches_[i].time, &it);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<RotationCostFunction, 3, 4, 4>(
            new RotationCostFunction(
                options_.optimizing_local_trajectory_builder_options()
                    .rotation_weight(),
                result.delta_rotation)),
        nullptr, batches_[i - 1].state.rotation.data(),
        batches_[i].state.rotation.data());
  }

  if (odometer_data_.size() > 1) {
    transform::TransformInterpolationBuffer interpolation_buffer;
    for (const auto& odometer_data : odometer_data_) {
      interpolation_buffer.Push(odometer_data.time, odometer_data.pose);
    }
    for (size_t i = 1; i < batches_.size(); ++i) {
      // Only add constraints for this range data if  we have bracketing data
      // from the odometer.
      if (!(interpolation_buffer.earliest_time() <= batches_[i - 1].time &&
            batches_[i].time <= interpolation_buffer.latest_time())) {
        continue;
      }
      const transform::Rigid3d previous_odometer_pose =
          interpolation_buffer.Lookup(batches_[i - 1].time);
      const transform::Rigid3d current_odometer_pose =
          interpolation_buffer.Lookup(batches_[i].time);
      const transform::Rigid3d delta_pose =
          current_odometer_pose.inverse() * previous_odometer_pose;
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<RelativeTranslationAndYawCostFunction,
                                          4, 3, 4, 3, 4>(
              new RelativeTranslationAndYawCostFunction(
                  options_.optimizing_local_trajectory_builder_options()
                      .odometry_translation_weight(),
                  options_.optimizing_local_trajectory_builder_options()
                      .odometry_rotation_weight(),
                  delta_pose)),
          nullptr, batches_[i - 1].state.translation.data(),
          batches_[i - 1].state.rotation.data(),
          batches_[i].state.translation.data(),
          batches_[i].state.rotation.data());
    }
  }

  ceres::Solver::Summary summary;
  ceres::Solve(ceres_solver_options_, &problem, &summary);
  // The optimized states in 'batches_' are in the submap frame and we transform
  // them in place to be in the local SLAM frame again. ???????????????????????????
  TransformStates(matching_submap->local_pose());
  if (num_accumulated_ < options_.scans_per_accumulation()) {
    return nullptr;
  }

  num_accumulated_ = 0;

  // deque::back()??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
  // T_global_newscan
  const transform::Rigid3d optimized_pose = batches_.back().state.ToRigid();
  sensor::RangeData accumulated_range_data_in_tracking = {
      Eigen::Vector3f::Zero(), {}, {}};

  for (const auto& batch : batches_) {
    // T_newscan_scan = T_global_newscan(???) * T_global_scan
    const transform::Rigid3f transform =
        (optimized_pose.inverse() * batch.state.ToRigid()).cast<float>();
    for (const Eigen::Vector3f& point : batch.points) {
      // ???????????????????????? p_newscan = T_newscan_scan * p_scan
      accumulated_range_data_in_tracking.returns.push_back(transform * point);
    }
  }

  return AddAccumulatedRangeData(time, optimized_pose,
                                 accumulated_range_data_in_tracking);
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::InsertionResult>
OptimizingLocalTrajectoryBuilder::AddAccumulatedRangeData(
    const common::Time time, const transform::Rigid3d& optimized_pose,  // T_global_newscan
    const sensor::RangeData& range_data_in_tracking) {
  const sensor::RangeData filtered_range_data = {
      range_data_in_tracking.origin,
      sensor::VoxelFiltered(range_data_in_tracking.returns,
                            options_.voxel_filter_size()),
      sensor::VoxelFiltered(range_data_in_tracking.misses,
                            options_.voxel_filter_size())};

  if (filtered_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }

  // ?????????????????????????????????????????????T_global_newscan??????????????????????????????
  last_pose_estimate_ = {
      time, optimized_pose,
      sensor::TransformPointCloud(filtered_range_data.returns,
                                  optimized_pose.cast<float>())};

  return InsertIntoSubmap(time, filtered_range_data, optimized_pose);
}

const OptimizingLocalTrajectoryBuilder::PoseEstimate&
OptimizingLocalTrajectoryBuilder::pose_estimate() const {
  return last_pose_estimate_;
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::InsertionResult>
OptimizingLocalTrajectoryBuilder::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_tracking,
    const transform::Rigid3d& pose_observation) {
  if (motion_filter_.IsSimilar(time, pose_observation)) {
    return nullptr;
  }
  std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  for (std::shared_ptr<Submap> submap : active_submaps_.submaps()) {
    insertion_submaps.push_back(submap);
  }
  // TODO(whess): Use an ImuTracker to track the gravity direction.
  const Eigen::Quaterniond kFakeGravityOrientation =
      Eigen::Quaterniond::Identity();
  // ???????????????????????????????????????????????????????????????
  // ???????????????submap???submap???local pose??????????????????????????????origin????????????????????????????????????
  active_submaps_.InsertRangeData(
      sensor::TransformRangeData(range_data_in_tracking,
                                 pose_observation.cast<float>()),  // ????????????????????????
      kFakeGravityOrientation);

  return std::unique_ptr<InsertionResult>(
      new InsertionResult{time, range_data_in_tracking, pose_observation,
                          std::move(insertion_submaps)});
}

// line185 PredictState(last_batch.state, last_batch.time, time)
// ??????IMU????????????state
OptimizingLocalTrajectoryBuilder::State
OptimizingLocalTrajectoryBuilder::PredictState(const State& start_state,
                                               const common::Time start_time,
                                               const common::Time end_time) {
  // cend()????????????????????????????????????????????????????????????????????????-?????????????????????
  auto it = --imu_data_.cend();
  while (it->time > start_time) {
    // cbegin()??????????????????????????????????????????????????????
    CHECK(it != imu_data_.cbegin());
    --it;
  }

  // ?????????start?????????????????????????????????end???delta ?????? ????????? it?????????????????????start_time?????????
  const IntegrateImuResult<double> result =
      IntegrateImu(imu_data_, start_time, end_time, &it);

  // ?????????????????????std::array<double, 4>???Eigen::Quaterniond
  const Eigen::Quaterniond start_rotation(
      start_state.rotation[0], start_state.rotation[1], start_state.rotation[2],
      start_state.rotation[3]);
  const Eigen::Quaterniond orientation = start_rotation * result.delta_rotation;
  const double delta_time_seconds = common::ToSeconds(end_time - start_time);

  // TODO(hrapp): IntegrateImu should integration position as well.
  // ?????????????????????std::array<double, 3>???Eigen::Vector3d
  // x_end = x_start + delta_t * v
  const Eigen::Vector3d position =
      Eigen::Map<const Eigen::Vector3d>(start_state.translation.data()) +
      delta_time_seconds *
          Eigen::Map<const Eigen::Vector3d>(start_state.velocity.data());
  // ????????????????????????????????????std::array<double, 3>???Eigen::Vector3d
  // v_end = v_start + T * delta_v - v_g
  const Eigen::Vector3d velocity =
      Eigen::Map<const Eigen::Vector3d>(start_state.velocity.data()) +
      start_rotation * result.delta_velocity -
      gravity_constant_ * delta_time_seconds * Eigen::Vector3d::UnitZ();

  // ????????????????????????
  return State(position, orientation, velocity);
}

}  // namespace mapping_3d
}  // namespace cartographer
