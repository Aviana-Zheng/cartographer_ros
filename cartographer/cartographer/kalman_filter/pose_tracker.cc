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

#include "cartographer/kalman_filter/pose_tracker.h"

#include <cmath>
#include <limits>
#include <utility>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/gaussian_distribution.h"
#include "cartographer/kalman_filter/unscented_kalman_filter.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace kalman_filter {

namespace {

// 定义函数AddDelta,当前状态State加上delta
PoseTracker::State AddDelta(const PoseTracker::State& state,
                            const PoseTracker::State& delta) {
  PoseTracker::State new_state = state + delta;
  const Eigen::Quaterniond orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(state[PoseTracker::kMapOrientationX],
                          state[PoseTracker::kMapOrientationY],
                          state[PoseTracker::kMapOrientationZ]));
  const Eigen::Vector3d rotation_vector(delta[PoseTracker::kMapOrientationX],
                                        delta[PoseTracker::kMapOrientationY],
                                        delta[PoseTracker::kMapOrientationZ]);
  CHECK_LT(rotation_vector.norm(), M_PI / 2.)
      << "Sigma point is far from the mean, recovered delta may be incorrect.";
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(rotation_vector);
  const Eigen::Vector3d new_orientation =
      transform::RotationQuaternionToAngleAxisVector(orientation * rotation);
  new_state[PoseTracker::kMapOrientationX] = new_orientation.x();
  new_state[PoseTracker::kMapOrientationY] = new_orientation.y();
  new_state[PoseTracker::kMapOrientationZ] = new_orientation.z();
  return new_state;
}

// 定义ComputeDelta()函数,计算State的差值delta
PoseTracker::State ComputeDelta(const PoseTracker::State& origin,
                                const PoseTracker::State& target) {
  PoseTracker::State delta = target - origin;
  const Eigen::Quaterniond origin_orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(origin[PoseTracker::kMapOrientationX],
                          origin[PoseTracker::kMapOrientationY],
                          origin[PoseTracker::kMapOrientationZ]));
  const Eigen::Quaterniond target_orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(target[PoseTracker::kMapOrientationX],
                          target[PoseTracker::kMapOrientationY],
                          target[PoseTracker::kMapOrientationZ]));
  const Eigen::Vector3d rotation =
      transform::RotationQuaternionToAngleAxisVector(
          origin_orientation.inverse() * target_orientation);
  delta[PoseTracker::kMapOrientationX] = rotation.x();
  delta[PoseTracker::kMapOrientationY] = rotation.y();
  delta[PoseTracker::kMapOrientationZ] = rotation.z();
  return delta;
}

// Build a model matrix for the given time delta.    
// 滤波器预测函数 3D, 返回delta时间后的状态
/**
 * @brief ModelFunction
 * 三维平面系统的预测函数　该函数被PoseTracker的Predict()函数调用
 * 主要功能为根据系统当前时刻的状态state　预测delta_t时刻后系统的状态
 * 这是三维平面的．
 * @param state     系统当前时刻的状态
 * @param delta_t   对应的delta_t
 * @return
 */
PoseTracker::State ModelFunction(const PoseTracker::State& state,
                                 const double delta_t) {
  CHECK_GT(delta_t, 0.);

  PoseTracker::State new_state;
  new_state[PoseTracker::kMapPositionX] =     //x=x+delta * v
      state[PoseTracker::kMapPositionX] +
      delta_t * state[PoseTracker::kMapVelocityX];
  new_state[PoseTracker::kMapPositionY] =      //y=y+delta* v
      state[PoseTracker::kMapPositionY] +
      delta_t * state[PoseTracker::kMapVelocityY];
  new_state[PoseTracker::kMapPositionZ] =       //z=z+ delta* v
      state[PoseTracker::kMapPositionZ] +
      delta_t * state[PoseTracker::kMapVelocityZ];

  new_state[PoseTracker::kMapOrientationX] =    //不变
      state[PoseTracker::kMapOrientationX];
  new_state[PoseTracker::kMapOrientationY] =
      state[PoseTracker::kMapOrientationY];
  new_state[PoseTracker::kMapOrientationZ] =
      state[PoseTracker::kMapOrientationZ];

  new_state[PoseTracker::kMapVelocityX] = state[PoseTracker::kMapVelocityX];   //不变
  new_state[PoseTracker::kMapVelocityY] = state[PoseTracker::kMapVelocityY];
  new_state[PoseTracker::kMapVelocityZ] = state[PoseTracker::kMapVelocityZ];

  return new_state;
}

}  // namespace

// 定义乘法操作,C=A*B
PoseAndCovariance operator*(const transform::Rigid3d& transform,
                            const PoseAndCovariance& pose_and_covariance) {
  GaussianDistribution<double, 6> distribution(
      Eigen::Matrix<double, 6, 1>::Zero(), pose_and_covariance.covariance);
  Eigen::Matrix<double, 6, 6> linear_transform;
  linear_transform << transform.rotation().matrix(), Eigen::Matrix3d::Zero(),
      Eigen::Matrix3d::Zero(), transform.rotation().matrix();
  return {transform * pose_and_covariance.pose,
          (linear_transform * distribution).GetCovariance()};
}

/*
trajectory_builder_3d.lua:
   pose_tracker = {
      orientation_model_variance = 5e-3,
      position_model_variance = 0.00654766,
      velocity_model_variance = 0.53926,
      -- This disables gravity alignment in local SLAM.
      imu_gravity_time_constant = 1e9,
      imu_gravity_variance = 0,
      num_odometry_states = 1,
    },
*/
proto::PoseTrackerOptions CreatePoseTrackerOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::PoseTrackerOptions options;
  options.set_position_model_variance(
      parameter_dictionary->GetDouble("position_model_variance"));
  options.set_orientation_model_variance(
      parameter_dictionary->GetDouble("orientation_model_variance"));
  options.set_velocity_model_variance(
      parameter_dictionary->GetDouble("velocity_model_variance"));
  options.set_imu_gravity_time_constant(
      parameter_dictionary->GetDouble("imu_gravity_time_constant"));
  options.set_imu_gravity_variance(
      parameter_dictionary->GetDouble("imu_gravity_variance"));
  options.set_num_odometry_states(
      parameter_dictionary->GetNonNegativeInt("num_odometry_states"));
  CHECK_GT(options.num_odometry_states(), 0);
  return options;
}

// kalman滤波器的初始化　返回一个均值为０方差为无穷小高斯分布
PoseTracker::Distribution PoseTracker::KalmanFilterInit() {
  State initial_state = State::Zero();
  // We are certain about the complete state at the beginning. We define the
  // initial pose to be at the origin and axis aligned. Additionally, we claim
  // that we are not moving.
  StateCovariance initial_covariance = 1e-9 * StateCovariance::Identity();
  return Distribution(initial_state, initial_covariance);
}

PoseTracker::PoseTracker(const proto::PoseTrackerOptions& options,
                         const common::Time time)
    : options_(options),  //一些配置参数　各个方差之类的
      time_(time),        //上一次更新的时刻
      kalman_filter_(KalmanFilterInit(), AddDelta, ComputeDelta),   //kalman滤波器
      imu_tracker_(options.imu_gravity_time_constant(), time),      //imu_tracker
      //odometrystate跟踪器(缓冲区)
      odometry_state_tracker_(options.num_odometry_states()) {}     

PoseTracker::~PoseTracker() {}

// 首先进行了时间的预测，然后从UKF中得到置信度
PoseTracker::Distribution PoseTracker::GetBelief(const common::Time time) {
  Predict(time);
  return kalman_filter_.GetBelief();
}

/**
 * @brief PoseTracker::GetPoseEstimateMeanAndCovariance
 * 得到t时刻，ukf滤波器估计的机器人的位姿和方差
 * @param time          对应的时间
 * @param pose          返回的机器人位姿
 * @param covariance    返回的机器人方差
 */
void PoseTracker::GetPoseEstimateMeanAndCovariance(const common::Time time,
                                                   transform::Rigid3d* pose,
                                                   PoseCovariance* covariance) {
  const Distribution belief = GetBelief(time);
  *pose = RigidFromState(belief.GetMean());
  static_assert(kMapPositionX == 0, "Cannot extract PoseCovariance.");
  static_assert(kMapPositionY == 1, "Cannot extract PoseCovariance.");
  static_assert(kMapPositionZ == 2, "Cannot extract PoseCovariance.");
  static_assert(kMapOrientationX == 3, "Cannot extract PoseCovariance.");
  static_assert(kMapOrientationY == 4, "Cannot extract PoseCovariance.");
  static_assert(kMapOrientationZ == 5, "Cannot extract PoseCovariance.");
  *covariance = belief.GetCovariance().block<6, 6>(0, 0);
  covariance->block<2, 2>(3, 3) +=
      options_.imu_gravity_variance() * Eigen::Matrix2d::Identity();
}

/**
 * @brief PoseTracker::BuildModelNoise
 * 构建间隔时间为t的噪声模型
 * 该噪声模型的噪声随着时间间隔的增大而增大
 * @param delta_t   对应的时间间隔
 * @return
 */
const PoseTracker::Distribution PoseTracker::BuildModelNoise(
    const double delta_t) const {
  // Position is constant, but orientation changes.
  StateCovariance model_noise = StateCovariance::Zero();
  
  // 对角线
  model_noise.diagonal() <<
      // Position in map.
      options_.position_model_variance() * delta_t,
      options_.position_model_variance() * delta_t,
      options_.position_model_variance() * delta_t,

      // Orientation in map.
      options_.orientation_model_variance() * delta_t,
      options_.orientation_model_variance() * delta_t,
      options_.orientation_model_variance() * delta_t,

      // Linear velocities in map.
      options_.velocity_model_variance() * delta_t,
      options_.velocity_model_variance() * delta_t,
      options_.velocity_model_variance() * delta_t;

  return Distribution(State::Zero(), model_noise);
}

/**
 * @brief PoseTracker::Predict
 * 预测time时刻的系统的状态
 * 滤波器预测函数　因此这这个系统中　imu中的数据都是用来预测系统位置的．
 * 因此每次imu数据的更新都需要调用这个函数来对系统状态进行新的预测
 * 这个预测函数会根据系统指定的model_function来自动选择系统的预测方程
 * @param time  对应的时刻
 */
void PoseTracker::Predict(const common::Time time) {
  imu_tracker_.Advance(time);
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  if (delta_t == 0.) {
    return;
  }
  kalman_filter_.Predict(
      // [ caputrue ] ( params ) opt -> ret { body; };
      [this, delta_t](const State& state) -> State {
        return ModelFunction(state, delta_t);
      },
      BuildModelNoise(delta_t));
  time_ = time;
}

void PoseTracker::AddImuLinearAccelerationObservation(
    const common::Time time, const Eigen::Vector3d& imu_linear_acceleration) {
  imu_tracker_.Advance(time);
  imu_tracker_.AddImuLinearAccelerationObservation(imu_linear_acceleration);
  Predict(time);
}

void PoseTracker::AddImuAngularVelocityObservation(
    const common::Time time, const Eigen::Vector3d& imu_angular_velocity) {
  imu_tracker_.Advance(time);
  imu_tracker_.AddImuAngularVelocityObservation(imu_angular_velocity);
  Predict(time);
}

void PoseTracker::AddPoseObservation(const common::Time time,
                                     const transform::Rigid3d& pose,
                                     const PoseCovariance& covariance) {
  Predict(time);

  // Noise covariance is taken directly from the input values.
  const GaussianDistribution<double, 6> delta(
      Eigen::Matrix<double, 6, 1>::Zero(), covariance);

  kalman_filter_.Observe<6>(
      [this, &pose](const State& state) -> Eigen::Matrix<double, 6, 1> {
        const transform::Rigid3d state_pose = RigidFromState(state);
        const Eigen::Vector3d delta_orientation =
            transform::RotationQuaternionToAngleAxisVector(
                pose.rotation().inverse() * state_pose.rotation());
        const Eigen::Vector3d delta_translation =
            state_pose.translation() - pose.translation();
        Eigen::Matrix<double, 6, 1> return_value;
        return_value << delta_translation, delta_orientation;
        return return_value;
      },
      delta);
}

// Updates from the odometer are in the odometer's map-like frame, called the
// 'odometry' frame. The odometer_pose converts data from the map frame
// into the odometry frame.
void PoseTracker::AddOdometerPoseObservation(
    const common::Time time, const transform::Rigid3d& odometer_pose,
    const PoseCovariance& covariance) {
  if (!odometry_state_tracker_.empty()) {
    const auto& previous_odometry_state = odometry_state_tracker_.newest();
    const transform::Rigid3d delta =
        previous_odometry_state.odometer_pose.inverse() * odometer_pose;
    const transform::Rigid3d new_pose =
        previous_odometry_state.state_pose * delta;
    AddPoseObservation(time, new_pose, covariance);
  }

  const Distribution belief = GetBelief(time);

  odometry_state_tracker_.AddOdometryState(
      {time, odometer_pose, RigidFromState(belief.GetMean())});
}

const mapping::OdometryStateTracker::OdometryStates&
PoseTracker::odometry_states() const {
  return odometry_state_tracker_.odometry_states();
}

/**
 * @brief PoseTracker::RigidFromState
 * 从机器人的位姿生成一个转换矩阵
 * @param state 机器人的位姿
 * @return
 */
transform::Rigid3d PoseTracker::RigidFromState(
    const PoseTracker::State& state) {
  return transform::Rigid3d(
      Eigen::Vector3d(state[PoseTracker::kMapPositionX],
                      state[PoseTracker::kMapPositionY],
                      state[PoseTracker::kMapPositionZ]),
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(state[PoseTracker::kMapOrientationX],
                          state[PoseTracker::kMapOrientationY],
                          state[PoseTracker::kMapOrientationZ])) *
          imu_tracker_.orientation()); 
          // 预测中的state方向角，是map中yaw角度的变化
          // imu_tracker_.orientation()是imu的重力方向，可以理解为roll和pitch
}

PoseCovariance BuildPoseCovariance(const double translational_variance,
                                   const double rotational_variance) {
  const Eigen::Matrix3d translational =
      Eigen::Matrix3d::Identity() * translational_variance;
  const Eigen::Matrix3d rotational =
      Eigen::Matrix3d::Identity() * rotational_variance;
  // clang-format off
  PoseCovariance covariance;
  covariance <<
      translational, Eigen::Matrix3d::Zero(),
      Eigen::Matrix3d::Zero(), rotational;
  // clang-format on
  return covariance;
}

}  // namespace kalman_filter
}  // namespace cartographer
