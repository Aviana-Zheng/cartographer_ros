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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/*
local_trajectory_builder_options.proto
  // Time constant in seconds for the orientation moving average based on
  // observed gravity via the IMU. It should be chosen so that the error
  // 1. from acceleration measurements not due to gravity (which gets worse when
  // the constant is reduced) and
  // 2. from integration of angular velocities (which gets worse when the
  // constant is increased) is balanced.
  optional double imu_gravity_time_constant = 17;
不同的ImuTracker类的初始化， imu_gravity_time_constant的值不同
imu_gravity_time_constant = 10.,
num_odometry_states = 1000,
构造函数初始化
*/
ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),   // 1796年
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(Eigen::Vector3d::UnitZ()),    //0,0,1，只有z轴有重力加速度。
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

/*
从time_增加到time。
dt=t-t_;
dθ=w*dt
*/
void ImuTracker::Advance(const common::Time time) {
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  //方向角
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));  // dθ=w*dt
  orientation_ = (orientation_ * rotation).normalized();   //更新角度
  gravity_vector_ = rotation.inverse() * gravity_vector_;  //根据方向角，更新重力加速度
  time_ = time;
}

/*
https://blog.csdn.net/qq_42700518/article/details/110913070
更新imu测量得到的加速度。
********** 滑动平均滤波 ***********
参数：vector3d，测量值
距离加速度校正的时间间隔越久, 就越不相信角速度预测的姿态
而更相信下一次加速度测量出来的姿态.
1),dt=t_-t;
// 计算权重    即 时间间隔越久  alpha 越接近1  那么 重力越倾向于 最新测量的结果
2),alpha=1-e^(-dt/g);  
// 一阶低通滤波  融合 IMU测量与后验重力
3),gravity_vector_=(1-alpha)*gv_+alpha*imu_line;
4),更新orientation_

exponential,指数.
由于角速度信息短时间比较准, 因此短时间内更偏向于角速度预测出来的姿态, 
而由于累计误差的存在,长时间角速度积累的误差较大, 而加速度测量与时间无关,
所以长时间更偏向于加速度的测量.
总之是一种非常朴素简单的思想, 精度肯定不高,但是对于机器人来说足够了.
*/
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  // 距离加速度校正的时间间隔越久, 就越不相信角速度预测的姿态
  // 而更相信下一次加速度测量出来的姿态.
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();   	
          // HUGE_VAL  展开成指示上溢的正 double 表达式，不必可表示为 float
  last_linear_acceleration_time_ = time_;
  // 计算权重    即 时间间隔越久  alpha 越接近1  那么 重力越倾向于 最新测量的结果
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  // 一阶低通滤波  融合 IMU测量与后验重力
  // gravity_vector_ 是预测结果    重力在IMU系下的表示
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  // Eigen::Quaterniond::FromTwoVectors(before, after)  result * before = after
  // 已知两个向量，其中一个向量由另外一个向量旋转得到
  // 求他们的旋转矩阵
  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      gravity_vector_, orientation_.inverse() * Eigen::Vector3d::UnitZ());
  orientation_ = (orientation_ * rotation).normalized();    //更新方向角
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

/*
更新imu测量得到的角速度
*/
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer

/* 
其实滑动平均模型的原理就是一阶滞后滤波法，其表达式如下： 
  new_value=(1−a)×value+a×old_value

其中a的取值范围[0,1]，具体就是：本次滤波结果=(1-a)本次采样值+a上次滤波结果

采用此算法的目的是： 
1、降低周期性的干扰； 
2、在波动频率较高的场景有很好的效果。

https://blog.csdn.net/Rex_WUST/article/details/85003743 */