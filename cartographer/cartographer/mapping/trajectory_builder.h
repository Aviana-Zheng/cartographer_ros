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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_

#include <functional>
#include <memory>
#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

// trajectory_builder.lua
proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);


/*
TrajectoryBuilder:虚基类,提供多个抽象接口
作用：根据轨迹Builder收集data。

成员函数有：
虚函数：
1),submaps()
2),AddSensorData()
3),pose_estimate()

非虚函数：
1),AddRangefinderData()
2),AddImuData()
3),AddOdometerData()

有一个内部类PoseEstimate,标识当前关键帧的Pose，包括估计位姿的时间,位置,点云

*/
// This interface is used for both 2D and 3D SLAM.
class TrajectoryBuilder {
 public:
  // Represents a newly computed pose. 'pose' is the end-user visualization of
  // orientation and 'point_cloud' is the point cloud, in the local map frame.
  /*
  PoseEstimate代表一个已经估计好的位姿,
  pose表示从start点看去的视觉定位,
  point_cloud表示局部帧的点云
  */
  struct PoseEstimate {
    PoseEstimate() = default;
    PoseEstimate(common::Time time, const transform::Rigid3d& pose,
                 const sensor::PointCloud& point_cloud)
        : time(time), pose(pose), point_cloud(point_cloud) {}

    common::Time time = common::Time::min();    //测量时间
    transform::Rigid3d pose = transform::Rigid3d::Identity();   //世界坐标转换
    sensor::PointCloud point_cloud;   //子图local map frame的点云
  };

  TrajectoryBuilder() {}
  virtual ~TrajectoryBuilder() {}

  TrajectoryBuilder(const TrajectoryBuilder&) = delete;
  TrajectoryBuilder& operator=(const TrajectoryBuilder&) = delete;

  virtual const PoseEstimate& pose_estimate() const = 0;   //子图位姿及其采集的点云

  // 根据sensor_id添加data，虚函数。
  virtual void AddSensorData(const string& sensor_id,
                             std::unique_ptr<sensor::Data> data) = 0;

  /*
  下面3个函数都是非虚函数。
  分别是添加雷达/imu/里程计的data。

  参数：
  1),sensor_id,标识传感器。
  2),time 测量时间
  3),PointCloud/Vector3d  /Rigid/Rigid3d 测量得到的数据
  */
  void AddRangefinderData(const string& sensor_id, common::Time time,
                          const Eigen::Vector3f& origin,
                          const sensor::PointCloud& ranges) {
    AddSensorData(sensor_id,
                  common::make_unique<sensor::Data>(
                      time, sensor::Data::Rangefinder{origin, ranges}));
  }

  void AddImuData(const string& sensor_id, common::Time time,
                  const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity) {
    AddSensorData(sensor_id, common::make_unique<sensor::Data>(
                                 time, sensor::Data::Imu{linear_acceleration,
                                                         angular_velocity}));
  }

  void AddOdometerData(const string& sensor_id, common::Time time,
                       const transform::Rigid3d& odometer_pose) {
    AddSensorData(sensor_id,
                  common::make_unique<sensor::Data>(time, odometer_pose));
  }
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_H_
