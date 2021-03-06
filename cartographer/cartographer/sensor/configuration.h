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

#ifndef CARTOGRAPHER_SENSOR_CONFIGURATION_H_
#define CARTOGRAPHER_SENSOR_CONFIGURATION_H_

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/sensor/proto/configuration.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace sensor {

// Creates the configuration for a singular sensor from 'parameter_dictionary'.
//从sensor配置文件解析sensor的数据参数。主要是sensor到机器坐标的转换
proto::Configuration::Sensor CreateSensorConfiguration(
    common::LuaParameterDictionary* parameter_dictionary);

// Creates the mapping from frame_id to Sensors defined in
// 'parameter_dictionary'.
// 求得多个sensor的配置的集合。
proto::Configuration CreateConfiguration(
    common::LuaParameterDictionary* parameter_dictionary);

// Returns true if 'frame_id' is mentioned in 'sensor_configuration'.
//系统是否支持某一传感器
bool IsEnabled(const string& frame_id,
               const sensor::proto::Configuration& sensor_configuration);

// Returns the transform which takes data from the sensor frame to the
// tracking frame.
// 将sensor采集的data经过3d坐标变换为机器坐标
transform::Rigid3d GetTransformToTracking(
    const string& frame_id,
    const sensor::proto::Configuration& sensor_configuration);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_CONFIGURATION_H_
