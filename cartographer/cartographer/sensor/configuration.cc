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

#include "cartographer/sensor/configuration.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/sensor/proto/configuration.pb.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {
/* assets_writer_backpack_2d.lua
configuration.proto   Sensor定义： 
message Sensor { 
  string frame_id = 2; //'frame_id' of the sensor.
 Rigid3d transform = 3; 
} */ 
proto::Configuration::Sensor CreateSensorConfiguration(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::Configuration::Sensor sensor;
  sensor.set_frame_id(parameter_dictionary->GetString("frame_id"));
  *sensor.mutable_transform() = transform::ToProto(transform::FromDictionary(
      parameter_dictionary->GetDictionary("transform").get()));
  return sensor;
}

// Configuration定义： message Configuration { repeated Sensor sensor = 15; } 
proto::Configuration CreateConfiguration(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::Configuration configuration;
  for (auto& sensor_parameter_dictionary :
       parameter_dictionary->GetArrayValuesAsDictionaries()) {
    *configuration.add_sensor() =
        CreateSensorConfiguration(sensor_parameter_dictionary.get());
  }
  return configuration;
}

// 判断系统是否支持某一传感器id
bool IsEnabled(const string& frame_id,
               const sensor::proto::Configuration& sensor_configuration) {
  for (const auto& sensor : sensor_configuration.sensor()) {
    if (sensor.frame_id() == frame_id) {
      return true;
    }
  }
  return false;
}

//将sensor坐标转换为机器坐标。以便于跟踪计算
transform::Rigid3d GetTransformToTracking(
    const string& frame_id,
    const sensor::proto::Configuration& sensor_configuration) {
  for (const auto& sensor : sensor_configuration.sensor()) {
    if (sensor.frame_id() == frame_id) {
      return transform::ToRigid3(sensor.transform());
    }
  }
  LOG(FATAL) << "No configuration found for sensor with frame ID '" << frame_id
             << "'.";
}

}  // namespace sensor
}  // namespace cartographer
