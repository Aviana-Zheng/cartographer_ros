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

#include "cartographer/sensor/point_cloud.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

/*. 
功能：根据3D变换，转换点云,
返回：转换后的新点云结果，原点云未变。
*/
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform) {
  PointCloud result;  //vector,元素是3*1f
  result.reserve(point_cloud.size());    //分配内存
  for (const Eigen::Vector3f& point : point_cloud) {
    // emplace_back函数的作用是减少对象拷贝和构造次数，
    // 是C++11中的新特性，主要适用于对临时对象的赋值。
    result.emplace_back(transform * point);
  }
  return result;
}

/*
功能：按范围丢弃点云，去掉z轴区域外的点云
返回：新的符合要求的点云。
*/
PointCloud Crop(const PointCloud& point_cloud, const float min_z,
                const float max_z) {
  PointCloud cropped_point_cloud;
  for (const auto& point : point_cloud) {
    if (min_z <= point.z() && point.z() <= max_z) {
      cropped_point_cloud.push_back(point);
    }
  }
  return cropped_point_cloud;
}

//序列化
proto::PointCloud ToProto(const PointCloud& point_cloud) {
  proto::PointCloud proto;
  for (const auto& point : point_cloud) {
    proto.add_x(point.x());
    proto.add_y(point.y());
    proto.add_z(point.z());
  }
  return proto;
}

//反序列化
PointCloud ToPointCloud(const proto::PointCloud& proto) {
  PointCloud point_cloud;
  const int size = std::min({proto.x_size(), proto.y_size(), proto.z_size()});
  point_cloud.reserve(size);   //最小，否则以下语句有bug
  for (int i = 0; i != size; ++i) {
    point_cloud.emplace_back(proto.x(i), proto.y(i), proto.z(i));
  }
  return point_cloud;
}

}  // namespace sensor
}  // namespace cartographer
