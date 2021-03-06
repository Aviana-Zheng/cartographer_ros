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

#ifndef CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"

/*
点云数据是指在一个三维坐标系统中的一组向量的集合。{p1,p2,p3,...}
这些向量通常以X,Y,Z三维坐标的形式表示p:{x,y,z}，
而且一般主要用来代表一个物体的外表面形状。
除{x,y,z}可以代表的几何位置信息之外，
点云数据还可以表示一个点的RGB颜色，灰度值，深度，分割结果等。

Eg..Pi={Xi, Yi, Zi,…….}表示空间中的一个点，
则Point Cloud={P1, P2, P3,…..Pn}表示一组点云数据。

cartographer的PointCloud是由Vector3f组成的vector。
PointCloudWithIntensities则是由点云和光线强度组成的struct类。
*/
namespace cartographer {
namespace sensor {

//vector，元素是3*1f
typedef std::vector<Eigen::Vector3f> PointCloud;

//点云+光线强度,{x,y,z}+intensity
struct PointCloudWithIntensities {
  PointCloud points;   //3*1的vector
  std::vector<float> intensities;
};

// Transforms 'point_cloud' according to 'transform'.  根据三维网格参数转换点云
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
/*去掉z轴区域外的点云,返回一个新的点云*/
PointCloud Crop(const PointCloud& point_cloud, float min_z, float max_z);

// Converts 'point_cloud' to a proto::PointCloud.
//序列化
proto::PointCloud ToProto(const PointCloud& point_cloud);

// Converts 'proto' to a PointCloud.
//反序列化
PointCloud ToPointCloud(const proto::PointCloud& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
