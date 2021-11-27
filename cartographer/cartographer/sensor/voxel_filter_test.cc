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

#include "cartographer/sensor/voxel_filter.h"

#include <cmath>

#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::ContainerEq;

TEST(VoxelFilterTest, ReturnsTheFirstPointInEachVoxel) {
  PointCloud point_cloud = {{0.f, 0.f, 0.f},
                            {0.1f, -0.1f, 0.1f},
                            {0.3f, -0.1f, 0.f},
                            {0.f, 0.f, 0.1f}};
  PointCloud point_cloud_filtered = VoxelFiltered(point_cloud, 0.3f);
  for(auto point : point_cloud_filtered){
    LOG(INFO) << point.x() << "," << point.y() << "," << point.z() << std::endl;
  }
  /*对每一个点云point,如果落在Cell中，则插入piont，并且后续不在接收落在此Cell的point。
  [point1] ,[point2],[point3],[point4]....
  [point11] ,[point12],[point13],[point14]....
  */
  EXPECT_THAT(VoxelFiltered(point_cloud, 0.3f),   //按照max_range滤波,
              ContainerEq(PointCloud{point_cloud[0], point_cloud[2]}));
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
