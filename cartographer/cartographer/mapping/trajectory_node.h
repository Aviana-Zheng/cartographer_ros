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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

/*
轨迹节点TrajectoryNode类，含有一个内部类ConstantData。
TrajectoryNode作用：在连续的轨迹上采样一些离散的点用于key frame，标识pose frame。

数据成员：
1), ConstantData* 
2) 位姿，pose


ConstantData类：

数据成员：
1，time ：时间
2，range_data_2d和range_data_3d：测量数据
3,trajectory_id:本节点所属的轨迹
4，Rigid3d ：tracking frame 到 pose frame的矩阵变换。
*/
struct TrajectoryNode {
  struct Data {
    common::Time time;  // 记录了扫描数据被插入子图的时刻

    // Range data in 'pose' frame. Only used in the 2D case.
    sensor::RangeData range_data_2d;   //测量得到的2D range数据

    // Range data in 'pose' frame. Only used in the 3D case.
    sensor::CompressedRangeData range_data_3d;   //测量得到的3D range数据

    // Transform from the 3D 'tracking' frame to the 'pose' frame of the range
    // data, which contains roll, pitch and height for 2D. In 3D this is always
    // identity.
    transform::Rigid3d tracking_to_pose;    // 节点在子图中的相对位姿
  };

  common::Time time() const { return constant_data->time; }
  //trimmed 修剪过的
  bool trimmed() const { return constant_data == nullptr; }

  // This must be a shared_ptr. If the data is used for visualization while the
  // node is being trimmed, it must survive until all use finishes.
  //常指针.指向某块内存,该内存块的数据不变，指针本身可以变。
  std::shared_ptr<const Data> constant_data;

  transform::Rigid3d pose; // 节点的全局位姿
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
