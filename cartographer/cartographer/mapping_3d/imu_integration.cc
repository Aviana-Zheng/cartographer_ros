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

#include "cartographer/mapping_3d/imu_integration.h"

namespace cartographer {
namespace mapping_3d {

// 返回以start位置为坐标系原点的，到end的delta 速度 旋转， it是最后一个小于start_time的位置
IntegrateImuResult<double> IntegrateImu(
    const std::deque<ImuData>& imu_data, const common::Time start_time,
    const common::Time end_time, std::deque<ImuData>::const_iterator* it) {
  return IntegrateImu<double>(imu_data, Eigen::Affine3d::Identity(),
                              Eigen::Affine3d::Identity(), start_time, end_time,
                              it);
}

}  // namespace mapping_3d
}  // namespace cartographer

/*
仿射变换矩阵实际上就是：平移向量+旋转变换组合而成，可以同时实现旋转，缩放，平移等空间变换。

Eigen库中，仿射变换矩阵的大致用法为： 

    创建Eigen::Affine3f 对象a。
    创建类型为Eigen::Translation3f 对象b，用来存储平移向量；
    创建类型为Eigen::Quaternionf 四元数对象c，用来存储旋转变换；
    最后通过以下方式生成最终Affine3f变换矩阵： a=b*c.toRotationMatrix();
    一个向量通过仿射变换时的方法是result_vector=test_affine*test_vector;
*/