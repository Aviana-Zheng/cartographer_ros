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

#ifndef CARTOGRAPHER_MAPPING_3D_CERES_POSE_H_
#define CARTOGRAPHER_MAPPING_3D_CERES_POSE_H_

#include <array>
#include <memory>

#include "Eigen/Core"
#include "cartographer/transform/rigid_transform.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping_3d {

class CeresPose {
 public:
  // 参考mapping2d-->scan_matching-->ceres.md: 基于Ceres库的扫描匹配器
  // LocalParameterization类的作用是解决非线性优化中的过参数化问题
  // 在SLAM中，当采用四元数表示位姿时，由于四元数本身的约束（模长为1），实际的自由度为3而非4。
  // 此时，若直接传递四元数进行优化，冗余的维数会带来计算资源的浪费
  CeresPose(
      const transform::Rigid3d& rigid,
      std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
      std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
      ceres::Problem* problem);

  CeresPose(const CeresPose&) = delete;
  CeresPose& operator=(const CeresPose&) = delete;

  const transform::Rigid3d ToRigid() const;

  double* translation() { return translation_.data(); }
  double* rotation() { return rotation_.data(); }

 private:
  std::array<double, 3> translation_;
  // Rotation quaternion as (w, x, y, z).
  std::array<double, 4> rotation_;
};

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_CERES_POSE_H_
