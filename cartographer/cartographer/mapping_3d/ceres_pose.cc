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

#include "cartographer/mapping_3d/ceres_pose.h"

namespace cartographer {
namespace mapping_3d {

CeresPose::CeresPose(
    const transform::Rigid3d& rigid,
    std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
    std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
    ceres::Problem* problem)
    : translation_({{rigid.translation().x(), rigid.translation().y(),
                     rigid.translation().z()}}),
      rotation_({{rigid.rotation().w(), rigid.rotation().x(),
                  rigid.rotation().y(), rigid.rotation().z()}}) {
  // std::unique_ptr， 独占所指向的对象,这种所有权仅能够通过std::move函数来转移。
  // 调用release 会切断unique_ptr 和它原来管理的对象的联系。
  // release 返回的指针通常被用来初始化另一个智能指针或给另一个智能指针赋值。
  // 如果不用另一个智能指针来保存release返回的指针，程序就要负责资源的释放。
  // 重构参数，优化时实际使用的是rotation_parametrization维度的等效旋转矢量
  problem->AddParameterBlock(translation_.data(), 3,
                             translation_parametrization.release());
  problem->AddParameterBlock(rotation_.data(), 4,
                             rotation_parametrization.release());
}

const transform::Rigid3d CeresPose::ToRigid() const {
  return transform::Rigid3d(
      Eigen::Map<const Eigen::Vector3d>(translation_.data()), // vector
      Eigen::Quaterniond(rotation_[0], rotation_[1], rotation_[2],
                         rotation_[3])); //Quaternion
}

}  // namespace mapping_3d
}  // namespace cartographer
