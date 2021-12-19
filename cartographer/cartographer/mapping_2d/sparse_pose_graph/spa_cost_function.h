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

#ifndef CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_SPA_COST_FUNCTION_H_
#define CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_SPA_COST_FUNCTION_H_

#include <array>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"

namespace cartographer {
namespace mapping_2d {
namespace sparse_pose_graph {

/* 位姿估计代价函数
 * cartographer paper-公式(SPA) (4) (5)
 * SPA（Sparse Pose Adjustment）的技术，根据路径节点和子图之间的约束关系，
 * 最终优化路径节点和子图的世界坐标
 * 路径节点和子图之间的约束关系是由约束构建器通过回环检测方法得到的。
 * 而所谓的SPA技术，本质上还是通过LM（Levenberg-Marquardt）方法进行非线性寻优，
 * 并且由于不是所有的子图和路径节点之间都能成功构建约束，以至于LM的增量方程中的H矩阵是一个稀疏矩阵，
 * 这样就可以通过使用一些优化手段来降低对内存的需求，提高求解效率。
 */

/*
 * SPA残差
 * Constraint与真实位置间的残差
 * Constraint: 只是一个Node j与一个Submap i间最好的匹配，二者匹配得最好，但只关注了局部，
 * 并不能保证整个Pose Graph匹配得最好，即与真实环境一致
 * SPA优化的目的：使用所有Node与所有的Submap整体上匹配得最好，即关注全局
 * SPA优化的方法：使所有残差之和最小，　残差为：constraint 的 pose 与真实pose之差最小
 * Ceres的求解参数的数据类型只支持：double*
 */
class SpaCostFunction {
 public:
  using Constraint = mapping::SparsePoseGraph::Constraint;

  explicit SpaCostFunction(const Constraint::Pose& pose) : pose_(pose) {}

  // Computes the error between the scan-to-submap alignment 'zbar_ij' and the
  // difference of submap pose 'c_i' and scan pose 'c_j' which are both in an
  // arbitrary common frame.
  // 论文公式(5),求差
  template <typename T>
  static std::array<T, 3> ComputeUnscaledError(
      const transform::Rigid2d& zbar_ij, // 第i个子图与第j个节点之间的相对位姿
      const T* const c_i,
      const T* const c_j) {
    // 由子图位姿写出的旋转矩阵，Rεmi
    // 对其求逆即可得到世界坐标系下的矢量在子图的局部坐标系下的转换关系
    // 此处直接计算cos sin
    const T cos_theta_i = cos(c_i[2]);
    const T sin_theta_i = sin(c_i[2]);
    const T delta_x = c_j[0] - c_i[0];
    const T delta_y = c_j[1] - c_i[1];
    // 由子图位姿写出的旋转矩阵 Rεmi 的逆，R−1εmi(tεmi−tεsj)
    // 将世界坐标下子图和节点位姿投影到子图的局部坐标系下，并计算两者的相对偏差量
    // c_j[2]、c_i[2]分别是子图位姿和节点位姿中的方向角，两者求差即是子图局部坐标系下的相对方向角
    const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                    -sin_theta_i * delta_x + cos_theta_i * delta_y,
                    c_j[2] - c_i[2]};
    // 论文公式 (4) 求差
    return {{T(zbar_ij.translation().x()) - h[0],
             T(zbar_ij.translation().y()) - h[1],
             common::NormalizeAngleDifference(T(zbar_ij.rotation().angle()) -
                                              h[2])}};
  }

  // Computes the error scaled by 'translation_weight' and 'rotation_weight',
  // storing it in 'e'.
  template <typename T>
  static void ComputeScaledError(const Constraint::Pose& pose,
                                 const T* const c_i, const T* const c_j,
                                 T* const e) {
    // 论文公式(5),求差
    const std::array<T, 3> e_ij =
        ComputeUnscaledError(transform::Project2D(pose.zbar_ij), c_i, c_j);
    // 协方差矩阵Σij来描述εij的可信度, 平移和旋转的权重，对应着Σij
    // 论文公式 (4)
    e[0] = e_ij[0] * T(pose.translation_weight);
    e[1] = e_ij[1] * T(pose.translation_weight);
    e[2] = e_ij[2] * T(pose.rotation_weight);
  }

  template <typename T>
  bool operator()(const T* const c_i, const T* const c_j, T* e) const {
    ComputeScaledError(pose_, c_i, c_j, e);
    return true;
  }

 private:
  // 描述节点j在子图i中的位姿，以及平移和旋转的权重，对应着Σij
  const Constraint::Pose pose_; 
};

}  // namespace sparse_pose_graph
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_SPA_COST_FUNCTION_H_
