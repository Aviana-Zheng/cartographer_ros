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

#ifndef CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_

#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/mapping/proto/sparse_pose_graph.pb.h"
#include "cartographer/mapping/proto/sparse_pose_graph_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// 根据sparse_pose_graph.lua文件设置option
proto::SparsePoseGraphOptions CreateSparsePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

/*
SparsePoseGraph:稀疏位姿图模型,虚基类,提供多个抽象接口,不可拷贝/赋值
有一个Constraint的内部类,
稀疏图用于闭环检测
*/
class SparsePoseGraph {
 public:
  // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
  // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
  // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
  // 约束 
  // cartographer paper-公式(SPA) (4) (5)
  struct Constraint {
    struct Pose {
      transform::Rigid3d zbar_ij;  // 在子图的 ,对应εij
      // 两个权重对应Σij，用于描述不确定度
      double translation_weight;   // 平移的权重
      double rotation_weight;      // 旋转的权重
    };

    // 有一个相同的字段trajectory_id，用于标记当前跟踪的轨迹。
    // 各自有一个从0开始计数的submap_index和node_index，
    // 分别为每个子图和节点提供一个唯一的编号
    SubmapId submap_id;  // 'i' in the paper.记录约束对应的子图索引
    NodeId node_id;      // 'j' in the paper.  记录节点索引

    // Pose of the scan 'j' relative to submap 'i'.
    Pose pose;

    // Differentiates between intra-submap (where scan 'j' was inserted into
    // submap 'i') and inter-submap constraints (where scan 'j' was not inserted
    // into submap 'i').
    // INTER_SUBMAP: loop closure constraints
    // Only loop closure constraints should have a loss function.
    enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
  };

  struct SubmapData {
    std::shared_ptr<const Submap> submap;
    transform::Rigid3d pose;
  };

  SparsePoseGraph() {}
  virtual ~SparsePoseGraph() {}

  SparsePoseGraph(const SparsePoseGraph&) = delete;
  SparsePoseGraph& operator=(const SparsePoseGraph&) = delete;

  // Adds a 'trimmer'. It will be used after all data added before it has been
  // included in the pose graph.
  virtual void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) = 0;

  // Computes optimized poses. 计算优化后的位姿估计
  virtual void RunFinalOptimization() = 0;

  // Gets the current trajectory clusters.获取已连接的轨迹集合
  virtual std::vector<std::vector<int>> GetConnectedTrajectories() = 0;

  // Return the number of submaps for the given 'trajectory_id'.
  virtual int num_submaps(int trajectory_id) = 0;

  // Returns the current optimized transform and submap itself for the given
  // 'submap_id'.
  virtual SubmapData GetSubmapData(const SubmapId& submap_id) = 0;

  // Returns data for all Submaps by trajectory.
  virtual std::vector<std::vector<SubmapData>> GetAllSubmapData() = 0;

  // Returns the transform converting data in the local map frame (i.e. the
  // continuous, non-loop-closed frame) into the global map frame (i.e. the
  // discontinuous, loop-closed frame).
  // 获取局部图的3D变换
  virtual transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) = 0;

  // Returns the current optimized trajectories.
  // 优化后的轨迹线
  virtual std::vector<std::vector<TrajectoryNode>> GetTrajectoryNodes() = 0;

  // Serializes the constraints and trajectories.
  proto::SparsePoseGraph ToProto();

  // Returns the collection of constraints.
  // 获取约束集
  virtual std::vector<Constraint> constraints() = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_
