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

#include "cartographer/mapping/sparse_pose_graph/optimization_problem_options.h"

#include "cartographer/common/ceres_solver_options.h"

namespace cartographer {
namespace mapping {
namespace sparse_pose_graph {

/*
从sparse_pose_graph.lua文件配置 获取参数
message OptimizationProblemOptions {
  optional double huber_scale = 1;
  optional double acceleration_weight = 8;
  optional double rotation_weight = 9;
  
  optional double consecutive_scan_translation_penalty_factor = 2;
  optional double consecutive_scan_rotation_penalty_factor = 3;
  optional bool log_solver_summary = 5;
  optional common.proto.CeresSolverOptions ceres_solver_options = 7;
}
*/
proto::OptimizationProblemOptions CreateOptimizationProblemOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::OptimizationProblemOptions options;
  options.set_huber_scale(parameter_dictionary->GetDouble("huber_scale"));
  options.set_acceleration_weight(
      parameter_dictionary->GetDouble("acceleration_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  options.set_consecutive_scan_translation_penalty_factor(
      parameter_dictionary->GetDouble(
          "consecutive_scan_translation_penalty_factor"));
  options.set_consecutive_scan_rotation_penalty_factor(
      parameter_dictionary->GetDouble(
          "consecutive_scan_rotation_penalty_factor"));
  options.set_log_solver_summary(
      parameter_dictionary->GetBool("log_solver_summary"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

}  // namespace sparse_pose_graph
}  // namespace mapping
}  // namespace cartographer
