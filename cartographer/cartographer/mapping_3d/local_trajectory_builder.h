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

#ifndef CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_

#include <memory>
#include <vector>

#include "cartographer/mapping_3d/local_trajectory_builder_interface.h"
#include "cartographer/mapping_3d/proto/local_trajectory_builder_options.pb.h"

namespace cartographer {
namespace mapping_3d {

// 根据trajectory_builder_3d.lua 中 use = "KALMAN",  -- or "OPTIMIZING".决定slam后端方法
std::unique_ptr<LocalTrajectoryBuilderInterface> CreateLocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions&
        local_trajectory_builder_options);

}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_LOCAL_TRAJECTORY_BUILDER_H_
