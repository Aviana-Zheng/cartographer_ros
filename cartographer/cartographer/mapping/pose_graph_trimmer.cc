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

#include "cartographer/mapping/pose_graph_trimmer.h"

#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// 修剪多个submaps至num_submaps_to_keep_
PureLocalizationTrimmer::PureLocalizationTrimmer(const int trajectory_id,
                                                 const int num_submaps_to_keep)
    : trajectory_id_(trajectory_id), num_submaps_to_keep_(num_submaps_to_keep) {
  // 至少保留3个submap，保留数量需要传参
  CHECK_GE(num_submaps_to_keep, 3);
}

void PureLocalizationTrimmer::Trim(Trimmable* const pose_graph) {
  const int total_num_submaps = pose_graph->num_submaps(trajectory_id_);
  while (total_num_submaps > num_submaps_trimmed_ + num_submaps_to_keep_) {
    const int submap_index_to_trim_next = num_submaps_trimmed_;
    pose_graph->MarkSubmapAsTrimmed(
        SubmapId{trajectory_id_, submap_index_to_trim_next});
    ++num_submaps_trimmed_;
  }
}

}  // namespace mapping
}  // namespace cartographer
