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

#include "cartographer/io/min_max_range_filtering_points_processor.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

//从.lua文件加载min_range和max_range,如:0-30
std::unique_ptr<MinMaxRangeFiteringPointsProcessor>
MinMaxRangeFiteringPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<MinMaxRangeFiteringPointsProcessor>(
      dictionary->GetDouble("min_range"), dictionary->GetDouble("max_range"),
      next);//assets_writer_backpack_2d.lua 1-60
}

// 构造函数,传递min ,max2个范围参数和 PointsProcessor* next指针
MinMaxRangeFiteringPointsProcessor::MinMaxRangeFiteringPointsProcessor(
    const double min_range, const double max_range, PointsProcessor* next)
    : min_range_(min_range), max_range_(max_range), next_(next) {}

void MinMaxRangeFiteringPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  std::vector<int> to_remove;   //待移除的索引
  for (size_t i = 0; i < batch->points.size(); ++i) {
    //计算sensor到远点的l2距离
    const float range = (batch->points[i] - batch->origin).norm();
    if (!(min_range_ <= range && range <= max_range_)) {
      //不在给定范围内,加入移除队列.
      to_remove.push_back(i);
    }
  }
  RemovePoints(to_remove, batch.get());
  //完成本轮过滤,接下来进行下一次过滤.即模拟流水线操作.
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult MinMaxRangeFiteringPointsProcessor::Flush() {
  return next_->Flush();
}

}  // namespace io
}  // namespace cartographer
