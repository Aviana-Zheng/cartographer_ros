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

#ifndef CARTOGRAPHER_IO_NULL_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_NULL_POINTS_PROCESSOR_H_

#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {
/*
NullPointsProcessor是PointsProcessor的第十个子类(10)
空操作类。通常用于pipline的尾端，丢弃所有的点云，表示整个处理流程已经完毕。
*/
// A points processor that just drops all points. The end of a pipeline usually.
class NullPointsProcessor : public PointsProcessor {
 public:
  NullPointsProcessor() {}
  ~NullPointsProcessor() override {}

  //不作任何事情
  void Process(std::unique_ptr<PointsBatch> points_batch) override {}
  //返回"kFinished"状态，此操作会传递给上一个流水线
  FlushResult Flush() override { return FlushResult::kFinished; }
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_NULL_POINTS_PROCESSOR_H_
