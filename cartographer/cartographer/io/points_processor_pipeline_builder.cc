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

#include "cartographer/io/points_processor_pipeline_builder.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/io/coloring_points_processor.h"
#include "cartographer/io/counting_points_processor.h"
#include "cartographer/io/fixed_ratio_sampling_points_processor.h"
#include "cartographer/io/hybrid_grid_points_processor.h"
#include "cartographer/io/intensity_to_color_points_processor.h"
#include "cartographer/io/min_max_range_filtering_points_processor.h"
#include "cartographer/io/null_points_processor.h"
#include "cartographer/io/outlier_removing_points_processor.h"
#include "cartographer/io/pcd_writing_points_processor.h"
#include "cartographer/io/ply_writing_points_processor.h"
#include "cartographer/io/xray_points_processor.h"
#include "cartographer/io/xyz_writing_points_processor.h"
#include "cartographer/mapping/proto/trajectory.pb.h"

namespace cartographer {
namespace io {
// register 登记  plain 清楚的 
template <typename PointsProcessorType>
void RegisterPlainPointsProcessor(
    PointsProcessorPipelineBuilder* const builder) {
  builder->Register(
      PointsProcessorType::kConfigurationFileActionName,
      [](common::LuaParameterDictionary* const dictionary,
         PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
        return PointsProcessorType::FromDictionary(dictionary, next);
      });
      /*https://blog.csdn.net/jiange_zh/article/details/79356417
      Lambda 表达式，实际上就是提供了一个类似匿名函数的特性，而匿名函数则是在需要一个函数，但是又不想费力去命名一个函数的情况下去使用的。

      Lambda 表达式的基本语法如下：
      [ caputrue ] ( params ) opt -> ret { body; };
      1) capture是捕获列表；
      2) params是参数表；(选填)
      3) opt是函数选项；可以填mutable,exception,attribute（选填）
      mutable说明lambda表达式体内的代码可以修改被捕获的变量，并且可以访问被捕获的对象的non-const方法。
      exception说明lambda表达式是否抛出异常以及何种异常。
      attribute用来声明属性。
      4) ret是返回值类型（拖尾返回类型）。(选填)
      5) body是函数体。
      */
}

template <typename PointsProcessorType>
void RegisterFileWritingPointsProcessor(
    FileWriterFactory file_writer_factory,
    PointsProcessorPipelineBuilder* const builder) {
  builder->Register(
      PointsProcessorType::kConfigurationFileActionName,
      [file_writer_factory](
          common::LuaParameterDictionary* const dictionary,
          PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
        return PointsProcessorType::FromDictionary(file_writer_factory,
                                                   dictionary, next);
      });
}

void RegisterBuiltInPointsProcessors(
    const mapping::proto::Trajectory& trajectory,
    FileWriterFactory file_writer_factory,
    PointsProcessorPipelineBuilder* builder) {
  RegisterPlainPointsProcessor<CountingPointsProcessor>(builder);
  RegisterPlainPointsProcessor<FixedRatioSamplingPointsProcessor>(builder);
  RegisterPlainPointsProcessor<MinMaxRangeFiteringPointsProcessor>(builder);
  RegisterPlainPointsProcessor<OutlierRemovingPointsProcessor>(builder);
  RegisterPlainPointsProcessor<ColoringPointsProcessor>(builder);
  RegisterPlainPointsProcessor<IntensityToColorPointsProcessor>(builder);
  RegisterFileWritingPointsProcessor<PcdWritingPointsProcessor>(
      file_writer_factory, builder);
  RegisterFileWritingPointsProcessor<PlyWritingPointsProcessor>(
      file_writer_factory, builder);
  RegisterFileWritingPointsProcessor<XyzWriterPointsProcessor>(
      file_writer_factory, builder);
  RegisterFileWritingPointsProcessor<HybridGridPointsProcessor>(
      file_writer_factory, builder);

  // X-Ray is an odd ball since it requires the trajectory to figure out the
  // different building levels we walked on to separate the images.
  builder->Register(
      XRayPointsProcessor::kConfigurationFileActionName,
      [&trajectory, file_writer_factory](
          common::LuaParameterDictionary* const dictionary,
          PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
        return XRayPointsProcessor::FromDictionary(
            trajectory, file_writer_factory, dictionary, next);
      });
}

void PointsProcessorPipelineBuilder::Register(const std::string& name,
                                              FactoryFunction factory) {
  CHECK(factories_.count(name) == 0) << "A points processor named '" << name
                                     << "' has already been registered.";
  factories_[name] = factory;
}

PointsProcessorPipelineBuilder::PointsProcessorPipelineBuilder() {}

std::vector<std::unique_ptr<PointsProcessor>>
PointsProcessorPipelineBuilder::CreatePipeline(
    common::LuaParameterDictionary* const dictionary) const {
  std::vector<std::unique_ptr<PointsProcessor>> pipeline;
  // The last consumer in the pipeline must exist, so that the one created after
  // it (and being before it in the pipeline) has a valid 'next' to point to.
  // The last consumer will just drop all points.
  pipeline.emplace_back(common::make_unique<NullPointsProcessor>());

  std::vector<std::unique_ptr<common::LuaParameterDictionary>> configurations =
      dictionary->GetArrayValuesAsDictionaries();

  // We construct the pipeline starting at the back.
  for (auto it = configurations.rbegin(); it != configurations.rend(); it++) {
    const string action = (*it)->GetString("action");
    auto factory_it = factories_.find(action);
    CHECK(factory_it != factories_.end())
        << "Unknown action '" << action
        << "'. Did you register the correspoinding PointsProcessor?";
    pipeline.push_back(factory_it->second(it->get(), pipeline.back().get()));
  }
  return pipeline;
}

}  // namespace io
}  // namespace cartographer
