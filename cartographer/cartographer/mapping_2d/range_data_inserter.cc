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

#include "cartographer/mapping_2d/range_data_inserter.h"

#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping_2d/ray_casting.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

proto::RangeDataInserterOptions CreateRangeDataInserterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::RangeDataInserterOptions options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_insert_free_space(
      parameter_dictionary->HasKey("insert_free_space")
          ? parameter_dictionary->GetBool("insert_free_space")
          : true);
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

RangeDataInserter::RangeDataInserter(
    const proto::RangeDataInserterOptions& options)
    : options_(options),
      hit_table_(mapping::ComputeLookupTableToApplyOdds(
          mapping::Odds(options.hit_probability()))),
      miss_table_(mapping::ComputeLookupTableToApplyOdds(
          mapping::Odds(options.miss_probability()))) {}

void RangeDataInserter::Insert(const sensor::RangeData& range_data,
                               ProbabilityGrid* const probability_grid) const {
  // 检查Grid不为空，且完成标志为未完成
  CHECK_NOTNULL(probability_grid)->StartUpdate();

  // By not starting a new update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
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
  CastRays(range_data, probability_grid->limits(),
           [this, &probability_grid](const Eigen::Array2i& hit) {
             probability_grid->ApplyLookupTable(hit, hit_table_);
           },
           [this, &probability_grid](const Eigen::Array2i& miss) {
             if (options_.insert_free_space()) {
               probability_grid->ApplyLookupTable(miss, miss_table_);
             }
           });
}

}  // namespace mapping_2d
}  // namespace cartographer
