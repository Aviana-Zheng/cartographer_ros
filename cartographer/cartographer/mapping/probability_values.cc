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

#include "cartographer/mapping/probability_values.h"

namespace cartographer {
namespace mapping {

namespace {

// 0 is unknown, [1, 32767] maps to [kMinProbability, kMaxProbability].
// 在计算时并不是用浮点数进行的计算，二是将0~1的概率值映射到1_32767的整数值
float SlowValueToProbability(const uint16 value) {
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == kUnknownProbabilityValue) {
    // Unknown cells have kMinProbability.
    return kMinProbability;
  }
  const float kScale = (kMaxProbability - kMinProbability) / 32766.f;
  return value * kScale + (kMinProbability - kScale);
}

/* 
 *把[1,32767]之间的所有value预先计算出来其映射到[lower_bound, upper_bound]
 *这个区间的对应浮点值，存到一个浮点型向量中： 
 */
const std::vector<float>* PrecomputeValueToProbability() {
  std::vector<float>* result = new std::vector<float>;
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      result->push_back(SlowValueToProbability(value));
    }
  }
  return result;
}

}  // namespace


const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability();

/* 
 *该函数的含义是，对于一个value~[1,32767], 如果有一个新的odds值的观测后，更新后的value应该是什么。
 *这里对所有可能的value都进行了计算，存在了一个列表中。odds只有两种情况，hit或misses. 
 *因此，可以预先计算出来两个列表。这样，在有一个新的odds时可根据原有的value值查表得到一个
 *新的value值，更新 
 */
std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  //这个表示这个表中的第一个元素对应了如果之前该点是unknown状态，更新的value应该是什么
  // 在ComputeLookupTableToApplyOdds这个转化里都加了一个kUpdateMarker，相当于整体有了一个偏移。
  // 虽然这样对整体代码逻辑没有影响，用来标记这个栅格已经被更新了，不要重复更新
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker);
  for (int cell = 1; cell != 32768; ++cell) {
    // LOG(INFO) << (*kValueToProbability)[cell] << std::endl;
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer
