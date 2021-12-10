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

#ifndef CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
#define CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_

#include <cmath>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

// 参考 https://zhuanlan.zhihu.com/p/49030629
namespace cartographer {
namespace mapping {
/*   
 *cartographer中 p(s=1) 称为Probability,后面公式中我用 p 表示；
 *p(s=0) 称为CorrespondenceCost, 后面公式中我用 c 表示
 *Odd(s)记为了Odds.  
 */

//由Probability计算odds
inline float Odds(float probability) {
  return probability / (1.f - probability);
}

//由odds计算probability
inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

constexpr float kMinProbability = 0.1f;      //最小概率为0.1
constexpr float kMaxProbability = 1.f - kMinProbability;    //最大概率为1-0.1=0.9

// Clamps probability to be in the range [kMinProbability, kMaxProbability].
// clamp(对角线)函数的含义是，如果给定参数小于最小值，则返回最小值；
// 如果大于最大值则返回最大值；其他情况正常返回参数
inline float ClampProbability(const float probability) {
  return common::Clamp(probability, kMinProbability, kMaxProbability);
}

//在没有任何先验信息情况下，Occupied和Free概率值都为0
constexpr uint16 kUnknownProbabilityValue = 0;
// 左移的话kUpdateMarker是2的15次方：32768，也就是所以概率值转化成整数value之后的最大范围。
// 所以程序中有判断是否越界
constexpr uint16 kUpdateMarker = 1u << 15;   

// Converts a probability to a uint16 in the [1, 32767] range.
/* 
 *cartongrapher为了尽量避免浮点运算，将[kMinProbability, kMaxProbability]或
 *[kMinCorrespondenceCost, kMaxCorrespondenceCost]之间的浮点数映射到了
 *整数区间：[1, 32767]， 由浮点数到整数区间的映射函数为： 
 */
inline uint16 ProbabilityToValue(const float probability) {
  const int value =
      common::RoundToInt((ClampProbability(probability) - kMinProbability) *
                         (32766.f / (kMaxProbability - kMinProbability))) +
      1;
  // DCHECK for performance.
  DCHECK_GE(value, 1);   //检查是否大于等于1
  DCHECK_LE(value, 32767);   //是否小于等于32767
  return value;
}

extern const std::vector<float>* const kValueToProbability;

// Converts a uint16 (which may or may not have the update marker set) to a
// probability in the range [kMinProbability, kMaxProbability].
// 整数区间value向浮点数映射：
inline float ValueToProbability(const uint16 value) {
  return (*kValueToProbability)[value];
}

std::vector<uint16> ComputeLookupTableToApplyOdds(float odds);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_PROBABILITY_VALUES_H_
