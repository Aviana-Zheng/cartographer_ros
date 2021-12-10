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

#ifndef CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_
#define CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_

#include "cartographer/mapping/proto/trajectory.pb.h"

#include "cartographer/common/time.h"

// 定义了关于3D扫描楼层的数据结构。
namespace cartographer {
namespace mapping {

/*
span,范围
Timespan表征时间范围。
*/
struct Timespan {
  common::Time start;
  common::Time end;
};

struct Floor {
  // The spans of time we spent on this floor. Since we might have walked up and
  // down many times in this place, there can be many spans of time we spent on
  // a particular floor.
  // 一个楼层对应多个扫描timespan：有可能重复的扫描多次, 但只有一个高度z。
  std::vector<Timespan> timespans;

  // The median z-value of this floor.
  double z;   //z轴的中值 
};

/*
heuristic:启发式,启发式搜索(Heuristically Search)又称为有信息搜索(Informed Search)，
它是利用问题拥有的启发信息来引导搜索，达到减少搜索范围、降低问题复杂度的目的，
这种利用启发信息的搜索过程称为启发式搜索。
使用启发式搜索寻找building的不同楼层的z值。
对楼层的要求：同一floor同一z值，只要有“楼梯”出现，即为“产生”一层
*/
// Uses a heuristic which looks at z-values of the poses to detect individual
// floors of a building. This requires that floors are *mostly* on the same
// z-height and that level changes happen *relatively* abrubtly, e.g. by taking
// the stairs.
// 使用启发式搜索寻找building的不同楼层的z值。
std::vector<Floor> DetectFloors(const proto::Trajectory& trajectory);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_
