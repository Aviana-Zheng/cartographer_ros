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

 /*
预备知识:
c++11 提供了语言级别的时间函数.包括duration和time_point
duration是时间段,指的是某单位时间上的一个明确的tick(片刻数)：
3分钟->"3个1分钟",
1.5个"1/3秒" :1.5是tick,1/3秒是时间单位

time_point是一个duration和一个epoch(起点)的组合：
2017年5月4日是"自1970,01,01"以来的126200000秒数
类模板 std::chrono::time_point 表示时间中的一个点。
它被实现成如同存储一个 Duration 类型的自 Clock 的纪元起始开始的时间间隔的值。

common/time.h主要功能是提供时间转换函数：
*/
#ifndef CARTOGRAPHER_COMMON_TIME_H_
#define CARTOGRAPHER_COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {
//719162 是0001年1月1日到1970年1月1日所经历的天数
constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>;   //千万分之一秒,0.1us.
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  /*time_point的模板参数是UniversalTimeScaleClock,
  那为何其可以做模板参数呢：？符合std::关于clock的类型定义和static成员*/
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;  //0.1微秒,0.1us
using Time = UniversalTimeScaleClock::time_point;  //时间点

/*Time::min()是chrono自带的函数。返回一个低于1970.01.01的数。

编译运行cpp/cppstdlib_2nd/util/chrono1.cpp:
epoch: Thu Jan  1 08:00:00 1970
now:   Tue Jul  4 19:39:29 2017
min:   Tue Sep 21 08:18:27 1677
max:   Sat Apr 12 07:47:16 2262

*/
// Convenience functions to create common::Durations.
//将秒数seconds转为c++的duration实例对象
Duration FromSeconds(double seconds);
Duration FromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.
//将的duration实例对象转为 秒数
double ToSeconds(Duration duration);

// Creates a time from a Universal Time Scale.
//将UTC时间(0.1微秒)转化为c++的time_point对象
Time FromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
//将c++的time_point对象转为TUC时间,单位是0.1us
int64 ToUniversal(Time time);

// For logging and unit tests, outputs the timestamp integer.
//重载<<操作符,将time_point以string输出
std::ostream& operator<<(std::ostream& os, Time time);

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TIME_H_


/*
linux下关于time转换：
http://blog.chinaunix.net/uid-20532339-id-1931780.html
https://stackoverflow.com/questions/2883576/how-do-you-convert-epoch-time-in-c/7844741#7844741
https://www.epochconverter.com/batch
*/