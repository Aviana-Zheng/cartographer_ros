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
std::chrono::duration_cast
不使用隐式转换。可能的情况下避免乘法和除法，若在编译时已知一或多个参数为 1 。
以最宽的可用类型进行计算，而如同用 static_cast 到结果类型的转换，只在完成时进行。 
*/

#include "cartographer/common/time.h"

#include <string>

namespace cartographer {
namespace common {

//duration_cast是c++ 11的时间显式转换函数.
Duration FromSeconds(const double seconds) {
  //将double类型的秒数转化为duration对象
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(seconds));  
      //std::chrono::duration<double>
      //_Period缺省值为std::ratio<1>，此时它的单位为秒
}

double ToSeconds(const Duration duration) {
  //反转化,count()返回时钟周期数,ticks
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

//先构造一个临时duration对象,再将其转化为time_point对象
//Duration(ticks)调用的是UniversalTimeScaleClock的构造函数
Time FromUniversal(const int64 ticks) { return Time(Duration(ticks)); }

//count()返回time_point自epoch以来的时钟周期数
int64 ToUniversal(const Time time) { return time.time_since_epoch().count(); }

//先将Time转化为 int64 , 再转为字符串形式
std::ostream& operator<<(std::ostream& os, const Time time) {
  os << std::to_string(ToUniversal(time));
  return os;
}

//mill是ms,micro是us,先将毫秒转为以毫秒计时的duration对象,再转化为以微妙计.
common::Duration FromMilliseconds(const int64 milliseconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
      //std::chrono::milliseconds 	
      //duration</*至少 45 位的有符号整数类型*/, std::milli>
}

}  // namespace common
}  // namespace cartographer
