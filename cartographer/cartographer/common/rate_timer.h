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

#ifndef CARTOGRAPHER_COMMON_RATE_TIMER_H_
#define CARTOGRAPHER_COMMON_RATE_TIMER_H_

#include <chrono>
#include <deque>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"

/*
 * RateTimer是,脉冲频率计数类,计算在一段时间内的脉冲率
 * RateTimer不可拷贝和赋值
 * RateTimer只有一个构造函数,提供时间段Duration
 * ComputeRate()返回事件脉冲率,单位hz
 * ComputeWallTimeRateRatio()返回真实时间与墙上挂钟时间的比率
 * 内部类Event封装了某个事件发生的时间点.
 * 调用一次Pulse()即产生了一次事件
 * */
namespace cartographer {
namespace common {

// Computes the rate at which pulses come in.默认模板参数是steady_clock
// system_clock：用在需要得到绝对时点的场景
// steady_clock：用在需要得到时间间隔，并且这个时间间隔不会因为修改系统时间而受影响的场景
template <typename ClockType = std::chrono::steady_clock>
class RateTimer {
 public:
  // Computes the rate at which pulses come in over 'window_duration' in wall
  // time.
  explicit RateTimer(const common::Duration window_duration)
      : window_duration_(window_duration) {}
  ~RateTimer() {}

  RateTimer(const RateTimer&) = delete;  //不可拷贝
  RateTimer& operator=(const RateTimer&) = delete;   //不可赋值

  // Returns the pulse rate in Hz.
  double ComputeRate() const {
    //计算频率
    if (events_.empty()) {
      return 0.;
    }
    //事件次数除以时间即为每秒钟发生多少次事件
    return static_cast<double>(events_.size() - 1) /
           common::ToSeconds((events_.back().time - events_.front().time));
           //最晚发生的时间-最早发生的时间 (事件产生时的真实时间)
  }

  // Returns the ratio of the pulse rate (with supplied times) to the wall time
  // rate. For example, if a sensor produces pulses at 10 Hz, but we call Pulse
  // at 20 Hz wall time, this will return 2.
  double ComputeWallTimeRateRatio() const {
    //返回比率
    if (events_.empty()) {
      return 0.;
    }
    //真实时间   events_.back().time - events_.front().time
    //墙上挂钟时间,->调用Pulse时的系统的时间
    return common::ToSeconds((events_.back().time - events_.front().time)) /
           std::chrono::duration_cast<std::chrono::duration<double>>(
               events_.back().wall_time - events_.front().wall_time)
               .count();
  }

  // Records an event that will contribute to the computed rate.
  void Pulse(common::Time time) {
    //产生一个脉冲
    events_.push_back(Event{time, ClockType::now()});
    while (events_.size() > 2 &&
           (events_.back().wall_time - events_.front().wall_time) >
               window_duration_) {
      events_.pop_front();
    }
  }

  // Returns a debug string representation.
  string DebugString() const {
    if (events_.size() < 2) {
      return "unknown";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << ComputeRate() << " Hz "
        << DeltasDebugString() << " (pulsed at "
        << ComputeWallTimeRateRatio() * 100. << "% real time)";
    return out.str();
  }

 private:
  struct Event {
    common::Time time;
    typename ClockType::time_point wall_time;
  };

  // Computes all differences in seconds between consecutive pulses.
  std::vector<double> ComputeDeltasInSeconds() const {
    CHECK_GT(events_.size(), 1);
    const size_t count = events_.size() - 1;
    std::vector<double> result;
    // vector中不断的push_back，会进行内存的重新自动分配的问题
    // 为了避免重新分配内存带来的问题，vector提供了reserve函数
    // reserve的作用是更改vector的容量（capacity），使vector至少可以容纳n个元素。
    // 如果n大于vector当前的容量，reserve会对vector进行扩容。
    // 其他情况下都不会重新分配vector的存储空间
    result.reserve(count);
    for (size_t i = 0; i != count; ++i) {
      result.push_back(
          common::ToSeconds(events_[i + 1].time - events_[i].time));
    }
    return result;
  }

  // Returns the average and standard deviation of the deltas.
  string DeltasDebugString() const {
    const auto deltas = ComputeDeltasInSeconds();
    /*
    累计范围内的值,  文末
    accumulate (InputIterator first, InputIterator last, T init, BinaryOperation binary_op);
    参数:
    first,last:将迭代器输入到序列中的初始位置和最终位置。使用的范围是[first,last)，
    它包含所有的元件第一和最后一个，包括由指向的元件第一但不被指向的元素最后。

    init:累加器的初始值。

    binary_op
    myfunction (int x, int y)；这样的函数时，init传入x，前面的范围和传入y，最后返回函数值。
    std::minus()；返回init-范围和
    int operator()(int x, int y)；和函数那个一样效果。

    返回值
    累积init：和范围内所有元素的结果[first,last)。
    */
    const double sum = std::accumulate(deltas.begin(), deltas.end(), 0.);
    const double mean = sum / deltas.size();

    double squared_sum = 0.;
    for (const double x : deltas) {
      squared_sum += common::Pow2(x - mean);
    }
    const double sigma = std::sqrt(squared_sum / (deltas.size() - 1));

    std::ostringstream out;
    out << std::scientific << std::setprecision(2) << mean << " s +/- " << sigma
        << " s";
    return out.str();
  }

  std::deque<Event> events_;
  const common::Duration window_duration_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_RATE_TIMER_H_


/*

std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

std::chrono是c++11引入的日期时间处理库，其中包含3种时钟：
system_clock， steady_clock，  high_resolution_clock。

system_clock：
对于system_clock，其起点是epoch，即1970-01-01 00:00:00 UTC，
其刻度是1个tick，也就是_XTIME_NSECS_PER_TICK纳秒。
用在需要得到绝对时点的场景

steady_clock：
steady_clock的刻度是1纳秒，起点并非1970-01-01 00:00:00 UTC，一般是系统启动时间，
这就是问题的关键。steady_clock的作用是为了得到不随系统时间修改而变化的时间间隔，
所以凡是想得到绝对时点的用法都是错误的。
steady_clock是没有to_time_t()的实现的，而system_clock是有的。
用在需要得到时间间隔，并且这个时间间隔不会因为修改系统时间而受影响的场景

high_resolution_clock：
精度是纳秒
是system_clock或steady_clock之一，根据情况使用

std::chrono::time_point   表示一个具体时间，如上个世纪80年代、
你的生日、今天下午、火车出发时间等等
*/

/*
#include <iostream>     // std::cout
#include <functional>   // std::minus
#include <numeric>      // std::accumulate

int myfunction (int x, int y) {return x+2*y;}
struct myclass {
    int operator()(int x, int y) {return x+3*y;}
} myobject;

int main () {
  int init = 100;
  int numbers[] = {10,20,30};

  std::cout << "using default accumulate: ";
  std::cout << std::accumulate(numbers,numbers+3,init);
  std::cout << '\n';

  std::cout << "using functional's minus: ";
  std::cout << std::accumulate (numbers, numbers+3, init, std::minus<int>());
  std::cout << '\n';

  std::cout << "using custom function: ";
  std::cout << std::accumulate (numbers, numbers+3, init, myfunction);
  std::cout << '\n';

  std::cout << "using custom object: ";
  std::cout << std::accumulate (numbers, numbers+3, init, myobject);
  std::cout << '\n';

  return 0;
}
*/