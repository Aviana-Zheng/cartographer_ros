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

#include "cartographer/mapping/collated_trajectory_builder.h"

#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace

CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    sensor::Collator* const sensor_collator, const int trajectory_id,
    const std::unordered_set<string>& expected_sensor_ids,
    std::unique_ptr<GlobalTrajectoryBuilderInterface>
        wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      last_logging_time_(std::chrono::steady_clock::now()) {
        sensor_collator_->AddTrajectory(
          trajectory_id, expected_sensor_ids,
          [this](const string& sensor_id, std::unique_ptr<sensor::Data> data) {
          HandleCollatedSensorData(sensor_id, std::move(data));
          }
        );
}

CollatedTrajectoryBuilder::~CollatedTrajectoryBuilder() {}

const TrajectoryBuilder::PoseEstimate&
CollatedTrajectoryBuilder::pose_estimate() const {
  return wrapped_trajectory_builder_->pose_estimate();
}

void CollatedTrajectoryBuilder::AddSensorData(
    const string& sensor_id, std::unique_ptr<sensor::Data> data) {
  sensor_collator_->AddSensorData(trajectory_id_, sensor_id, std::move(data));
}

void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const string& sensor_id, std::unique_ptr<sensor::Data> data) {
  auto it = rate_timers_.find(sensor_id);
  if (it == rate_timers_.end()) {
    it = rate_timers_
             .emplace(
               // emplace map的特殊情况, 文末
               // forward_as_tuple:完美转发. (以tuple规则比较2者),tuple定义了<运算符
                 std::piecewise_construct, std::forward_as_tuple(sensor_id),
                 std::forward_as_tuple(
                     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
             .first;
  }
  it->second.Pulse(data->time);

  if (std::chrono::steady_clock::now() - last_logging_time_ >
      common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();
  }

  switch (data->type) {
    case sensor::Data::Type::kImu:
      wrapped_trajectory_builder_->AddImuData(data->time,
                                              data->imu.linear_acceleration,
                                              data->imu.angular_velocity);
      return;

    case sensor::Data::Type::kRangefinder:
      wrapped_trajectory_builder_->AddRangefinderData(
          data->time, data->rangefinder.origin, data->rangefinder.ranges);
      return;

    case sensor::Data::Type::kOdometer:
      wrapped_trajectory_builder_->AddOdometerData(data->time,
                                                   data->odometer_pose);
      return;
  }
  LOG(FATAL);
}

}  // namespace mapping
}  // namespace cartographer

/*
map 的特殊情况
map 类型的 emplace 处理比较特殊，因为和其他的容器不同，
map 的 emplace 函数把它接收到的所有的参数都转发给 pair 的构造函数。
对于一个 pair 来说，它既需要构造它的 key 又需要构造它的 value。
如果我们按照普通的 的语法使用变参模板，我们无法区分哪些参数用来构造 key, 哪些用来构造 value。 
比如下面的代码：

map<string, complex> scp;
scp.emplace(“hello”, 1, 2); // 无法区分哪个参数用来构造 key 哪些用来构造 value
// string s(“hello”, 1), complex cpx(2) ???
// string s(“hello”), complex cpx(1, 2) ???
所以我们需要一种方式既可以接受异构变长参数，又可以区分 key 和 value，
解决 方式是使用 C++11 中提供的 tuple。

pair<string, complex> scp(make_tuple(“hello”), make_tuple(1, 2));
然后这种方式是有问题的，因为这里有歧义，第一个 tuple 会被当成是 key，
第二 个tuple会被当成 value。最终的结果是类型不匹配而导致对象创建失败，
为了解决 这个问题，C++11 设计了 piecewise_construct_t 这个类型用于解决这种歧义，
它 是一个空类，存在的唯一目的就是解决这种歧义，
全局变量 std::piecewise_construct 就是该类型的一个变量。所以最终的解决方式如下：

pair<string, complex> scp(piecewise_construct, make_tuple(“hello”), make_tuple(1, 2));
当然因为 map 的 emplace 把参数原样转发给 pair 的构造，
所以你需要使用同样 的语法来完成 emplace 的调用，
当然你可以使用 forward_as_tuple 替代 make_tuple，
该函数会帮你构造一个 tuple 并转发给 pair 构造。

map<string, complex> scp;
scp.emplace(piecewise_construct,
forward_as_tuple(“hello”),
forward_as_tuple(1, 2));
所以对于 map 来说你虽然避免了临时变量的构造，但是你却需要构建两个 tuple 。 
这种 traedoff 是否值得需要代码编写者自己考虑，从方便性和代码优雅性上来说：

scp.insert({“world”, {1, 2}});
这种写法都要胜过前面这个 emplace 版本。所以个人认为对于临时变量构建代价不是 很大的对象
（比如基础类型）推荐使用 insert 而不是 emplace。
*/