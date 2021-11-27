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

#include "cartographer/sensor/collator.h"

namespace cartographer {
namespace sensor {

// 一个轨迹线,多个传感器
/*
struct QueueKey {
  int trajectory_id;// 轨线id;
  string sensor_id; //传感器id
  }
*/
void Collator::AddTrajectory(
    const int trajectory_id,
    // unordered_set是基于哈希表
    // 哈希表是根据关键码值而进行直接访问的数据结构
    // 无序集合容器（unordered_set）是一个存储唯一(unique，即无重复）
    // 的关联容器（Associative container），容器中的元素无特别的秩序关系，
    // 该容器允许基于值的快速元素检索，同时也支持正向迭代。
    // 在一个unordered_set内部，元素不会按任何顺序排序，
    // 而是通过元素值的hash值将元素分组放置到各个槽(Bucker，也可以译为“桶”），
    // 这样就能通过元素值快速访问各个对应的元素（均摊耗时为O（1））
    const std::unordered_set<string>& expected_sensor_ids,
    const Callback callback) {
  // 遍历时，expected_sensor_ids顺序不是按照
  // {"horizontal_rangefinder", "vertical_rangefinder", "imu", "odometry"}
  // 这样的顺序的，而是
  // {"horizontal_rangefinder", "imu", "vertical_rangefinder", "odometry"}
  for (const auto& sensor_id : expected_sensor_ids) {
    //对于每一个轨迹线+传感器,设置一个key
    const auto queue_key = QueueKey{trajectory_id, sensor_id};
    //添加一个名为key的队列,并设置回调函数处理data
    queue_.AddQueue(queue_key,
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });
    //map<int,vector<key>>:添加轨迹线对应的key
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}

//队列不再接收数据
void Collator::FinishTrajectory(const int trajectory_id) {
  for (const auto& queue_key : queue_keys_[trajectory_id]) {
    queue_.MarkQueueAsFinished(queue_key);
  }
}

//主要的操作,添加传感器数据,数据形式是:key+data
void Collator::AddSensorData(const int trajectory_id, const string& sensor_id,
                             std::unique_ptr<Data> data) {
  //找到key，再move(data)
  queue_.Add(QueueKey{trajectory_id, sensor_id}, std::move(data));
}

void Collator::Flush() { queue_.Flush(); }

int Collator::GetBlockingTrajectoryId() const {
  return queue_.GetBlocker().trajectory_id;
}

}  // namespace sensor
}  // namespace cartographer
