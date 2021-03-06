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

#define sout(Xit)  {std::cout<<__LINE__<<" "<< Xit <<""<<std::endl;}

#include "cartographer/sensor/collator.h"

#include <array>
#include <memory>

#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

TEST(Collator, Ordering) {
  const std::array<string, 4> kSensorId = {
      {"horizontal_rangefinder", "vertical_rangefinder", "imu", "odometry"}};
  //构造6个传感器数据
  Data zero(common::FromUniversal(0), Data::Rangefinder{});
  Data first(common::FromUniversal(100), Data::Rangefinder{});
  Data second(common::FromUniversal(200), Data::Rangefinder{});
  Data third(common::FromUniversal(300), Data::Imu{});
  Data fourth(common::FromUniversal(400), Data::Rangefinder{});
  Data fifth(common::FromUniversal(500), Data::Rangefinder{});
  Data sixth(common::FromUniversal(600), transform::Rigid3d::Identity());

  //添加一个轨迹线0,多个传感器,并指定回调函数:所有的接收数据都存储在received中
  std::vector<std::pair<string, Data>> received;
  Collator collator;
  collator.AddTrajectory(
      0, std::unordered_set<string>(kSensorId.begin(), kSensorId.end()),
      [&received](const string& sensor_id, std::unique_ptr<Data> data) {
        received.push_back(std::make_pair(sensor_id, *data));
      });

  constexpr int kTrajectoryId = 0;

  // Establish a common start time.
  //轨迹0,传感器0产生一个数据,产生时间是0us
  collator.AddSensorData(kTrajectoryId, kSensorId[0],
                         common::make_unique<Data>(zero));
  collator.AddSensorData(kTrajectoryId, kSensorId[1],
                         common::make_unique<Data>(zero));
  collator.AddSensorData(kTrajectoryId, kSensorId[2],
                         common::make_unique<Data>(zero));
  collator.AddSensorData(kTrajectoryId, kSensorId[3],
                         common::make_unique<Data>(zero));
  for(auto i:received)
  {
      LOG(INFO) << i.second.time << std::endl;  //此处才开始有0   {0}
  }
  collator.AddSensorData(kTrajectoryId, kSensorId[0],
                         common::make_unique<Data>(first));
  for(auto i:received)
  {
      LOG(INFO) << i.second.time << std::endl;  //  {0, 0}
  }
  collator.AddSensorData(kTrajectoryId, kSensorId[3],
                         common::make_unique<Data>(sixth));
  for(auto i:received)
  {
      LOG(INFO) << i.second.time << std::endl;   //  {0,0}
  }
  collator.AddSensorData(kTrajectoryId, kSensorId[0],
                         common::make_unique<Data>(fourth));
  for(auto i:received)
  {
      LOG(INFO) << i.second.time << std::endl;  //{0， 0}
  }
  collator.AddSensorData(kTrajectoryId, kSensorId[1],
                         common::make_unique<Data>(second));
  for(auto i:received)
  {
      LOG(INFO) << i.second.time << std::endl;   //{0， 0}
  }
  collator.AddSensorData(kTrajectoryId, kSensorId[1],
                         common::make_unique<Data>(fifth));
  for(auto i:received)
  {
      LOG(INFO) << i.second.time << std::endl;   // {0， 0}
  }
  collator.AddSensorData(kTrajectoryId, kSensorId[2],
                         common::make_unique<Data>(third));
  // collator.cc line:45
  // 遍历时，expected_sensor_ids顺序不是按照
  // {"horizontal_rangefinder", "vertical_rangefinder", "imu", "odometry"}
  // 这样的顺序的，而是
  // {"horizontal_rangefinder", "imu", "vertical_rangefinder", "odometry"}
  // 所以，IMU队列无数据，不能用回调函数处理数据，必须等IMU队列有数据之后，才可以用回调函数处理数据
  for(auto i:received)
  {
      LOG(INFO) << i.second.time << std::endl;   // {0,0,0,0,100,200,300}
  }

/*
h:{0,100,400}
v:{0,200,500}
i:{0,300}
o:{0,600}
*/

  ASSERT_EQ(7, received.size());    //{0,0,0,0,100,200,300}
  EXPECT_EQ(100, common::ToUniversal(received[4].second.time));
  EXPECT_EQ(kSensorId[0], received[4].first);
  EXPECT_EQ(200, common::ToUniversal(received[5].second.time));
  EXPECT_EQ(kSensorId[1], received[5].first);
  EXPECT_EQ(300, common::ToUniversal(received[6].second.time));
  EXPECT_EQ(kSensorId[2], received[6].first);

  collator.Flush();    //刷新

  //10个数据,所有sensor采集的数据
  ASSERT_EQ(10, received.size());
  EXPECT_EQ(kSensorId[0], received[7].first);
  EXPECT_EQ(500, common::ToUniversal(received[8].second.time));
  EXPECT_EQ(kSensorId[1], received[8].first);
  EXPECT_EQ(600, common::ToUniversal(received[9].second.time));
  EXPECT_EQ(kSensorId[3], received[9].first);
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
