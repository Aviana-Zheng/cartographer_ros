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

#include "cartographer/mapping/trajectory_connectivity.h"

#include <algorithm>
#include <unordered_set>

#include "cartographer/mapping/proto/trajectory_connectivity.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

TrajectoryConnectivity::TrajectoryConnectivity()
    : lock_(), forest_(), connection_map_() {}

void TrajectoryConnectivity::Add(const int trajectory_id) {
  common::MutexLocker locker(&lock_);
  forest_.emplace(trajectory_id, trajectory_id);
}

void TrajectoryConnectivity::Connect(const int trajectory_id_a,
                                     const int trajectory_id_b) {
  common::MutexLocker locker(&lock_);
  // union即为联合，它是一种特殊的类。通过关键字union进行定义
  Union(trajectory_id_a, trajectory_id_b);
  // minmax()函数是算法标头的库函数，用于查找最小和最大值，它接受两个值并返回一对最小和最大值，
  // 该对中的第一个元素包含最小值，并且该对中的第二个元素包含最大值。 
  auto sorted_pair = std::minmax(trajectory_id_a, trajectory_id_b);
  // sorted_pair 在connection_map_对应的值加1
  ++connection_map_[sorted_pair];
}

void TrajectoryConnectivity::Union(const int trajectory_id_a,
                                   const int trajectory_id_b) {
  forest_.emplace(trajectory_id_a, trajectory_id_a);
  forest_.emplace(trajectory_id_b, trajectory_id_b);
  const int representative_a = FindSet(trajectory_id_a);
  const int representative_b = FindSet(trajectory_id_b);
  forest_[representative_a] = representative_b;
}

int TrajectoryConnectivity::FindSet(const int trajectory_id) {
  auto it = forest_.find(trajectory_id);
  CHECK(it != forest_.end());
  if (it->first != it->second) {
    it->second = FindSet(it->second);
  }
  return it->second;
}

bool TrajectoryConnectivity::TransitivelyConnected(const int trajectory_id_a,
                                                   const int trajectory_id_b) {
  common::MutexLocker locker(&lock_);
  // 使用count，返回的是被查找元素的个数。如果有，返回1；
  // 否则，返回0。注意，map中不存在相同元素，所以返回值只能是1或0。
  if (forest_.count(trajectory_id_a) == 0 ||
      forest_.count(trajectory_id_b) == 0) {
    return false;
  }
  return FindSet(trajectory_id_a) == FindSet(trajectory_id_b);
}

std::vector<std::vector<int>> TrajectoryConnectivity::ConnectedComponents() {
  // Map from cluster exemplar -> growing cluster.
  std::unordered_map<int, std::vector<int>> map;
  common::MutexLocker locker(&lock_);
  for (const auto& trajectory_id_entry : forest_) {
    map[FindSet(trajectory_id_entry.first)].push_back(
        trajectory_id_entry.first);
  }

  std::vector<std::vector<int>> result;
  result.reserve(map.size());
  for (auto& pair : map) {
    result.emplace_back(std::move(pair.second));
  }
  return result;
}

int TrajectoryConnectivity::ConnectionCount(const int trajectory_id_a,
                                            const int trajectory_id_b) {
  common::MutexLocker locker(&lock_);
  const auto it =
      connection_map_.find(std::minmax(trajectory_id_a, trajectory_id_b));
  return it != connection_map_.end() ? it->second : 0;
}

proto::TrajectoryConnectivity ToProto(
    std::vector<std::vector<int>> connected_components) {
  proto::TrajectoryConnectivity proto;
  for (auto& connected_component : connected_components) {
    std::sort(connected_component.begin(), connected_component.end());
  }
  std::sort(connected_components.begin(), connected_components.end());
  for (const auto& connected_component : connected_components) {
    auto* proto_connected_component = proto.add_connected_component();
    for (const int trajectory_id : connected_component) {
      proto_connected_component->add_trajectory_id(trajectory_id);
    }
  }
  return proto;
}

proto::TrajectoryConnectivity::ConnectedComponent FindConnectedComponent(
    const proto::TrajectoryConnectivity& trajectory_connectivity,
    const int trajectory_id) {
  for (const auto& connected_component :
       trajectory_connectivity.connected_component()) {
    if (std::find(connected_component.trajectory_id().begin(),
                  connected_component.trajectory_id().end(),
                  trajectory_id) != connected_component.trajectory_id().end()) {
      return connected_component;
    }
  }

  proto::TrajectoryConnectivity::ConnectedComponent connected_component;
  connected_component.add_trajectory_id(trajectory_id);
  return connected_component;
}

}  // namespace mapping
}  // namespace cartographer

/*
https://blog.csdn.net/hou09tian/article/details/80816445
union即为联合，它是一种特殊的类。通过关键字union进行定义，一个union可以有多个数据成员。例如
union Token{
   char cval;
   int ival;
   double dval;
};

互斥赋值
在任意时刻，联合中只能有一个数据成员可以有值。当给联合中某个成员赋值之后，
该联合中的其它成员就变成未定义状态了。

Token token;
 
token.cval = 'a';
 
token.ival = 1;
 
token.dval = 2.5;

*/

/*
#include<string>
#include<cstring>
#include<iostream>
#include<queue>
#include<map>
#include<algorithm>
using namespace std;
int main(){
    map<string,int> test;
    test.insert(make_pair("test1",1));//test["test1"]=1
    test.insert(make_pair("test2",2));//test["test2"]=2
    map<string,int>::iterator it;
    it=test.find("test0");
    cout<<"test0 find:";
    if(it==test.end()){
        cout<<"test0 not found"<<endl;
    }
    else{
        cout<<it->second<<endl;
    }
    cout<<"test0 count:";
    cout<<test.count("test1")<<endl;
 
    cout<<"test1 find:";
    it=test.find("test1");
    if(it==test.end()){
        cout<<"test1 not found"<<endl;
    }
    else{
        cout<<it->second<<endl;
    }
    cout<<"test1 count:";
    cout<<test.count("test1")<<endl;
 
    cout<<"after inserting test1"<<endl;
    test.insert(make_pair("test1",2));
    cout<<"test1 find:";
    it=test.find("test1");
    if(it==test.end()){
        cout<<"test1 not found"<<endl;
    }
    else{
        cout<<it->second<<endl;
    }
    cout<<"test1 count:";
    cout<<test.count("test1")<<endl;
    return 0;
}
*/