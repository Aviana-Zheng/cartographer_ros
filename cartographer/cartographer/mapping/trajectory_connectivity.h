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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_CONNECTIVITY_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_CONNECTIVITY_H_

#include <map>
#include <unordered_map>

#include "cartographer/common/mutex.h"
#include "cartographer/mapping/proto/trajectory_connectivity.pb.h"
#include "cartographer/mapping/submaps.h"

namespace cartographer {
namespace mapping {

// A class that tracks the connectivity structure between trajectories.
//
// Connectivity includes both the count ("How many times have I _directly_
// connected trajectories i and j?") and the transitive connectivity.
//
// This class is thread-safe.
/*

TrajectoryConnectivity用于解决不同轨迹线的连通性问题.

多条轨迹构成一颗森林，而相互联通的轨迹应该合并。

不可拷贝/赋值
包含3个数据成员
1,互斥锁lock_
2,forest_  不同轨迹线组成的森林
3,connection_map_ 连通图
成员函数
Add():添加一条轨迹线
Connect():将2条轨迹线联通
TransitivelyConnected():判断是否处于同一个连通域
ConnectionCount():返回直接联通的数量

the transitive connectivity:传递连通性

ConnectedComponents():由联通分量id组成的已联通分类组
*/
class TrajectoryConnectivity {
 public:
  TrajectoryConnectivity();

  TrajectoryConnectivity(const TrajectoryConnectivity&) = delete;
  TrajectoryConnectivity& operator=(const TrajectoryConnectivity&) = delete;

  // Add a trajectory which is initially connected to nothing.
  // 添加一条轨迹线，默认不连接到任何轨迹线
  void Add(int trajectory_id) EXCLUDES(lock_);

  // Connect two trajectories. If either trajectory is untracked, it will be
  // tracked. This function is invariant to the order of its arguments. Repeated
  // calls to Connect increment the connectivity count.
  // 将轨迹a和轨迹b联通
  void Connect(int trajectory_id_a, int trajectory_id_b) EXCLUDES(lock_);

  // Determines if two trajectories have been (transitively传递性) connected. If
  // either trajectory is not being tracked, returns false. This function is
  // invariant to the order of its arguments.
  // 判断是否处于同一个连通域
  bool TransitivelyConnected(int trajectory_id_a, int trajectory_id_b)
      EXCLUDES(lock_);

  // Return the number of _direct_ connections between 'trajectory_id_a' and
  // 'trajectory_id_b'. If either trajectory is not being tracked, returns 0.
  // This function is invariant to the order of its arguments.
  //返回直接联通的数量
  int ConnectionCount(int trajectory_id_a, int trajectory_id_b) EXCLUDES(lock_);

  // The trajectory IDs, grouped by connectivity.
  std::vector<std::vector<int>> ConnectedComponents() EXCLUDES(lock_);

 private:
  // Find the representative(代表) and compresses the path to it.
  int FindSet(int trajectory_id) REQUIRES(lock_);
  void Union(int trajectory_id_a, int trajectory_id_b) REQUIRES(lock_);

  common::Mutex lock_;  //互斥锁
  // Tracks transitive connectivity using a disjoint set forest, i.e. each
  // entry points towards the representative for the given trajectory.
  // 不同轨迹线组成的森林，即连通域问题。
  std::map<int, int> forest_ GUARDED_BY(lock_);
  // Tracks the number of direct connections between a pair of trajectories.
  //直接链接的轨迹线
  std::map<std::pair<int, int>, int> connection_map_ GUARDED_BY(lock_);
};

// Returns a proto encoding connected components.
// 编码已联通成分到proto文件
proto::TrajectoryConnectivity ToProto(
    std::vector<std::vector<int>> connected_components);

// Returns the connected component containing 'trajectory_index'.
// 返回连接到联通id的所有联通分量。
proto::TrajectoryConnectivity::ConnectedComponent FindConnectedComponent(
    const cartographer::mapping::proto::TrajectoryConnectivity&
        trajectory_connectivity,
    int trajectory_id);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_CONNECTIVITY_H_

/*
是在Clang Thread Safety Analysis（线程安全分析）中定义的属性，
Clang Thread Safety Analysis是C ++语言扩展，它警告代码中潜在的竞争条件。
分析是完全静态的（即编译时）；没有运行时开销。该分析仍在积极开发中，但已经足够成熟，
可以在工业环境中进行部署。它是由Google与CERT / SEI合作开发的，并广泛用于Google的内部代码库中。

GUARDED_BY
GUARDED_BY是数据成员的属性，该属性声明数据成员受给定功能保护。对数据的读操作需要共享访问，
而写操作则需要互斥访问。该 GUARDED_BY属性声明线程必须先锁定listener_list_mutex才能对
其进行读写listener_list，从而确保增量和减量操作是原子的

EXCLUDES
EXCLUDES是函数或方法的属性，该属性声明调用方不拥有给定的功能。该注释用于防止死锁。
许多互斥量实现都不是可重入的，因此，如果函数第二次获取互斥量，则可能发生死锁。
在上面代码中的EXCLUDES表示的意思是：调用listener_disconnect()函数的调用用不能
拥有listener_list_mutex锁。


...
static ListenerList& listener_list GUARDED_BY(listener_list_mutex) = *new ListenerList();
...
static void listener_disconnect(void* arg, atransport*) EXCLUDES(listener_list_mutex) {
...
}

*/