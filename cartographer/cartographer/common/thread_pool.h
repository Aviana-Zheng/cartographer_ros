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

#ifndef CARTOGRAPHER_COMMON_THREAD_POOL_H_
#define CARTOGRAPHER_COMMON_THREAD_POOL_H_

#include <deque>
#include <functional>
#include <thread>
#include <vector>

#include "cartographer/common/mutex.h"

namespace cartographer {
namespace common {

/*
ThreadPool 是对c++11 thread的封装.
ThreadPool是线程数量固定的线程池，不可拷贝 和复制.

1，构造函数ThreadPool(int num_threads) 初始化一个线程数量固定的线程池。
2，Schedule(std::function<void()> work_item)添加想要ThreadPool执行的函数,
   std::thread会在线程后台依次排队执行相关函数.
3，数据成员pool_是具体的线程，work_queue_是待执行的函数队列。
*/

// A fixed number of threads working on a work queue of work items. Adding a
// new work item does not block, and will be executed by a background thread
// eventually. The queue must be empty before calling the destructor. The thread
// pool will then wait for the currently executing work items to finish and then
// destroy the threads.
class ThreadPool {
 public:
  explicit ThreadPool(int num_threads);
  ~ThreadPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  void Schedule(std::function<void()> work_item);

 private:
  void DoWork();

  Mutex mutex_;   //互斥锁，保证线程安全
  //running_只是一个监视哨,只有线程池在running_状态时,才能往work_queue_加入函数.
  bool running_ GUARDED_BY(mutex_) = true;
  std::vector<std::thread> pool_ GUARDED_BY(mutex_);
  std::deque<std::function<void()>> work_queue_ GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_THREAD_POOL_H_
