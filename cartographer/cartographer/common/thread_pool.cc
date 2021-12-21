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

#include "cartographer/common/thread_pool.h"

#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <numeric>

#include "glog/logging.h"

namespace cartographer {
namespace common {
  /*
pool_是vector,每个线程初始化时,执行DoWork()函数.

*/

ThreadPool::ThreadPool(int num_threads) {
  MutexLocker locker(&mutex_);
  for (int i = 0; i != num_threads; ++i) {
    // emplace_back函数的作用是减少对象拷贝和构造次数，
    // 是C++11中的新特性，主要适用于对临时对象的赋值。
    // 如果A函数调用了B函数，B函数排除了某个能力，A函数就不能递归排除该功能
    pool_.emplace_back([this]() { ThreadPool::DoWork(); });
  }
}

/*
只有线程池的work_queue_=0时,才能析构,此时所有的线程已经开始被pool执行,
只有等待pool结束所有的线程执行函数(join结束),ThreadPool才能析构完成
*/
ThreadPool::~ThreadPool() {
  { // locker的作用域在{}，出了作用域则自动析构，唤醒所有线程
    MutexLocker locker(&mutex_);
    CHECK(running_);
    running_ = false;
    CHECK_EQ(work_queue_.size(), 0);
  }
  for (std::thread& thread : pool_) {
    thread.join();  // 等待所有线程执行完毕
  }
}

/*
使想要线程池执行的work_item函数加入到等待队列中,然后排队等待空闲线程"领取"该函数.
*/
void ThreadPool::Schedule(std::function<void()> work_item) {
  MutexLocker locker(&mutex_);
  CHECK(running_);
  work_queue_.push_back(work_item);
}

/*
每个线程都要执行这个函数.
ThreadPool利用条件变量通知此函数,work_queue_不为零时,说明有需要执行的函数,此时加入执行
*/
void ThreadPool::DoWork() {
#ifdef __linux__
  // This changes the per-thread nice level of the current thread on Linux. We
  // do this so that the background work done by the thread pool is not taking
  // away CPU resources from more important foreground threads.
  CHECK_NE(nice(10), -1);
#endif
  for (;;) {
    std::function<void()> work_item;
    {
      // 加锁,同一时刻只能一个线程"领取"某个函数
      MutexLocker locker(&mutex_);
      // 在work_queue_不为空或者running状态为false的前提下，才可以唤醒该线程
      locker.Await([this]() REQUIRES(mutex_) {
        return !work_queue_.empty() || !running_;
        //注意,队列不为空或者不在running_状态,条件变量都需要通知该函数
      });
      if (!work_queue_.empty()) {
        work_item = work_queue_.front();    //领取
        work_queue_.pop_front();      //删除
      } else if (!running_) {
        return;
      }
    }
    // 抢到的线程开始执行work_item
    CHECK(work_item);
    work_item();
  }
}

}  // namespace common
}  // namespace cartographer

/*
 * 在C/C++中大括号指明了变量的作用域，
 * 在大括号内声明的局部变量其作用域自变量声明开始，到大括号之后终结。
 * { } 里的内容是一个“块”，单独的{ }在执行顺序上没有改变，仍然是顺序执行
eg:
void MyProcess(MyType input, MyType &output)
 
{
   MyType filter = input;
 
   {
      MyType temp;
      step1(filter,temp);
   }
 
 
   {
      MyType temp;
      step2(filter,temp);
   }
 
 
   {
      MyType temp;
      step3(filter,temp);
   }
 
   output = filter;
}
 

 以上程序实现了简单的管道/过滤器结构：

          temp1    temp2      temp3
           ↓         ↓          ↓ 
input --> step1 --> step2 --> step3 --> output

temp都是临时变量，
如果没有大括号的约束，每个临时变量都存在于函数作用域中，那么频繁增减流程时出错的概率大大增加。
放在大括号中，不仅程序阅读起来很清楚，而且也不容易出错

 */