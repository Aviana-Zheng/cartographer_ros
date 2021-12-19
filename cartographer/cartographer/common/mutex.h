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

common/mutex.h主要是对c++11 的mutex的封装。

*/

#ifndef CARTOGRAPHER_COMMON_MUTEX_H_
#define CARTOGRAPHER_COMMON_MUTEX_H_

#include <condition_variable>
#include <mutex>

#include "cartographer/common/time.h"

namespace cartographer {
namespace common {


/*不使用clang时,以下宏定义全部为空操作.故看见宏定义时可暂时忽略*/
// Enable thread safety attributes only with clang.
// The attributes can be safely erased when compiling with other compilers.
#if defined(__SUPPORT_TS_ANNOTATION__) || defined(__clang__)
#define THREAD_ANNOTATION_ATTRIBUTE__(x) __attribute__((x))
#else
#define THREAD_ANNOTATION_ATTRIBUTE__(x)  // no-op,空操作
#endif

#define CAPABILITY(x) THREAD_ANNOTATION_ATTRIBUTE__(capability(x))

#define SCOPED_CAPABILITY THREAD_ANNOTATION_ATTRIBUTE__(scoped_lockable)

#define GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(guarded_by(x))

#define PT_GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(pt_guarded_by(x))

#define REQUIRES(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(requires_capability(__VA_ARGS__))

#define ACQUIRE(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(acquire_capability(__VA_ARGS__))

#define RELEASE(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(release_capability(__VA_ARGS__))

#define EXCLUDES(...) THREAD_ANNOTATION_ATTRIBUTE__(locks_excluded(__VA_ARGS__))

#define NO_THREAD_SAFETY_ANALYSIS \
  THREAD_ANNOTATION_ATTRIBUTE__(no_thread_safety_analysis)

/*
Mutex是c++11 mutex的封装，
Mutex类有一个内部类Locker。

Locker 是一个RAII的类型.
在构造Locker时,对mutex上锁,在析构Locker时对mutex解锁.
本质是使用std::unique_lock和std::condition_variable实现的

Locker类提供2个成员函数Await()和AwaitWithTimeout()
功能是利用c++11的条件变量和unique_lock实现在谓词predicate为真的情况下对mutex解锁。
*/


// Defines an annotated mutex that can only be locked through its scoped locker
// implementation.
// 没有传递参数给`ACQUIRE`或`RELEASE`，则假定参数为`this`  ACQUIRE(mutex)  RELEASE()
// 需配合 CAPABILITY 使用，将`Container`转为能力
class CAPABILITY("mutex") Mutex {
 public:
  // A RAII class that acquires a mutex in its constructor, and
  // releases it in its destructor. It also implements waiting functionality on
  // conditions that get checked whenever the mutex is released.
  class SCOPED_CAPABILITY Locker {
   public:
    // ACQUIRE(mutex)能力解释：能力为mutex，在进入Locker后持有，退出后持有
    Locker(Mutex* mutex) ACQUIRE(mutex) : mutex_(mutex), lock_(mutex->mutex_) {}
    
    // RELEASE() 调用函数或方法时，释放mutex,即函数进入前已持有，退出前释放
    ~Locker() RELEASE() {
      lock_.unlock();    //解锁
      mutex_->condition_.notify_all();     //条件变量通知解锁
    }

    template <typename Predicate>
    // REQUIRES(this)能力解释：能力为this,在函数进入前持有，退出时后必须持有
    void Await(Predicate predicate) REQUIRES(this) {
      /*wait()实质是分3步：
      1.对lock_解锁
      2.等待predicate谓词为真，此时调用端阻塞。
      3.对lock_ 重新上锁。
      */
      mutex_->condition_.wait(lock_, predicate);
    }

    template <typename Predicate>
    // REQUIRES(this)能力解释：能力为this,在函数进入前持有，退出时后必须持有
    bool AwaitWithTimeout(Predicate predicate, common::Duration timeout)
        REQUIRES(this) {
          //等待谓词为真或者直到超时timeout返回。
      return mutex_->condition_.wait_for(lock_, timeout, predicate);
    }

   private:
    Mutex* mutex_;
    // 方便线程对互斥量上锁，但提供了更好的上锁和解锁控制 提供了更好的上锁和解锁控制
    std::unique_lock<std::mutex> lock_;
  };

 private:
  // https://www.cnblogs.com/haippy/p/3252041.html
  // C++11 并发指南五(std::condition_variable 详解)
  std::condition_variable condition_;
  std::mutex mutex_;
};

using MutexLocker = Mutex::Locker;

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_MUTEX_H_
