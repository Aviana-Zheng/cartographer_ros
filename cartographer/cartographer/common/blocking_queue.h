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

#ifndef CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
#define CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_

#include <cstddef>
#include <deque>
#include <memory>

#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "glog/logging.h"
/*
 * BlockingQueue是线程安全的阻塞队列,(生产者消费者模式)
 * 不可拷贝不可赋值
 * 构造函数初始化队列大小,kInfiniteQueueSize=0默认不限制容量。queue_size限制容量：通过条件变量做到.
 * Push()添加元素,容量不够时,阻塞等待
 * Pop()删除元素,没有元素时,阻塞等待
 * Peek()返回下一个应该弹出的元素
 * PushWithTimeout(),添加元素,若超时则返回false
 * PopWithTimeout(),删除元素，若超时则返回false
 *
 * */

namespace cartographer {
namespace common {

// A thread-safe blocking queue that is useful for producer/consumer patterns.
// 'T' must be movable.
template <typename T>
class BlockingQueue {
 public:
  // const并未区分出编译期常量和运行期常量
  // constexpr限定在了编译期常量
  static constexpr size_t kInfiniteQueueSize = 0;

  // Constructs a blocking queue with infinite queue size.
  BlockingQueue() : BlockingQueue(kInfiniteQueueSize) {}   //默认不限制容量

  BlockingQueue(const BlockingQueue&) = delete;
  BlockingQueue& operator=(const BlockingQueue&) = delete;

  // Constructs a blocking queue with a size of 'queue_size'.  限制容量
  // C++中的explicit关键字只能用于修饰只有一个参数的类构造函数, 
  // 它的作用是表明该构造函数是显示的, 而非隐式的, 
  // 跟它相对应的另一个关键字是implicit, 意思是隐藏的,
  // 类构造函数默认情况下即声明为implicit(隐式).
  // explicit关键字的作用就是防止类构造函数的隐式自动转换.
  explicit BlockingQueue(const size_t queue_size) : queue_size_(queue_size) {}

  // Pushes a value onto the queue. Blocks if the queue is full.
  void Push(T t) {
    MutexLocker lock(&mutex_);    // 加锁
    // 队列未满才能push
    lock.Await([this]() REQUIRES(mutex_) { return QueueNotFullCondition(); });
    // 移动而非拷贝
    deque_.push_back(std::move(t));
  }

  // Like push, but returns false if 'timeout' is reached.
  bool PushWithTimeout(T t, const common::Duration timeout) {
    // 最多等待timeout时间
    MutexLocker lock(&mutex_);
    if (!lock.AwaitWithTimeout(
            [this]() REQUIRES(mutex_) { return QueueNotFullCondition(); },
            timeout)) {
      return false;
    }
    deque_.push_back(std::move(t));
    return true;
  }

  // Pops the next value from the queue. Blocks until a value is available.
  T Pop() {
    // 加锁
    MutexLocker lock(&mutex_);
    lock.Await([this]() REQUIRES(mutex_) { return QueueNotEmptyCondition(); });
    // 队列不为空才能pop()
    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  // Like Pop, but can timeout. Returns nullptr in this case.
  T PopWithTimeout(const common::Duration timeout) {
    MutexLocker lock(&mutex_);
    if (!lock.AwaitWithTimeout(
            [this]() REQUIRES(mutex_) { return QueueNotEmptyCondition(); },
            timeout)) {
      return nullptr;
    }
    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  // Returns the next value in the queue or nullptr if the queue is empty.
  // Maintains ownership. This assumes a member function get() that returns
  // a pointer to the given type R.
  template <typename R>
  const R* Peek() {
    MutexLocker lock(&mutex_);
    if (deque_.empty()) {
      return nullptr;
    }
    return deque_.front().get();
  }

  // Returns the number of items currently in the queue.
  size_t Size() {
    MutexLocker lock(&mutex_);
    return deque_.size();
  }

 private:
  // Returns true iff the queue is not empty.
  bool QueueNotEmptyCondition() REQUIRES(mutex_) { return !deque_.empty(); }

  // Returns true iff the queue is not full.
  bool QueueNotFullCondition() REQUIRES(mutex_) {
    return queue_size_ == kInfiniteQueueSize || deque_.size() < queue_size_;
  }

  Mutex mutex_;
  const size_t queue_size_ GUARDED_BY(mutex_);
  std::deque<T> deque_ GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
