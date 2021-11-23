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

#include "cartographer/common/blocking_queue.h"

#include <memory>
#include <thread>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(BlockingQueueTest, testPushPeekPop) {
  BlockingQueue<std::unique_ptr<int>> blocking_queue;   //元素是智能指针,不限容
  blocking_queue.Push(common::make_unique<int>(42));    //添加
  ASSERT_EQ(1, blocking_queue.Size());                  //size==1
  blocking_queue.Push(common::make_unique<int>(24));    //添加
  ASSERT_EQ(2, blocking_queue.Size());                  //size==2
  EXPECT_EQ(42, *blocking_queue.Peek<int>());           //顶元素==42
  ASSERT_EQ(2, blocking_queue.Size());                  //队列大小是2
  EXPECT_EQ(42, *blocking_queue.Pop());
  ASSERT_EQ(1, blocking_queue.Size());
  EXPECT_EQ(24, *blocking_queue.Pop());
  ASSERT_EQ(0, blocking_queue.Size());
  EXPECT_EQ(nullptr, blocking_queue.Peek<int>());
  ASSERT_EQ(0, blocking_queue.Size());
}

TEST(BlockingQueueTest, testPushPopSharedPtr) {
  BlockingQueue<std::shared_ptr<int>> blocking_queue;          //不限容
  blocking_queue.Push(std::make_shared<int>(42));
  blocking_queue.Push(std::make_shared<int>(24));
  EXPECT_EQ(42, *blocking_queue.Pop());
  EXPECT_EQ(24, *blocking_queue.Pop());
}

TEST(BlockingQueueTest, testPopWithTimeout) {
  BlockingQueue<std::unique_ptr<int>> blocking_queue;
  //没有插入,所以是nullptr
  EXPECT_EQ(nullptr,
            blocking_queue.PopWithTimeout(common::FromMilliseconds(150)));
}

TEST(BlockingQueueTest, testPushWithTimeout) {
  BlockingQueue<std::unique_ptr<int>> blocking_queue(1);   //限容
  //插入成功
  EXPECT_EQ(true,
            blocking_queue.PushWithTimeout(common::make_unique<int>(42),
                                           common::FromMilliseconds(150)));
  //失败,因为队列大小是1
  EXPECT_EQ(false,
            blocking_queue.PushWithTimeout(common::make_unique<int>(15),
                                           common::FromMilliseconds(150)));
  EXPECT_EQ(42, *blocking_queue.Pop());
  EXPECT_EQ(0, blocking_queue.Size());
}

TEST(BlockingQueueTest, testPushWithTimeoutInfinteQueue) {
  // 不限容量
  BlockingQueue<std::unique_ptr<int>> blocking_queue;
  EXPECT_EQ(true,
            blocking_queue.PushWithTimeout(common::make_unique<int>(42),
                                           common::FromMilliseconds(150)));
  EXPECT_EQ(true,
            blocking_queue.PushWithTimeout(common::make_unique<int>(45),
                                           common::FromMilliseconds(150)));
  EXPECT_EQ(42, *blocking_queue.Pop());
  EXPECT_EQ(45, *blocking_queue.Pop());
  EXPECT_EQ(0, blocking_queue.Size());
}

TEST(BlockingQueueTest, testBlockingPop) {
  BlockingQueue<std::unique_ptr<int>> blocking_queue;
  ASSERT_EQ(0, blocking_queue.Size());

  int pop = 0;
  // 多线程测试
  // template <class Fn, class... Args>   
  // explicit thread (Fn&& fn, Args&&... args);
  // 按照值传递 
  std::thread thread([&blocking_queue, &pop] { pop = *blocking_queue.Pop(); });

  std::this_thread::sleep_for(common::FromMilliseconds(100));
  blocking_queue.Push(common::make_unique<int>(42));
  //threads[i].joinable();//判断线程是否可以join 
  thread.join();    //主线程等待当前线程执行完成再退出 
  ASSERT_EQ(0, blocking_queue.Size());
  EXPECT_EQ(42, pop);
}

TEST(BlockingQueueTest, testBlockingPopWithTimeout) {
  BlockingQueue<std::unique_ptr<int>> blocking_queue;
  ASSERT_EQ(0, blocking_queue.Size());

  int pop = 0;
  // 多线程测试
  std::thread thread([&blocking_queue, &pop] {
    pop = *blocking_queue.PopWithTimeout(common::FromMilliseconds(2500));
  });

  std::this_thread::sleep_for(common::FromMilliseconds(100));
  blocking_queue.Push(common::make_unique<int>(42));
  thread.join();
  ASSERT_EQ(0, blocking_queue.Size());
  EXPECT_EQ(42, pop);
}

}  // namespace
}  // namespace common
}  // namespace cartographer
