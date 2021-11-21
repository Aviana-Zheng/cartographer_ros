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

#include "cartographer_ros/ros_log_sink.h"

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "glog/log_severity.h"
#include "ros/console.h"

namespace cartographer_ros {
//对头文件中重载的两个函数send和WaitTillSent进行了实现，
//首先定义了一个GetBasename用于在给定文件路径的情况下获取文件的基础文件名。

namespace {
/**
 * @brief 根据给定的文件全路径名, 获取文件名
 * 
 * @param[in] filepath 
 * @return const char* 返回文件名
 */

const char* GetBasename(const char* filepath) {
  // 找到 '/' 最后一次在filepath中出现的位置
  // strchr这个函数主要是为了寻找一个字符ch在给定字符串str中最后一次出现的位置，
  // 如果找到，则返回指向ch的指针，如果找不到则返 回NULL
  const char* base = std::strrchr(filepath, '/');
  // 找到'/',就将'/'之后的字符串返回；找不到'/', 就将整个filepath返回
  return base ? (base + 1) : filepath;
}

}  // namespace

/**
 * @brief 在构造函数中调用AddLogSink(), 将ScopedRosLogSink类注册到glog中
 */
/* 这里的AddLogSink和RemoveLogSink是glog中实现的函数，
主要的功能是添加或移除一个LogSink作为日志数据的使用者。 
同时这个函数是线程安全的。直观理解就是将日志数据转发给ScopedRosLogSink类。 */
ScopedRosLogSink::ScopedRosLogSink() : will_die_(false) { AddLogSink(this); }
ScopedRosLogSink::~ScopedRosLogSink() { RemoveLogSink(this); }

// 这个类要重载的核心函数send
// 这个函数的主要目的就是将传入的日志数据
// 整合成一个普通字符串并根据日志的严重性等级调用对应的ROS日志宏进行输出。
/**
 * @brief 重载了send()方法, 使用ROS_INFO进行glog消息的输出
 * 
 * @param[in] severity 消息级别
 * @param[in] filename 全路径文件名
 * @param[in] base_filename 文件名
 * @param[in] line 消息所在的文件行数
 * @param[in] tm_time 消息的时间
 * @param[in] message 消息数据本体
 * @param[in] message_len 消息长度
 */
void ScopedRosLogSink::send(const ::google::LogSeverity severity,
                            const char* const filename,
                            const char* const base_filename, const int line,
                            const struct std::tm* const tm_time,
                            const char* const message,
                            const size_t message_len) {
  // 这里的::google::LogSink::ToString函数主要是将输入的glog日志数据整合成为一个
  // 普通的文本字符串消息用于输出
  const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, tm_time, message, message_len);
  switch (severity) {
    case ::google::GLOG_INFO:
      ROS_INFO_STREAM(message_string);
      break;

    case ::google::GLOG_WARNING:
      ROS_WARN_STREAM(message_string);
      break;

    case ::google::GLOG_ERROR:
      ROS_ERROR_STREAM(message_string);
      break;

    case ::google::GLOG_FATAL:
      ROS_FATAL_STREAM(message_string);
      will_die_ = true;
      break;
  }
}

// 重载基类的函数WaitTillSent
// WaitTillSent()会在每次send后调用, 用于一些异步写的场景
// 重新定义这个来实现等待接收器的日志逻辑来完成。
// 它将在每次 send() 返回后调用，但在 LogMessage 退出或崩溃之前。
// 默认情况下，此函数不执行任何操作。
// 配合这个说明以及只有在FATAL级别的消息的情况情况下才将will_die_设置为true以执行休眠，
// 我们可以推断出这个函数的主要目的是在接收到FATAL级别的消息后等候一段时间，
// 以便ROS消息可以在程序崩溃之前发送出去。
void ScopedRosLogSink::WaitTillSent() {
  if (will_die_) {
    // Give ROS some time to actually publish our message.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

}  // namespace cartographer_ros
