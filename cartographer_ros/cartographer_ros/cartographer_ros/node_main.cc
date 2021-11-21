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

#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_ros {
namespace {

// std::tuple是类似pair的模板。每个pair的成员类型都不相同，但每个pair都恰好有两个成员。
// 不同std::tuple类型的成员类型也不相同，但一个std::tuple可以有任意数量的成员。
// 每个确定的std::tuple类型的成员数目是固定的，
// 但一个std::tuple类型的成员数目可以与另一个std::tuple类型不同。
// 可以将std::tuple看作一个”快速而随意”的数据结构
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions() {
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<string>{FLAGS_configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                         CreateTrajectoryOptions(&lua_parameter_dictionary));
}

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 1e6;
  // 设置tf侦听器
  // 一旦创建了侦听器，它就开始通过连接接收tf2转换，并对它们进行长达tf_buffer(如10秒)的缓冲
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  // std::tie会将变量的引用整合成一个tuple，从而实现批量赋值。
  std::tie(node_options, trajectory_options) = LoadOptions();

  Node node(node_options, &tf_buffer);
  node.StartTrajectoryWithDefaultTopics(trajectory_options);

  ::ros::spin();

  node.FinishAllTrajectories();
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  //::是域操作符，前面什么都不写代表的是全局函数，此函数不属于某个特定的类
  ::ros::init(argc, argv, "cartographer_node");
  // ros::NodeHandler构造函数执行会调用ros::start()
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  //调用ros::shutdown()函数来关闭节点,会终结所有开放的订阅，发布，服务，调用
  ::ros::shutdown();
}
