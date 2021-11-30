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

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

/*
PlyWritingPointsProcessor是PointsProcessor的第五个子类(5).
PlyWritingPointsProcessor负责将点云point以PLY的格式写入磁盘.
*/

// Streams a PLY file to disk. The header is written in 'Flush'.
class PlyWritingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "write_ply";
  // 指定文件写入的工厂和流水线next指针。
  PlyWritingPointsProcessor(std::unique_ptr<FileWriter> file,
                            PointsProcessor* next);

  static std::unique_ptr<PlyWritingPointsProcessor> FromDictionary(
      FileWriterFactory file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~PlyWritingPointsProcessor() override {}

  PlyWritingPointsProcessor(const PlyWritingPointsProcessor&) = delete;
  PlyWritingPointsProcessor& operator=(const PlyWritingPointsProcessor&) =
      delete;

  //按照ply格式写点云信息 。然后直接流水线到下一PointsProcessor
  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  PointsProcessor* const next_;

  int64 num_points_;
  bool has_colors_;   //是否是RGB
  std::unique_ptr<FileWriter> file_;
};

}  // namespace io
}  // namespace cartographer
