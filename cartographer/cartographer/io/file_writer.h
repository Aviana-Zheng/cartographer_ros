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

#ifndef CARTOGRAPHER_IO_FILE_WRITER_H_
#define CARTOGRAPHER_IO_FILE_WRITER_H_

#include <fstream>
#include <functional>
#include <memory>

#include "cartographer/common/port.h"

namespace cartographer {
namespace io {
/*
FileWriter负责文件写入的虚基类,不可拷贝/赋值
没有数据成员.只提供一系列抽象接口.
1),WriteHeader(),写入文件head
2),Write(),写入数据
3),Close(),关闭文件
*/

// Simple abstraction for a file.
class FileWriter {
 public:
  FileWriter() {}
  FileWriter(const FileWriter&) = delete;
  FileWriter& operator=(const FileWriter&) = delete;

  virtual ~FileWriter() {}

  // Write 'data' to the beginning of the file. This is required to overwrite
  // fixed size headers which contain the number of points once we actually know
  // how many points there are.
  virtual bool WriteHeader(const char* data, size_t len) = 0;

  virtual bool Write(const char* data, size_t len) = 0;
  virtual bool Close() = 0;
};

// An Implementation of file using std::ofstream.
  /*
  StreamFileWriter 文件流写入类,继承自FileWriter
  数据成员只有一个std::ofstream out_负责将文件写入磁盘
  */
class StreamFileWriter : public FileWriter {
 public:
  ~StreamFileWriter() override;

  StreamFileWriter(const string& filename);

  bool Write(const char* data, size_t len) override;
  bool WriteHeader(const char* data, size_t len) override;    //从文件开始处写入,覆盖旧数据
  bool Close() override;

 private:
  std::ofstream out_;
};

/*工厂模式,
创建一个FileWriter对象,由智能指针管理生命期,
返回值是std::unique_ptr<FileWriter>,
函数参数是string.
*/
using FileWriterFactory =
    std::function<std::unique_ptr<FileWriter>(const string& filename)>;

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_FILE_WRITER_H_
