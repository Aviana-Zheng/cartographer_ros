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

#include "cartographer/io/file_writer.h"

namespace cartographer {
namespace io {

//out:File open for writing: the internal stream buffer supports output operations.
//binary:Operations are performed in binary mode rather than text.

StreamFileWriter::StreamFileWriter(const string& filename)
    : out_(filename, std::ios::out | std::ios::binary) {}   //打开,并以2进制方式写入

StreamFileWriter::~StreamFileWriter() {}

//写入len个char
bool StreamFileWriter::Write(const char* const data, const size_t len) {
  if (out_.bad()) {
    return false;
  }
  out_.write(data, len);
  return !out_.bad();
}

bool StreamFileWriter::Close() {    //关闭文件
  if (out_.bad()) {
    return false;
  }
  out_.close();
  return !out_.bad();
}

bool StreamFileWriter::WriteHeader(const char* const data, const size_t len) {
  if (out_.bad()) {
    return false;
  }
  out_.flush();    //清空buffer
  // 在最后阶段才写入head,但不是在文件最后写入head,而是out_.seekp(0); 偏移到0处
  out_.seekp(0);    //偏移量为0
  return Write(data, len);
}

}  // namespace io
}  // namespace cartographer
