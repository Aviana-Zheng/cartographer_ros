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

#include "cartographer/io/proto_stream.h"

// std::ostream 参考https://zhuanlan.zhihu.com/p/365950428

namespace cartographer {
namespace io {

namespace {

// First eight bytes to identify our proto stream format.
const size_t kMagic = 0x7b1d1f7b5bf501db;
/*
put函数使用例子如下:

#include <iostream>
#include <fstream>

using namespace std;

int main()
{
    filebuf buf;
    if ( buf.open("/proc/self/fd/1", ios::out) == nullptr )
    {
        cerr << "stdout open failed" << endl;
        return -1;
    }
    ostream out(&buf);
    char c = 'X';
    out.put('c').put('=').put(c).put('\n');
    return 0;
}
这里因为put函数返回的是ostream&类型，所以可以连着使用put函数，代码编译后执行结果如下：
c=X
*/

void WriteSizeAsLittleEndian(size_t size, std::ostream* out) {
  for (int i = 0; i != 8; ++i) {
    // ostream头文件中put函数:   往缓冲区中插入一个字符
    // put顺序：(char)db 01 f5 5b 7b 1f 1d 7b
    out->put(size & 0xff);  // 一个字节由8个二进制位组成的,一个char占1个字节,put一个char
    // 移位操作都是针对补码进行的，正数的原码、补码、反码相同，负数的原码取反得反码，反码加一得补码
    // >>是右移运算符, x >>= 1等价于 x = x>>1 
    size >>= 8;
  }
}

/*

 std::istream::get
get()含有三种重载模式

单字符：
single character:  int get();（如果到文件尾就返回EOF）

　　　　　　　　istream& get(char & c);

从流中读取一个字符，结果保存在引用c中，如果到文件尾就返回空字符。
如file.get(x);表示从文件中读取一个字符，并把读取的字符保存在x中。

c字符串：
c-string:   istream& get(char* s,streamsize n);　　　　

　　　　 istream& get(char* s,streamsize n,char delim);

从流中提取字符，并将其作为c字符串存储在s中，直到读入了n个字符或者遇到定界字符位置，
定界符为‘\n’或delim。

如ifstream &get(char *buf,int num,char delim='\n')；把字符读入由buf指向的数组，
直到读入了num个字符或遇到了delim指定的字符，如果没使用delim这个参数，将使用缺省值换行符'\n'。
eg:

// C++ code for basic_istream::get() 
#include <bits/stdc++.h> 
using namespace std; 
  
// Driver Code 
int main() 
{ 
    // Declare string stream 
    istringstream gfg("Computer"); 
  
    // Here we get C 
    char a = gfg.get(); 
    cout << "First character is:"
         << a << endl; 
  
    char b; 
    // Here we got o 
    gfg.get(b); 
    cout << "After reading:" << a 
         << " We got " << b << endl; 
  
    char c; 
    // Here we got m 
    gfg.get(c); 
    cout << "Now we got:"
         << c << endl; 
  
    return 0; 
}

输出：

First character is:C
After reading:C We got o
Now we got:m

*/
bool ReadSizeAsLittleEndian(std::istream* in, size_t* size) {
  *size = 0;
  for (int i = 0; i != 8; ++i) {
    *size >>= 8;
    // 一个字节由8个二进制位组成的,一个char占1个字节,get一个char
    // get顺序：(char)db 01 f5 5b 7b 1f 1d 7b, 所以需要相加再 >> 右移8位 变为kMagic
    *size += static_cast<size_t>(in->get()) << 56;
  }
  return !in->fail();
}

}  // namespace

ProtoStreamWriter::ProtoStreamWriter(const string& filename)
    : out_(filename, std::ios::out | std::ios::binary) {
  WriteSizeAsLittleEndian(kMagic, &out_);
}

ProtoStreamWriter::~ProtoStreamWriter() {}

/*
ostream的write函数原型如下：
#include <iostream>
#include <fstream>

using namespace std;

int main()
{
    filebuf buf;
    if ( buf.open("/proc/self/fd/1", ios::out) == nullptr )
    {
        cerr << "stdout open failed" << endl;
        return -1;
    }
    ostream out(&buf);
    if ( !out.good())
    {
        cerr << "stream buf state is bad" << endl;
        return -1;
    }
    out.write("aaa\n", 4);  
    return 0;
}

good函数是ostream继承于父类ios的一个成员函数，它用来检查流的状态是否正常，正常则返回true。

代码编译执行后结果如下：

[root@mylinux ~]# ./a.out 
aaa
[root@mylinux ~]#

*/
void ProtoStreamWriter::Write(const string& uncompressed_data) {
  string compressed_data;
  common::FastGzipString(uncompressed_data, &compressed_data);
  WriteSizeAsLittleEndian(compressed_data.size(), &out_);
  out_.write(compressed_data.data(), compressed_data.size());
}

bool ProtoStreamWriter::Close() {
  out_.close();
  return !out_.fail();
}

ProtoStreamReader::ProtoStreamReader(const string& filename)
    : in_(filename, std::ios::in | std::ios::binary) {
  size_t magic;
  if (!ReadSizeAsLittleEndian(&in_, &magic) || magic != kMagic) {
    in_.setstate(std::ios::failbit);
  }
}

ProtoStreamReader::~ProtoStreamReader() {}

bool ProtoStreamReader::Read(string* decompressed_data) {
  size_t compressed_size;
  if (!ReadSizeAsLittleEndian(&in_, &compressed_size)) {
    return false;
  }
  string compressed_data(compressed_size, '\0');
  if (!in_.read(&compressed_data.front(), compressed_size)) {
    return false;
  }
  common::FastGunzipString(compressed_data, decompressed_data);
  return true;
}

}  // namespace io
}  // namespace cartographer
