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

#ifndef CARTOGRAPHER_COMMON_MAKE_UNIQUE_H_
#define CARTOGRAPHER_COMMON_MAKE_UNIQUE_H_

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

namespace cartographer {
namespace common {
  /*
make_unique.h在不支持c++14的环境下实现 std::make_unique的功能
实现细节:完美转发和移动语义

*/

// Implementation of c++14's std::make_unique, taken from
// https://isocpp.org/files/papers/N3656.txt
template <class T>
struct _Unique_if {
  typedef std::unique_ptr<T> _Single_object;
};

template <class T>
struct _Unique_if<T[]> {
  typedef std::unique_ptr<T[]> _Unknown_bound;   //不支持数组(不定长)
};

template <class T, size_t N>
struct _Unique_if<T[N]> {
  typedef void _Known_bound;   //不支持数组(定长)
};

template <class T, class... Args>
typename _Unique_if<T>::_Single_object make_unique(Args&&... args) {
  // 完美转发参数  std::forward
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
/*
  // std::move和std::forward本质就是一个转换函数，
  // std::move执行到右值的无条件转换，
  // std::forward执行到右值的有条件转换，
  // 在参数都是右值时，二者就是等价的
完美转发:
完美转发实现了参数在传递过程中保持其值属性的功能，
即若是左值，则传递之后仍然是左值，若是右值，则传递之后仍然是右值。
*/

template <class T>
typename _Unique_if<T>::_Unknown_bound make_unique(size_t n) {
  typedef typename std::remove_extent<T>::type U;
  return std::unique_ptr<T>(new U[n]());
}
/*
std::remove_extent
 返回数组降低一个维度后的数据类型。不改变数据类型的限制属性(const, volatile, const volatile)
 一维数组降低到0维度；
 二维数组降低到一维数组；
 三维数组降低到二维数组；
*/

template <class T, class... Args>
//不能使用定长数组
typename _Unique_if<T>::_Known_bound make_unique(Args&&...) = delete;

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_MAKE_UNIQUE_H_
