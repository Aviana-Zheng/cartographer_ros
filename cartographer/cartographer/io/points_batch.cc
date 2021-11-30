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

#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

/*
先对to_remove做降序排列,然后按照to_remove依次移除某些point.

batch->points是vector,index 是要移除的vector元素的索引.

时间复杂度:o(m*n),m是to_remove的大小,n是to_remove[0]到batch的end()的距离.

注:erase()的复杂度是:
Linear on the number of elements erased (destructions)*
the number of elements after the last element deleted (moving).
返回值是一个迭代器，指向删除元素下一个元素;
使用erase()要注意迭代器失效的问题.

使用降序排序,即从end开始erase()可避免失效,
*/

void RemovePoints(std::vector<int> to_remove, PointsBatch* batch) {
  std::sort(to_remove.begin(), to_remove.end(), std::greater<int>());  //降序排列
  for (const int index : to_remove) {
    batch->points.erase(batch->points.begin() + index);  //移除point
    if (!batch->colors.empty()) {
      batch->colors.erase(batch->colors.begin() + index);   //point对应的rgb
    }
    if (!batch->intensities.empty()) {     //移除光强度
      batch->intensities.erase(batch->intensities.begin() + index);
    }
  }
}

}  // namespace io
}  // namespace cartographer
