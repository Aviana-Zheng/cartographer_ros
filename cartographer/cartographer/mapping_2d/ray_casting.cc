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

#include "cartographer/mapping_2d/ray_casting.h"

namespace cartographer {
namespace mapping_2d {

namespace {

// 对于“范围数据”中的每条射线，在适当的单元格上调用“命中访问者”和“错过访问者”。 
// 在未命中之前处理命中。 
// Factor for subpixel accuracy of start and end point.
// 起点和终点的亚像素精度的因素 
constexpr int kSubpixelScale = 1000;

// We divide each pixel in kSubpixelScale x kSubpixelScale subpixels. 'begin'
// and 'end' are coordinates at subpixel precision. We compute all pixels in
// which some part of the line segment connecting 'begin' and 'end' lies.
// 我们将每个像素划分为 kSubpixelScale x kSubpixelScale 子像素。 'begin' 和 'end' 是亚像素精度的坐标。 
// 我们计算连接“开始”和“结束”的线段的某些部分所在的所有像素。 
void CastRay(const Eigen::Array2i& begin, const Eigen::Array2i& end,
             const std::function<void(const Eigen::Array2i&)>& visitor) {
  // For simplicity, we order 'begin' and 'end' by their x coordinate.
  // 为简单起见，我们按 x 坐标对“开始”和“结束”进行排序。 
  // 保证dx > 0，方便后面计算
  if (begin.x() > end.x()) {
    CastRay(end, begin, visitor);
    return;
  }

  CHECK_GE(begin.x(), 0);
  CHECK_GE(begin.y(), 0);
  CHECK_GE(end.y(), 0);

  // Special case: We have to draw a vertical line in full pixels, as 'begin'
  // and 'end' have the same full pixel x coordinate.
  // 处理当射线为一条竖线的情况
  if (begin.x() / kSubpixelScale == end.x() / kSubpixelScale) {
    Eigen::Array2i current(begin.x() / kSubpixelScale,
                           std::min(begin.y(), end.y()) / kSubpixelScale);
    const int end_y = std::max(begin.y(), end.y()) / kSubpixelScale;
    for (; current.y() <= end_y; ++current.y()) {
      visitor(current);
    }
    return;
  }

  // 已知起始点和终止点坐标（1000倍细分），求该两点连线经过的栅格？
  const int64 dx = end.x() - begin.x();
  const int64 dy = end.y() - begin.y();
  // denominator是用于Bresenham算法的避免了浮点运算的因子
  // 本质上是x轴扩大两倍，一个栅格的面积，即kSubpixelScale *（2*dx）
  const int64 denominator = 2 * kSubpixelScale * dx;

  // The current full pixel coordinates. We begin at 'begin'.
  Eigen::Array2i current = begin / kSubpixelScale;

  // To represent subpixel centers, we use a factor of 2 * 'kSubpixelScale' in
  // the denominator.
  // +-+-+-+ -- 1 = (2 * kSubpixelScale) / (2 * kSubpixelScale)
  // | | | |
  // +-+-+-+
  // | | | |
  // +-+-+-+ -- top edge of first subpixel = 2 / (2 * kSubpixelScale)
  // | | | | -- center of first subpixel = 1 / (2 * kSubpixelScale)
  // +-+-+-+ -- 0 = 0 / (2 * kSubpixelScale)

  // The center of the subpixel part of 'begin.y()' assuming the
  // 'denominator', i.e., sub_y / denominator is in (0, 1).
  // (begin.y() % kSubpixelScale) + 1/2，起始点中心距离所在栅格x边的距离
  // 乘以2 * dx，即矩形面积
  int64 sub_y = (2 * (begin.y() % kSubpixelScale) + 1) * dx;

  // The distance from the from 'begin' to the right pixel border, to be divided
  // by 2 * 'kSubpixelScale'.
  // 起始点中心距离所在栅格y边的距离，扩大了2倍
  const int first_pixel =
      2 * kSubpixelScale - 2 * (begin.x() % kSubpixelScale) - 1;
  // The same from the left pixel border to 'end'.
  // 终点中心距离所在栅格y边的距离，扩大了2倍
  const int last_pixel = 2 * (end.x() % kSubpixelScale) + 1;

  // The full pixel x coordinate of 'end'.
  const int end_x = std::max(begin.x(), end.x()) / kSubpixelScale;

  // Move from 'begin' to the next pixel border to the right.
  sub_y += dy * first_pixel;
  // 实际上是 B+C 与 kSubpixelScale进行对比
  if (dy > 0) {
    while (true) {
      visitor(current);
      // 取当前点上方的点
      while (sub_y > denominator) {
        sub_y -= denominator;
        ++current.y();
        visitor(current);
      }
      // 取当前点右方的点
      ++current.x();
      // 取当前点右上方的点
      if (sub_y == denominator) {
        sub_y -= denominator;
        ++current.y();
      }
      if (current.x() == end_x) {
        break;
      }
      // Move from one pixel border to the next.
      // 加了一个斜率单位的高度
      sub_y += dy * 2 * kSubpixelScale;
    }
    // Move from the pixel border on the right to 'end'.
    sub_y += dy * last_pixel;
    visitor(current);
    while (sub_y > denominator) {
      sub_y -= denominator;
      ++current.y();
      visitor(current);
    }
    CHECK_NE(sub_y, denominator);
    CHECK_EQ(current.y(), end.y() / kSubpixelScale);
    return;
  }

  // Same for lines non-ascending in y coordinates.
  while (true) {
    visitor(current);
    while (sub_y < 0) {
      sub_y += denominator;
      --current.y();
      visitor(current);
    }
    ++current.x();
    if (sub_y == 0) {
      sub_y += denominator;
      --current.y();
    }
    if (current.x() == end_x) {
      break;
    }
    sub_y += dy * 2 * kSubpixelScale;
  }
  sub_y += dy * last_pixel;
  visitor(current);
  while (sub_y < 0) {
    sub_y += denominator;
    --current.y();
    visitor(current);
  }
  CHECK_NE(sub_y, 0);
  CHECK_EQ(current.y(), end.y() / kSubpixelScale);
}

}  // namespace

// 传入雷达点，子图大小信息，hit和miss的更新函数指针
void CastRays(const sensor::RangeData& range_data, const MapLimits& limits,
              const std::function<void(const Eigen::Array2i&)>& hit_visitor,
              const std::function<void(const Eigen::Array2i&)>& miss_visitor) {
  // 这里把要更新的子图的分辨率再缩小1000倍，用于再精细化起始点和终止点的坐标
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  // 注意这里为更新子图重新构建一个信息结构，地图的limits.max()没有变化
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  // 获取雷达在map坐标系中的坐标作为雷达更新的起始点
  const Eigen::Array2i begin =
      superscaled_limits.GetXYIndexOfCellContainingPoint(range_data.origin.x(),
                                                         range_data.origin.y());

  // Compute and add the end points.
  // 更新所有hit点的概率值，直接调用hit_visitor即可
  std::vector<Eigen::Array2i> ends;
  ends.reserve(range_data.returns.size());
  for (const Eigen::Vector3f& hit : range_data.returns) {
    ends.push_back(
        superscaled_limits.GetXYIndexOfCellContainingPoint(hit.x(), hit.y()));
    hit_visitor(ends.back() / kSubpixelScale);
  }

  // Now add the misses.  更新miss射线部分的概率栅格
  for (const Eigen::Array2i& end : ends) {
    CastRay(begin, end, miss_visitor);
  }

  // Finally, compute and add empty rays based on misses in the scan.
  // 更新无反馈的雷达射线，借用miss模型
  for (const Eigen::Vector3f& missing_echo : range_data.misses) {
    CastRay(begin,
            superscaled_limits.GetXYIndexOfCellContainingPoint(
                missing_echo.x(), missing_echo.y()),
            miss_visitor);
  }
}

}  // namespace mapping_2d
}  // namespace cartographer
