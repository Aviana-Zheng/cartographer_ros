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

#include "cartographer/mapping_3d/scan_matching/rotational_scan_matcher.h"

#include <map>
#include <vector>

#include "cartographer/common/math.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

namespace {

constexpr float kMinDistance = 0.2f;
constexpr float kMaxDistance = 0.9f;
constexpr float kSliceHeight = 0.2f;

// 直方图权重的计算方法：
void AddValueToHistogram(float angle, const float value,
                         Eigen::VectorXf* histogram) {
  // Map the angle to [0, pi), i.e. a vector and its inverse are considered to
  // represent the same angle.
  while (angle > static_cast<float>(M_PI)) {
    angle -= static_cast<float>(M_PI);
  }
  while (angle < 0.f) {
    angle += static_cast<float>(M_PI);
  }
  const float zero_to_one = angle / static_cast<float>(M_PI);
  // 本来是四舍五入取整，减去0.5变成向下取整；注意是负数；
  // 总是一个小于1大于0的数；
  // 确保bucket一定在[0，size - 1]之间；
  const int bucket = common::Clamp<int>(
      common::RoundToInt(histogram->size() * zero_to_one - 0.5f), 0,
      histogram->size() - 1);
  // 将【0，pi】的范围分割成了一定离散范围；当前点和参考点的向量与当前点和中心的向量，越垂直权重越大
  // 直方图的理解：相当于将所有的点云分了size个类别，分类的标准为当前点和参考点之间与x轴的夹角，这样每个点都能划分到
  // 一个类别中，它在该类别中的占比为当前点和参考点的的连线与当前点与质心点之间连线的同向行，越垂直值越大，因为越平行
  // 说明可能是外点或对同一个点的不同测量
  (*histogram)(bucket) += value;
}

// 计算该slice中点云的均值；
Eigen::Vector3f ComputeCentroid(const sensor::PointCloud& slice) {
  CHECK(!slice.empty());
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  for (const Eigen::Vector3f& point : slice) {
    sum += point;
  }
  return sum / static_cast<float>(slice.size());
}

struct AngleValuePair {
  float angle;
  float value;
};

// 将每个slice中排好序的点映射到histogram直方图中
void AddPointCloudSliceToValueVector(
    const sensor::PointCloud& slice,
    std::vector<AngleValuePair>* value_vector) {
  if (slice.empty()) {
    return;
  }
  // We compute the angle of the ray from a point to the centroid of the whole
  // point cloud. If it is orthogonal(正交) to the angle we compute between points, we
  // will add the angle between points to the histogram with the maximum weight.
  // This is to reject, e.g., the angles observed on the ceiling and floor.
  // 又要计算每个slice的质心 c
  const Eigen::Vector3f centroid = ComputeCentroid(slice);
  // 设置第一个点为参考点 a，因为排过序，所以第一个点应为与x轴角度最小的点
  Eigen::Vector3f last_point = slice.front();
  for (const Eigen::Vector3f& point : slice) {
    // 连接当前点 b（参考点的下一个点）和参考点 a，并计算它与x轴的夹角，按照夹角映射到直方图；
    const Eigen::Vector2f delta = (point - last_point).head<2>();
    const Eigen::Vector2f direction = (point - centroid).head<2>();
    const float distance = delta.norm();
    // 第一个点，distance = 0，所以跳过；
    // 确保前后两个点之间，以及当前点和中心点之间的距离都大于阈值；
    if (distance < kMinDistance || direction.norm() < kMinDistance) {
      continue;
    }
    // 当当前点和参考点超出一定距离时，则重置参考点；所有点的都按照角度排过序；
    if (distance > kMaxDistance) {
      last_point = point;
      continue;
    }
    // 这是当前点和参考点之间的连线和x轴所成的夹角
    const float angle = common::atan2(delta);
    // 计算该点的权重，即ab与bc的垂直程度，直方图元素即为权重的累加值。
    // 如果两个向量垂直，dot=0，如果两个向量平行，dot=1，这表示的是当前点和参考点之间，以及当前点和中心点之间向量的点积值；
    // 所以，如果两个向量垂直，value值越大，两个向量越接平行，value值越小；
    const float value = std::max(
        0.f, 1.f - std::abs(delta.normalized().dot(direction.normalized())));
    // 按照angle将value值累加在histogram中；
    value_vector->push_back(AngleValuePair{angle, value});
  }
}

// A function to sort the points in each slice by angle around the centroid.
// This is because the returns from different rangefinders are interleaved in
// the data.
// 将每个slice中的点进行排序, 按照沿x轴的角度进行排序
// a. 计算该slice中点云的均值；
// b. 并求每个point与质心连线与x轴所成的角度；	
//​	c. 按照角度排序所有点。
sensor::PointCloud SortSlice(const sensor::PointCloud& slice) {
  struct SortableAnglePointPair {
    // 按照角度排序
    bool operator<(const SortableAnglePointPair& rhs) const {
      return angle < rhs.angle;
    }

    float angle;
    Eigen::Vector3f point;
  };
  // 计算该slice中点云的均值；
  const Eigen::Vector3f centroid = ComputeCentroid(slice);
  std::vector<SortableAnglePointPair> by_angle;
  by_angle.reserve(slice.size());
  // 求每个point与质心连线与x轴所成的角度；
  for (const Eigen::Vector3f& point : slice) {
    const Eigen::Vector2f delta = (point - centroid).head<2>();
    // 确保这个点距离中心点足够远
    if (delta.norm() < kMinDistance) {
      continue;
    }
    // atan2(delta)是一个在xoy*面内的角度值，具体的就是delta向量和x轴的角度；
    by_angle.push_back(SortableAnglePointPair{common::atan2(delta), point});
  }
  // 将该slice中的点按照距离中点的在xoy*面内的角度值排序；
  std::sort(by_angle.begin(), by_angle.end());
  // 然后把排序好的结果中的point取出；
  sensor::PointCloud result;
  for (const auto& pair : by_angle) {
    result.push_back(pair.point);
  }
  return result;
}

std::vector<AngleValuePair> GetValuesForHistogram(
    const sensor::PointCloud& point_cloud) {
  std::map<int, sensor::PointCloud> slices;
  // 将点云的z值放大5倍，然后四舍五入取整，将点云分割成为一些slices，每个slices中存放原始点集；
  for (const Eigen::Vector3f& point : point_cloud) {
    slices[common::RoundToInt(point.z() / kSliceHeight)].push_back(point);
  }
  std::vector<AngleValuePair> result;
  for (const auto& slice : slices) {
    AddPointCloudSliceToValueVector(SortSlice(slice.second), &result);
  }
  return result;
}

// 将直方图进行旋转
void AddValuesToHistogram(const std::vector<AngleValuePair>& value_vector,
                          const float rotation, Eigen::VectorXf* histogram) {
  for (const AngleValuePair& pair : value_vector) {
    AddValueToHistogram(pair.angle + rotation, pair.value, histogram);
  }
}

}  // namespace

RotationalScanMatcher::RotationalScanMatcher(
    const std::vector<mapping::TrajectoryNode>& nodes, const int histogram_size)
    : histogram_(Eigen::VectorXf::Zero(histogram_size)) {
  for (const mapping::TrajectoryNode& node : nodes) {
    AddValuesToHistogram(
        GetValuesForHistogram(sensor::TransformPointCloud(
            node.constant_data->range_data_3d.returns.Decompress(),
            node.pose.cast<float>())),
        0.f, &histogram_);
  }
}

std::vector<float> RotationalScanMatcher::Match(
    const sensor::PointCloud& point_cloud,
    const std::vector<float>& angles) const {   // angles属于（-theta， theta）范围；
  std::vector<float> result;
  result.reserve(angles.size());
  const std::vector<AngleValuePair> value_vector =
      GetValuesForHistogram(point_cloud);
  
  for (const float angle : angles) {
    Eigen::VectorXf scan_histogram = Eigen::VectorXf::Zero(histogram_.size());
    // 将直方图进行旋转
    AddValuesToHistogram(value_vector, angle, &scan_histogram);
    // 计算两个直方图的相似程度，余弦距离；
    result.push_back(MatchHistogram(scan_histogram));
  }
  return result;
}

// 计算两个直方图的余弦距离，即相似程度；
float RotationalScanMatcher::MatchHistogram(
    const Eigen::VectorXf& scan_histogram) const {
  // We compute the dot product of normalized histograms as a measure of
  // similarity.
  const float scan_histogram_norm = scan_histogram.norm();
  const float histogram_norm = histogram_.norm();
  const float normalization = scan_histogram_norm * histogram_norm;
  if (normalization < 1e-3f) {
    return 1.f;
  }
  return histogram_.dot(scan_histogram) / normalization;
}

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer
