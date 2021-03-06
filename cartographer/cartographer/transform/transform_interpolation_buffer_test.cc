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

#include "cartographer/transform/transform_interpolation_buffer.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace transform {
namespace {

TEST(TransformInterpolationBufferTest, testHas) {
  TransformInterpolationBuffer buffer;
  EXPECT_FALSE(buffer.Has(common::FromUniversal(50)));   //false 50us
  buffer.Push(common::FromUniversal(50), transform::Rigid3d::Identity());
  EXPECT_FALSE(buffer.Has(common::FromUniversal(25)));     //false，只有1个元素。
  EXPECT_TRUE(buffer.Has(common::FromUniversal(50)));    //true，1个元素
  EXPECT_FALSE(buffer.Has(common::FromUniversal(75)));   //false
  buffer.Push(common::FromUniversal(100), transform::Rigid3d::Identity());
  EXPECT_FALSE(buffer.Has(common::FromUniversal(25)));    //false，50-100才是true
  EXPECT_TRUE(buffer.Has(common::FromUniversal(50)));    
  EXPECT_TRUE(buffer.Has(common::FromUniversal(75)));   //在50-100之间
  EXPECT_TRUE(buffer.Has(common::FromUniversal(100)));    
  EXPECT_FALSE(buffer.Has(common::FromUniversal(125)));   //不在
  EXPECT_EQ(common::FromUniversal(50), buffer.earliest_time());
  EXPECT_EQ(common::FromUniversal(100), buffer.latest_time());
}

TEST(TransformInterpolationBufferTest, testLookup) {
  TransformInterpolationBuffer buffer;
  buffer.Push(common::FromUniversal(50), transform::Rigid3d::Identity());
  // The rotation needs to be relatively small in order for the interpolation to
  // remain a z-axis rotation.
  buffer.Push(common::FromUniversal(100),
              transform::Rigid3d::Translation(Eigen::Vector3d(10., 10., 10.)) *
                  transform::Rigid3d::Rotation(
                      Eigen::AngleAxisd(2., Eigen::Vector3d::UnitZ())));
  const common::Time time = common::FromUniversal(75);
  // 75在50-100之间，可以使用插值计算。
  const transform::Rigid3d interpolated = buffer.Lookup(time);
  EXPECT_THAT(
      interpolated,
      IsNearly(transform::Rigid3d::Translation(Eigen::Vector3d(5., 5., 5.)) *
                   transform::Rigid3d::Rotation(
                       Eigen::AngleAxisd(1., Eigen::Vector3d::UnitZ())),
               1e-6));
}

}  // namespace
}  // namespace transform
}  // namespace cartographer
