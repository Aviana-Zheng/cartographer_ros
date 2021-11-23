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

#ifndef CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
#define CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_

#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"

namespace cartographer {
namespace transform {
/*
几个名词细微的区别 :

transformation:变换
translation:平移
rotation:旋转
scaling:缩放
clock wise rotation:顺时针旋转
counter clock wise rotation:逆时针旋转，counter：反向
Identity():单位矩阵,不同于线性代数的“单位”。在不同的语义下，含义会不同。


Rigid2是对二维刚性变换【旋转与平移】的封装,SE2.对应论文公式(1):


Rigid2表现为先将vector[x0,y0]旋转θ角度得到[x',y']，然后再叠加[dx,dy]得到[x1,y1]
即:
x1=x'+dx,
y1=y'+dy,


模板参数必须是浮点型。(int类型为错误用法)

提供2个操作：
1，按照指定角度θ逆时针旋转。此时dx,dy=0
2，按照指定平移向量dx,dy平移. 此时θ=0

静态成员函数包括：
1,Rotation()
2,Translation()
3,Identity()

Translation()指定对象的2D translation（2D平移）。
第一个参数对应X轴，第二个参数对应Y轴。默认是单位变换。　　　


[x'       [cosθ , -sinθ           [ x
 y']   =   sinθ ,  cosθ ]   *       y ]
*/

template <typename FloatType>
//FloatType不能是int类型，不然Rotation2D的cos0==0,丢失角度信息，无法得到正确结果
class Rigid2 {
 public:
  using Vector = Eigen::Matrix<FloatType, 2, 1>;   //2行1列
  using Rotation2D = Eigen::Rotation2D<FloatType>;    

// 无参构造函数,平移向量为单位向量[1,0]^t,旋转角度为0,debugstring()输出:[1,0,0]
  Rigid2()
      : translation_(Vector::Identity()), rotation_(Rotation2D::Identity()) {}

//Rotation2D(double ): Construct a 2D counter clock wise rotation from the angle a in radian. 
//双参构造函数，给定平移向量[dx,dy]和旋转角度0，进行旋转变换: 
  Rigid2(const Vector& translation, const Rotation2D& rotation)
      : translation_(translation), rotation_(rotation) {}
//同上，给定旋转角度θ,double是弧度值
  Rigid2(const Vector& translation, const double rotation)
      : translation_(translation), rotation_(rotation) {}

//类的静态成员函数，返回Rigid2,debugstring()是[0,0,θ ]
  static Rigid2 Rotation(const double rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }
//同上
  static Rigid2 Rotation(const Rotation2D& rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }
//旋转角度是单位矩阵,即θ为0 ,[dx,dy,0]
  static Rigid2 Translation(const Vector& vector) {
    return Rigid2(vector, Rotation2D::Identity());
  }
//静态成员函数，全为0：
  static Rigid2<FloatType> Identity() {
    //返回Rigid2,[0,0,0]
    return Rigid2<FloatType>(Vector::Zero(), Rotation2D::Identity());
  }

/*为什么要加translation_.template：
参见成员模板函数的写法：
https://stackoverflow.com/questions/29754251/issue-casting-c-eigenmatrix-types-via-templates
https://stackoverflow.com/questions/12676190/how-to-call-a-template-member-function
https://stackoverflow.com/questions/4942703/why-do-i-get-an-error-trying-to-call-a-template-member-function-with-an-explicit?answertab=votes

cast()按照指定的参数类型将数据成员进行类型转换：
Rigid2数据成员原本是double,对象rigid2转换成float可调用：
rigid2.cast<float>()
*/
  template <typename OtherType>
  Rigid2<OtherType> cast() const {
    return Rigid2<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }
//返回平移向量,[dx,dy]
  const Vector& translation() const { return translation_; }

/*返回旋转矩阵:
[cosθ , -sinθ  
 sinθ ,  cosθ ] */
  Rotation2D rotation() const { return rotation_; }

//归一化角度 ,弧度[-pi;pi]
  double normalized_angle() const {
    return common::NormalizeAngleDifference(rotation().angle());
    //方位角θ.以弧度计。
  }


/*
平移变换矩阵的逆矩阵与原来的平移量相同，但是方向相反。
旋转变换矩阵的逆矩阵与原来的旋转轴相同,但是角度相反。

Rotation2D重载了operator*(maxtrix<sclar,2,1>).故可以使用乘法。
乘以rotation：[dx,dy]是旋转了θ以后的[dx,dy]
而求逆是反方向旋转θ，故需要求[dx,dy]在旋转θ之前的方向的投影。
translation加负号的原因：x1=x'+dx，所以x'=x1-dx

----公式推导----------

P'=R*P+t;
P=R^(-1)*(P'-t)=R^(-1)* P'-R^(-1)*t.
--------------------

*/
  Rigid2 inverse() const {
    //取负,得到-θ。即上面公式中的R^(-1)
    const Rotation2D rotation = rotation_.inverse();
    //得到[-dx',-dy']。即上面公式中的-R^(-1)*t
    const Vector translation = -(rotation * translation_);
    return Rigid2(translation, rotation);
    //返回一个新2维刚性变换矩阵.[-dx’,-dy’,-θ]。即上面公式中的R^(-1)和-R^(-1)*t。
  }

  string DebugString() const {
    string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append("], r: [");
    out.append(std::to_string(rotation().angle()));
    out.append("] }");
    return out;
  }

 private:
  //2行1列 的矩阵.平移向量[dx,dy]
  Vector translation_;
  //旋转角度。Eigen::Rotation2D,方向角,旋转变换 [θ]
  Rotation2D rotation_;

/*
Rotation2D介绍：
Rotation2D 是二维旋转里根据逆时针旋转θ角而生成的旋转矩阵:
[cosθ , -sinθ     
 sinθ ,  cosθ ]
*/
};

/*
定义 * 乘法操作符:2个Rigid2相乘,得到第三个Rigid2,
等效于连续变换2次
实现细节：
1，最终的[dx'',dy'']等于lhs在rhs方向上的投影[dx',dy']加上rhs自身的[dx,dy]
2，角度相加(等效于旋转矩阵相乘)

------公式推导--------

P'=R1*P+t1;

P''=R2*P'+t2,即

P''=R2(R1*P+t1)+t2=R2*R1*P+R2*t1+t2.
--------------------
*/

template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs,
                            const Rigid2<FloatType>& rhs) {
  return Rigid2<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),    //对应R2*t1+t2.
      lhs.rotation() * rhs.rotation());                          //对应R2*R1
}

/*
该函数即为公式(1):

参数1：【旋转+平移】矩阵
参数2：要进行上述变换的point。

返回值：刚性变换后的point。
等效于对某个2维point旋转0角，再叠加平移矩阵[dx,dy]。


实现：
1，先对vector[x0,y0]自身旋转0，得到[x',y'].
2，[x',y']再加上dx,dy.得到vector[x1,y1]


--------公式推导---------
P'=R*P+t;
------------------------
*/

template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
    const Rigid2<FloatType>& rigid,
    const typename Rigid2<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
  /*
Rotation2D的乘法原型：
Vector2 Eigen::Rotation2D< Scalar >::operator*  ( const Vector2 &   vec ) const
:
Applies the rotation to a 2D vector

  */
}

// 定义 << 输出运算符
// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const cartographer::transform::Rigid2<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

//模板特化
using Rigid2d = Rigid2<double>;
using Rigid2f = Rigid2<float>;


/*
预备概念：

四元数,通常用quaternion来计算3D物体的旋转角度，与Matrix相比，quaternion更加高效。
在数学上，quaternion表示复数w+xi+yj+zk，其中i,j,k都是虚数单位。
四元数也可以表示为[w,iv].其中虚数v=[x,y,z]，用四元数描述旋转需要的存储空间很小。
更为关键的是可以使用被称为球面线性插值（Slerp Algorithm）的方法对四元数进行插值运算，
从而解决了平滑旋转的插值问题。

3D旋转可以分解为绕x,y,z三个方向的旋转的叠加。
而Eigen的AngleAxis表示3D旋转绕任意轴axis的旋转。
并且与MatrixBase::Unit{X,Y,Z}组合,AngleAxis 能非常容易的得到欧垃角。

*/

/*
Rigid3是三维刚性变换.SE3
该类含义2个数据成员：

Vector translation_;
Quaternion rotation_;

1),3个构造函数分别初始化平移矩阵和旋转矩阵。

2),3个静态成员函数分别按照给定方式旋转.
   Rotation(),Translation(),Identity()
构造函数应为单位四元数

*/
template <typename FloatType>
class Rigid3 {
 public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
  using Quaternion = Eigen::Quaternion<FloatType>;
  using AngleAxis = Eigen::AngleAxis<FloatType>;

  Rigid3()
      : translation_(Vector::Identity()), rotation_(Quaternion::Identity()) {}
  Rigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }

  static Rigid3 Rotation(const Quaternion& rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }

  static Rigid3 Translation(const Vector& vector) {
    return Rigid3(vector, Quaternion::Identity());
  }

  static Rigid3<FloatType> Identity() {
    return Rigid3<FloatType>(Vector::Zero(), Quaternion::Identity());
  }

  template <typename OtherType>
  Rigid3<OtherType> cast() const {
    return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }

  Rigid3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

  string DebugString() const {
    string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append(", ");
    out.append(std::to_string(translation().z()));
    out.append("], q: [");
    out.append(std::to_string(rotation().w()));
    out.append(", ");
    out.append(std::to_string(rotation().x()));
    out.append(", ");
    out.append(std::to_string(rotation().y()));
    out.append(", ");
    out.append(std::to_string(rotation().z()));
    out.append("] }");
    return out;
  }

 private:
  Vector translation_;
  Quaternion rotation_;
};

template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
}

template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
    const Rigid3<FloatType>& rigid,
    const typename Rigid3<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const cartographer::transform::Rigid3<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;

// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.
Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);

// Returns an transform::Rigid3d given a 'dictionary' containing 'translation'
// (x, y, z) and 'rotation' which can either we an array of (roll, pitch, yaw)
// or a dictionary with (w, x, y, z) values as a quaternion.
Rigid3d FromDictionary(common::LuaParameterDictionary* dictionary);

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
