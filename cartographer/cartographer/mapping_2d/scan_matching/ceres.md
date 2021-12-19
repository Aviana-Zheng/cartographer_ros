

# 基于Ceres库的扫描匹配器
http://gaoyichao.com/Xiaotu/?book=Cartographer%E6%BA%90%E7%A0%81%E8%A7%A3%E8%AF%BB&title=%E5%9F%BA%E4%BA%8ECeres%E5%BA%93%E7%9A%84%E6%89%AB%E6%8F%8F%E5%8C%B9%E9%85%8D%E5%99%A8

通过分析[Local SLAM的业务主线](http://gaoyichao.com/Xiaotu/?book=Cartographer源码解读&title=Loca_SLAM的业务主线_AddRangeData)，我们发现Cartographer主要使用一种基于Ceres库的扫描匹配器，    完成激光扫描数据与地图之间的匹配工作，输出最可能的机器人位姿。    该扫描匹配器在[LocalTrajectoryBuilder2D](http://gaoyichao.com/Xiaotu/?book=Cartographer源码解读&title=Local_SLAM的核心_LocalTrajectoryBuilder2D)中以对象ceres_scan_matcher_的形式存在，    其数据类型为[CeresScanMatcher2D](https://github.com/googlecartographer/cartographer/blob/1.0.0/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h)。

本文中，将简单介绍一下Ceres库，然后详细分析类CeresScanMatcher2D。



## 1. Ceres库

[Ceres库](http://www.ceres-solver.org/)主要用于求解无约束或者有界约束的最小二乘问题。其数学形式如下：
$$
\begin{equation}
	\begin{split}
        \min_{\boldsymbol{x}} &\quad \frac{1}{2}\sum_{i} \rho_i\left(\left\|f_i\left(x_1, \cdots ,x_k\right)\right\|^2\right) \\
        \text{s.t.}           &\quad l_j \le x_j \le u_j
    \end{split}
\end{equation}
$$
我们的任务就是找到一组满足约束 $l_j \le x_j \le u_j$的 $x_1, \cdots, x_k$， 使得优化目标函数 $\begin{split}\frac{1}{2}\sum_{i} \rho_i\left(\left\|f_i\left(x_1, \cdots ,x_k\right)\right\|^2\right)\end{split}$取值最小。在Ceres库中，优化参数  $x_1, \cdots, x_k$被称为**参数块(ParameterBlock)**，它们的取值就是我们要寻找的解。 $l_j, u_j$分别是第*j*个优化参数 $x_j$的下界和上界。表达式 $\rho_i\left(\left\|f_i\left(x_1, \cdots ,x_k\right)\right\|^2\right)$被称为**残差项(ResidualBlock)**。    其中，是 $f_i(\cdot)$**代价函数(CostFunction)**， $\rho_i(\cdot)$则是关于代价函数平方的**核函数(LossFunction)**。    核函数存在的意义主要是为了降低野点(outliers)对于解的影响。

很多时候我们说最小二乘都是拿来做曲线拟合的，实际只要能够把问题描述成式(1)的形式，就都可以使用Ceres来求解。使用起来也比较简单， 只要按照[教程](http://www.ceres-solver.org/tutorial.html)介绍的套路，提供CostFunction的计算方式，描述清楚每个ResidualBlock以及LossFunction即可。    如下边的示例代码所示，一般我们需要定义三个对象，problem用于描述将要求解的问题，options提供了很多配置项，而summary用于记录求解过程。

```c++
ceres::Problem problem;
ceres::Solver::Options options;
ceres::Solver::Summary summary;
```

Ceres的求解过程包括构建最小二乘和求解最小二乘问题两部分，其中构建最小二乘问题的相关方法均包含在`Ceres::Problem`类中，涉及的成员函数主要包括`Problem::AddResidualBlock()`和`Problem::AddParameterBlock()`。

https://blog.csdn.net/weixin_43991178/article/details/100532618

### 1.1 Problem::AddResidualBlock( )

`AddResidualBlock()`顾名思义主要用于向`Problem`类传递残差模块的信息，函数原型如下，传递的参数主要包括代价函数模块、损失函数模块和参数模块。

关于cost_function，除了要提供代价函数的计算方法之外，还要明确其求导方法。求导方法我们完全可以以仿函数的形式自己定义一个，但更多的时候都在使用Ceres提供的AutoDiffCostFunction进行求导。    所以，我们经常能够看到类似下面示例的调用方法。

```c++
problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CostFunction, m, n>(new CostFunction(/* 构造参数 */)),loss_function, params);

ResidualBlockId Problem::AddResidualBlock(CostFunction *cost_function, 
										  LossFunction *loss_function, 
										  const vector<double *> parameter_blocks)
										  
ResidualBlockId Problem::AddResidualBlock(CostFunction *cost_function, 
										  LossFunction *loss_function,
										  double *x0, double *x1, ...)
```

类AutoDiffCostFunction是一个模板类，其模板参数列表中的CostFunction是计算残差项代价的仿函数类型，m是CostFunction所提供的残差项数量，n则是优化参数的数量。

1. 代价函数：包含了参数模块的维度信息，内部使用仿函数定义误差函数的计算方式。AddResidualBlock( )函数会检测传入的参数模块是否和代价函数模块中定义的维数一致，维度不一致时程序会强制退出。代价函数模块的详解参见Ceres详解（二） CostFunction。
2. 损失函数：用于处理参数中含有野值的情况，避免错误量测对估计的影响，常用参数包括HuberLoss、CauchyLoss等（完整的参数列表参见Ceres API文档）；该参数可以取NULL或nullptr，此时损失函数为单位函数。
3. 参数模块：待优化的参数，可一次性传入所有参数的指针容器vector<double*>或依次传入所有参数的指针double*。

代价函数本身的计算，要求我们以仿函数的形式，提供一个重载了运算符"()"的类，并在该重载中完成代价的计算。所以，我们会看到类似下侧示例代码的定义。    重载的运算符"()"有两个输入参数，其中params是优化的参数值，cost则记录了各个残差项中代价函数的输出。

```c++
class CostFunction {
public:
    template <typename T>
    bool operator()(const T* const params, T* cost) const {
        // 必要的运算之后，更新各个cost
        cost[0] = value_n;
        ...
            cost[m] = value_m;
        return true;
    }
}
```



正如上面AutoDiffCostFunction类型的模板参数m那样，一个CostFunction可能提供了多个残差项的代价，我们需要保证在重载的运算符"()"中更新的cost数量与模板参数m一致。

CostFunction只是提供代价函数的计算方式，而残差的计算则有Ceres根据核函数的选择自己计算了。核函数并不是必须提供的，当不需要的时候可以空指针(nullptr)来代替，    此时残差项为  $\rho_i\left(\left\|f_i\left(x_1, \cdots ,x_k\right)\right\|^2\right) = \left\|f_i\left(x_1, \cdots ,x_k\right)\right\|^2$。



### 1.2 Problem::AddParameterBlock( )

用户在调用AddResidualBlock( )时其实已经隐式地向Problem传递了参数模块，但在一些情况下，需要用户显示地向Problem传入参数模块（通常出现在需要对优化参数进行重新参数化的情况）。Ceres提供了Problem::AddParameterBlock( )函数用于用户显式传递参数模块：

```c++
void Problem::AddParameterBlock(double *values, int size)

void Problem::AddParameterBlock(double *values, int size, LocalParameterization *local_parameterization)
```

#### 1.2.1 LocalParameterization

LocalParameterization类的作用是解决非线性优化中的过参数化问题。所谓过参数化，即待优化参数的实际自由度小于参数本身的自由度。例如在SLAM中，当采用四元数表示位姿时，由于四元数本身的约束（模长为1），实际的自由度为3而非4。此时，若直接传递四元数进行优化，冗余的维数会带来计算资源的浪费，需要使用Ceres预先定义的QuaternionParameterization对优化参数进行重构：

```c++
problem.AddParameterBlock(quaternion, 4);// 直接传递4维参数

ceres::LocalParameterization* local_param = new ceres::QuaternionParameterization();
problem.AddParameterBlock(quaternion, 4, local_param)//重构参数，优化时实际使用的是3维的等效旋转矢量
```

#### 1.2.2 自定义LocalParameterization

`LocalParaneterization`本身是一个虚基类，详细定义如下。用户可以自行定义自己需要使用的子类，或使用Ceres预先定义好的子类。

```c++
class LocalParameterization {
 public:
  virtual ~LocalParameterization() {}
  //
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const = 0;//参数正切空间上的更新函数
  virtual bool ComputeJacobian(const double* x, double* jacobian) const = 0; //雅克比矩阵
  virtual bool MultiplyByJacobian(const double* x,
                                  const int num_rows,
                                  const double* global_matrix,
                                  double* local_matrix) const;//一般不用
  virtual int GlobalSize() const = 0; // 参数的实际维数
  virtual int LocalSize() const = 0; // 正切空间上的参数维数
};
```

上述成员函数中，需要我们改写的主要为`GlobalSize()`、`ComputeJacobian()`、`GlobalSize()`和`LocalSize()`，这里我们以ceres预先定义好的`QuaternionParameterization`为例具体说明，类声明如下：

```c++
class CERES_EXPORT QuaternionParameterization : public LocalParameterization {
 public:
  virtual ~QuaternionParameterization() {}
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x,
                               double* jacobian) const;
  virtual int GlobalSize() const { return 4; }
  virtual int LocalSize() const { return 3; }
};
```

- 可以看到，GlobalSize()的返回值为4，即四元数本身的实际维数；由于在内部优化时，ceres采用的是旋转矢量，维数为3，因此LocalSize()的返回值为3。
- 重载的Plus函数给出了四元数的更新方法，接受参数分别为优化前的四元数x，用旋转矢量表示的增量delta，以及更新后的四元数x_plus_delta。函数首先将增量由旋转矢量转换为四元数，随后采用标准四元数乘法对四元数进行更新。

```c++
bool QuaternionParameterization::Plus(const double* x,
                                      const double* delta,
                                      double* x_plus_delta) const {
  // 将旋转矢量转换为四元数形式
  const double norm_delta =
      sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
  if (norm_delta > 0.0) {
    const double sin_delta_by_delta = (sin(norm_delta) / norm_delta);
    double q_delta[4];
    q_delta[0] = cos(norm_delta);
    q_delta[1] = sin_delta_by_delta * delta[0];
    q_delta[2] = sin_delta_by_delta * delta[1];
    q_delta[3] = sin_delta_by_delta * delta[2];
    // 采用四元数乘法更新
    QuaternionProduct(q_delta, x, x_plus_delta);
  } else {
    for (int i = 0; i < 4; ++i) {
      x_plus_delta[i] = x[i];
    }
  }
  return true;
}
```

- ComputeJacobian函数给出了四元数相对于旋转矢量的雅克比矩阵计算方法，即 $\mathcal{J}_{4 \times 3}=d \mathcal{q} / d \mathcal{v} = d [q_w, q_x, q_y, q_z]^T /d [x,y,z] $，对应Jacobian维数为4行3列，存储方式为行主序。

```c++
bool QuaternionParameterization::ComputeJacobian(const double* x,
                                                 double* jacobian) const {
  jacobian[0] = -x[1]; jacobian[1]  = -x[2]; jacobian[2]  = -x[3];  // NOLINT
  jacobian[3] =  x[0]; jacobian[4]  =  x[3]; jacobian[5]  = -x[2];  // NOLINT
  jacobian[6] = -x[3]; jacobian[7]  =  x[0]; jacobian[8]  =  x[1];  // NOLINT
  jacobian[9] =  x[2]; jacobian[10] = -x[1]; jacobian[11] =  x[0];  // NOLINT
  return true;
}
```

#### 1.2.3 ceres预定义LocalParameterization

除了上面提到的`QuaternionParameterization`外，ceres还提供下述预定义`LocalParameterization`子类：

- `EigenQuaternionParameterization`：除四元数排序采用Eigen的实部最后外，与`QuaternionParameterization`完全一致；
- `IdentityParameterizationconst`：`LocalSize与GlobalSize`一致，相当于不传入`LocalParameterization`；
- `SubsetParameterization`：`GlobalSize`为2，`LocalSize`为1，用于第一维不需要优化的情况；
- `HomogeneousVectorParameterization`：具有共面约束的空间点；
- `ProductParameterization`：7维位姿变量一同优化，而前4维用四元数表示的情况（这里源文档只举了一个例子，具体用法有待深化）；

### 1.3 其他成员函数

`Probelm`还提供了其他关于`ResidualBlock`和`ParameterBlock`的函数，例如获取模块维数、判断是否存在模块、存在的模块数目等，这里只列出几个比较重要的函数，完整的列表参见[ceres API](http://www.ceres-solver.org/nnls_modeling.html#problem)：

```c++
// 设定对应的参数模块在优化过程中保持不变
void Problem::SetParameterBlockConstant(double *values)
// 设定对应的参数模块在优化过程中可变
void Problem::SetParameterBlockVariable(double *values)
// 设定优化下界
void Problem::SetParameterLowerBound(double *values, int index, double lower_bound)
// 设定优化上界
void Problem::SetParameterUpperBound(double *values, int index, double upper_bound)
// 该函数紧跟在参数赋值后，在给定的参数位置求解Problem，给出当前位置处的cost、梯度以及Jacobian矩阵；
bool Problem::Evaluate(const Problem::EvaluateOptions &options, 
					   double *cost, vector<double>* residuals, 
					   vector<double> *gradient, CRSMatrix *jacobian)
```



完成了上述的准备工作之后，我们就可以调用ceres::Solve求解了。最后的解保存在准备工作中的params对象中，需要强调的是，在求解之前应当给params一个比较合理的迭代初值。

```c++
ceres::Solve(&options, &problem, &summary);
```

## 2. 类CeresScanMatcher2D

下面我们来看一下Cartographer是如何使用Ceres库完成扫描匹配的，类[CeresScanMatcher2D](https://github.com/googlecartographer/cartographer/blob/1.0.0/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h)只定义了两个成员变量。    其中，options_用于记录扫描匹配器的相关配置，而变量ceres_solver_options_则是优化过程的配置。而且该类还屏蔽了拷贝构造和拷贝赋值。

下面是类CeresScanMatcher2D的构造函数和析构函数的定义。构造函数无非是记录下配置项，通过函数CreateCeresSolverOptions和配置项ceres_solver_options装载Ceres优化库的配置。    析构函数则什么都没有做。

```c++
CeresScanMatcher2D::CeresScanMatcher2D(const proto::CeresScanMatcherOptions2D& options)
            : options_(options),
ceres_solver_options_(common::CreateCeresSolverOptions(options.ceres_solver_options())) {
            ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
        }
CeresScanMatcher2D::~CeresScanMatcher2D() {}
```

该类的核心就在下面这个Match函数的实现上，它有6个参数，在给定机器人的初始位姿估计initial_pose_estimate的情况下，尽可能的将扫描的点云数据point_cloud放置到栅格地图grid上，    同时返回优化后的位姿估计pose_estimate和优化迭代信息summary。而参数target_translation主要用于约束位姿估计的xy坐标，    在[Local SLAM的业务主线](http://gaoyichao.com/Xiaotu/?book=Cartographer源码解读&title=Loca_SLAM的业务主线_AddRangeData)调用该函数的时候传递的是[位姿估计器](http://gaoyichao.com/Xiaotu/?book=Cartographer源码解读&title=位姿估计器)的估计值，Cartographer认为优化后的机器人位置应当与该估计值的偏差不大。

```c++
void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                                       const transform::Rigid2d& initial_pose_estimate,
                                       const sensor::PointCloud& point_cloud,
                                       const Grid2D& grid,
                                       transform::Rigid2d* const pose_estimate,
                                       ceres::Solver::Summary* const summary) const {
```

在函数的一开始用一个double数组记录下初始位姿，作为Ceres优化迭代的初值。

```c++
double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                             initial_pose_estimate.translation().y(),
                                             initial_pose_estimate.rotation().angle()};
```

接下来构建了ceres::Problem对象，并通过接口AddResidualBlock添加残差项。从代码看来，残差主要有三个方面的来源：(1) 占用栅格与扫描数据的匹配度，(2) 优化后的位置相对于target_translation的距离，    (3) 旋转角度相对于迭代初值的偏差。关于这三类残差的计算本文后面会详细解释。此外，还应注意到在配置文件中为这三类来源定义了权重。

```c++
ceres::Problem problem;
CHECK_GT(options_.occupied_space_weight(), 0.);

problem.AddResidualBlock(OccupiedSpaceCostFunction2D::CreateAutoDiffCostFunction(options_.occupied_space_weight() / std::sqrt(static_cast<double>(point_cloud.size())), point_cloud, grid),nullptr /* loss function */, ceres_pose_estimate);

CHECK_GT(options_.translation_weight(), 0.);

problem.AddResidualBlock(TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(options_.translation_weight(), target_translation),nullptr /* loss function */, ceres_pose_estimate);

CHECK_GT(options_.rotation_weight(), 0.);
           problem.AddResidualBlock(RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(options_.rotation_weight(), ceres_pose_estimate[2]),nullptr /* loss function */, ceres_pose_estimate);
```

最后求解并更新位姿估计。

```c++
ceres::Solve(ceres_solver_options_, &problem, summary);

*pose_estimate = transform::Rigid2d({ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}
```

## 3. 残差计算

正如我们刚刚看到的，Cartographer为三类不同的残差源分别定义了一个类，并通过其静态函数CreateAutoDiffCostFunction提供使用Ceres原生的自动求导方法的代价函数计算。

下面的代码片段是[类RotationDeltaCostFunctor2D](https://github.com/googlecartographer/cartographer/blob/1.0.0/cartographer/mapping/internal/2d/scan_matching/rotation_delta_cost_functor_2d.h)中关于构造和拷贝的函数定义。可以看到它屏蔽了拷贝构造和拷贝赋值，同时还显式的将唯一的构造函数定义称为私有的，这就意味着我们不能直接的构造其对象，    只能通过其静态函数CreateAutoDiffCostFunction间接的创建。

```c++
class RotationDeltaCostFunctor2D {
private:
    explicit RotationDeltaCostFunctor2D(const double scaling_factor, const double target_angle): scaling_factor_(scaling_factor), angle_(target_angle) {}
    RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D&) = delete;
    RotationDeltaCostFunctor2D& operator=(const RotationDeltaCostFunctor2D&) = delete;
};
```

下面是函数CreateAutoDiffCostFunction的实现，它是类RotationDeltaCostFunctor2D公开的静态成员。有两个参数，其中scaling_factor是角度偏差的权重，可以通过配置文件指定; target_angle则是参考角度，     实际的调用传参是优化之前的角度估计。该函数最终构造和返回的是Ceres的AutoDiffCostFunction对象，从第2行中的模板列表中可以看到类RotationDeltaCostFunctor2D只提供一个残差项，其参与优化的参数有3个。

```c++
static ceres::CostFunction* CreateAutoDiffCostFunction(const double scaling_factor, const double target_angle) {
    return new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor2D, 1 /* residuals */, 3 /* pose variables */>(
                        new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
}
```

下边是类RotationDeltaCostFunctor2D对运算符"()"的重载，在第四行中计算角度偏差量并乘上权重得到残差项的代价。    这里的scaling_factor_和angle_是在类RotationDeltaCostFunctor2D中定义的两个私有成员变量分别记录了权重和参考角度。它们通过构造函数和静态函数CreateAutoDiffCostFunction的传参赋值。

```c++
// RotationDeltaCostFunctor2D
template <typename T>
bool operator()(const T* const pose, T* residual) const {    
    residual[0] = scaling_factor_ * (pose[2] - angle_);
    return true;
}
```

类[TranslationDeltaCostFunctor2D](https://github.com/googlecartographer/cartographer/blob/1.0.0/cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h)和[OccupiedSpaceCostFunction2D](https://github.com/googlecartographer/cartographer/blob/1.0.0/cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h)也是以相同的套路实现的，把构造函数定义成私有的，防止用户直接构造对象。    在静态函数CreateAutoDiffCostFunction中以这两个类型为基础构建AutoDiffCostFunction对象。类TranslationDeltaCostFunctor2D定义了成员变量x_和y_用于记录位移偏差量的参考坐标。    类OccupiedSpaceCostFunction2D则使用成员变量point_cloud_和grid_分别记录待匹配的激光点云数据和栅格地图。

下边是类TranslationDeltaCostFunctor2D对运算符"()"的重载，它有两个残差项的代价需要计算，分别对应着x轴和y轴上的偏差量。在第4,5行中计算位置偏差量并乘上权重来更新对应残差项的代价。

```c++
// TranslationDeltaCostFunctor2D
template <typename T>
bool operator()(const T* const pose, T* residual) const {    
    residual[0] = scaling_factor_ * (pose[0] - x_);
    residual[1] = scaling_factor_ * (pose[1] - y_);
    return true;
}
```

对于优化结果添加位置偏差量和角度偏差量的限定，我个人理解，通过位姿估计器推测的位姿基本上就在真值附近，如果优化后的结果出现了较大的偏差说明位姿跟丢了。    刚刚讨论的这两残差项顶多就是对匹配位姿的约束，真正的扫描匹配的主角是类OccupiedSpaceCostFunction2D。在分析它的代价计算方式之前，    我们先来看一下[论文的IV章C节](http://gaoyichao.com/Xiaotu//papers/2016 - Real-time loop closure in 2D LIDAR SLAM - Hess et al.pdf)是如何计算两者的匹配度的。

文中提到扫描匹配的目的就是在地图中找到一个位姿估计，使得在该位姿下所有hit点在局部地图中的占用概率之和最大。    记 $H = \{h_1, \cdots, h_k, \cdots, h_K\}$为传感器扫描到的 $K$个hit点集合，$h_k$是第 $k$个hit点在机器人坐标系下的位置坐标。    那么 $h_k$在地图坐标系下的坐标可以表示为：
$$
\begin{equation}
	T_{\varepsilon} h_k = 
	\begin{bmatrix} 
		\cos \varepsilon_{\theta} & - \sin \varepsilon_{\theta} \\ 
		\sin \varepsilon_{\theta} & \cos \varepsilon_{\theta} 
	\end{bmatrix}  h_k + 
	\begin{bmatrix} 
		\varepsilon_x \\ 
		\varepsilon_y 
	\end{bmatrix}
\end{equation}
$$
其中， $\varepsilon = \begin{bmatrix} \varepsilon_x & \varepsilon_y & \varepsilon_{\theta} \end{bmatrix}^T$表示位姿估计 $\varepsilon_x, \varepsilon_y$分别是机器人在地图坐标系下的坐标， $\varepsilon_{\theta}$是机器人的方向角。用 $T_{\varepsilon}$表示位姿估计描述的坐标变换。我们根据hit点在地图坐标系下的位置坐标查找对应的占用概率，    但由于栅格坐标相对来说是一个比较稀疏离散的采样，所以Cartographer在它的基础上进行了一次双三次(bicubic)插值 $M_{smooth}\left(T_{\varepsilon} h_k\right)$。    因此优化的目标就是:
$$
\begin{equation}
	\underset{\varepsilon}{\text{argmin}} \sum_{k = 1}^K \left(1 -  M_{smooth}\left(T_{\varepsilon} h_k\right)\right)^2
\end{equation}
$$


下面是类OccupiedSpaceCostFunction2D对运算符"()"的重载。在函数的一开始，先把迭代查询的输入参数pose转换为坐标变换 $T_{\varepsilon}$用临时变量transform记录。

```c++
template <typename T>
bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);
```

然后使用Ceres库原生提供的双三次插值迭代器。

```c++
const GridArrayAdapter adapter(grid_);
ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
const MapLimits& limits = grid_.limits();
```

接着根据式([3](http://gaoyichao.com/Xiaotu/?book=Cartographer源码解读&title=基于Ceres库的扫描匹配器#mjx-eqn-f3))针对每个hit点计算对应的残差代价。第13行通过hit点坐标与transform的相乘得到其在地图坐标系下的坐标 $T_{\varepsilon} h_k$。    在第14行通过刚刚构建的迭代器，和地图坐标，获取在hit点出现的概率。该函数调用有三个参数，前两个参数用来描述x,y轴索引，    第三个参数用于记录插值后的结果。这里的xy索引计算的比较奇怪，它通过GridArrayAdapter类中成员函数[GetValue](https://github.com/googlecartographer/cartographer/blob/1.0.0/cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h#L83)获取栅格数据，这里不再细述。    此外由于[占用栅格](http://gaoyichao.com/Xiaotu/?book=Cartographer源码解读&title=占用栅格的数据结构)中原本存储的就是栅格空闲的概率，    所以这里查询出来的概率就是 $\left(1 -  M_{smooth}\left(T_{\varepsilon} h_k\right)\right)$。

```c++
for (size_t i = 0; i < point_cloud_.size(); ++i) {
    const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].x())), (T(point_cloud_[i].y())), T(1.));
    const Eigen::Matrix<T, 3, 1> world = transform * point;
    interpolator.Evaluate((limits.max().x() - world[0]) / limits.resolution() - 0.5 + static_cast(kPadding),(limits.max().y() - world[1]) / limits.resolution() - 0.5 + static_cast(kPadding),&residual[i]);
    residual[i] = scaling_factor_ * residual[i];
}
```

最后返回退出。

```c++
	return true;
}
```

## 4. 完

本文中我们简单介绍了使用Ceres库求解问题的基本套路。然后分析了扫描匹配器CeresScanMatcher2D，发现它除了要求hit点在占用栅格上出现的概率最大化之外，    还通过两个残差项约束了优化后的位姿估计在原始估计的附近。