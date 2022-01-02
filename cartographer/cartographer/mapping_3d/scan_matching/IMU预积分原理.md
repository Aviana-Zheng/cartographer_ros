# 姿态估计与李群和流形---收回(retraction)操作的选择

原文https://zhuanlan.zhihu.com/p/376342834



## 1 增量角度 $\theta = w\Delta t$ 和物体姿态的变化量  $\Delta q$ 之间的关系

在姿态估计的计算过程中，第一个要解决的问题就是，当我们在一段时间 $\Delta t$ 内测量得到角速度    $w$ ，得到物体变化的增量角度 $\theta = w\Delta t$ ，它和物体姿态的变化量  $\Delta q$ 之间的关系，也就是所谓的映射关系，是什么？看多了书，会发现这种映射关系居然不是唯一的，可以有好几种。

这种映射关系在流形中被称为收回(retraction)操作。在引入流形之后，就会发觉，存在有多种映射关系，其实是我们的选择。流形不只是完美的球面一种，我们相信底下的流形是什么样子，我们才选择了什么样的收回(retraction)操作。

## 2 形象理解映射关系

### 2.1 单位四元数的形象化

单位四元数 $q$ :
$$
q &= &r + x \pmb{\mathcal{i}} + y \pmb{\mathcal{j}} + z \pmb{\mathcal{k}} \\
  &= &r + \pmb{\mathcal{q}}
$$
可以表示三维空间的姿态。但是单位四元数 $q$ 要想用图形表示出来，也是一个困难的事情，因为单位四元数 $q$ 所在的球面是嵌入 $\mathbb{R}^4$ 空间中的 $\mathcal{S}^3$ 球面。所以有一种方法是用类似复平面的方式表示:

![](assets/ceres1.jpg)

其中，实数轴做为南北轴。因为单位四元数 $q$ 表示旋转是两倍覆盖，所以一般选择 $r \ge 0$ 

的北半球表示一个三维旋转。另外，图中的角度 $\theta$ 代表的是:

$tan(\theta) = \frac{||q||}{r}$ 

### 2.2 映射关系的类型

在参考文献[1-3]这里，流形的收回(retraction)操作所选择指数映射函数 $exp()$ ，或大写指数映射函数 $\operatorname{Exp}()$ 的运算和常见的四元数形式关联为:
$$
\operatorname{Exp}(\theta a^{\wedge}) &= &exp([\theta \times])    \\
&= &cos(\frac{\theta}{2})+ sin(\frac{\theta}{2}) \pmb{\mathbb{u}}
$$
其实这个收回(retraction)操作，在不同的论文和项目里，有不同的版本。这个 $\frac{1}{2}$  系数，是因为四元数 $\mathcal{S}^3$   流形对 SO( 3 ) 是双倍覆盖。于是在旋转矩阵形式的版本中使用的是全角  $\theta$  ，而在四元数版本中使用的是半角 $\frac{\theta}{2}$ 。在参考文献[4]中，总结了常见的4种：

1. 正交投影 (Orthographic, O) 

   $ \left(\begin{array}{c}\sqrt{1-\|\boldsymbol{\theta}\|^{2} / 4} \\ \boldsymbol{\theta} / 2\end{array}\right) $

2. Rodrigues 参数 (Rodrigues Parameters, RP)

   $\frac{1}{\sqrt{4+\|\boldsymbol{\theta}\|^{2}}}\left(\begin{array}{l}2 \\ \boldsymbol{\theta}\end{array}\right) $

3. 改进型 Rodrigues 参数 (Modified Rodrigues Parameters, MRP)  

   $\frac{1}{16+\|\boldsymbol{\theta}\|^{2}}\left(\begin{array}{c}16-\|\boldsymbol{\theta}\|^{2} \\ 8 \boldsymbol{\theta}\end{array}\right) $

4. 旋转向量 (Rotation Vector, RV)

   $\left(\begin{array}{c}\cos (\boldsymbol{\theta} / 2) \\ (\boldsymbol{\theta} /\|\boldsymbol{\theta}\|) \sin (\boldsymbol{\theta} / 2)\end{array}\right) $

5. 参考文献[1-3]所选择的指数映射函数 

   $\exp ()\left(\begin{array}{c}\cos (\theta) \\ (\theta /\|\theta\|) \sin (\theta)\end{array}\right) $

前三种是我们所知道的球极平面投影 (stereographic projection){这三种投影分别具体称为正交投影  (Orthographic)、日晷投影 (Gnomonic) 和球极平面投影 (Stereographic)}。最后是所谓的等距投影  (Equidistant)。下面我们通过图形理解这些映射。

#### 2.2.1 正交投影(Orthographic)

其中 , $\mathbb{E}^3$ 空间在  $r \in [0, 1]$ 之间截取投影:

![](assets/ceres2.jpg)

#### 2.2.2 Rodrigues 参数 (Rodrigues Parameters, RP)

又称Gibbs向量。这是[Dr. F. Landis Markley](https://link.zhihu.com/?target=http%3A//www.acsu.buffalo.edu/~johnc/markley/)的最爱，因为它是单映射， $+q$ 和  $-q$ 都映射到同一个Gibbs向量，对于同一个旋转来说，这是 $1:1$ 表示。并且因为 $tan()$  函数和高斯分布有一点点相像，就是两端有无限长尾。所以(NASA)大佬们的论文里更喜欢选择这个映射。在文献[4]里也偏向采用这种映射。这种映射属于日晷投影 (Gnomonic)， $\mathbb{E}^3$ 空间在  $r = 1$  ，通过圆心的两个单位四元数投影到同一个Gibbs向量

![](assets/ceres3.jpg)

#### 2.2.3  改进型 Rodrigues 参数  (Modified Rodrigues Parameters, MRP)

这种映射与旋转向量映射有许多共同的特点，包括离散跳跃的需求，但又避免了超越函数。这种映射属于球极平面投影 (Stereographic)， $\mathbb{E}^3$ 空间在  $r = 0$  ， $\mathcal{S}^3$ 的一个半球在三维 $\pmb{\mathcal{p}}$  空间中投射到单位球体的内部，而 $\mathcal{S}^3$ 的另一个半球则投射到单位 $\pmb{\mathcal{p}} - sphere$  的外部。

![](assets/ceres4.jpg)

#### 2.2.4 旋转向量 (Rotation Vector, RV)

际上就是参考文献[1-3]所说的指数映射函数 $\operatorname{exp}()$ ，在旋转矩阵形式的版本中使用的是全角 $\theta$ ，而在四元数版本中使用的是半角  $\frac{\theta}{2}$  。这种映射属于等距投影 (Equidistant)， $\mathbb{E}^3$ 空间位于幺元处，并且向量的模长和测地线(geodesic)的弧长是  $1:1$ 关系.

![](assets/ceres5.jpg)



## 3 映射关系的选择

同时存在这么多种映射关系，这往往体现了不同理论之间，还有理论和工程之间的差异。

如果说前三种映射关系是第四种的某种近似，因为这四种映射关系在零点附近的小角度情况下数值很接近，但大角度之后就明显不再是球面，如果球面是完美模型，选择前三种映射关系可能是因为某种原因，例如选择正交投影  (Orthographic)是因为它可以简单地把单位四元数转换为轴-角向量，选择Rodrigues参数(RP)是因为它是单映射，选择改进型Rodrigues参数(MRP)是因为它避免了超越函数的运算。此外，因为在工程中IMU的取样间隔时间 $\Delta t$  很小，所以一般增量角度  $\pmb{\theta = w \Delta t}$ 都很小，所以都会应用三角函数小角度近似的方法。

难道就没有一个高下么？在一些论文里，例如在文献[4]中，做过模拟检验，前四种映射关系在数值精度上其实没有一个有特别优势。如果加入第五种映射关系经行对比，我估计结果也类似。因为都是在卡尔曼滤波器中采用这些映射关系，而卡尔曼是一个神奇的东西。虽然物理模型有差异，在测量更新阶段的残差增大，那首先影响的是协方差矩阵，接着是卡尔曼增益系数，但是最后的状态输出会趋向于类似的稳定数值上，因为测量校正发挥了影响力。最后可能只是内部的协方差矩阵的数量级从 $10^{-9}$ 变成了 $10^{-8}$ ，虽然差了一个数量级，但都是一个很小的数值，而且协方差矩阵和卡尔曼增益等比例稳定之后，输出结果也会稳定趋同。

## 4 总结

大佬选啥我跟啥。城门失火，殃及池鱼。神仙打架，小鬼遭殃。最后还得等大佬们吵架吵出一个结果之后我们好跟随。

## 5 参考文献

1. [A micro Lie theory for state estimation in robotics](https://link.zhihu.com/?target=https%3A//arxiv.org/abs/1812.01537)
2. [Lie theory for the roboticist](https://link.zhihu.com/?target=https%3A//www.youtube.com/watch%3Fv%3DnHOcoIyJj2o)
3. [Joan Solà - Lie theory for the Roboticist](https://link.zhihu.com/?target=https%3A//www.youtube.com/watch%3Fv%3DQR1p0Rabuww)
4. [Kalman Filtering for Attitude Estimation with Quaternions and Concepts from Manifold Theory](https://link.zhihu.com/?target=https%3A//www.mdpi.com/1424-8220/19/1/149)
5. [Full-Order Solution to the Attitude Reset Problem for Kalman Filtering of Attitudes](https://link.zhihu.com/?target=https%3A//arc.aiaa.org/doi/10.2514/1.G004134)

# 四元数的导数

参考：

[四元数速查手册](https://aipiano.github.io/2019/01/11/%E5%9B%9B%E5%85%83%E6%95%B0%E9%80%9F%E6%9F%A5%E6%89%8B%E5%86%8C/#:~:text=%E5%9B%9B%E5%85%83%E6%95%B0%E7%9A%84%E5%AF%BC%E6%95%B0%20%E5%AF%B9%E6%97%B6%E9%97%B4%E6%B1%82%E5%AF%BC,%E5%9C%A8%E5%8A%A8%E5%8A%9B%E5%AD%A6%E7%B3%BB%E7%BB%9F%E4%B8%AD%EF%BC%8C%E7%94%A8%E5%9B%9B%E5%85%83%E6%95%B0%E8%A1%A8%E7%A4%BA%E6%9F%90%E4%B8%AA%E7%89%A9%E4%BD%93%E7%9A%84%E5%A7%BF%E6%80%81%EF%BC%88%E6%97%8B%E8%BD%AC%E9%87%8F%EF%BC%89%E6%97%B6%EF%BC%8C%E5%9B%9B%E5%85%83%E6%95%B0%E6%98%AF%E6%97%B6%E9%97%B4%E7%9A%84%E5%87%BD%E6%95%B0%EF%BC%8C%E5%88%99%E5%85%B6%E5%85%B3%E4%BA%8E%E6%97%B6%E9%97%B4%E7%9A%84%E5%8F%98%E5%8C%96%E7%8E%87%EF%BC%88%E5%AF%BC%E6%95%B0%EF%BC%89%E4%B8%BA%20%E5%85%B6%E4%B8%AD%E6%98%AF%E5%B1%80%E9%83%A8%EF%BC%88Body%EF%BC%89%E5%9D%90%E6%A0%87%E7%B3%BB%E4%B8%8B%E7%89%A9%E4%BD%93%E7%9A%84%E8%A7%92%E9%80%9F%E5%BA%A6%EF%BC%8C%E6%98%AF%E4%B8%80%E4%B8%AA%E8%BD%B4%E8%A7%92%E8%A1%A8%E7%A4%BA%E7%9A%84%E4%B8%89%E7%BB%B4%E7%9F%A2%E9%87%8F%EF%BC%8C%E5%B8%A6%E5%85%A5%E4%B8%8A%E5%BC%8F%E8%AE%A1%E7%AE%97%E6%97%B6%E8%A6%81%E5%85%88%E5%8F%98%E4%B8%BA%E7%BA%AF%E5%9B%9B%E5%85%83%E6%95%B0%E3%80%82)

[三角函数公式（超全）](https://zhuanlan.zhihu.com/p/362443307)

[四元数的微分形式](https://blog.csdn.net/qq_39554681/article/details/88909564)

## 对时间求导

### 关于四元数表示旋转的一些知识

#### 1.四元数表示向量旋转
定义: $ p_v = [p_x \enspace p_y \enspace p_z]^T$为三维空间中的一点，将其转换成纯四元数形式即： $p=[0 \enspace p_v]^T$，令 
$$
q=
 \begin{bmatrix}
 cos(\frac{\theta}{2}) & vsin(\frac{\theta}{2})
 \end{bmatrix} ^T
$$


为单位四元数，则有
$$
p' = q \thinspace \cdot  p \thinspace \cdot  q^{-1}
   = 
   \begin{bmatrix}
   0 & p_v'
   \end{bmatrix}
   ^T
$$
其中 $p_v' = [p_x' \enspace p_y' \enspace p_z']^T$表示 $p_v$ 绕旋转轴 $v$ 旋转 $\theta$ 角度后得到的新向量在原三维空间中的坐标表示。



以1.为例，先做 $q_1$ 旋转，再做 $q_2$ 旋转之后向量  $x$ 在新坐标系中的表示为  $x'$ ，则有：
$$
x_{temp} &= &q_1 \thinspace \cdot x \thinspace \cdot q_1^{-1} \\
x' &= &q_2 \thinspace \cdot x_{temp} \thinspace \cdot q_2^{-1} \\
x' &= &q_2 q_1 \thinspace \cdot x \thinspace \cdot q_2^{-1} q_1^{-1}
$$
四元数 $q_2q_1$ 表示了连续两次的旋转。



#### 2.四元数表示坐标系旋转
定义向量 $v_0$在 $oxyz$坐标系中的表示为 $v_1=[v_x \enspace v_y \enspace v_z]^T$，令坐标系 $oxyz$ 绕单位旋转轴 $v$ 旋转 $\theta$ 角度，得到新坐标系  $o'x'y'z'$，此时 $v_0$ 在新坐标系中的坐标表示为 $v_1'=[v_x' \enspace v_y' \enspace v_z']^T$ 。那么，向量 $v_0$ 在两个坐标系之间的坐标转换关系为：
$$
\begin{bmatrix}
0 & v_1'
\end{bmatrix}
= q^{-1} \thinspace \cdot \thinspace
\begin{bmatrix}
0 & v_1
\end{bmatrix}

\thinspace \cdot \thinspace q
$$


其中
$$
q = 
\begin{bmatrix}
cos(\frac{\theta}{2}) & vsin(\frac{\theta}{2})
\end{bmatrix}
$$
其中 $v_1'=[v_x' \enspace v_y' \enspace v_z']^T$表示，原三维空间中的坐标系绕旋转轴 $v$ 旋转 $\theta$ 角度后得到了一个新的坐标系， 向量 $v_0$ 在新的坐标系中的坐标表示。



以2.为例，先做 $q_1$ 旋转，再做 $q_2$ 旋转之后向量 $x$ 在新坐标系中的表示为 $x'$ ，则有：
$$
x_{temp} &= &q_1^{-1} \thinspace \cdot x \thinspace \cdot q_1 \\
x' &= &q_2^{-1} \thinspace \cdot x_{temp} \thinspace \cdot q_2 \\
x' &= &q_2^{-1} q_1^{-1} \thinspace \cdot x \thinspace \cdot q_1 q_2  
$$
四元数 $q_1q_2$ 表示了连续两次的旋转。



**值得注意的是，上述两种四元数表示旋转的方式得到的四元数旋转关系是不同的，这是因为1.表示向量在同一个坐标系中的旋转，而2.中旋转的是坐标系**。

### 四元数微分形式推导

以无人机的姿态表示为例子，我们定义单位四元数  $q(t)$ 来表示从无人机的地理系E到机体系B的旋转关系。在 $t+\bigtriangleup t$ 时刻，旋转可表示为  $q(t + \bigtriangleup t)$；即在 $\bigtriangleup t$ 过程中，机体坐标系又经过了一个微小旋转，这个微小旋转的瞬时旋转角速度为 $w$；接着对瞬时旋转轴做单位化处理
$$
\hat{w} = w/||w||
$$
在 $\bigtriangleup t$ 转过的角度为：
$$
\bigtriangleup \theta = \bigtriangleup t ||w||
$$
则这次的微小旋转可由如下形式的单位四元数表示：
$$
\bigtriangleup q &= &cos(\frac{\bigtriangleup \theta}{2}) + \hat{w} sin(\frac{\bigtriangleup \theta}{2})  \\
&= &cos(\frac{||w||}{2} \bigtriangleup t) + \hat{w} sin(\frac{||w||}{2} \bigtriangleup t)
$$
假定地理系到机体系的旋转四元数为: $q_e^b = [q_0  \enspace q_1 \enspace q_2 \enspace q_3]^T$ 

则根据上述2.有
$$
\begin{bmatrix}
0 & ^{e}r
\end{bmatrix}
= (q_b^e)^{-1} 
\begin{bmatrix}
0 & ^br
\end{bmatrix}
q_b^e
$$
即(**注意**：此处转化为与1.相同的表示形式，但两者的物理意义不同)：
$$
\begin{bmatrix}
0 & ^{e}r
\end{bmatrix}
= (q_e^b)
\begin{bmatrix}
0 & ^br
\end{bmatrix}
(q_e^b)^{-1}
$$
那么根据上面的推导，连续两次的旋转可以表示为：

$q(t + \bigtriangleup t) = \bigtriangleup q \enspace \cdot \enspace q$

则有：
$$
q(t + \bigtriangleup t) - q(t) &= &\Big(cos(\frac{||w||}{2} \bigtriangleup t) + \hat{w} sin(\frac{||w||}{2} \bigtriangleup t) \Big) q(t) -q(t)
\\
&= &\Big(-2sin^2(\frac{||w||}{4} \bigtriangleup t)\Big)q(t) + \Big(\hat{w} sin(\frac{||w||}{2} \bigtriangleup t)\Big)q(t)
$$


略去高阶项可得：
$$
q(t + \bigtriangleup t) - q(t) = \Big(\hat{w} sin(\frac{||w||}{2} \bigtriangleup t)\Big)q(t)
$$
即:
$$
\dot{q(t)} &= &\lim_{\bigtriangleup t \to 0} \frac{q(t + \bigtriangleup t) - q(t)}{\bigtriangleup t}
\\
&= &\lim_{\bigtriangleup t \to 0} \frac{\Big(\hat{w} sin(\frac{||w||}{2} \bigtriangleup t)\Big)q(t)}{\bigtriangleup t}
\\
&= &\frac{1}{2} \thinspace \cdot \thinspace \hat{w} \thinspace \cdot \thinspace ||w|| \thinspace \cdot \thinspace q(t)
\\
&= &\frac{1}{2} \thinspace \cdot \thinspace w \thinspace \cdot \thinspace q(t)
$$
注意：此处的 $\cdot$ 表示四元数乘法； $w$ 为角速度的纯四元数表示，$w = [0 \enspace w_x \enspace w_y \enspace w_z]^T$


由于实际工程中我们都是通过固连在机体上的陀螺仪等传感器来获知机体角速度

$w^b$

它与地理坐标系下的角速度表示有如下关系

$w = q(t) w^b q(t)^{-1}$

带入上式即可得到姿态解算过程中常用的四元数的微分形式

$\dot{q(t)} = \frac{1}{2}q(t)w^b$



可以看出，通过一次四元数乘法运算便可得到四元数的微分。

上式可以写成如下的矩阵形式：
$$
\begin{bmatrix}
\dot{q_0} \\
\dot{q_1} \\
\dot{q_2} \\
\dot{q_3} 
\end{bmatrix}
= \frac{1}{2} 
\begin{bmatrix}
q_0 & -q_1 & -q_2 & -q_3 \\
q_1 &  q_0 & -q_3 &  q_2 \\
q_2 &  q_3 &  q_0 & -q_1 \\
q_3 & -q_2 &  q_1 &  q_0
\end{bmatrix}
\begin{bmatrix}
0 \\
w_x^b \\
w_y^b \\
w_z^b
\end{bmatrix}
$$
或者
$$
\begin{bmatrix}
\dot{q_0} \\
\dot{q_1} \\
\dot{q_2} \\
\dot{q_3} 
\end{bmatrix}
= \frac{1}{2} 
\begin{bmatrix}
    0 & -w_x^b & -w_y^b & -w_z^b \\
w_x^b &      0 &  w_z^b & -w_y^b \\
w_y^b & -w_z^b &      0 &  w_x^b \\
w_z^b &  w_y^b & -w_x^b &      0
\end{bmatrix}
\begin{bmatrix}
q_0 \\
q_1 \\
q_2 \\
q_3
\end{bmatrix}
$$



<font color = '#ff0000'>`QuaternionParameterization`四元数的流型:</font>

$\textcolor{#00ff00}{\boxplus(x,\Delta) = \Big[cos(|\Delta|), \frac{sin(|\Delta|)}{|\Delta|}\Delta \Big] * x}$

# 三维旋转：欧拉角、四元数、旋转矩阵、轴角之间的转换

原文：

[三维旋转：欧拉角、四元数、旋转矩阵、轴角之间的转换](https://zhuanlan.zhihu.com/p/45404840)

参考文献：

[1]Henderson, D.M.. Euler angles, quaternions, and transformation matrices for space shuttle analysis[C]//NASA, Jun 09, 1977.

[2] [https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion](https://link.zhihu.com/?target=https%3A//en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles%23Euler_Angles_to_Quaternion_Conversion)

[3] [https://en.wikipedia.org/wiki/Euler_angles](https://link.zhihu.com/?target=https%3A//en.wikipedia.org/wiki/Euler_angles)

[4] [https://en.wikipedia.org/wiki/Rotation_matrix](https://link.zhihu.com/?target=https%3A//en.wikipedia.org/wiki/Rotation_matrix)

[5] Slabaugh G G. Computing Euler angles from a rotation matrix[J]. 1999.

[6] Mike Day, Converting a Rotation Matrix to a Quaternion. [https://d3cw3dd2w32x2b.cloudfront.net](https://link.zhihu.com/?target=https%3A//d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf)

[7] Tomas K.M. , Eric H., Naty H.. Real Time Rendering 3rd Edition , p68-p69,p76-p77， 2008.

[9] [https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula](https://link.zhihu.com/?target=https%3A//en.wikipedia.org/wiki/Rodrigues%27_rotation_formula)

[10] [https://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication](https://link.zhihu.com/?target=https%3A//en.wikipedia.org/wiki/Cross_product%23Conversion_to_matrix_multiplication)

[11] [http://mathworld.wolfram.com/RodriguesRotationFormula.html](https://link.zhihu.com/?target=http%3A//mathworld.wolfram.com/RodriguesRotationFormula.html)

[12] [https://zh.wikipedia.org/wiki/%E5%9B%9B%E5%85%83%E6%95%B8](https://link.zhihu.com/?target=https%3A//zh.wikipedia.org/wiki/%E5%9B%9B%E5%85%83%E6%95%B8)

[13] [https://blog.csdn.net/silangquan/article/details/39008903](https://link.zhihu.com/?target=https%3A//blog.csdn.net/silangquan/article/details/39008903)

[14] Quaternion and Rotations, [http://run.usc.edu/cs520-s12/quaternions/quaternions-cs520.pdf](https://link.zhihu.com/?target=http%3A//run.usc.edu/cs520-s12/quaternions/quaternions-cs520.pdf)

[15] [https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation](https://link.zhihu.com/?target=https%3A//en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) 

![](assets/Quaternion1.jpg)

## **1 欧拉角(Euler Angle)与旋转矩阵(Rotation Matrix)**

### **1.1 欧拉角 ----> 旋转矩阵**

首先欧拉角旋转序列(Euler Angle Rotational Sequence)一共有12种顺规，6种绕三条轴的旋转(也叫**Tait-Bryan Angle**，XYZ,XZY,YXZ,YZX,ZXY,ZYX)，另外6种只绕两条轴的旋转(也叫**Proper Euler Angle**，XYX,YXY,XZX,ZXZ,YZY,ZYZ)。如果相邻两次旋转是绕同一条轴，例如XXY，那么其实可以坍缩成XY。那么只绕一条轴旋转就根本不够自由度就不需要说了。也就是说，一共有12种基础旋转的组合顺序，它们可以旋转出三维的所有旋转状态。所以一共是12种旋转顺规（可以表示所有旋转的集合），DirectXMath库采用的是**ZXY顺规**，分别对应着Z-Roll，X-Pitch，Y-Yaw。
$$
\begin{aligned}
R(\alpha, \beta, \gamma) = & R_{y}(\alpha) R_{x}(\beta) R_{z}(\gamma) \\
= &\left[\begin{array}{ccc}
\cos \alpha & 0 & \sin \alpha \\
0 & 1 & 0 \\
-\sin \alpha & 0 & \cos \alpha
\end{array}\right]\left[\begin{array}{ccc}
1 & 0 & 0 \\
0 & \cos \beta & -\sin \beta \\
0 & \sin \beta & \cos \beta
\end{array}\right]\left[\begin{array}{cc}
\cos \gamma & -\sin \gamma & 0 \\
\sin \gamma & \cos \gamma & 0 \\
0 & 0 & 1
\end{array}\right] \\
= &\left[\begin{array}{ccc}
c_{1} & 0 & s_{1} \\
0 & 1 & 0 \\
-s_{1} & 0 & c_{1}
\end{array}\right]\left[\begin{array}{cccc}
1 & 0 & 0 \\
0 & c_{2} & -s_{2} \\
0 & s_{2} & c_{2}
\end{array}\right]\left[\begin{array}{ccc}
c_{3} & -s_{3} & 0 \\
s_{3} & c_{3} & 0 \\
0 & 0 & 1
\end{array}\right] \\
= &\left[\begin{array}{ccc}
c_{1} & s_{1} s_{2} & s_{1} c_{2} \\
0 & c_{2} & -s_{2} \\
-s_{1} & c_{1} s_{2} & c_{1} c_{2}
\end{array}\right]\left[\begin{array}{ccc}
c_{3} & -s_{3} & 0 \\
s_{3} & c_{3} & 0 \\
0 & 0 & 1
\end{array}\right] \\
= &\left[\begin{array}{cccc}
c_{1} c_{3}+s_{1} s_{2} s_{3} & c_{3} s_{1} s_{2}-c_{1} s_{3} & c_{2} s_{1} \\
c_{2} s_{3} & c_{2} c_{3} & -s_{2} \\
c_{1} s_{2} s_{3}-s_{1} c_{3} & s_{1} s_{3}+c_{1} c_{3} s_{2} & c_{1} c_{2}
\end{array}\right]

\\
\\
其中:
\\
\\
c_{1} = &\cos (\alpha)=\cos \left(Y_{y a w}\right), s_{1}=\sin \alpha= \sin \left(Y_{y a w}\right) \\
c_{2} = &\cos (\beta)=\cos \left(X_{p i t c h}\right), s_{2}=\sin \beta=\sin \left(X_{p i t c h}\right) \\
c_{3} = &\cos (\gamma)=\cos \left(Z_{\text {roll }}\right), s_{3}=\sin \gamma=\sin \left(Z_{\text {roll }}\right)
\end{aligned}
$$
这里要提一嘴**内旋(intrinsic rotation)**和外旋**(extrinsic rotation)：内旋的每个elemental绕的是object space basis的轴，外旋的每个elemental rotation绕的是world space（惯性系）basis的轴**。上面的矩阵之所以是这么个顺序，是因为：

1. 采用了ZXY顺规
2. 采用列主向量
3. 采用外旋的约定（毕竟在图形学里常用euler angle去表示物体相对于惯性系的旋转姿态）

那么给定一个向量 $v$ ，上面的矩阵左乘以这个向量，就是对它做一次主动旋转，得到变换后的向量 $v^{'}$：

$v^{'} = Rv$ 

在这规定下，上面的矩阵就是先roll Z，再pitch X，再yaw Y。

上面的欧拉角--->矩阵的结果与维基百科Euler Angles[3] ![[公式]](https://www.zhihu.com/equation?tex=Y_1X_2Z_3) 给出的结果一致，那应该稳了：

![](assets/Quaternion2.jpg)

### **1.2 旋转矩阵----> 欧拉角**

参考一篇NASA的关于姿态描述的技术报告[1]的Appendix-A6和[5]，我们可以用**旋转矩阵元素的相乘、相除、反三角函数等操作去“凑”出欧拉角**。[5]给出了从XYZ顺规提取欧拉角的方法、步骤、思路，[1]则给出了全部12种顺规的欧拉角提取公式，但是没有给一些细节注意事项。所以总结一下，根据[1]、[5]、[7]《Real Time Rendering 3rd  Edition》4.2.2和自己的推导，从ZXY顺规旋转矩阵提取欧拉角的公式是（[1]原文下标似乎有点小问题）：

- Y axis yaw  angle:

  $\alpha=\operatorname{atan} 2(\sin \alpha \cos \beta, \cos \alpha \cos \beta)=\operatorname{atan} 2\left(m_{13}, m_{33}\right) $

- X axis pitch angle:

  $\beta=\arcsin (\sin \beta)=\arcsin \left(-m_{23}\right) $

- Z axis roll angle:

  $\gamma=\operatorname{atan} 2(\cos \beta \sin \gamma, \cos \beta \cos \gamma)=\operatorname{atan} 2\left(m_{21}, m_{22}\right)$

注意到一点，注意到矩阵的每一个元素都是pitch angle $\beta$  的函数…所以当 $m_{23} = -sin\beta = \pm 1$  即 $cos \beta = 0$  的时候，这时候其他的欧拉角提取表达式就凉凉了（分子分母都是0, arctan和atan2都没有意义了）….其实pitch angle $\beta = \pm \pi /2$ 恰好就是Gimbal Lock的位置。在Gimbal Lock的时候，旋转矩阵会退化为：
$$
\begin{aligned}
R(\alpha, \beta, \gamma) &=\left[\begin{array}{ccc}
c_{1} c_{3} \pm s_{1} s_{3} & \pm c_{3} s_{1}-c_{1} s_{3} & 0 \\
0 & 0 & \pm 1 \\
\pm c_{1} s_{3}-s_{1} c_{3} & s_{1} s_{3} \pm c_{1} c_{3} & 0
\end{array}\right] \\
&=\left[\begin{array}{ccc}
\cos (\alpha \pm \gamma) & \sin (\alpha \pm \gamma) & 0 \\
0 & 0 & \pm 1 \\
-\sin (\alpha \pm \gamma) & -\cos (\alpha \pm \gamma) & 0
\end{array}\right]
\end{aligned}
$$
那么要进一步处理万向节死锁的corner case就需要分两种情况：

- $\beta = - \pi /2$ ，此时 $sin\beta = - 1, cos \beta = 0$
  $$
  \begin{aligned}
  R(\alpha, \beta, \gamma) &=\left[\begin{array}{ccc}
  c_{1} c_{3}-s_{1} s_{3} & -c_{3} s_{1}-c_{1} s_{3} & 0 \\
  0 & 0 & \pm 1 \\
  -c_{1} s_{3}-s_{1} c_{3} & s_{1} s_{3}-c_{1} c_{3} & 0
  \end{array}\right] \\
  &=\left[\begin{array}{ccc}
  \cos (\alpha+\gamma) & -\sin (\alpha+\gamma) & 0 \\
  0 & 0 & 1 \\
  -\sin (\alpha+\gamma) & -\cos (\alpha+\gamma) & 0
  \end{array}\right] \\
  \Rightarrow \alpha+\gamma &=\operatorname{atan} 2\left(-m_{12}, m_{11}\right)
  \end{aligned}
  $$
  要给 $\alpha$ 或者 $\gamma$ 其中一个欧拉角赋值，另外一个就按等式计算出来。

- $\beta =  \pi /2$ ，此时 $sin\beta = 1, cos \beta = 0$
  $$
  \begin{aligned}
  R(\alpha, \beta, \gamma) &=\left[\begin{array}{ccc}
  c_{1} c_{3}+s_{1} s_{3} & c_{3} s_{1}-c_{1} s_{3} & 0 \\
  0 & 0 & -1 \\
  c_{1} s_{3}-s_{1} c_{3} & s_{1} s_{3}+c_{1} c_{3} & 0
  \end{array}\right] \\
  &=\left[\begin{array}{ccc}
  \cos (\alpha-\gamma) & \sin (\alpha-\gamma) & 0 \\
  0 & 0 & -1 \\
  -\sin (\alpha-\gamma) & \cos (\alpha-\gamma) & 0
  \end{array}\right] \\
  \Rightarrow \alpha-\gamma &=\operatorname{atan} 2\left(m_{12}, m_{11}\right)
  \end{aligned}
  $$
  

 同样的，要给 $\alpha$ 或者 $\gamma$ 其中一个欧拉角赋值，另外一个就按等式计算出来。

从旋转矩阵提取欧拉角的公式跟欧拉角顺规的选取有关，因为旋转矩阵的元素会略有不同，但是思路都是一样的，就是**根据旋转矩阵的解析表达式+反三角函数凑出来**23333。

## **2 四元数(Quaternion)与旋转矩阵**

### **2.1 四元数---->旋转矩阵**

众所周知的是，欧拉旋转是有万向节死锁(Gimbal Lock)的问题的。幸好我们有四元数(Quaternion)这种数学工具可以避免这个情况。一般来说，我们都会用单位四元数 $\mathbf{q} = w + x \pmb{\mathbf{i}} + y\pmb{\mathbf{j}} + z\pmb{\mathbf{k}}$ 来表示旋转，其中 $\|\mathbf{q}\| = x^2 + y^2 + z^2 + w^2$ 。那么给定一个单位四元数，可以构造旋转矩阵(column major)[1][4][8][14][15]：
$$
R(q)=\left[\begin{array}{ccc}
1-2 y^{2}-2 z^{2} & 2 x y-2 z w & 2 x z+2 y w \\
2 x y+2 z w & 1-2 x^{2}-2 z^{2} & 2 y z-2 x w \\
2 x z-2 y w & 2 y z+2 x w & 1-2 x^{2}-2 y^{2}
\end{array}\right]
$$
这个四元数构造的大概思路就是把**四元数的旋转操作写成矩阵形式**（注：给定一个用于旋转的单位四元数  $\mathbf{q} = w + x \pmb{\mathbf{i}} + y\pmb{\mathbf{j}} + z\pmb{\mathbf{k}}$ 和被旋转的三维向量 $\mathbf{v}$ ，那么要直接用四元数旋转这个向量，则我们首先要构造一个纯四元数 $\mathbf{p} = (\mathbf{v},0)$ ，设旋转后的向量为 $\mathbf{v}^{'}$  ，旋转后的向量构造的纯四元数为 $\mathbf{p}^{'} = (\mathbf{v}^{'},0)$ ，那么 $\mathcal{p}^{'} = \mathbf{q} \mathbf{p} \mathbf{q}^{-1}$ ）。因为是用四元数来构造矩阵的，所以这个矩阵构造公式就没有欧拉角顺规的说法了。



### **2.2 旋转矩阵---->四元数**

那第一步肯定是判断3x3矩阵是一个正交矩阵啦（满足 $\mathbf{R} \mathbf{R}^T = \mathbf{R}^T \mathbf{R} = I$ ）。那么如果这个矩阵已经是一个合法的旋转矩阵了，要从旋转矩阵里提取四元数，也是可以像提取欧拉角那样，**用参数化过的矩阵的表达式凑出来**。参考[8]《Real Time Rendering 3rd edition》Chapter4的思路，我们观察一下用四元数分量进行参数化的矩阵 $\mathbf{R}(q)$ ，然后经过一顿操作，我们发现：
$$
\begin{array}{l}
m_{32}-m_{23}=(2 y z+2 x w)-(2 y z-2 x w)=4 x w \\
m_{13}-m_{31}=(2 x z+2 y w)-(2 x z-2 y w)=4 y w \\
m_{21}-m_{12}=(2 x y+2 z w)-(2 x y-2 z w)=4 z w
\end{array}
$$
于是我们再凑出个实分量 $w$ ，就可以把四元数四个分量都用矩阵元素表示出来了。于是我们又机智地发现了一个等式：
$$
\begin{aligned}
\operatorname{tr}(R(q)) &=m_{11}+m_{22}+m_{33} \\
&=3-4\left(x^{2}+y^{2}+z^{2}\right) \\
&=4\left(1-\left(x^{2}+y^{2}+z^{2}\right)\right)-1 \\
&=4 w^{2}-1
\end{aligned}
$$


其中 $\operatorname{tr}(M)$ 是矩阵 $M$ 的迹(trace)，也就是矩阵对角元素的和。因为这里用的是3x3矩阵，跟其他资料里面的表示有一点不同。所以我们可以把四元数的四个分量都用矩阵元素凑出来了：
$$
\begin{aligned}
w &=\frac{\sqrt{(\operatorname{tr}(R)+1}}{2} \\
x &=\frac{m_{32}-m_{23}}{4 w} \\
y &=\frac{m_{13}-m_{31}}{4 w} \\
z &=\frac{m_{21}-m_{12}}{4 w}
\end{aligned}
$$
有一点《Real Time Rendering》提到的， ![[公式]](https://www.zhihu.com/equation?tex=w) 绝对值比较小的时候，可能会出现数值不稳定的情况，那么想要数值稳定的话就得用一种不用除法的方式来凑，在这不展开了，可以看一下RTR 2333。

## **3 欧拉角与四元数**

### **3.1 欧拉角---->四元数**

首先提一下四元数的乘积：
$$
\begin{aligned}
\mathbf{p} &=w_{1}+\mathbf{v}_{1}=w_{1}+x_{1} \mathbf{i}+y_{1} \mathbf{j}+z_{1} \mathbf{k} \\
\mathbf{q} &=w_{2}+\mathbf{v}_{2}=w_{2}+x_{2} \mathbf{i}+y_{2} \mathbf{j}+z_{2} \mathbf{k} \\
\Rightarrow \mathbf{p q} &=w_{1} w_{2}-\mathbf{v}_{1} \cdot \mathbf{v}_{2}+w_{2} \mathbf{v}_{1}+w_{1} \mathbf{v}_{2}+\mathbf{v}_{1} \times \mathbf{v}_{2} \\
&=\left[\begin{array}{l}
x_{1} w_{2}+w_{1} x_{2}+y_{1} z_{2}-z_{1} y_{2} \\
y_{1} w_{2}+w_{1} y_{2}+z_{1} x_{2}-x_{1} z_{2} \\
z_{1} w_{2}+w_{1} z_{2}+x_{1} y_{2}-y_{1} x_{2} \\
w_{1} w_{2}-x_{1} x_{2}-y_{1} y_{2}-z_{1} z_{2}
\end{array}\right]
\end{aligned}
$$
参考维基百科[2]的思路，欧拉角构造四元数，跟欧拉角构造旋转矩阵一样，就是**把三个基础旋转Elemental Rotation组合在一起。**

那么用于旋转的四元数 $q(x,y,z,w)$ 的表达式是：
$$
\begin{aligned}
\mathbf{q}_{\mathrm{IB}} &=\left[\begin{array}{c}
\cos (\psi / 2) \\
0 \\
0 \\
\sin (\psi / 2)
\end{array}\right]\left[\begin{array}{c}
\cos (\theta / 2) \\
0 \\
\sin (\theta / 2) \\
0
\end{array}\right]\left[\begin{array}{c}
\cos (\phi / 2) \\
\sin (\phi / 2) \\
0 \\
0
\end{array}\right] \\
&=\left[\begin{array}{l}
\cos (\phi / 2) \cos (\theta / 2) \cos (\psi / 2)+\sin (\phi / 2) \sin (\theta / 2) \sin (\psi / 2) \\
\sin (\phi / 2) \cos (\theta / 2) \cos (\psi / 2)-\cos (\phi / 2) \sin (\theta / 2) \sin (\psi / 2) \\
\cos (\phi / 2) \sin (\theta / 2) \cos (\psi / 2)+\sin (\phi / 2) \cos (\theta / 2) \sin (\psi / 2) \\
\cos (\phi / 2) \cos (\theta / 2) \sin (\psi / 2)-\sin (\phi / 2) \sin (\theta / 2) \cos (\psi / 2)
\end{array}\right]
\end{aligned}
$$


### **3.2 四元数---->欧拉角**

本来我以为，从四元数提取欧拉角的思路可以跟旋转矩阵提取欧拉角类似，也是用四元数的元素运算和反三角函数凑出公式来。后来我发现这简直就是一个极其硬核的任务，展开之后每一项都是六次多项式，画面有一丢暴力且少儿不宜，直接强行凑的话画风大概是这样：
$$
\begin{array}{c}
x w &= &\left(c_{1} s_{2} c_{3}+s_{1} c_{2} s_{3}\right)\left(c_{1} c_{2} c_{3}+s_{1} s_{2} s_{3}\right) \\
&= &c_{1}^{2} c_{2} s_{2} c_{3}^{2}+c_{1} s_{2} s_{2}^{2} c_{3} s_{3}+c_{1} s_{1} c_{2}^{2} c_{3} s_{3}+s_{1}^{2} c_{2} s_{2} s_{3}^{2} \\
y z &= &\left(s_{1} c_{2} c_{3}-c_{1} s_{2} s_{3}\right)\left(-s_{1} s_{2} c_{3}+c_{1} c_{2} s_{3}\right) \\
&= &-s_{1}^{2} c_{2} s_{2} c_{3}^{2}+c_{1} s_{1} c_{2}^{2} c_{3} s_{3}+c_{1} s_{1} s_{2}^{2} c_{3} s_{3}-c_{1}^{2} c_{2} s_{2} s_{3}^{2} \\
\Rightarrow 2(x w-y z) &= &2\left(c_{1}^{2} c_{2} s_{2}+s_{1}^{2} c_{2} s_{2}\right)=2 c_{2} s_{2}=\sin \beta \\
\Rightarrow \beta &= &\arcsin (2(x w-y z))=\arcsin \left(-m_{23}\right)
\end{array}
$$
这个结果跟欧拉角参数化的旋转矩阵的  $m_{23} = -sin\beta = 2yz - 2xw$  的表达式是吻合的。但这还只是最好凑的那一个，惹不起惹不起。所以舒服的思路还是**四元数-->旋转矩阵-->欧拉角**，想一步到位的话，把四元数分量参数化的旋转矩阵、欧拉角参数化的旋转矩阵结合在一起，参考下旋转矩阵转欧拉角的方法，替换下元素就完事了。这里就不把公式展开了，因为四元数直接转欧拉角 跟 旋转矩阵转欧拉角一样，依旧是要处理gimbal lock的corner case，还是那么麻烦，所以这里先鸽了23333

## **4 轴-角(Axis-Angle)**

### **4.1 轴角---->四元数**

轴-角(Axis-Angle)顾名思义就是绕某条单位轴旋转一定角度，从这个意义上看，它构造四元数是非常舒服的，毕竟直观的几何意义有一点点类似，绕单位轴 $u$ 旋转 $\theta$ 的四元数是：
$$
\pmb{\mathbf{q}}(w,\pmb{\mathbf{v}}) = (cos \frac{\theta}{2}, \pmb{\mathbf{u}} sin \frac{\theta}{2} )
$$


### **4.2 轴角---->旋转矩阵**

Axis Angle转Rotation Matrix可以从[9]罗德里格斯旋转公式Rodrigues Rotation Formula开始推导。

[Rodrigues' rotation formula](https://link.zhihu.com/?target=https%3A//en.wikipedia.org/wiki/Rodrigues%27_rotation_formula)

设 $\mathbf{v}$ 是我们要旋转的单位向量，旋转轴为 $\mathbf{k}$ ，  $\mathbf{v}$  绕 $\mathbf{k}$  旋转角度 $\theta$ ，那么旋转后的向量为：
$$
\mathbf{v}_{r o t}=\mathbf{v} \cos \theta+(\mathbf{k} \times \mathbf{v}) \sin \theta+\mathbf{k}(\mathbf{k} \cdot \mathbf{v})(1-\cos \theta)
$$
这个公式的推导思路是这样子的，我们先对向量 $\mathbf{v}$ 进行正交分解，分解成投影到旋转轴    $\mathbf{k}$  的分量和垂直于   $\mathbf{k}$  的分量：
$$
\mathbf{v} = \mathbf{v}_{\bot} + \mathbf{v}_{\parallel}
$$
其中：
$$
\mathbf{v}_{\parallel} &=  &(\mathbf{v} \cdot \mathbf{k})\mathbf{k}
\\
\mathbf{v}_{\bot} &=  &-\mathbf{k} \times (\mathbf{k} \times \mathbf{v})
$$
![](assets/Quaternion3.png)

于是绕 $\mathbf{k}$ 旋转向量 $\mathbf{v}$  其实就是把上面**正交投影后的向量分别旋转之后再加起来**。那么很明显的，投影到旋转轴上的部分 $\mathbf{v}_{\parallel}$ 都跟旋转轴共享了，那么自然旋转之后的结果就没有变化了，于是我们只需要旋转和旋转轴垂直的部分  $\mathbf{v}_{\bot}$  。那么这个 $\mathbf{v}_{\bot}$  旋转后的表达式就是：
$$
\mathbf{v}_{\perp-\text { rotated }}=\cos \theta \mathbf{v}_{\perp}+\sin \theta \mathbf{k} \times \mathbf{v}
$$
然后我们不按wikipedia里面坑爹的、不考虑下文的变形，自己推一波：
$$
\begin{aligned}
\mathbf{v}_{\text {rotated }} &=\mathbf{v}_{\perp-\text { rotated }}+\mathbf{v}_{\|-\text {rotated }} \\
&=\cos \theta \mathbf{v}_{\perp}+\sin \theta \mathbf{k} \times \mathbf{v}+\mathbf{v}_{\|} \\
&=-\cos \theta \mathbf{k} \times(\mathbf{k} \times \mathbf{v})+\sin \theta \mathbf{k} \times \mathbf{v}+\left(\mathbf{v}-\mathbf{v}_{\perp}\right) \\
&=-\cos \theta \mathbf{k} \times(\mathbf{k} \times \mathbf{v})+\sin \theta \mathbf{k} \times \mathbf{v}+(\mathbf{v}-(-\mathbf{k} \times(\mathbf{k} \times \mathbf{v}))) \\
&=\mathbf{v}+(1-\cos \theta) \mathbf{k} \times(\mathbf{k} \times \mathbf{v})+\sin \theta \mathbf{k} \times \mathbf{v}
\end{aligned}
$$
这里我们把旋转后向量的表达式**变形得只剩下叉积(cross product)，去掉点积(dot product)了**，这样子我们才可以把这个绕轴旋转的表达式写成矩阵形式。怎么写呢？首先叉积可以写成矩阵形式：
$$
\begin{array}{c}
\mathbf{M}=\left[\begin{array}{ccc}
0 & -k_{z} & k_{y} \\
k_{z} & 0 & -k_{x} \\
-k_{y} & k_{x} & 0
\end{array}\right] \\
\mathbf{M v}=\mathbf{k} \times \mathbf{v}
\end{array}
$$
于是罗德里格斯旋转公式的变换就可以写成矩阵形式：
$$
R(\mathbf{k}, \theta)=\mathbf{I}+\sin \theta \mathbf{M}+(1-\cos \theta) \mathbf{M}^{2}
$$
展开之后就是：
$$
R(\mathbf{k}, \theta) = 
	\begin{bmatrix}
   		\cos \theta+k_{x}^{2}(1-\cos \theta) & -\sin \theta k_{z}+(1-\cos \theta) k_{x} k_{y} & \sin \theta k_{y}+(1-\cos \theta) k_{x} k_{z} \\
		\sin \theta k_{z}+(1-\cos \theta) k_{x} k_{y} & \cos \theta+k_{y}^{2}(1-\cos \theta) & -\sin \theta k_{x}+(1-\cos \theta) k_{y} k_{z} \\
		-\sin \theta k_{y}+(1-\cos \theta) k_{x} k_{z} & \sin \theta k_{x}+(1-\cos \theta) k_{y} k_{z} & \cos \theta+k_{z}^{2}(1-\cos \theta)
	\end{bmatrix}
$$

# IMU预积分原理

以VINS-Mono为例

原文：

[如何将大象放进去：SLAM技术之IMU预积分原理[1]](https://zhuanlan.zhihu.com/p/385180002)

## **1 前言**

