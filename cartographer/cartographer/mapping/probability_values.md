https://zhuanlan.zhihu.com/p/49030629

1. 占据栅格图(Occupancy Grid Map)更新公式推导

对于栅格化地图中的一个cell,或者说一个pixel,它的状态s可能有两个状态：
$s = 1$: 表示该点被occupied的

$s = 0$ : 表示该点是Free的

通常，我们用一个概率 $\mathcal{p}(\mathcal{s}=1)$来表示该pixel被占据(occupied)的概率。那么，该点Free的概率就是  $\mathcal{p}(\mathcal{s}=0)=1-\mathcal{p}(\mathcal{s}=1)$. 

但是，对于同一个点用两个值表达比较麻烦，而且也不便于概率值的更新。所以，这里会引入一个新的变量Odd(s)——用两个概率的比值表示：
$$
\operatorname{Odd}(s)=\frac{p(s=1)}{p(s=0)}
$$
这种情况下，Odd(s)值等于1时，表征一半对一半，该点被occupied和free的概率各为0.5；

如果Odd(s)值大于1，表征该点被occupied的概率更大；Odd(s)越大，occupied的概率越大。范围为1～+ ![[公式]](https://www.zhihu.com/equation?tex=%5Cinfty) ；

如果Odd(s)值小于1，表征该点free的概率更大；Odd(s)越小，free的概率越大。范围为0~1。



那么对于这个cell, 新来了一个测量值 z——即传感器测得该点为occupied或Free, 那么我们如何根据该测量值来更新该点的状态呢？

分为两种情况讨论：

Case 1：假设之前我们对该点没有任何信息，那么我们会直接把测量值赋给该点： 
$$
\begin{array}{l}
p(s=1)=z(s=1) \\
\operatorname{odd}(s)=\frac{p(s=1)}{p(s=0)}
\end{array}
$$
Case 2: 假设测量值来之前，该点已有一个测量值odd(s).

我们要求的实际上是一个条件概率：
$$
\operatorname{odd}(s \mid z)=\frac{p(s=1 \mid z)}{p(s=0 \mid z)}
$$
即，在存在观测值z 的条件下，该点的实际状态为 $s=1$的概率与该点的实际状态为 $s = 0$的比值.

那么，根据贝叶斯的条件概率公式，我们有：
$$
\begin{array}{c}
p(s=1 \mid z)=\frac{p(z \mid s=1) p(s=1)}{p(z)} \\
p(s=0 \mid z)=\frac{p(z \mid s=0) p(s=0)}{p(z)}
\end{array}
$$
所以，我们可以得到：
$$
\operatorname{odd}(s \mid z)=\frac{p(z \mid s=1) p(s=1)}{p(z \mid s=0) p(s=0)}=\frac{p(z \mid s=1)}{p(z \mid s=0)} \cdot \frac{p(s=1)}{p(s=0)}
$$
其中， $\frac{p(s=1)}{p(s=0)}$即为上一次的状态，即odd(s). 所以，我们可以得到更新模型为：
$$
\operatorname{odd}(s \mid z)=\frac{p(z \mid s=1)}{p(z \mid s=0)} \operatorname{odd}(s)
$$
其中，  $\frac{p(z \mid s=1)}{p(z \mid s=0)}$表征的是传感器的测量模型。  $p(z \mid s=1)$的含义是在实际状态为occupied的条件下，测量结果是z的概率。同样， $p(z \mid s=0)$的含义是在实际状态为free的条件下，测量结果是z 的概率。这两个值是由传感器的测量精度、可靠性等因素决定。当一个传感器制造完成后，该值是定值，不会随时间、位置等因素改变(当然，如果你的传感器是随着时间、位置等因素而变化的，这不在我们讨论范围内，你需要一个新的传感器模型。



为了更方便计算，可以对两边取log，可以得到：
$$
\log (\operatorname{odd}(s \mid z))=\log ( \frac{p(z \mid s=1)}{p(z \mid s=0)}) + \log(\operatorname{odd}(s))
$$
那么，根据测量结果z是occupied或free的情况，我们可以预先求出来两个值：
$$
\begin{array}{c}
S_{\text {free }}=\log \left(\frac{p(z=0 \mid s=1)}{p(z=0 \mid s=0)}\right) \\
S_{\text {occupied }}=\log \left(\frac{p(z=1 \mid s=1)}{p(z=1 \mid s=0)}\right)
\end{array}
$$
则：
$$
S_{z}=\log (\frac{p(z \mid s=1)}{p(z \mid s=0)})=\left\{\begin{array}{ll}
S_{\text {free }} & z=0 \\
S_{\text {occupied }} & z=1
\end{array}\right.
$$
记：
$$
S^+ = \operatorname{odd}(s \mid z), \qquad S^- = \operatorname{odd}(s)
$$


则我们的更新模型就可以得到:
$$
S^+ = S^- + S_z
$$
