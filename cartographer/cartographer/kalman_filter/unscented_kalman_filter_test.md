
```c++
Eigen::Matrix<double, 1, 1> Scalar(double value) {
  return value * Eigen::Matrix<double, 1, 1>::Identity();
}

Eigen::Matrix<double, 2, 1> g(const Eigen::Matrix<double, 2, 1>& state) {
  Eigen::Matrix<double, 2, 1> new_state;
  new_state << state[0], state[0];
  return new_state;
}

Eigen::Matrix<double, 1, 1> h(const Eigen::Matrix<double, 2, 1>& state) {
  return Scalar(state[0]) - Scalar(5.);
}


TEST(KalmanFilterTest, testObserve) {
  UnscentedKalmanFilter<double, 2> filter(GaussianDistribution<double, 2>(
      Eigen::Vector2d(0., 42.), 10. * Eigen::Matrix2d::Identity()));
  for (int i = 0; i < 3; ++i) {
    filter.Predict(
        g, GaussianDistribution<double, 2>(Eigen::Vector2d(0., 0.),
                                           Eigen::Matrix2d::Identity() * 1e-9));
    filter.Observe<1>(
        h, GaussianDistribution<double, 1>(Scalar(0.), Scalar(1e-2)));
  }
```

`TEST(KalmanFilterTest, testObserve)`第一次运行计算过程：

```c++
  全局变量：
  constexpr static FloatType kAlpha = 1e-3;
  constexpr static FloatType kKappa = 0.;
  constexpr static FloatType kBeta = 2.;
  constexpr static FloatType kLambda = sqr(kAlpha) * (N + kKappa) - N;
  constexpr static FloatType kMeanWeight0 = kLambda / (N + kLambda);
  constexpr static FloatType kCovWeight0 =
      kLambda / (N + kLambda) + (1. - sqr(kAlpha) + kBeta);
  constexpr static FloatType kMeanWeightI = 1. / (2. * (N + kLambda));
  constexpr static FloatType kCovWeightI = kMeanWeightI;
  局部变量：
  const FloatType kSqrtNPlusLambda = std::sqrt(N + kLambda);
```

$$
参数: 
\begin{cases}
kSqrtNPlusLambda = \sqrt{2 * 0.001^2} \thickapprox  0.00141421\\
kMeanWeight0 = \frac{2 * 0.001^2 - 2}{2 * 0.001^2} = -999999 \\
kCovWeight0 = \frac{2 * 0.001^2 - 2}{2 * 0.001^2} + 1 - 0.001^2 + 2 \thickapprox -999996\\
kMeanWeightI = kCovWeightI = \frac{1}{2 * 2 * 0.001^2} = 250000
\end{cases}
$$



```shell
I1202 16:43:07.335314 13739 unscented_kalman_filter.h:213] kSqrtNPlusLambda: 0.00141421
I1202 16:43:07.335325 13739 unscented_kalman_filter.h:214] kMeanWeight0: -999999
I1202 16:43:07.335335 13739 unscented_kalman_filter.h:215] kCovWeight0: -999996
I1202 16:43:07.335343 13739 unscented_kalman_filter.h:216] kCovWeightI: 250000
```



`Predict`:
$$
mu = \overbrace{\begin{bmatrix}
0\\
42
\end{bmatrix}}^{\text{begin}}
\\
\\
sqrt_{sigma} = \overbrace{\begin{bmatrix}
\sqrt{10} & 0\\
0 & \sqrt{10}
\end{bmatrix}}^{\text{begin}}\\
\\
\overbrace{\begin{bmatrix}
0\\
0
\end{bmatrix} \begin{bmatrix}
\sqrt{20} * 10^{-3}\\
\sqrt{20} * 10^{-3}
\end{bmatrix} \begin{bmatrix}
-\sqrt{20} * 10^{-3}\\
-\sqrt{20} * 10^{-3}
\end{bmatrix} \begin{bmatrix}
0\\
0
\end{bmatrix} \begin{bmatrix}
0\\
0
\end{bmatrix}}^{\text{Y}}\\
\\
new_{mu} = \overbrace{\begin{bmatrix}
0\\
0
\end{bmatrix}}^{\text{new-mu}} + \overbrace{\begin{bmatrix}
0\\
0
\end{bmatrix}}^{\text{epsilon.GetMean()}} = \overbrace{\begin{bmatrix}
0\\
0
\end{bmatrix}}^{\text{predict}}
\\
\\计算2*N + 1次(此处N为2)：
\\
new_{sigma} = \overbrace{\begin{bmatrix}
0 & 0\\
0 & 0
\end{bmatrix}}^{\text{1}} + \overbrace{\begin{bmatrix}
5 & 5\\
5 & 5
\end{bmatrix}}^{\text{2}} + 
\overbrace{\begin{bmatrix}
5 & 5\\
5 & 5
\end{bmatrix}}^{\text{3}} + 
\overbrace{\begin{bmatrix}
0 & 0\\
0 & 0
\end{bmatrix}}^{\text{4}} + 
\overbrace{\begin{bmatrix}
0 & 0\\
0 & 0
\end{bmatrix}}^{\text{5}} = 
\overbrace{\begin{bmatrix}
10 & 10\\
10 & 10
\end{bmatrix}}^{\text{end}}\\
\\
new_{sigma} = new_{sigma} + \overbrace{\begin{bmatrix}
1e-9 & 0\\
0 & 1e-9
\end{bmatrix}}^{\text{epsilon.GetCovariance()}} \thickapprox \overbrace{\begin{bmatrix}
10 & 10\\
10 & 10
\end{bmatrix}}^{\text{predict}}
$$


`Observe`:
$$
mu = \overbrace{\begin{bmatrix}
0\\
0
\end{bmatrix}}^{\text{predict}}
\\
sqrt_{sigma} = \overbrace{\begin{bmatrix}
\sqrt{5} & \sqrt{5}\\
\sqrt{5} & \sqrt{5}
\end{bmatrix}}^{\text{predict}}\\
\\
\overbrace{\begin{bmatrix}
0\\
0
\end{bmatrix} \begin{bmatrix}
\sqrt{10} * 10^{-3}\\
\sqrt{10} * 10^{-3}
\end{bmatrix} \begin{bmatrix}
-\sqrt{10} * 10^{-3}\\
-\sqrt{10} * 10^{-3}
\end{bmatrix} \begin{bmatrix}
\sqrt{10} * 10^{-3}\\
\sqrt{10} * 10^{-3}
\end{bmatrix} \begin{bmatrix}
-\sqrt{10} * 10^{-3}\\
-\sqrt{10} * 10^{-3}
\end{bmatrix}}^{\text{W}}
\\
\\
\overbrace{\begin{bmatrix}
-5
\end{bmatrix} \begin{bmatrix}
\sqrt{10} * 10^{-3} - 5
\end{bmatrix} \begin{bmatrix}
-\sqrt{10} * 10^{-3} - 5
\end{bmatrix} \begin{bmatrix}
\sqrt{10} * 10^{-3} - 5
\end{bmatrix} \begin{bmatrix}
-\sqrt{10} * 10^{-3} - 5
\end{bmatrix}}^{\text{Z}}
\\
\\
Z_{hat}  = -5
\\
\\
加上噪声，新的传感器不确定性:S = 10 + 0.01 = 10.01  
\\
\\
sigma_{bar-xz} = \overbrace{\begin{bmatrix}
0\\
0 
\end{bmatrix}}^{\text{1}} + \overbrace{\begin{bmatrix}
2.5\\
2.5
\end{bmatrix}}^{\text{2}} + 
\overbrace{\begin{bmatrix}
2.5\\
2.5
\end{bmatrix}}^{\text{3}} + 
\overbrace{\begin{bmatrix}
2.5\\
2.5
\end{bmatrix}}^{\text{4}} + 
\overbrace{\begin{bmatrix}
2.5\\
2.5
\end{bmatrix}}^{\text{5}} = 
\overbrace{\begin{bmatrix}
10\\
10
\end{bmatrix}}^{\text{end}}\\
\\
kalman_{gain} = \begin{bmatrix}
\frac{10}{10.01}\\
\frac{10}{10.01}
\end{bmatrix}\\
\\
new_{mu} = \overbrace{\begin{bmatrix}
0\\
0
\end{bmatrix}}^{\text{predict}} + \overbrace{\begin{bmatrix}
\frac{10}{10.01}\\
\frac{10}{10.01}
\end{bmatrix}}^{\text{kalman-gain}} * \overbrace{\begin{bmatrix}
-5
\end{bmatrix}}^{\text{Z-hat}} \thickapprox  \overbrace{\begin{bmatrix}
4.995\\
4.995
\end{bmatrix}}^{\text{observe}}\\
\\
new_{sigma} = \overbrace{\begin{bmatrix}
10 & 10\\
10 & 10
\end{bmatrix}}^{\text{predict}} - \overbrace{\begin{bmatrix}
\frac{10}{10.01}\\
\frac{10}{10.01}
\end{bmatrix}}^{\text{kalman-gain}} * \overbrace{\begin{bmatrix}
10.01
\end{bmatrix}}^{\text{S}} * \overbrace{\begin{bmatrix}
\frac{10}{10.01} \frac{10}{10.01}
\end{bmatrix}}^{\text{kalman-gain-transpose}}  \thickapprox \overbrace{\begin{bmatrix}
0.00999001 & 0.00999001\\
0.00999001 & 0.00999001
\end{bmatrix}}^{\text{observe}}
$$

$$

$$





