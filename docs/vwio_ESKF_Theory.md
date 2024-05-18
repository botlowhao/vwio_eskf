# vwio ESKF Theory

## 1 Kalman Filter Formula Review

### 1.1 Construct Predict Model And Mesure Model Of System

#### 1) Predict Model Equations
$$
x_t = F_t x_{t-1} + B_t u_{t-1} + w_k
$$

$$
w_k \sim \mathcal{N}(0, Q)
$$

#### 2) Mesure Model Equations
$$
z_t = H x_t + v_k
$$

$$
v_k \sim \mathcal{N}(0, R)
$$


### 1.2 Predict Step

#### 1) Priori Estimate
$$
\hat{x}_t^- = F_t \hat{x}_{t-1} + B_t u_{t-1}
$$

#### 2) Priori Estimate Covariance
$$
P_t^- = F_t P_{t-1} F_t^T + Q
$$

### 1.3 Status Update Step

#### 1) Kalman Gein
$$
K_t = \frac{P_t^- H^T}{H P_t^- H^T + R}
$$

#### 2) Posterior Estimate
$$
\hat{x}_t = \hat{x}_t^- + K_t(z_t - H \hat{x}_t^-)
$$

#### 3) Posterior Estimate Covariance
$$
P_t = (I - K_t H)P_t^-
$$

## 2 Analysis Of Algorithms ESKF For WO & IMU & VO

### 2.1 ESKF Init Step

1) Set intial position/velocity/posture value from WO(Wheel Odometry) and timestamp.



2) Initialize the state covariance matrix and set the noise for the position/velocity/posture estimation of the robot's initial state.



### 2.2 ESKF Predict Step

#### 2.2.1 Construct Predict Model Of System

1) Set the ESKF truth state to the formula (1)

$$
\mathbf{x}_t = [\mathbf{p}_t, \mathbf{v}_t, \mathbf{R}_t, \mathbf{b}_{at}, \mathbf{b}_{gt}, \mathbf{g}_t]^\mathrm{T}\tag{1}
$$

2) Set the ESKF error state to the formula (2)

$$
\delta \mathbf{x} =
\begin{bmatrix}
\delta \mathbf{p} \\ 
\delta \mathbf{v} \\ 
\delta \boldsymbol{\theta} \\
\delta \mathbf{b}_a \\ 
\delta \mathbf{b}_g
\end{bmatrix}
\in \mathbb{R}^{15 \times 1}
\tag{2}
$$

3) State transition equation in discrete time to formula set (3).
$$
\begin{aligned}
\begin{cases}
\mathbf{p}(t+\Delta t) &= \mathbf{p}(t) + \mathbf{v} \Delta t + \frac{1}{2} \left(\mathbf{R}(\tilde{\mathbf{a}}-\mathbf{b}_a) \right) \Delta t^2 + \frac{1}{2} \mathbf{g} \Delta t^2 \\
\mathbf{v}(t+\Delta t) &= \mathbf{v}(t) + \mathbf{R} (\tilde{\mathbf{a}} - \mathbf{b}_a) \Delta t + \mathbf{g} \Delta t \\
\mathbf{R}(t+\Delta t) &= \mathbf{R}(t) \mathrm{Exp} \left( (\tilde{\boldsymbol{\omega}}-\mathbf{b}_g) \Delta t \right) \\
\mathbf{b}_g(t+\Delta t) &= \mathbf{b}_g(t) \\
\mathbf{b}_a(t+\Delta t) &= \mathbf{b}_a(t) \\
\mathbf{g}(t+\Delta t) &= \mathbf{g}(t)
\end{cases}
\end{aligned}
\tag{3}
$$

&emsp;&emsp;where $ \mathbf{u}_{m} = [\tilde{\mathbf{a}}, \mathbf{b}_a,\tilde{\boldsymbol{\omega}},\mathbf{b}_g]^\mathrm{T} $ from the ``linear acceleration and angular acceleration of the IMU in the WIO odometer and their deviation values``.




4) Error state equation in discrete time to formula set (4).

$$
\begin{aligned}
\begin{cases}
\delta \mathbf{p}(t+\Delta t) &= \delta \mathbf{p} + \delta \mathbf{v} \Delta t \\
\delta \mathbf{v}(t+\Delta t) &= \delta \mathbf{v} + \left( - \mathbf{R}(\tilde{\mathbf{a}} - \mathbf{b}_a)^\wedge \delta \boldsymbol{\theta} - \mathbf{R} \delta \mathbf{b}_a  + \delta \mathbf{g} \right) \Delta t + \boldsymbol{\eta}_{v} \\
\delta \boldsymbol{\theta} (t+\Delta t) &= \mathrm{Exp}\left( -(\tilde{\boldsymbol{\omega}} - \mathbf{b}_g) \Delta t \right) \delta \boldsymbol{\theta} - \delta \mathbf{b}_g \Delta t - \boldsymbol{\eta}_{\theta} \\
\delta \mathbf{b}_g (t+\Delta t) &= \delta \mathbf{b}_g + \boldsymbol{\eta}_g \\
\delta \mathbf{b}_a (t+\Delta t)&= \delta \mathbf{b}_a + \boldsymbol{\eta}_a \\
\delta \mathbf{g} (t+\Delta t) &= \delta \mathbf{g}
\end{cases}
\end{aligned}
\tag{4}
$$


5) $ f(.) $ is the nonlinear state function of the system. The motion equations of Error status $\delta \mathbf{x}$ in Discrete Time to formula (5)

$$
\delta \mathbf{x}_{(t+\Delta t)} = f(\delta \mathbf{x}) + \mathbf{w}, \mathbf{w} \sim \mathcal{N}(0, \mathbf{Q})\tag{5}
$$


6) Linearization of Equations of Motion in Discrete Time to formula (6)

$$
\delta \mathbf{x}_{(t+\Delta t)} = \mathbf{F_x} \delta \mathbf{x} + \mathbf{F_i}\mathbf{w}\tag{6}
$$ 

&emsp;&emsp;where $ F_x $ and $ F_i $ equal to formula (7), formula(8) respectively.

$$
\mathbf{F_x} = \left.\frac{\partial f}{\partial \delta \mathbf{x}}\right|_{\mathbf{x}, \mathbf{u}_{m}} =
\begin{bmatrix}
\mathbf{I} & \mathbf{I} \Delta t & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{I} & - \mathbf{R}(\tilde{\mathbf{a}} - \mathbf{b}_a)^\wedge \Delta t & -\mathbf{R} \Delta t & \mathbf{0} & \mathbf{I} \Delta t \\
\mathbf{0} & \mathbf{0} & \mathrm{Exp}\left( -(\tilde{\boldsymbol{\omega}} - \mathbf{b}_g) \Delta t \right) & \mathbf{0} & -\mathbf{I} \Delta t & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\tag{7}
$$

$$
\mathbf{F}_{\mathbf{i}}=
\left.\frac{\partial f}{\partial \mathbf{i}}\right|_{\mathbf{x}, \mathbf{u}_{m}}=
\left[\begin{array}{llll}
0 & 0 & 0 & 0 \\
\mathbf{I} & 0 & 0 & 0 \\
0 & \mathbf{I} & 0 & 0 \\
0 & 0 & \mathbf{I} & 0 \\
0 & 0 & 0 & \mathbf{I} \\
0 & 0 & 0 & 0 
\end{array}\right]
\tag{8}
$$


#### 2.2.2 Priori Estimate

$$
\begin{aligned}
\delta \mathbf{\hat x}_{(t+\Delta t)}^- &= \mathbf{F_x} \delta \mathbf{x} 
\end{aligned}
\tag{9}
$$

<!-- &emsp;&emsp;where $ \delta \mathbf{x}_{(t+\Delta t)}^- = \mathbf {\hat x}_{(t+\Delta t)}^- - \mathbf {\hat x}_t^- $ -->



#### 2.2.3 Priori Estimate Covariance

$$
\begin{aligned}
\mathbf{P}_{(t+\Delta t)}^- &= \mathbf{F_x} \mathbf{P} \mathbf{F_x}^\mathrm{T} + \mathbf{F_i}\mathbf{Q} \mathbf{F_i}^{\mathrm{T}}
\end{aligned}
\tag{10}
$$

### 2.3 ESKF Correct Step

#### 2.3.1 Construct The Observe Model Of The System

1) Assume that the ``visual sensor (Visual Odometry)`` can observe state variables, and its nonlinear observation function is $ h(.) $
, can be written as the formula (11), where $ \mathbf{z} =  [\mathbf{p}_t^z, \mathbf{v}_t^z, \mathbf{R}_t^z]^\mathrm{T} $ is the observation data from the visual odometry, $ v $ is the observation noise, $ V $ is the covariance matrix of the noise.

$$
\mathbf{z} = h(\mathbf{x}) + \mathbf{v}, \mathbf{v} \sim \mathcal{N}(0, \mathbf{V})\tag{11}
$$


&emsp;&emsp;``In the traditional EKF``: Linearize the observation equation, find the Jacobian matrix of the observation equation relative to the state variable, and then update the Kalman filter.

&emsp;&emsp;``In ESKF``, we currently have an estimate of the nominal state $ \mathbf{x} $ and an estimate of the error state $\delta \mathbf{x} $, and we want to update the error state, so we need to calculate the Jacobian matrix $ H $ of the observation equation compared to the error state.

$$
\mathbf{H} = \frac{\partial h}{\partial \delta \mathbf{x}} = \frac{\partial h}{\partial \mathbf{x}} \cdot \frac{\partial \mathbf{x}}{\partial \delta \mathbf{x}}
\tag{12}
$$


### 2.3.2 Calcurate Klaman Gein

$$
\mathbf{K} = \frac{\mathbf{P}_{(t+\Delta t)}^- \cdot \mathbf{H}^T}{\mathbf{H} \cdot \mathbf{P}_{(t+\Delta t)}^- \cdot \mathbf{H}^T + \mathbf{V}}
\tag{13}
$$

### 2.3.3 Posterior Estimate

$$
\begin{aligned}
\delta \mathbf{\hat x}_{\mathrm{(t+\Delta t)}} &= \mathbf{K} (\mathbf{z} - \mathbf{H} \cdot \delta  \mathbf {\hat x}_{\mathrm{(t+\Delta t)}}^- ) 
\end{aligned}
\tag{14}
$$

### 2.3.4 Posterior Estimate Covariance

$$
\begin{aligned}
\mathbf{P}_{\mathrm{(t+\Delta t)}} &= (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P}_{\mathrm{(t+\Delta t)}}^-
\end{aligned}
\tag{15}
$$


## 2.4 ESKF State Update

$$
\mathbf{x}_{(t+\Delta t)} = \mathbf{x}_t \oplus \delta \mathbf{x}_{(t+\Delta t)}
\tag{16}
$$

&emsp;&emsp;The formula (15) equal to the formula set (16) below.

$$
\begin{aligned}
\begin{cases}
\mathbf{p}_{k+1} &= \mathbf{p}_k + \delta \mathbf{p}_k \\
\mathbf{v}_{k+1} &= \mathbf{v}_k + \delta \mathbf{v}_k \\
\mathbf{R}_{k+1} &= \mathbf{R}_k \mathrm{Exp}(\delta \boldsymbol{\theta}_k) \\
\mathbf{b}_{g, k+1} &= \mathbf{b}_{g,k} + \delta \mathbf{b}_{g,k} \\
\mathbf{b}_{a, k+1} &= \mathbf{b}_{a,k} + \delta \mathbf{b}_{a,k} \\
\mathbf{g}_{k+1} &= \mathbf{g}_{k} + \delta \mathbf{g}_{k}
\end{cases}
\end{aligned}
\tag{17}
$$


## 2.5 ESKF Error State Reset

$$
\begin{aligned}
\delta \mathbf{x}_{(t+\Delta t)} = 
\begin{bmatrix}
\delta \mathbf{p}_{(t+\Delta t)} \\
\delta \mathbf{v}_{(t+\Delta t)} \\
\mathrm \delta \boldsymbol{\theta}_{(t+\Delta t)} \\
\mathbf{b}_{g,(t+1)} \\
\delta \mathbf{b}_{a,(t+1)} \\
\delta \mathbf{g}_{(t+1)}
\end{bmatrix}
=
\begin{bmatrix}
0 \\
0 \\
0 \\
0 \\
0 \\
0 \\
\end{bmatrix}
\end{aligned}
\tag{18}
$$





