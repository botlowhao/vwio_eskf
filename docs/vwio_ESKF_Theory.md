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

#### 2) Priori Status Estimate Covariance
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

1) Set **intial position value** from WO(Wheel Odometry) according to the formula (1).

$$
\begin{aligned}
\mathbf{p}_t =  [\mathbf{p}_{xt}^{wo}, \mathbf{p}_{yt}^{wo}, \mathbf{p}_{zt}^{wo}]^\mathrm{T}\tag{1}
\end{aligned}
$$


2) Initialize the **State Estimate Covariance** matrix and set the noise for the position/velocity/posture estimation of the robot's initial state according to the formula (2).


$$
\mathbf{P}_t^{init} = 
\begin{pmatrix}
\sigma_{\text{p}}^2 \mathbf{I}_{3 \times 3} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \sigma_{\text{v}}^2 \mathbf{I}_{3 \times 3} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \sigma_{\theta}^2 \mathbf{I}_{3 \times 3} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \sigma_{\text{a}}^2 \mathbf{I}_{3 \times 3} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \sigma_{\alpha}^2 \mathbf{I}_{3 \times 3} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \sigma_{g}^2 \mathbf{I}_{3 \times 3}
\end{pmatrix}
\in \mathbb{R}^{18 \times 18}
\tag{2}
$$

&emsp;&emsp;Where $ \sigma_{\text{p}}^2 = 1.2 $, $ \sigma_{\text{v}}^2 = 10.0 $ and $ \sigma_{\theta}^2 = 1.0 $ while $ \sigma_{a}^2 = \sigma_{\alpha}^2 = \sigma_{g}^2 = 0 $ .


### 2.2 ESKF Predict Step

#### 2.2.1 Construct Predict Model Of System

1) Set the **ESKF truth state** to the formula (3)

$$
\mathbf{x}_t = [\mathbf{p}_t, \mathbf{v}_t, \mathbf{R}_t, \mathbf{b}_{at}, \mathbf{b}_{\alpha t}, \mathbf{g}_t]^\mathrm{T}\tag{3}
$$

2) Set the **ESKF error state** to the formula (4)

$$
\delta \mathbf{x} =
\begin{bmatrix}
\delta \mathbf{p} \\ 
\delta \mathbf{v} \\ 
\delta \boldsymbol{\theta} \\
\delta \mathbf{b}_a \\ 
\delta \mathbf{b}_{\alpha} \\
\delta \mathbf{g}
\end{bmatrix}
\in \mathbb{R}^{18 \times 1}
\tag{4}
$$

3) State transition equation in discrete time to formula set (5).
$$
\begin{aligned}
\begin{cases}
\mathbf{p}(t+\Delta t) = \mathbf{p}(t) + \mathbf{v} \Delta t + \frac{1}{2} \left(\mathbf{R}(\tilde{\mathbf{a}}-\mathbf{b}_a) \right) \Delta t^2 + \frac{1}{2} \mathbf{g} \Delta t^2 \\
\mathbf{v}(t+\Delta t) = \mathbf{v}(t) + \mathbf{R} (\tilde{\mathbf{a}} - \mathbf{b}_a) \Delta t + \mathbf{g} \Delta t \\
\mathbf{R}(t+\Delta t) = \mathbf{R}(t) \mathrm{Exp} \left( (\tilde{\boldsymbol{\alpha}}-\mathbf{b}_{\alpha}) \Delta t \right) \\
\mathbf{b}_a(t+\Delta t) = \mathbf{b}_a(t) \\
\mathbf{b}_{\alpha}(t+\Delta t) = \mathbf{b}_{\alpha}(t) \\
\mathbf{g}(t+\Delta t) = \mathbf{g}(t)
\end{cases}
\end{aligned}
\tag{5}
$$

&emsp;&emsp;where $\mathbf{u}_{m} = [\tilde{\mathbf{a}}, \mathbf{b}_a,\tilde{\boldsymbol{\alpha}},\mathbf{b}_{\alpha}]^\mathrm{T}$ **from the information of the IMU's linear acceleration and angular acceleration in the WIO odometer and their deviation values**.




4) Error state equation in discrete time to formula set (6).

$$
\begin{aligned}
\begin{cases}
\delta \mathbf{p}(t+\Delta t) = \delta \mathbf{p} + \delta \mathbf{v} \Delta t \\
\delta \mathbf{v}(t+\Delta t) = \delta \mathbf{v} + \left( - \mathbf{R}(\tilde{\mathbf{a}} - \mathbf{b}_a)^\wedge \delta \boldsymbol{\theta} - \mathbf{R} \delta \mathbf{b}_a  + \delta \mathbf{g} \right) \Delta t + \boldsymbol{\eta}_{v} \\
\delta \boldsymbol{\theta} (t+\Delta t) = \mathrm{Exp}\left( -(\tilde{\boldsymbol{\omega}} - \mathbf{b}_{\omega}) \Delta t \right) \delta \boldsymbol{\theta} - \delta \mathbf{b}_{\omega} \Delta t - \boldsymbol{\eta}_{\theta} \\
\delta \mathbf{b}_a (t+\Delta t) = \delta \mathbf{b}_a + \boldsymbol{\eta}_a \\
\delta \mathbf{b}_{\omega} (t+\Delta t) = \delta \mathbf{b}_{\omega} + \boldsymbol{\eta}_{\omega} \\
\delta \mathbf{g} (t+\Delta t) = \delta \mathbf{g}
\end{cases}
\end{aligned}
\tag{6}
$$


5) $ f(.) $ is the nonlinear state function of the system and $\mathbf{Q}$ is the **Variance of Process Noise matrix**. The motion equations of Error status $\delta \mathbf{x}$ in Discrete Time to formula (7).

$$
\delta \mathbf{x} = f(\delta \mathbf{x}) + \mathbf{w}, \mathbf{w} \sim \mathcal{N}(0, \mathbf{Q})\tag{7}
$$

$$
\mathbf{Q} = 
\begin{pmatrix}
{\Delta t}^2 \sigma_{a}^2 \mathbf{I}_{3 \times 3} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & {\Delta t}^2 \sigma_{\alpha}^2 \mathbf{I}_{3 \times 3} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \Delta t \sigma_{ba}^2 \mathbf{I}_{3 \times 3} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \Delta t \sigma_{b \alpha}^2 \mathbf{I}_{3 \times 3}
\end{pmatrix}
\in \mathbb{R}^{12 \times 12}
\tag{8}
$$

&emsp;&emsp;Where $\sigma_{a}^2 = \sigma_{\alpha}^2 = \sigma_{ba}^2 = \sigma_{b \alpha}^2 = 5 \times 10^{-4}$.


6) Linearization of Equations of Motion in Discrete Time to formula (9)

$$
\delta \mathbf{x} = \mathbf{F_x} \delta \mathbf{x} + \mathbf{F_i}\mathbf{w}\tag{9}
$$ 

&emsp;&emsp;where $ F_x $ and $ F_i $ equal to formula (10), formula(11) respectively.

$$
\mathbf{F_x} = \left.\frac{\partial f}{\partial \delta \mathbf{x}}\right|_{\mathbf{x}, \mathbf{u}_{m}} =
\begin{bmatrix}
\mathbf{I} & \mathbf{I} \Delta t & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{I} & - \mathbf{R}(\tilde{\mathbf{a}} - \mathbf{b}_a)^\wedge \Delta t & -\mathbf{R} \Delta t & \mathbf{0} & \mathbf{I} \Delta t \\
\mathbf{0} & \mathbf{0} & \mathrm{Exp}\left( -(\tilde{\boldsymbol{\alpha}} - \mathbf{b}_{\alpha}) \Delta t \right) & \mathbf{0} & -\mathbf{I} \Delta t & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
\in \mathbb{R}^{18 \times 18}
\tag{10}
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
\in \mathbb{R}^{18 \times 12}
\tag{11}
$$


#### 2.2.2 Priori Estimate

$$
\begin{aligned}
\delta \mathbf{\hat x}^- &= \mathbf{F_x} \delta \mathbf{x} 
\end{aligned}
\tag{12}
$$

<!-- &emsp;&emsp;where $ \delta \mathbf{x}_{(t+\Delta t)}^- = \mathbf {\hat x}_{(t+\Delta t)}^- - \mathbf {\hat x}_t^- $ -->



#### 2.2.3 Priori State Estimate Covariance

$$
\begin{aligned}
\mathbf{P}_t^- &= \mathbf{F_x} \mathbf{P}_t \mathbf{F_x}^\mathrm{T} + \mathbf{F_i}\mathbf{Q} \mathbf{F_i}^{\mathrm{T}}
\end{aligned}
\tag{13}
$$





### 2.3 ESKF Correct Step

#### 2.3.1 Construct The Observe Model Of The System

1) Assume that the **visual sensor (Visual Odometry)** can observe state variables, and its nonlinear observation function is $h(.)$, can be written as the formula (14), where $\mathbf{z} =  [\mathbf{p}_{xt}^{vo}, \mathbf{p}_{yt}^{vo}, \mathbf{p}_{zt}^{vo}]^\mathrm{T}$ is the **observation data** from the visual odometry, $v$ is the observation noise, $V$ is the **Variance of Collection Noise matrix**.


$$
\mathbf{z} = h(\mathbf{x}) + \mathbf{v}, \mathbf{v} \sim \mathcal{N}(0, \mathbf{V})\tag{14}
$$ 


$$
\mathbf{V} = 
\begin{pmatrix}
\sigma_{\text{observe}}^2 & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \sigma_{\text{observe}}^2 & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \sigma_{\text{observe}}^2 \\
\end{pmatrix}
\in \mathbb{R}^{3 \times 3} ,\ (\sigma_{observe}^2 = 1.2 )
\tag{15}
$$




&emsp;&emsp;In the traditional **Extended Kalman Filter (EKF)**, the observation equation is linearized, **the Jacobian matrix of the observation equation with respect to the state variable is found**, and then the Kalman filter is updated.

&emsp;&emsp;In the **Error State Kalman Filter (ESKF)**, an estimate of the nominal state $\mathbf{x}$ and an estimate of the error state $\delta \mathbf{x}$ are available. To update the error state, **the Jacobian matrix** $H$ **of the observation equation with respect to the error state is calculated**.


$$
\mathbf{H} = \frac{\partial h}{\partial \delta \mathbf{x}} = \frac{\partial h}{\partial \mathbf{x}} \cdot \frac{\partial \mathbf{x}}{\partial \delta \mathbf{x}}
\tag{16}
$$


#### 2.3.2 Calcurate Klaman Gein

$$
\mathbf{K} = \frac{\mathbf{P}_t^- \cdot \mathbf{H}^T}{\mathbf{H} \cdot \mathbf{P}_t^- \cdot \mathbf{H}^T + \mathbf{V}}
\tag{17}
$$

#### 2.3.3 Posterior Estimate

$$
\begin{aligned}
\delta \mathbf{\hat x} &= \mathbf{K} (\mathbf{z} - \mathbf{H} \cdot \delta  \mathbf {\hat x}^- ) 
\end{aligned}
\tag{18}
$$

#### 2.3.4 Posterior State Estimate Covariance

$$
\begin{aligned}
\mathbf{P}_t &= (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P}_t^-
\end{aligned}
\tag{19}
$$


## 2.4 ESKF State Update

&emsp;&emsp;The true state $\mathbf{x}_t$ equal to the **Combination Operations** of nomal state $\mathbf{x}$ and error state $\delta{\hat x}_t$.

$$
\mathbf{x}_t = \mathbf{x} \oplus \delta \mathbf{\hat x}
\tag{20}
$$

&emsp;&emsp;The formula (15) equal to the formula set (16) below.

$$
\begin{aligned}
\begin{cases}
\mathbf{p}_{t} = \mathbf{p} + \delta \mathbf{p} \\
\mathbf{v}_{t} = \mathbf{v} + \delta \mathbf{v} \\
\mathbf{R}_{t} = \mathbf{R} \mathrm{Exp}(\delta \boldsymbol{\theta}) \\
\mathbf{b}_{a, t} = \mathbf{b}_{a} + \delta \mathbf{b}_{a} \\
\mathbf{b}_{{\alpha}, t} = \mathbf{b}_{{\alpha}} + \delta \mathbf{b}_{{\alpha}} \\
\mathbf{g}_{t} = \mathbf{g} + \delta \mathbf{g}
\end{cases}
\end{aligned}
\tag{21}
$$


## 2.5 ESKF Error State Reset

&emsp;&emsp;The error state $\delta x$ will be **RESET to ZERO**  after ESKF State Update according to the formula (22) below.

$$
\begin{aligned}
\delta \mathbf{\hat x} = 
\begin{bmatrix}
\delta \mathbf{p} \\
\delta \mathbf{v} \\
\mathrm \delta \boldsymbol{\theta} \\
\delta \mathbf{b}_{a} \\
\delta \mathbf{b}_{\alpha} \\
\delta \mathbf{g}
\end{bmatrix} =
\begin{bmatrix}
0 \\
0 \\
0 \\
0 \\
0 \\
0 \\
\end{bmatrix}
\end{aligned}
\tag{22}
$$






