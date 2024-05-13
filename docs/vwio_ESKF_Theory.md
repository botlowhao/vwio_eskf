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

### 2.2 ESKF Predict Step

#### 2.2.1 Construct Predict Model Of System

``Position Of The Predict Model:``
$$
p_t = p_{t-1} + v_{t-1} \cdot \Delta t + \frac{1}{2} [R(q_{t-1}) \cdot (acc_{t-1}^{wio} - abias_{t-1}^{x}) + g_{t-1}^x] \cdot (\Delta t)^2
$$

``Velocity Of The Predict Model:``
$$
v_t = v_{t-1} + [R(q_{t-1}) \cdot (a_{t-1}^{wio} - abias_{t-1}^{x}) + g_{t-1}^x] \cdot \Delta t
$$

``Quaternion Of The Predict Model:``
$$
q_t = q_{t-1} \cdot [q(\omega_{t-1}^{wio} - \omega bias_{t-1}^x) \cdot \Delta t]
$$

``Process Noise Of The Predict Model:``
$$
w_k \sim \mathcal{N}(0, Q)
$$

``Convert to the state space equation of the robot:``

$$
x_t = F_t \cdot x_{t-1} + B_t \cdot u_{t-1} + Q
$$

where,

1) ``The vectors represent the state quantities of the robot at time t and time t-1 respectively. They are defined as the formula below.``
$$
x_t = 
\begin{bmatrix}
p_t^x \\  
v_t^x \\ 
q_t^x \\
\end{bmatrix}
,
x_{t-1} =
\begin{bmatrix}
p_{t-1}^x \\  
v_{t-1}^x \\ 
q_{t-1}^x \\
\end{bmatrix}
$$


2) ``F_t represents state transition matrix.``
$$
F_t = 
\begin{bmatrix} 
1 & \Delta t & 0 \\ 0 & 1 & 0 \\ 0 & 0 & q(\omega_{t-1}^{wio} - \omega bias_{t-1}^x) \cdot \Delta t 
\end{bmatrix} 
$$


3) ``B_t represents input control matrix.``  
$$
B_t = 
\begin{bmatrix} 
\frac{1}{2} [R(q_{t-1}) \cdot (acc_{t-1}^{wio} - abias_{t-1}^{x}) + g_{t-1}^x] & 0 & 0 \\ 0 & [R(q_{t-1}) \cdot (a_{t-1}^{wio} - abias_{t-1}^{x}) + g_{t-1}] & 0 \\ 0 & 0 & 0 
\end{bmatrix} 
$$

4) ``u_t represents control input.``  
$$
u_{t-1} =
\begin{bmatrix}
(\Delta t)^2 \\ \Delta t \\ 1 
\end{bmatrix} \\
$$

#### 2.2.2 Priori Estimate
$$
x_t = F_t x_{t-1} + B_t
$$


#### 2.2.3 Priori Estimate Covariance
$$
P_t^- = F_t P_{t-1} F_t^T + Q
$$


### 2.3 ESKF Correct Step

#### 2.3.1 Construct The Observe Model Of The System

$$
z_t = H_t x_t + v_k
$$
which,

1) ``z_t represents observations from visual information.``

$$
z_t = 
\begin{bmatrix}
p_t^z \\  
v_t^z \\ 
q_t^z \\
\end{bmatrix}
(v_t^z=0, q_t^z=0) 
$$

2) ``x_t represent the state quantities of the robot.``

$$
x_t = 
\begin{bmatrix}
p_t^x \\  
v_t^x \\ 
q_t^x \\
\end{bmatrix}
$$

3) ``H_t represents Jacobian matrix of observation model.``

$$
H_t = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}
$$


4) ``v_k represents the noise of observation.``

$$
v_k \sim \mathcal{N}(0, V)
$$


### 2.3.2 Calcurate Klaman Gein

$$
K_t = \frac{P_t^- H^T}{H P_t^- H^T + V}
$$

### 2.3.3 Posterior Estimate
$$
\hat{x}_t = \hat{x}_t^- + K_t(z_t - H \hat{x}_t^-)
$$

### 2.3.4 Posterior Estimate Covariance
$$
P_t = (I - K_t H)P_t^-
$$


## 2.4 ESKF State Update



