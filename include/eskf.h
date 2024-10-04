#ifndef __ESKF__
#define __ESKF__

#include <iostream>
#include "state_variable.h"
#include "vwio_eskf/Q1Data.h"
#include "vwio_eskf/V1Data.h"

using namespace std;

class ESKF
{

public:
    vwio_eskf::Q1Data custom_q1_data;
    vwio_eskf::V1Data custom_v1_data;

private:
    // Init
    double position_noise = 1.0;
    double velocity_noise = 8.0;
    double posture_noise = 1.0;

    // Predict
    double acc_noise = 5e-4;
    double gyro_noise = 5e-4;
    double acc_bias_noise = 5e-4;
    double gyro_bias_noise = 5e-4;
    Eigen::Matrix<double, 18, 18> Fx;
    Eigen::Matrix<double, 18, 12> Fi;
    Eigen::Matrix<double, 18, 15> Fi_fis;
    Eigen::Matrix<double, 12, 12> Qi;
    Eigen::Matrix<double, 15, 15> Qi_fis;

    // last wio orientation
    Eigen::Quaterniond last_wio_orientation;

    // Correct
    double pose_noise = 4.3;
    Eigen::Matrix<double, 3, 18> H;

public:
    // Init
    ESKF();
    ~ESKF();

    // ESKF state estimation

    /**
     * @brief Initialize filter state estimate by Wheel Odometry Message
     *
     * @param odom_msg Wheel Odometry Message
     * @param x Robot status
     */
    void Init(const nav_msgs::OdometryConstPtr &odom_msg, State &x);
    /**
     * @brief Predict the robot's status by WIO Data
     *
     * @param wio_data Wheel-IMU Odometry Data
     * @param dt sampling time
     * @param x Robot status
     */
    void Predict(const WIO_Data &wio_data, const WO_Data &wo_data, const double &dt, State &x);

    /**
     * @brief Correct the robot's status by Inter-frame Visual Odometry Data
     * @param wvo Wheel-Visual Odometry
     * @param x Robot status
     */
    void Correct(const WVO_Data &wvo_data, State &x);
    void CorrectWithIterators(const WVO_Data &wvo_data, int max_iterations, double convergence_threshold, State &x);

    void State_update(State &x);
    void Error_State_Reset(State &x);

    // Quaternion
    Eigen::Quaterniond kronecker_product(const Eigen::Quaterniond &p, const Eigen::Quaterniond &q);
    Eigen::Matrix3d skewsym_matrix(const Eigen::Vector3d &vec);
    Eigen::Quaterniond euler_to_quatertion(Eigen::Vector3d euler);

    // Predict
    Eigen::Matrix<double, 18, 18> calcurate_Jacobian_Fx(Eigen::Vector3d acc, Eigen::Vector3d acc_bias, Eigen::Matrix3d R, const double dt);
    // Eigen::Matrix<double, 18, 18> calcurate_Jacobian_Fx(Eigen::Vector3d acc, Eigen::Vector3d acc_bias, Eigen::Vector3d gyro, Eigen::Vector3d gyro_bias, Eigen::Matrix3d R, const double dt);

    // Compute Q Without FIS_WO
    Eigen::Matrix<double, 18, 12> calcurate_Jacobian_Fi();
    Eigen::Matrix<double, 12, 12> calcurate_Jacobian_Qi(const double dt);

    // Compute Q With FIS_WO
    Eigen::Matrix<double, 18, 15> calcurate_Jacobian_Fi_WithFIS();
    Eigen::Matrix<double, 15, 15> calcurate_Jacobian_Qi_WithFIS(const double dt);

    // Correct
    Eigen::Matrix<double, 3, 18> calcurate_Jacobian_H(State &x);

    // test code
    Eigen::Quaterniond getQuaFromAA(Eigen::Vector3d vec);
    Eigen::Matrix<double, 3, 3> getRotFromAA(Eigen::Vector3d vec);

    void predict_position_filter(State &x);
    void predict_quaternion_filter(const WO_Data &wo_data, State &x);
};

/***********************************************************************
 * Initialize
 **********************************************************************/
ESKF::ESKF()
{
    cout << "ESKF Start!" << endl;
}

ESKF::~ESKF()
{
    cout << "ESKF Finish" << endl;
}

void ESKF::Init(const nav_msgs::OdometryConstPtr &odom_msg, State &x)
{
    // Set the timestamp of the state to the timestamp of the GPS data
    x.timestamp = ros::Time::now().toSec();

    Eigen::Vector3d odom_position;
    odom_position << odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z;

    // Set the position of the state to the NED (North-East-Down) coordinates of the GPS data
    x.position = odom_position;
    // x.position = odom_msg->pose.pose.position;

    // Set the position part of the state covariance matrix to position noise times the identity matrix
    x.PEst.block<3, 3>(0, 0) = position_noise * Eigen::Matrix3d::Identity();

    // Set the velocity part of the state covariance matrix to velocity noise times the identity matrix
    x.PEst.block<3, 3>(3, 3) = velocity_noise * Eigen::Matrix3d::Identity();

    // Set the posture part of the state covariance matrix to posture noise times the identity matrix
    x.PEst.block<3, 3>(6, 6) = posture_noise * Eigen::Matrix3d::Identity();
}

// https://qiita.com/rsasaki0109/items/e969ad632cf321e25a6a
/***********************************************************************
 * ESKF Predict Step
 **********************************************************************/
/*
 * x  = [p v q] = [x y z vx vy vz qx qy qz qw]
 * dx = [dp dv dth] = [dx dy dz dvx dvy dvz dthx dthy dthz]
 *
 * pos_k = pos_{k-1} + vel_k * dt + (1/2) * (Rot(q_{k-1}) acc_{k-1}^{imu} - g) *dt^2
 * vel_k = vel_{k-1} + (Rot(quat_{k-1})) acc_{k-1}^{imu} - g) *dt
 * quat_k = Rot(w_{k-1}^{imu}*dt)*quat_{k-1}
 *
 * covariance
 * P_{k} = F_k P_{k-1} F_k^T + L Q_k L^T
 */
void ESKF::Predict(const WIO_Data &wio_data, const WO_Data &wo_data, const double &dt, State &x)
{
    Eigen::Matrix3d R = x.quaternion.toRotationMatrix();
    // Eigen::Matrix3d R1 = Eigen::Quaterniond::Identity().toRotationMatrix();
    // std::cout << "R1 =" << R1 << std::endl;

    // state update //
    x.position = x.position + x.velocity * dt + 0.5 * (R * (wio_data.acc - x.acc_bias) + x.gravity) * dt * dt;
    x.velocity = x.velocity + (R * (wio_data.acc - x.acc_bias) + x.gravity) * dt;
    // x.quaternion = kronecker_product(x.quaternion, euler_to_quatertion((wio_data.gyro - x.gyro_bias) * dt));

    // Eigen::Vector3d q_v = (wio_data.gyro - x.gyro_bias) * dt;
    // x.quaternion = x.quaternion * getQuaFromAA(q_v)

    // Update quaternion if both current and last quaternions are valid
    if (wio_data.quaternion.norm() != 0 && last_wio_orientation.norm() != 0)
    {
        // Quaternion valid, update orientation according to delta quaternion
        Eigen::Quaterniond q_q = last_wio_orientation.inverse() * wio_data.quaternion;
        // std::cout << "q_q: (x=" << q_q.x() << ", y=" << q_q.y() << ", z=" << q_q.z() << ", w=" << q_q.w() << ")" << std::endl;
        x.quaternion = x.quaternion * q_q;

        // Update orientation using gyroscope data as well if significant
        const Eigen::Vector3d q_v = (-x.gyro_bias) * dt;
        if (q_v.norm() >= 1e-12)
        {
            // std::cout << "q_v[(-x.gyro_bias) * dt]: " << q_v << std::endl;
            x.quaternion = x.quaternion * getQuaFromAA(q_v);
        }
    }
    else
    {
        const Eigen::Vector3d q_v = (wio_data.gyro - x.gyro_bias) * dt;
        if (q_v.norm() >= 1e-12)
        {
            // std::cout << "q_v[(wio_data.gyro - x.gyro_bias) * dt]: " << q_v << std::endl;
            x.quaternion = x.quaternion * getQuaFromAA(q_v);
        }
    }

    if (wio_data.quaternion.norm() != 0)
    {
        last_wio_orientation = wio_data.quaternion;
    }

    // filterd the position and quaternion from Predict Step
    predict_position_filter(x);
    predict_quaternion_filter(wo_data, x);

    // calcurate Jacobian Fx
    Fx = calcurate_Jacobian_Fx(wio_data.acc, x.acc_bias, R, dt);
    // Fx = calcurate_Jacobian_Fx(wio_data.acc, x.acc_bias, wio_data.gyro, x.gyro_bias, R, dt);

    // calcurate Jacobian Fi
    Fi = calcurate_Jacobian_Fi();
    Fi_fis = calcurate_Jacobian_Fi_WithFIS();

    // ccalcurate Jacobian Qi
    Qi = calcurate_Jacobian_Qi(dt);
    Qi_fis = calcurate_Jacobian_Qi_WithFIS(dt);

    // calcurate PEst
    x.PEst = Fx * x.PEst * Fx.transpose() + Fi * Qi * Fi.transpose();

    // Calcurate PEst When FIS-WO Have Activated
    // x.PEst = Fx * x.PEst * Fx.transpose() + Fi_fis * Qi_fis * Fi_fis.transpose();
}

Eigen::Matrix<double, 18, 18> ESKF::calcurate_Jacobian_Fx(Eigen::Vector3d acc, Eigen::Vector3d acc_bias, Eigen::Matrix3d R, const double dt)
{
    Eigen::Matrix<double, 18, 18> Fx = Eigen::Matrix<double, 18, 18>::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(3, 6) = -skewsym_matrix(R * (acc - acc_bias)) * dt;
    Fx.block<3, 3>(3, 9) = -R * dt;
    Fx.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(6, 12) = -R * dt;

    return Fx;
}

/*
Eigen::Matrix<double, 18, 18> ESKF::calcurate_Jacobian_Fx(Eigen::Vector3d acc, Eigen::Vector3d acc_bias, Eigen::Vector3d gyro, Eigen::Vector3d gyro_bias, Eigen::Matrix3d R, const double dt)
{
    Eigen::Matrix<double, 18, 18> Fx = Eigen::Matrix<double, 18, 18>::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(3, 6) = - R * skewsym_matrix((acc - acc_bias)) * dt;
    Fx.block<3, 3>(3, 9) = - R * dt;
    Fx.block<3, 3>(6, 6) = getRotFromAA((gyro - gyro_bias) * dt);
    Fx.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(6, 12) = - R * dt;

    return Fx;
}
*/

Eigen::Matrix<double, 18, 12> ESKF::calcurate_Jacobian_Fi()
{
    Eigen::Matrix<double, 18, 12> Fi = Eigen::Matrix<double, 18, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    return Fi;
}

Eigen::Matrix<double, 18, 15> ESKF::calcurate_Jacobian_Fi_WithFIS()
{
    Eigen::Matrix<double, 18, 15> Fi_fis = Eigen::Matrix<double, 18, 15>::Zero();
    Fi_fis.block<12, 15>(3, 0) = Eigen::Matrix<double, 12, 15>::Identity();

    return Fi_fis;
}

Eigen::Matrix<double, 12, 12> ESKF::calcurate_Jacobian_Qi(const double dt)
{
    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = dt * dt * acc_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = dt * dt * gyro_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = dt * acc_bias_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = dt * gyro_bias_noise * Eigen::Matrix3d::Identity();

    return Qi;
}

Eigen::Matrix<double, 15, 15> ESKF::calcurate_Jacobian_Qi_WithFIS(const double dt)
{
    Eigen::Matrix<double, 15, 15> Qi_fis = Eigen::Matrix<double, 15, 15>::Zero();
    Qi_fis.block<3, 3>(0, 0) = dt * dt * custom_q1_data.q1 * Eigen::Matrix3d::Identity();
    Qi_fis.block<3, 3>(3, 3) = dt * dt * acc_noise * Eigen::Matrix3d::Identity();
    Qi_fis.block<3, 3>(6, 6) = dt * dt * gyro_noise * Eigen::Matrix3d::Identity();
    Qi_fis.block<3, 3>(9, 9) = dt * acc_bias_noise * Eigen::Matrix3d::Identity();
    Qi_fis.block<3, 3>(12, 12) = dt * gyro_bias_noise * Eigen::Matrix3d::Identity();

    // std:cout << "Qi_fis = " << Qi_fis << std::endl;
    return Qi_fis;
}

// https://qiita.com/rsasaki0109/items/e969ad632cf321e25a6a
/***********************************************************************
 * ESKF Correct Step
 **********************************************************************/
/*
 * y = pobs = [xobs yobs zobs]
 *
 * K = P_k H^T (H P_k H^T + R)^{-1}
 *
 * dx = K (y_k - p_k )
 *
 * p_x = p_{k-1} + dp_k
 * v_k = v_{k-1} + dv_k
 * q_k = Rot(dth) q_{k-1}
 *
 * P_k = (I - KH)*P_{k-1}
 */
void ESKF::Correct(const WVO_Data &wvo_data, State &x)
{
    Eigen::Vector3d Y(wvo_data.position[0],
                      wvo_data.position[1],
                      wvo_data.position[2]);

    Eigen::Vector3d X(x.position[0],
                      x.position[1],
                      x.position[2]);

    Eigen::Matrix3d V = pose_noise * Eigen::Matrix3d::Identity();

    // calcurate Jacobian H
    H = calcurate_Jacobian_H(x);

    // calcurate Klaman Gein
    Eigen::MatrixXd K = x.PEst * H.transpose() * (H * x.PEst * H.transpose() + V).inverse();

    // calcurate error
    x.error = K * (Y - X);

    // calcurate PEst
    x.PEst = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * x.PEst;
}

void ESKF::CorrectWithIterators(const WVO_Data &wvo_data, int max_iterations, double convergence_threshold, State &x)
{
    // Measurement vector
    Eigen::Vector3d Y(wvo_data.position[0],
                      wvo_data.position[1],
                      wvo_data.position[2]);

    // Current state estimate's position
    Eigen::Vector3d X(x.position[0],
                      x.position[1],
                      x.position[2]);

    // Noise covariance matrix
    Eigen::Matrix3d V = pose_noise * Eigen::Matrix3d::Identity();

    // std::cout << "===================================" << std::endl;
    // std::cout << "pose noise" << pose_noise << std::endl;

    // Noise covariance matrix From VOFIS
    Eigen::Matrix3d V1 = 20.00 * custom_v1_data.v1 * Eigen::Matrix3d::Identity();

    // std::cout << "custom_v1_data" << custom_v1_data.v1 << std::endl;
    // std::cout << "===================================" << std::endl;


    // Initialize Kalman gain K and Jacobian H outside the loop
    Eigen::Matrix<double, 3, 18> H;
    Eigen::MatrixXd K;

    // Iterative correction
    for (int i = 0; i < max_iterations; ++i)
    {
        // Calculate Jacobian H
        H = calcurate_Jacobian_H(x);

        // Calculate Kalman gain K
        K = x.PEst * H.transpose() * (H * x.PEst * H.transpose() + V).inverse();

        // Calculate innovation error
        Eigen::VectorXd dx = K * (Y - X);

        // Save the error for further use
        x.error = dx;

        // Check for convergence
        if (dx.norm() < convergence_threshold)
            break;
    }

    // Update covariance matrix PEst using the last calculated K and H
    x.PEst = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * x.PEst;
}

Eigen::Matrix<double, 3, 18> ESKF::calcurate_Jacobian_H(State &x)
{
    Eigen::Matrix<double, 3, 19> Hx = Eigen::Matrix<double, 3, 19>::Zero();
    Hx.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 19, 18> Xx = Eigen::Matrix<double, 19, 18>::Identity();
    Eigen::Matrix<double, 4, 3> Q_theta;
    Q_theta << -x.quaternion.x(), -x.quaternion.y(), -x.quaternion.z(),
        x.quaternion.w(), x.quaternion.z(), -x.quaternion.y(),
        -x.quaternion.z(), x.quaternion.w(), x.quaternion.x(),
        x.quaternion.y(), -x.quaternion.x(), x.quaternion.w();
    Q_theta *= 0.5;
    Xx.block<4, 3>(6, 6) = Q_theta;

    Eigen::Matrix<double, 3, 18> H = Hx * Xx;

    return H;
}

/***********************************************************************
 * State Update
 **********************************************************************/
void ESKF::State_update(State &x)
{
    Eigen::Vector3d error_pos = Eigen::Map<Eigen::Vector3d>(x.error.block<3, 1>(0, 0).transpose().data());
    Eigen::Vector3d error_vel = Eigen::Map<Eigen::Vector3d>(x.error.block<3, 1>(3, 0).transpose().data());
    Eigen::Vector3d error_ori = Eigen::Map<Eigen::Vector3d>(x.error.block<3, 1>(6, 0).transpose().data());
    Eigen::Quaterniond error_quat = euler_to_quatertion(error_ori);

    Eigen::Vector3d error_acc_bias = Eigen::Map<Eigen::Vector3d>(x.error.block<3, 1>(9, 0).transpose().data());
    Eigen::Vector3d error_gyr_bias = Eigen::Map<Eigen::Vector3d>(x.error.block<3, 1>(12, 0).transpose().data());
    Eigen::Vector3d error_gra = Eigen::Map<Eigen::Vector3d>(x.error.block<3, 1>(15, 0).transpose().data());

    x.position = x.position + error_pos;
    x.velocity = x.velocity + error_vel;
    // x.quaternion = kronecker_product(error_quat, x.quaternion);
    x.quaternion = x.quaternion * getQuaFromAA(error_ori);

    x.acc_bias = x.acc_bias + error_acc_bias;
    x.gyro_bias = x.gyro_bias + error_gyr_bias;
    x.gravity = x.gravity + error_gra;
}

void ESKF::Error_State_Reset(State &x)
{
    x.error.setZero();
}

/***********************************************************************
 * Quaternion
 **********************************************************************/
Eigen::Quaterniond ESKF::kronecker_product(const Eigen::Quaterniond &p, const Eigen::Quaterniond &q)
{
    Eigen::Quaterniond res;
    res.w() = p.w() * q.w() - p.x() * q.x() - p.y() * q.y() - p.z() * q.z();
    res.x() = p.w() * q.x() + p.x() * q.w() + p.y() * q.z() - p.z() * q.y();
    res.y() = p.w() * q.y() - p.x() * q.z() + p.y() * q.w() + p.z() * q.x();
    res.z() = p.w() * q.z() + p.x() * q.y() - p.y() * q.x() + p.z() * q.w();
    return res;
}

Eigen::Matrix3d ESKF::skewsym_matrix(const Eigen::Vector3d &vec)
{
    Eigen::Matrix3d mat;
    mat << 0.0, -vec(2), vec(1),
        vec(2), 0.0, -vec(0),
        -vec(1), vec(0), 0.0;
    return mat;
}

// https://qiita.com/take4eng/items/ae487c82a6f7d60ceba8
Eigen::Quaterniond ESKF::euler_to_quatertion(Eigen::Vector3d euler)
{
    double roll = euler[0];
    double pitch = euler[1];
    double yaw = euler[2];

    double cr = cos(0.5 * roll);
    double sr = sin(0.5 * roll);
    double cp = cos(0.5 * pitch);
    double sp = sin(0.5 * pitch);
    double cy = cos(0.5 * yaw);
    double sy = sin(0.5 * yaw);

    Eigen::Vector4d vq;

    vq[0] = cy * cp * cr + sy * sp * sr;
    vq[1] = cy * cp * sr - sy * sp * cr;
    vq[2] = sy * cp * sr + cy * sp * cr;
    vq[3] = sy * cp * cr - cy * sp * sr;

    Eigen::Quaterniond q(vq);

    return q;
}

// tranform angleaxis to quaternion
Eigen::Quaterniond ESKF::getQuaFromAA(Eigen::Vector3d vec)
{
    double theta = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    if (theta == 0.0)
    {
        Eigen::Vector4d vq;
        vq[0] = 1.0;
        vq[1] = 0.0;
        vq[2] = 0.0;
        vq[3] = 0.0;

        Eigen::Quaterniond q(vq);

        return q;
    }

    Eigen::Vector3d unit_axis = vec / theta;

    double w = cos(0.5 * theta);
    double sin_t = sin(0.5 * theta);
    double x = unit_axis[0] * sin_t;
    double y = unit_axis[1] * sin_t;
    double z = unit_axis[2] * sin_t;

    Eigen::Quaterniond qua(w, x, y, z);
    qua.normalized();

    return qua;
}

// get rotation matrix from angleaxis by rodrigues formula
Eigen::Matrix<double, 3, 3> ESKF::getRotFromAA(Eigen::Vector3d vec)
{
    double theta = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    if (0 == theta)
    {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix<double, 3, 1> unit_axis = vec / theta;
    Eigen::Matrix<double, 3, 3> unit_mat = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 3> R;
    double sin_th = sin(theta);
    double cos_th = cos(theta);
    R = unit_mat * cos_th + (1 - cos_th) * unit_axis * unit_axis.transpose() + sin_th * skewsym_matrix(unit_axis);

    return R;
}

void ESKF::predict_position_filter(State &x)
{
    // Define the maximum allowable change in position (example threshold values)
    Eigen::Vector3d max_position_change(10.0, 10.0, 0.5); // Adjust these values as needed

    Eigen::Vector3d previous_position;

    // Calculate the change in position
    Eigen::Vector3d position_change = x.position - previous_position;

    // Apply saturation filter to limit the change in position
    for (int i = 0; i < 3; ++i)
    {
        if (std::abs(position_change[i]) > max_position_change[i])
        {
            position_change[i] = max_position_change[i] * (position_change[i] > 0 ? 1 : -1);
        }
    }

    // Update the position with the limited change
    x.position = previous_position + position_change;

    // Store the current position for the next iteration
    previous_position = x.position;
}

void ESKF::predict_quaternion_filter(const WO_Data &wo_data, State &x)
{
    // 假设 wo_data.quaternion 和 x.quaternion 是 Eigen::Quaterniond 类型
    Eigen::Quaterniond wo_data_quat(wo_data.quaternion.w(), wo_data.quaternion.x(), wo_data.quaternion.y(), wo_data.quaternion.z());
    Eigen::Quaterniond x_quat(x.quaternion.w(), x.quaternion.x(), x.quaternion.y(), x.quaternion.z());

    // Step 1: 将四元数转换为欧拉角 (roll, pitch, yaw)
    Eigen::Vector3d wo_euler = wo_data_quat.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX顺序 (yaw, pitch, roll)
    Eigen::Vector3d x_euler = x_quat.toRotationMatrix().eulerAngles(2, 1, 0);        // ZYX顺序 (yaw, pitch, roll)

    // Step 2: 使用 wo_data 的 roll 和 pitch，结合 x 的 yaw
    double new_roll = wo_euler(2);  // roll 来自 wo_data
    double new_pitch = wo_euler(1); // pitch 来自 wo_data
    double new_yaw = x_euler(0);    // yaw 来自 x

    // Step 3: 将组合后的欧拉角 (roll, pitch, yaw) 转换为新的四元数
    Eigen::Quaterniond quat_from_euler;
    quat_from_euler = Eigen::AngleAxisd(new_yaw, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(new_pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(new_roll, Eigen::Vector3d::UnitX());

    // Step 4: 检查四元数方向，纠正符号问题
    Eigen::Quaterniond fused_quat;
    if (quat_from_euler.dot(x_quat) < 0)
    {
        // 使用共轭实现取反
        fused_quat = -quat_from_euler.coeffs(); // 直接创建负四元数
    }
    else
    {
        fused_quat = quat_from_euler;
    }
    // 更新状态
    x.quaternion = fused_quat;

    // 输出结果
    // std::cout << "Fused Quaternion (w, x, y, z): " << x.quaternion.w() << ", " << x.quaternion.x() << ", "
    //           << x.quaternion.y() << ", " << x.quaternion.z() << std::endl;
}

#endif // ESKF