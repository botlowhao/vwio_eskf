#ifndef __ESKF__
#define __ESKF__

#include <iostream>
#include "state_variable.h"

using namespace std;

class ESKF
{
private:
    // Init
    double position_noise = 1.2;
    double velocity_noise = 10.0;
    double posture_noise = 1.0;

    // Predict
    double acc_noise = 5e-4;
    double gyro_noise = 5e-4;
    double acc_bias_noise = 5e-4;
    double gyro_bias_noise = 5e-4;
    Eigen::Matrix<double, 18, 18> Fx;
    Eigen::Matrix<double, 18, 12> Fi;
    Eigen::Matrix<double, 12, 12> Qi;

    // last wio orientation
    Eigen::Quaterniond last_wio_orientation;

    // Correct
    double pose_noise = 1.2;
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
    void Predict(const WIO_Data &wio_data, const double &dt, State &x);

    /**
     * @brief Correct the robot's status by Inter-frame Visual Odometry Data
     * @param vo Inter-frame Visual Odometry
     * @param x Robot status
     */
    void Correct(const nav_msgs::Odometry &vo, State &x);
    void State_update(State &x);
    void Error_State_Reset(State &x);

    // Quaternion
    Eigen::Quaterniond kronecker_product(const Eigen::Quaterniond &p, const Eigen::Quaterniond &q);
    Eigen::Matrix3d skewsym_matrix(const Eigen::Vector3d &vec);
    Eigen::Quaterniond euler_to_quatertion(Eigen::Vector3d euler);

    // Predict
    Eigen::Matrix<double, 18, 18> calcurate_Jacobian_Fx(Eigen::Vector3d acc, Eigen::Vector3d acc_bias, Eigen::Matrix3d R, const double dt);
    // Eigen::Matrix<double, 18, 18> calcurate_Jacobian_Fx(Eigen::Vector3d acc, Eigen::Vector3d acc_bias, Eigen::Vector3d gyro, Eigen::Vector3d gyro_bias, Eigen::Matrix3d R, const double dt);
    Eigen::Matrix<double, 18, 12> calcurate_Jacobian_Fi();
    Eigen::Matrix<double, 12, 12> calcurate_Jacobian_Qi(const double dt);

    // Correct
    Eigen::Matrix<double, 3, 18> calcurate_Jacobian_H(State &x);

    // test code
    Eigen::Quaterniond getQuaFromAA(Eigen::Vector3d vec);
    Eigen::Matrix<double, 3, 3> getRotFromAA(Eigen::Vector3d vec);
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
void ESKF::Predict(const WIO_Data &wio_data, const double &dt, State &x)
{

    Eigen::Matrix3d R = x.quaternion.toRotationMatrix();

    // state update //
    x.position = x.position + x.velocity * dt + 0.5 * (R * (wio_data.acc - x.acc_bias) + x.gravity) * dt * dt;
    x.velocity = x.velocity + (R * (wio_data.acc - x.acc_bias) + x.gravity) * dt;
    // x.quaternion = kronecker_product(x.quaternion, euler_to_quatertion((imu_data.gyro - x.gyro_bias) * dt));

    // Eigen::Vector3d q_v = (wio_data.gyro - x.gyro_bias) * dt;
    // x.quaternion = x.quaternion * getQuaFromAA(q_v);

    // Update quaternion if both current and last quaternions are valid
    if (wio_data.orientation.norm() != 0 && last_wio_orientation.norm() != 0)
    {
        // Quaternion valid, update orientation according to delta quaternion
        Eigen::Quaterniond q_q = last_wio_orientation.inverse() * wio_data.orientation;
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

    if (wio_data.orientation.norm() != 0)
    {
        last_wio_orientation = wio_data.orientation;
    }

    // calcurate Jacobian Fx
    Fx = calcurate_Jacobian_Fx(wio_data.acc, x.acc_bias, R, dt);
    // Fx = calcurate_Jacobian_Fx(imu_data.acc, x.acc_bias, imu_data.gyro, x.gyro_bias, R, dt);

    // calcurate Jacobian Fi
    Fi = calcurate_Jacobian_Fi();

    // ccalcurate Jacobian Qi
    Qi = calcurate_Jacobian_Qi(dt);

    // calcurate PEst
    x.PEst = Fx * x.PEst * Fx.transpose() + Fi * Qi * Fi.transpose();
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

Eigen::Matrix<double, 12, 12> ESKF::calcurate_Jacobian_Qi(const double dt)
{
    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = dt * dt * acc_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = dt * dt * gyro_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = dt * acc_bias_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = dt * gyro_bias_noise * Eigen::Matrix3d::Identity();

    return Qi;
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
 void ESKF::Correct(const nav_msgs::Odometry &vo, State &x)
{
    Eigen::Vector3d Y(vo.pose.pose.position.x,
                      vo.pose.pose.position.y,
                      vo.pose.pose.position.z);
    
    Eigen::Vector3d X(x.position[0],
                      x.position[1],
                      x.position[2]);

    /*
    
    // 限制协方差矩阵P的值
    const double max_covariance_value = 10;         // 设置最大的协方差值
    x.PEst = x.PEst.cwiseMin(max_covariance_value); // 将协方差矩阵中的每个元素限制在最大值内

    // 控制噪声矩阵V的值     
    const double min_noise_value = 0.8; // 设置最小的噪声值
    Eigen::Matrix3d V = pose_noise * Eigen::Matrix3d::Identity();
    V = V.cwiseMax(min_noise_value); // 将噪声矩阵中的每个元素限制在最小值以上

    */

    Eigen::Matrix3d V = pose_noise * Eigen::Matrix3d::Identity();

    // calcurate Jacobian H
    H = calcurate_Jacobian_H(x);

    
    /*
    // 计算卡尔曼增益矩阵 K
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(x.PEst.rows(), Y.size()); // 初始化 K 矩阵
    if (!std::isnan(x.PEst.sum()) && !std::isnan(H.sum()) && !std::isnan(V.sum()))
    {
        // 如果输入的矩阵没有 NaN 值，才进行计算
        K = x.PEst * H.transpose() * (H * x.PEst * H.transpose() + V).inverse();
    }
    else
    {
        ROS_WARN("NaN detected in covariance or Jacobian or noise matrix. Skipping correction step.");
        return;
    }

    // 检查计算结果是否有 NaN 值
    if (std::isnan(K.sum()))
    {
        ROS_WARN("NaN detected in Kalman Gain. Skipping correction step.");
        return;
    }
    */

    // calcurate Klaman Gein
    Eigen::MatrixXd K = x.PEst * H.transpose() * (H * x.PEst * H.transpose() + V).inverse();

    // calcurate error
    x.error = K * (Y - X);

    // calcurate PEst
    x.PEst = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * x.PEst;                                              

    // Convert K matrix to string
    std::stringstream ss;
    ss << "Jacobian Gain K:\n"
       << K;

    // Output K matrix using ROS_INFO
    ROS_INFO("%s", ss.str().c_str());
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

#endif // ESKF