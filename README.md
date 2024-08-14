# vwio_eskf
ROS1 Package Using ESKF/IESKF Method for Muti-Sensor Fusion(Wheel Odometry, Inertial Odometry, Visual Odometry)
## Overview
### System Structure Diagram With FIS
![FIS-ESKF_Structure](./FIS-ESKF_Structure.jpg)
### Functionality completed
- [x] Construst **Wheel-Inertial Odometry** (WIO).
- [x] Construst **Wheel-Visual Odometry** (WVO).
- [x] Wheel-Inertial Odometry (WIO) and Wheel-Visual Odometry (WVO) Fusion Localization by **ESKF/IESKF method**.
- [x] Construct **Wheel Odometry Fuzzy Inference System**(WOFIS) with another ROS Package [fis_wo](https://github.com/botlowhao/fis_wo).
  - [x] Custom define ROS data type(vwio_eskf::WOFISData) to send **ros_wofisdata**(including Wheel Velocity Difference and Z-Axies Angular Velocity) by robot's linear and angular velocity.
  - [x] Custom define ROS data type(vwio_eskf::Q1Data) to receive **custom_q1_data** which is used to adjust the Process Covariance Matrix dynamically.
### Functionality to be completed
- [ ] Construct **Visual Odometry Fuzzy Inference System** by Number of Feature Points and Reprojection Error from ORB features.

## Develop Environment
OS:
- Ubuntu 20.04 with NVIDIA Jeston Origin Nano(8GB RAM)
- Ubuntu 20.04 with PC(Intel Core i5 8th Gen, 8GB RAM)

ROS:
- noetic

3rdParty:
- [Eigen 3.3.7](https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz)
```
mkdir -p ~/3rdParty
cd ~/3rdParty
git clone https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz

sudo tar -xzvf eigen-3.3.7.tar.gz
mv eigen-3.3.7 eigen

cd eigen
mkdir build && cd build
cmake ..
sudo make install
sudo cp -r /usr/local/include/eigen3/Eigen /usr/local/include
```
- [Ceres-Solver 2.2.0](https://github.com/ceres-solver/ceres-solver/releases/tag/2.2.0)

```
# Install Ceres-Solver 2.2.0
sudo apt-get update
sudo apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev
cd 3rdParty
git clone -b 2.2.0 https://github.com/ceres-solver/ceres-solver.git

cd ceres-solver
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

## Input And Output ROS Topics
### Input
- Wheel Odometry messages /wo_sync (nav_msgs/Odometry)
- Inertial Odometry messages /imu_sync (sensor_msgs/Imu)
- Visual Odometry messages /vo_sync (nav_msgs/Odometry)
- q1 data messages /Q1_msg (vwio_eskf::Q1Data)
### Output
- Wheel Odometry Path message /wo_path (nav_msgs/Path)
- Wheel-Inertial Odometry Path message /wio_path (nav_msgs/Path)
- Wheel-Visual Odometry Path message /wvo_path (nav_msgs/Path)
- Estimated Path message /estimated_path (nav_msgs/Path)
- Wheel Odometry message /wo_pose (nav_msgs/Odometry)
- Wheel-Inertial Odometry message /wio_pose (nav_msgs/Odometry)
- Wheel-Visual Odometry message /wvo_pose (nav_msgs/Odometry)
- Estimated Pose message /state_pose (nav_msgs/Odometry)
- WOFIS data message /WOFIS_msg (vwio_eskf::WOFISData)

## Class Diagram

```mermaid
classDiagram
    class WO_Data {
        double timestamp
        Eigen::Vector3d position
        Eigen::Quaterniond quaternion
        Eigen::Vector3d linear_vel
        Eigen::Vector3d angular_vel
        double yaw_angle
    }
    class WIO_Data {
        double timestamp
        Eigen::Vector3d position
        Eigen::Quaterniond quaternion
        Eigen::Vector3d linear_vel
        Eigen::Vector3d angular_vel
        Eigen::Vector3d acc
        Eigen::Vector3d gyro
        double yaw_angle
    }
    class WVO_Data{
        double timestamp
        Eigen::Vector3d position
        Eigen::Quaterniond quaternion
    }
    class State {
        double timestamp
        Eigen::Vector3d position
        Eigen::Vector3d velocity
        Eigen::Quaterniond quaternion
        Eigen::Vector3d acc_bias
        Eigen::Vector3d gyro_bias
        Eigen::Vector3d gravity
        Eigen::Matrix<double, 18, 18> PEst
        Eigen::Matrix<double, 18, 1> error
    }
    class WOFISData{
        double delta_v
        double w_z
    }
    class ESKF {
        +vwio_eskf::Q1Data custom_q1_data
        -State x
        -double position_noise
        -double velocity_noise
        -double posture_noise
        -double acc_noise
        -double gyro_noise
        -double acc_bias_noise
        -double gyro_bias_noise
        -Eigen::Matrix<double, 18, 18> Fx
        -Eigen::Matrix<double, 18, 12> Fi
        -Eigen::Matrix<double, 18, 15> Fi_fis
        -Eigen::Matrix<double, 12, 12> Qi
        -Eigen::Matrix<double, 15, 15> Qi_fis
        -Eigen::Quaterniond last_wio_orientation
        -double pose_noise
        -Eigen::Matrix<double, 3, 18> H
        +ESKF()
        +~ESKF()
        +void Init(const GPS_Data&, State&)
        +void Predict(const IMU_Data&, State&)
        +void Correct(const GPS_Data&, State&)
        +void CorrectWithIterators(const WVO_Data&, int, double, State&)
        +void State_update(State &)
        +void Error_State_Reset(State&)
        +Eigen::Quaterniond kronecker_product(const Eigen::Quaterniond&, const Eigen::Quaterniond&)
        +Eigen::Matrix3d skewsym_matrix(const Eigen::Vector3d&)
        +Eigen::Quaterniond euler_to_quatertion(Eigen::Vector3d)
        +Eigen::Matrix<double, 18, 18> calcurate_Jacobian_Fx(Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d, const double)
        +Eigen::Matrix<double, 18, 12> calcurate_Jacobian_Fi()
        +Eigen::Matrix<double, 18, 15> calcurate_Jacobian_Fi_WithFIS()
        +Eigen::Matrix<double, 12, 12> calcurate_Jacobian_Qi(const double)
        +Eigen::Matrix<double, 15, 15> calcurate_Jacobian_Qi_WithFIS(const double)
        +Eigen::Matrix<double, 3, 18> calcurate_Jacobian_H(State&)
        +Eigen::Quaterniond getQuaFromAA(Eigen::Vector3d)
        +Eigen::Matrix<double, 3, 3> getRotFromAA(Eigen::Vector3d)
    }
    class ROS_Interface {
        -ros::NodeHandle nh
        -bool init
        -ros::Publisher odom_path_pub
        -ros::Publisher wio_path_pub
        -ros::Publisher wo_path_pub
        -ros::Publisher wvo_path_pub
        -ros::Publisher estimated_path_pub
        -ros::Publisher state_odom_pub_
        -ros::Publisher wio_odom_pub_
        -ros::Publisher wo_odom_pub_
        -ros::Publisher wvo_odom_pub_
        -ros::Publisher wofis_pub_
        -ros::Subscriber odom_sub
        -ros::Subscriber imu_sub
        -ros::Subscriber visual_odom_sub
        -ros::Subscriber q1_sub
        -nav_msgs::Path odom_path
        -nav_msgs::Path wio_path
        -nav_msgs::Path wo_path
        -nav_msgs::Path wvo_path
        -nav_msgs::Path odom_path
        -nav_msgs::Path estimated_path
        -nav_msgs::Odometry state_pose
        -nav_msgs::Odometry wo_pose
        -nav_msgs::Odometry wo_pose
        -nav_msgs::Odometry wio_pose
        -nav_msgs::Odometry wvo_pose
        -vwio_eskf::WOFISData ros_wofisdata
        -ros::Time current_odom_time
        -ros::Time prev_odom_time
        -State x
        -WIO_Data wio_data
        -WO_Data wo_data
        -WVO_Data wvo_data
        -ESKF eskf
        -FIS_Getdata fis_getdatay
        -nav_msgs::OdometryConstPtr wo_msg
        -double delta_yaw
        -static const int WVO_DATA_INDEX
        +ROS_Interface(ros::NodeHandle&)
        +~ROS_Interface()
        +void odom_callback(const nav_msgs::OdometryConstPtr&)
        +void imu_callback(const sensor_msgs::ImuConstPtr&)
        +void visual_odom_callback(const nav_msgs::OdometryConstPtr&)
        +void data_conversion_wio(const sensor_msgs::ImuConstPtr&, WIO_Data&, double&)
        +void data_conversion_wo(double)
        +void data_conversion_wvo(const nav_msgs::OdometryConstPtr&, const WO_Data&, WVO_Data&)
        +void filter_wvo_data()
        +void publish_WOFIS()
        +void Q1_msg_callback(const vwio_eskf::Q1Data&)
        -double calculateOrientation(const Eigen::Quaterniond&)
    }
    class Filter{
        -std::vector<std::vector<Eigen::Vector3d>> m_dataLis
        -const int MAX_SENSOR_NUM
        -const int MAX_DATA_NUM
        -const int WINDOW_DATA_NUM
        +Filter(int, int, int)
        +Eigen::Vector3d slidingWindowAvgFilter(int, const Eigen::Vector3d&)
    }
    class FIS_Getdata{
        +FIS_Getdata()
        +~FIS_Getdata()
        +WOFISData getValueForWOFIS(double, double)
    }
    ROS_Interface --|> ESKF: Uses
    ROS_Interface --|> WO_Data: Uses
    ROS_Interface --|> WIO_Data: Uses
    ROS_Interface --|> WVO_Data: Uses
    ROS_Interface --|> State: Uses
    ROS_Interface --|> WOFISData: Uses
    ROS_Interface --|> Filter: Uses
    ROS_Interface --|> FIS_Getdata: Uses
    ESKF --|> State: Uses
    ESKF --|> WO_Data: Uses
    ESKF --|> WIO_Data: Uses
    ESKF --|> WVO_Data: Uses
    FIS_Getdata --|> WOFISData: Uses

```

