#ifndef __ROS_INTERFACE__
#define __ROS_INTERFACE__

#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
// #include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include "eskf.h"
#include "state_variable.h"
#include "fis_getdata.h"
#include "filter.h"
#include "vwio_eskf/WOFISData.h" 
#include "vwio_eskf/Q1Data.h"
#include "vwio_eskf/V1Data.h"

#include <fstream> // Include this for file handling

using namespace std;

class ROS_Interface
{
private:
    ros::NodeHandle nh;
    bool init;

    // Publisher
    // ros::Publisher gps_path_pub;
    ros::Publisher odom_path_pub;
    ros::Publisher wio_path_pub;
    ros::Publisher wo_path_pub;
    ros::Publisher wvo_path_pub;
    ros::Publisher estimated_path_pub;
    ros::Publisher state_odom_pub_;
    ros::Publisher wio_odom_pub_;
    ros::Publisher wo_odom_pub_;
    ros::Publisher wvo_odom_pub_;
    ros::Publisher wofis_pub_;

    // Subscriberorientation
    // ros::Subscriber gps_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber visual_odom_sub;
    ros::Subscriber q1_sub;
    ros::Subscriber v1_sub;

    // tf publish
    tf::TransformBroadcaster odom_to_baselink_broadcaster;
    geometry_msgs::TransformStamped odom_to_baselink;

    // publish data
    // nav_msgs::Path gps_path;
    nav_msgs::Path odom_path;
    nav_msgs::Path wio_path;
    nav_msgs::Path wo_path;
    nav_msgs::Path wvo_path;
    nav_msgs::Path estimated_path;
    nav_msgs::Odometry state_pose;
    nav_msgs::Odometry wo_pose;
    nav_msgs::Odometry wio_pose;
    nav_msgs::Odometry wvo_pose;
    vwio_eskf::WOFISData ros_wofisdata;

    // ROS_Time
    ros::Time current_odom_time;
    ros::Time prev_odom_time;

    // ESKF variable
    State x;
    WIO_Data wio_data;
    WO_Data wo_data;
    WVO_Data wvo_data;
    // GPS_Data gps_data;
    // map_projection_reference map_ref;
    // double lat0;
    // double lon0;
    // double alt0;

    // ESKF Instance
    ESKF eskf;
    
    // FIS_GetData Instance
    FIS_Getdata fis_getdata;

    // GEOGRAPHY Instance
    // GEOGRAPHY geography;

    nav_msgs::OdometryConstPtr wo_msg; // Member variable to store and update odom_msg

    // calculate yaw angle
    // double current_yaw;
    // double prev_yaw;
    double delta_yaw;

    // Sensor Data source for SlidingWindowAvg
    static const int WVO_DATA_INDEX = 0;

private:
    /**
     * @brief calculate the RPY angle by Quternion(w,x,y,z)
     *
     * @param orientation orientation
     * @return double yaw_angle
     */
    double calculateOrientation(const Eigen::Quaterniond &orientation);

public:
    // Init
    /**
     * @brief Construct a new ros interface object
     *
     * @param n NodeHander
     */
    ROS_Interface(ros::NodeHandle &n);

    ~ROS_Interface();

    // callback function

    /**
     * @brief Wheel Odometry Message Callback && Complete ESKF Initialization
     *
     * @param odom_msg Wheel Odometry Message(nav_msgs::Odometry)
     */
    void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg);

    /**
     * @brief IMU Message Callback && Complete ESKF Prediction
     *
     * @param imu_msg IMU Message(sensor_msgs::Imu)
     */
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);

    /**
     * @brief Visual Odometry Message Callback && Complete ESKF Correction, ESKF State_update, ESKF Error_State_Reset
     *
     * @param vodom_msg Visual Odometry Message(nav_msgs::Odometry)
     */
    void visual_odom_callback(const nav_msgs::OdometryConstPtr &vodom_msg);

    /**
     * @brief Create the Wheel-IMU Odometry(WIO)
     *
     * @param imu_msg IMU Message(sensor_msgs::Imu)
     * @param wio_data Wheel-IMU Odometry Data
     */
    void data_conversion_wio(const sensor_msgs::ImuConstPtr &imu_msg, WIO_Data &wio_data, double &dt);

    /**
     * @brief Create the Wheel Pdpmetry(WO)
     *
     * @param odom_msg Create the Wheel Odometry(WO)
     * @param dt sampling time
     * @param wo_data Wheel Odometry(WO) Data
     */
    void data_conversion_wo(double dt);

    /**
     * @brief Use the position information of WO_Data as the initial value,
     *        and then use vodom_msg to extract the inter-frame visual mileage as an increment for ESKF Correction
     * @param vodom_msg visual odometry
     * @param wo_data Wheel Odometry Data
     * @param wvo_data Wheel-Visual Odometry Data
     */
    void data_conversion_wvo(const nav_msgs::OdometryConstPtr &vodom_msg, const WO_Data &wo_data, WVO_Data &wvo_data);

    /**
     * @brief filter the Wheel-Visual odometry(WVO) data by SlidingWindowAvg
     */
    void filter_wvo_data();

    /**
     * @brief Publish Ros Topic OF WOFIS including Velocity Difference and Angualr Velocity  
     * 
     */
    void publish_WOFIS();

    /**
     * @brief Q1 noise
     * 
     */
    void Q1_msg_callback(const vwio_eskf::Q1Data &ros_q1_data);

    
    void V1_msg_callback(const vwio_eskf::V1Data &ros_v1_data);
};

/***********************************************************************
 * Initialize
 **********************************************************************/
ROS_Interface::ROS_Interface(ros::NodeHandle &n)
{
    // init ROS
    cout << "ROS_Interface Start!" << endl;
    nh = n;
    init = false;

    // Publisher
    // gps_path_pub = nh.advertise<nav_msgs::Path>("/gps_path", 10);
    odom_path_pub = nh.advertise<nav_msgs::Path>("/odom_path", 10);
    wio_path_pub = nh.advertise<nav_msgs::Path>("/wio_path", 10);
    wo_path_pub = nh.advertise<nav_msgs::Path>("/wo_path", 10);
    wvo_path_pub = nh.advertise<nav_msgs::Path>("/wvo_path", 10);
    estimated_path_pub = nh.advertise<nav_msgs::Path>("/estimated_path", 10);
    state_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/state_pose", 50);
    wo_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/wo_pose", 50);
    wio_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/wio_pose", 50);
    wvo_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/wvo_pose", 50);
    wofis_pub_ = nh.advertise<vwio_eskf::WOFISData>("/WOFIS_msg", 100);

    // Subscriber
    // gps_sub = nh.subscribe("/fix", 10, &ROS_Interface::gps_callback, this);
    odom_sub = nh.subscribe("/wo_sync", 10, &ROS_Interface::odom_callback, this);
    imu_sub = nh.subscribe("/imu_sync", 10, &ROS_Interface::imu_callback, this);
    visual_odom_sub = nh.subscribe("/vo_sync", 10, &ROS_Interface::visual_odom_callback, this);
    q1_sub = nh.subscribe("/Q1_msg", 10, &ROS_Interface::Q1_msg_callback, this);
    v1_sub = nh.subscribe("/V1_msg", 10, &ROS_Interface::V1_msg_callback, this);

    // init odom_path
    odom_path.header.frame_id = "odom";
    odom_path.header.stamp = ros::Time::now();
    odom_path.header.seq = 0;

    // init wio_path
    wio_path.header.frame_id = "odom";
    wio_path.header.stamp = ros::Time::now();
    wio_path.header.seq = 0;

    // init wo_path
    wo_path.header.frame_id = "odom";
    wo_path.header.stamp = ros::Time::now();
    wo_path.header.seq = 0;

    // init wvo_path
    wvo_path.header.frame_id = "odom";
    wvo_path.header.stamp = ros::Time::now();
    wvo_path.header.seq = 0;

    // init estimated_path
    estimated_path.header.frame_id = "odom";
    estimated_path.header.stamp = ros::Time::now();
    estimated_path.header.seq = 0;
}

ROS_Interface::~ROS_Interface()
{
    cout << "ROS_Interface Finish" << endl;
}

/***********************************************************************
 * Callback function(odom/imu/visual message)
 **********************************************************************/

void ROS_Interface::odom_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{

    double delta_time = 0.0;
    wo_msg = odom_msg; // store and update the information of odom_msgodom

    // If the system have NOT been INITIALIZED
    if (!init)
    {
        eskf.Init(odom_msg, x);
        init = true;
    }
    else
    {
        printf("ESKF_Init Status: %d\n", init);
    }
}

void ROS_Interface::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    const char* RESET   = "\033[0m";
    const char* GREEN   = "\033[32m";
    const char* YELLOW  = "\033[33m";

    double dt = 0.0;
    delta_yaw = 0.0;
    data_conversion_wio(imu_msg, wio_data, dt);
    data_conversion_wo(dt);
    publish_WOFIS();

    // If the system have NOT been INITIALIZED
    if (!init)
    {
        return;
    }

    eskf.Predict(wio_data, wo_data, dt, x);

    // std::cout << GREEN << "WIO Position: [" 
    //           << wio_data.position[0] << ", " 
    //           << wio_data.position[1] << ", " 
    //           << wio_data.position[2] << "]" 
    //           << RESET << std::endl;

    // std::cout << YELLOW << "Predict Position: [" 
    //           << x.position[0] << ", " 
    //           << x.position[1] << ", " 
    //           << x.position[2] << "]" 
    //           << RESET << std::endl;

    // std::cout << GREEN << "WIO Orientation: [" 
    //           << wio_data.quaternion.w() << ", " 
    //           << wio_data.quaternion.x() << ", " 
    //           << wio_data.quaternion.y() << ", " 
    //           << wio_data.quaternion.z() << "]" 
    //           << RESET << std::endl;

    // std::cout << YELLOW << "Predict Orientation: [" 
    //           << x.quaternion.w() << ", " 
    //           << x.quaternion.x() << ", " 
    //           << x.quaternion.y() << ", " 
    //           << x.quaternion.z() << "]" 
    //           << RESET << std::endl; 

    // Eigen::Matrix3d R2 = wo_data.quaternion.toRotationMatrix();
    // std::cout << GREEN << "R2 =" << R2 << std::endl;

}

void ROS_Interface::data_conversion_wio(const sensor_msgs::ImuConstPtr &imu_msg, WIO_Data &wio_data, double &dt)
{
    // previous linear velocity
    static Eigen::Vector3d prev_linear_vel;

    // previous angular velocity
    static Eigen::Vector3d prev_angular_vel;

    // timestamp
    wio_data.timestamp = ros::Time::now().toSec();

    // calcurate sampling time
    dt = wio_data.timestamp - x.timestamp;
    x.timestamp = wio_data.timestamp;

    // linear_velocity
    wio_data.linear_vel = Eigen::Vector3d(wo_msg->twist.twist.linear.x,
                                          wo_msg->twist.twist.linear.y,
                                          wo_msg->twist.twist.linear.z);

    // angular_velocity
    wio_data.angular_vel = Eigen::Vector3d(imu_msg->angular_velocity.x,
                                           imu_msg->angular_velocity.y,
                                           imu_msg->angular_velocity.z);

    // linear_acceration
    if (dt != 0)
    {
        wio_data.acc = (wio_data.linear_vel - prev_linear_vel) / dt;
    }
    else
    {
        wio_data.acc.setZero();
    }
    prev_linear_vel = wio_data.linear_vel;

    // angular_acceration
    if (dt != 0)
    {
        wio_data.gyro = (wio_data.angular_vel - prev_angular_vel) / dt;
    }
    else
    {
        wio_data.gyro.setZero();
    }

    // Orientation
    wio_data.quaternion = Eigen::Quaterniond(imu_msg->orientation.w,
                                             imu_msg->orientation.x,
                                             imu_msg->orientation.y,
                                             imu_msg->orientation.z);

    // current_yaw = calculateOrientation(wio_data.orientation);

    // Calculate the yaw angle change
    delta_yaw = wio_data.angular_vel[2] * dt;

    // ROS_INFO("\033[1;33mdelta_yaw: %.2f degrees\037", delta_yaw * 180.0 / M_PI);

    // Check if the change in yaw angle is within a small threshold
    if (delta_yaw * 180.0 / M_PI < 0.1 && delta_yaw * 180.0 / M_PI > -0.1)
    {
        // If the change in yaw angle is negligible, set it to zero
        delta_yaw = 0.0;
    }

    // Update and accumulate yaw angle
    wio_data.yaw_angle += delta_yaw;

    // Limit yaw angle to range [-π, π]
    wio_data.yaw_angle = std::fmod(wio_data.yaw_angle, 2 * M_PI);
    if (wio_data.yaw_angle > M_PI)
    {
        wio_data.yaw_angle -= 2 * M_PI;
    }
    else if (wio_data.yaw_angle < -M_PI)
    {
        wio_data.yaw_angle += 2 * M_PI;
    }

    // ROS_INFO("\033[1;31mWIO_Data.yaw_angle: %.2f degrees\033[0m", wio_data.yaw_angle * 180.0 / M_PI);

    // ROS_INFO("\033[1;31mWIO_Data.linear_vel[0]: %.2f m/s\033[0m", wio_data.linear_vel[0]);

    wio_data.position[0] += wio_data.linear_vel[0] * cos(wio_data.yaw_angle) * dt;
    wio_data.position[1] += wio_data.linear_vel[0] * sin(wio_data.yaw_angle) * dt;
    wio_data.position[2] = 0.0;
}

void ROS_Interface::data_conversion_wo(double dt)
{

    // Linear_velocity
    wo_data.linear_vel = Eigen::Vector3d(wo_msg->twist.twist.linear.x,
                                         wo_msg->twist.twist.linear.y,
                                         wo_msg->twist.twist.linear.z);

    // Angular_velocity
    wo_data.angular_vel = Eigen::Vector3d(wo_msg->twist.twist.angular.x,
                                          wo_msg->twist.twist.angular.y,
                                          wo_msg->twist.twist.angular.z);

    // Orientation
    wo_data.quaternion = Eigen::Quaterniond(wo_msg->pose.pose.orientation.w,
                                            wo_msg->pose.pose.orientation.x,
                                            wo_msg->pose.pose.orientation.y,
                                            wo_msg->pose.pose.orientation.z);

    // yaw_angle
    wo_data.yaw_angle = calculateOrientation(wo_data.quaternion);

    // position
    wo_data.position[0] += wo_data.linear_vel[0] * cos(wo_data.yaw_angle) * dt;
    wo_data.position[1] += wo_data.linear_vel[0] * sin(wo_data.yaw_angle) * dt;
    wo_data.position[2] = 0.0;
}

void ROS_Interface::visual_odom_callback(const nav_msgs::OdometryConstPtr &vodom_msg)
{
    // Wheel-Visual-Odometry
    data_conversion_wvo(vodom_msg, wo_data, wvo_data);

    // filter the Wheel-Visual-Odometry data by SlidingWindowAvg
    filter_wvo_data();

    // ESKF Correction
    // eskf.Correct(wvo_data, x);
    eskf.CorrectWithIterators(wvo_data, 5, 1e-6, x);

    // ESKF State_update
    eskf.State_update(x);

    // ESKF Error_State_Reset
    eskf.Error_State_Reset(x);

    /*---------------------------------------------------------------------
     * Publish nav_msgs::OdometryPath
     --------------------------------------------------------------------*/

    // publish odom_path
    geometry_msgs::PoseStamped odom_point;
    odom_point.header.frame_id = "odom";
    odom_point.header.stamp = ros::Time::now();
    odom_point.pose.position.x = wo_msg->pose.pose.position.x;
    odom_point.pose.position.y = wo_msg->pose.pose.position.y;
    odom_point.pose.position.z = 0.0;
    odom_point.pose.orientation.w = wo_msg->pose.pose.orientation.w;
    odom_point.pose.orientation.x = wo_msg->pose.pose.orientation.x;
    odom_point.pose.orientation.y = wo_msg->pose.pose.orientation.y;
    odom_point.pose.orientation.z = wo_msg->pose.pose.orientation.z;
    odom_path.poses.push_back(odom_point);
    odom_path_pub.publish(odom_path);

    // publish wio_path
    geometry_msgs::PoseStamped wio_point;
    wio_point.header.frame_id = "odom";
    wio_point.header.stamp = ros::Time::now();
    wio_point.pose.position.x = wio_data.position[0];
    wio_point.pose.position.y = wio_data.position[1];
    wio_point.pose.position.z = 0.0;
    wio_point.pose.orientation.w = wio_data.quaternion.w();
    wio_point.pose.orientation.x = wio_data.quaternion.x();
    wio_point.pose.orientation.y = wio_data.quaternion.y();
    wio_point.pose.orientation.z = wio_data.quaternion.z();
    wio_path.poses.push_back(wio_point);
    wio_path_pub.publish(wio_path);

    // publish wo_path
    geometry_msgs::PoseStamped wo_point;
    wo_point.header.frame_id = "odom";
    wo_point.header.stamp = ros::Time::now();
    wo_point.pose.position.x = wo_data.position[0];
    wo_point.pose.position.y = wo_data.position[1];
    wo_point.pose.position.z = 0.0;
    wo_point.pose.orientation.w = wo_data.quaternion.w();
    wo_point.pose.orientation.x = wo_data.quaternion.x();
    wo_point.pose.orientation.y = wo_data.quaternion.y();
    wo_point.pose.orientation.z = wo_data.quaternion.z();
    wo_path.poses.push_back(wo_point);
    wo_path_pub.publish(wo_path);

    // publish wvo_path
    geometry_msgs::PoseStamped wvo_point;
    wvo_point.header.frame_id = "odom";
    wvo_point.header.stamp = ros::Time::now();
    wvo_point.pose.position.x = wvo_data.position[0];
    wvo_point.pose.position.y = wvo_data.position[1];
    wvo_point.pose.position.z = 0.0;
    wvo_path.poses.push_back(wvo_point);
    wvo_path_pub.publish(wvo_path);

    // publish estimated_path
    geometry_msgs::PoseStamped estimated_point;
    estimated_point.header.frame_id = "odom";
    estimated_point.header.stamp = ros::Time::now();
    estimated_point.pose.position.x = x.position[0];
    estimated_point.pose.position.y = x.position[1];
    estimated_point.pose.position.z = 0.0;
    estimated_point.pose.orientation.x = x.quaternion.x();
    estimated_point.pose.orientation.y = x.quaternion.y();
    estimated_point.pose.orientation.z = x.quaternion.z();
    estimated_point.pose.orientation.w = x.quaternion.w();

    estimated_path.poses.push_back(estimated_point);
    estimated_path_pub.publish(estimated_path);

    /*---------------------------------------------------------------------
     * Publish nav_msgs::Odometry
     --------------------------------------------------------------------*/

    static ros::Time base_time = ros::Time::now(); // 使用静态变量保存基础时间
    ros::Duration time_increment(0.02);            // 0.02秒的时间增量

    // publish wo_pose
    base_time += time_increment;
    wo_pose.header.frame_id = "odom";
    wo_pose.child_frame_id = "base_link";
    wo_pose.header.stamp = base_time;
    wo_pose.pose.pose.position.x = wo_data.position[0];
    wo_pose.pose.pose.position.y = wo_data.position[1];
    wo_pose.pose.pose.position.z = 0.0;
    wo_pose.pose.pose.orientation.x = wo_data.quaternion.x();
    wo_pose.pose.pose.orientation.y = wo_data.quaternion.y();
    wo_pose.pose.pose.orientation.z = wo_data.quaternion.z();
    wo_pose.pose.pose.orientation.w = wo_data.quaternion.w();
    wo_odom_pub_.publish(wo_pose);

    // publish wio_pose
    base_time += time_increment;
    wio_pose.header.frame_id = "odom";
    wio_pose.child_frame_id = "base_link";
    wio_pose.header.stamp = base_time;
    wio_pose.pose.pose.position.x = wio_data.position[0];
    wio_pose.pose.pose.position.y = wio_data.position[1];
    wio_pose.pose.pose.position.z = 0.0;
    wio_pose.pose.pose.orientation.x = wio_data.quaternion.x();
    wio_pose.pose.pose.orientation.y = wio_data.quaternion.y();
    wio_pose.pose.pose.orientation.z = wio_data.quaternion.z();
    wio_pose.pose.pose.orientation.w = wio_data.quaternion.w();
    wio_odom_pub_.publish(wio_pose);

    // publish wvo_pose
    base_time += time_increment;
    wvo_pose.header.frame_id = "odom";
    wvo_pose.child_frame_id = "base_link";
    wvo_pose.header.stamp = base_time;
    wvo_pose.pose.pose.position.x = wvo_data.position[0];
    wvo_pose.pose.pose.position.y = wvo_data.position[1];
    wvo_pose.pose.pose.position.z = 0.0;
    wvo_pose.pose.pose.orientation.x = wvo_data.quaternion.x();
    wvo_pose.pose.pose.orientation.y = wvo_data.quaternion.y();
    wvo_pose.pose.pose.orientation.z = wvo_data.quaternion.z();
    wvo_pose.pose.pose.orientation.w = wvo_data.quaternion.w();
    wvo_odom_pub_.publish(wvo_pose);

    // publish state_pose
    base_time += time_increment;
    state_pose.header.frame_id = "odom";
    state_pose.child_frame_id = "base_link";
    state_pose.header.stamp = base_time;
    state_pose.pose.pose.position.x = x.position[0];
    state_pose.pose.pose.position.y = x.position[1];
    state_pose.pose.pose.position.z = 0.0;
    state_pose.pose.pose.orientation.x = x.quaternion.x();
    state_pose.pose.pose.orientation.y = x.quaternion.y();
    state_pose.pose.pose.orientation.z = x.quaternion.z();
    state_pose.pose.pose.orientation.w = x.quaternion.w();
    state_odom_pub_.publish(state_pose);


    /*---------------------------------------------------------------------
     * Publish geometry_msgs::TransformStamped
     --------------------------------------------------------------------*/

    // /odom to /base_link transform broadcast
    odom_to_baselink.header.stamp = ros::Time::now();
    odom_to_baselink.header.frame_id = "odom";
    odom_to_baselink.child_frame_id = "base_link";
    odom_to_baselink.transform.translation.x = x.position[0];
    odom_to_baselink.transform.translation.y = x.position[1];
    odom_to_baselink.transform.translation.z = 0.0;
    odom_to_baselink.transform.rotation.x = x.quaternion.x();
    odom_to_baselink.transform.rotation.y = x.quaternion.y();
    odom_to_baselink.transform.rotation.z = x.quaternion.z();
    odom_to_baselink.transform.rotation.w = x.quaternion.w();
    odom_to_baselink_broadcaster.sendTransform(odom_to_baselink);

}

void ROS_Interface::data_conversion_wvo(const nav_msgs::OdometryConstPtr &vodom_msg, const WO_Data &wo_data, WVO_Data &wvo_data)
{
    // Set the threshold
    const double min_threshold = 0.5;
    const double max_threshold = 65.0;

    // Define a flag to check if initialization is done
    static bool is_initialized = false;

    // Previous visual odometry
    static Eigen::Vector3d prev_vodom;

    // Current visual odometry
    Eigen::Vector3d current_vodom(vodom_msg->pose.pose.position.x,
                                  vodom_msg->pose.pose.position.y,
                                  0.0);

    // If not initialized, set the initial values
    if (!is_initialized)
    {
        prev_vodom = current_vodom;
        is_initialized = true;
    }

    // If WO data indicates the robot has not moved, ignore visual odometry data
    if (wo_data.position[0] == 0.0 && wo_data.position[1] == 0.0)
    {
        // Reset prev_vodom to current_vodom to avoid large jumps when WO data starts updating
        prev_vodom = current_vodom;
        // Use WO data directly
        wvo_data.position[0] = 0.0;
        wvo_data.position[1] = 0.0;
        wvo_data.position[2] = 0.0;
        return;
    }

    // Calculate incremental position
    Eigen::Vector3d incremental_vo = current_vodom - prev_vodom;

    // Update the previous visual odometry only if the change is within the threshold
    prev_vodom = current_vodom;

    // Check if the incremental position exceeds the threshold
    if (incremental_vo.norm() >= min_threshold && incremental_vo.norm() <= max_threshold)
    {
        // If the change is too large, ignore this visual odometry data
        incremental_vo = Eigen::Vector3d::Zero();
    }

    // Update WVO position data
    wvo_data.position[0] = wo_data.position[0] + incremental_vo[0];
    wvo_data.position[1] = wo_data.position[1] + incremental_vo[1];
    wvo_data.position[2] = 0.0;
}

double ROS_Interface::calculateOrientation(const Eigen::Quaterniond &orientation)
{
    // Convert Eigen::Quaterniond to rotation matrix and extract roll, pitch, yaw
    Eigen::Matrix3d mat = orientation.toRotationMatrix();

    // Calculate roll, pitch, yaw
    double roll, pitch, yaw;
    pitch = asin(-mat(2, 0));

    // Check for singularities at the poles
    if (fabs(pitch - M_PI / 2) < 1.0e-3) // Near the North Pole
    {
        roll = 0.0; // Assume roll is 0
        yaw = atan2(mat(1, 2), mat(0, 2));
    }
    else if (fabs(pitch + M_PI / 2) < 1.0e-3) // Near the South Pole
    {
        roll = 0.0; // Assume roll is 0
        yaw = atan2(-mat(1, 2), -mat(0, 2));
    }
    else
    {
        roll = atan2(mat(2, 1), mat(2, 2));
        yaw = atan2(mat(1, 0), mat(0, 0));
    }

    // ROS_INFO("\033[1;32mWO Orientation: Roll=%f, Pitch=%f, Yaw=%f\033[0m",
    //          roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

    return yaw;
}

void ROS_Interface::filter_wvo_data()
{
    const int max_sensor_num = 9;
    const int max_data_num = 9;
    const int window_data_num = 7;

    static Filter filter(max_sensor_num, max_data_num, window_data_num);

    Eigen::Vector3d wvo_position(wvo_data.position[0], wvo_data.position[1], wvo_data.position[2]);

    Eigen::Vector3d filtered_wvo_position = filter.slidingWindowAvgFilter(WVO_DATA_INDEX, wvo_position);

    wvo_data.position[0] = filtered_wvo_position[0];
    wvo_data.position[1] = filtered_wvo_position[1];
    wvo_data.position[2] = filtered_wvo_position[2];
}

void ROS_Interface::publish_WOFIS()
{
    // 获取 WOFISData 类型数据
    WOFISData custom_wofisdata = fis_getdata.getValueForWOFIS(wo_data.linear_vel[0], wo_data.angular_vel[2]);

    // 将 WOFISData 类型转换为 vwio_eskf::WOFISData 类型
    ros_wofisdata.delta_v = custom_wofisdata.delta_v;
    ros_wofisdata.w_z = custom_wofisdata.w_z;
    ros_wofisdata.v_x = custom_wofisdata.v_x;
    wofis_pub_.publish(ros_wofisdata);
}

void ROS_Interface::Q1_msg_callback(const vwio_eskf::Q1Data &ros_q1_data)
{
    eskf.custom_q1_data.q1 = ros_q1_data.q1;
}

void ROS_Interface::V1_msg_callback(const vwio_eskf::V1Data &ros_v1_data)
{
    eskf.custom_v1_data.v1 = ros_v1_data.v1;
}

#endif // ROS_INTERFACE