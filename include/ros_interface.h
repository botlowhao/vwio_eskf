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
    ros::Publisher vo_path_pub;
    ros::Publisher estimated_path_pub;
    ros::Publisher estimated_pose_pub;

    // Subscriber
    // ros::Subscriber gps_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber visual_odom_sub;

    // tf publish
    tf::TransformBroadcaster odom_to_baselink_broadcaster;
    geometry_msgs::TransformStamped odom_to_baselink;

    // publish data
    // nav_msgs::Path gps_path;
    nav_msgs::Path odom_path;
    nav_msgs::Path wio_path;
    nav_msgs::Path vo_path;
    nav_msgs::Path estimated_path;
    nav_msgs::Odometry estimated_pose;

    // ESKF variable
    State x;
    WIO_Data wio_data;
    // IMU_Data imu_data;
    // GPS_Data gps_data;
    // map_projection_reference map_ref;
    // double lat0;
    // double lon0;
    // double alt0;

    // ESKF Instance
    ESKF eskf;

    // GEOGRAPHY Instance
    // GEOGRAPHY geography;

    nav_msgs::OdometryConstPtr wo_msg; // Member variable to store and update odom_msg

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
     * @brief Use the position information of WIO_Data as the initial value,
     *        and then use vodom_msg to extract the inter-frame visual mileage as an increment for ESKF Correction
     * @param vodom_msg visual odometry
     * @param wio_data Wheel-IMU Odometry Data
     * @param vo Inter-frame visual Odometry
     */
    void data_conversion_vo(const nav_msgs::OdometryConstPtr &vodom_msg, const WIO_Data &wio_data, nav_msgs::Odometry &vo);
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
    vo_path_pub = nh.advertise<nav_msgs::Path>("/vo_path", 10);
    estimated_path_pub = nh.advertise<nav_msgs::Path>("/estimated_path", 10);
    estimated_pose_pub = nh.advertise<nav_msgs::Odometry>("/estimated_pose", 10);

    // Subscriber
    // gps_sub = nh.subscribe("/fix", 10, &ROS_Interface::gps_callback, this);
    odom_sub = nh.subscribe("/wo_sync", 10, &ROS_Interface::odom_callback, this);
    imu_sub = nh.subscribe("/imu_sync", 10, &ROS_Interface::imu_callback, this);
    visual_odom_sub = nh.subscribe("/vo_sync", 10, &ROS_Interface::visual_odom_callback, this);

    // init odom_path
    odom_path.header.frame_id = "map";
    odom_path.header.stamp = ros::Time::now();
    odom_path.header.seq = 0;

    // init wio_path
    wio_path.header.frame_id = "map";
    wio_path.header.stamp = ros::Time::now();
    wio_path.header.seq = 0;

    // init vo_path
    vo_path.header.frame_id = "map";
    vo_path.header.stamp = ros::Time::now();
    vo_path.header.seq = 0;

    // init estimated_path
    estimated_path.header.frame_id = "map";
    estimated_path.header.stamp = ros::Time::now();
    estimated_path.header.seq = 0;

    // init estimated_pose
    estimated_pose.header.frame_id = "map";
    estimated_pose.child_frame_id = "base_link";
    estimated_pose.header.stamp = ros::Time::now();

    // init state
    /*
    x.position = Eigen::Vector3d::Zero();
    x.velocity = Eigen::Vector3d::Zero();
    */
    // x.quaternion = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    /*
    x.acc_bias = Eigen::Vector3d::Zero();
    x.gyro_bias = Eigen::Vector3d::Zero();
    */

    // x.gravity = Eigen::Vector3d(0., 0., 9.81007); // ned frame
    x.gravity = Eigen::Vector3d(0., 0., -9.81007); // enu frame
    /*
    x.PEst = Eigen::Matrix<double, 18, 18>::Zero();
    x.error = Eigen::Matrix<double, 18, 1>::Zero();
    */

    // init reference lat, lon projection
    // geography.map_projection_init(&map_ref, lat, lon);
    // lat0 = lat;
    // lon0 = lon;
    // alt0 = 0.0;
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

    wo_msg = odom_msg; // store and update the information of odom_msg

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
    double dt = 0.0;
    data_conversion_wio(imu_msg, wio_data, dt);

    // If the system have NOT been INITIALIZED
    if (!init)
    {
        return;
    }

    eskf.Predict(wio_data, dt, x);
}

void ROS_Interface::data_conversion_wio(const sensor_msgs::ImuConstPtr &imu_msg, WIO_Data &wio_data, double &dt)
{
    // previous linear velocity
    static Eigen::Vector3d prev_linear_vel;

    // previous linear velocity
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
    prev_angular_vel = wio_data.angular_vel;

    // Orientation
    wio_data.orientation = Eigen::Quaterniond(imu_msg->orientation.w,
                                              imu_msg->orientation.x,
                                              imu_msg->orientation.y,
                                              imu_msg->orientation.z);

    // yaw_angle
    double yaw_angle = yaw_angle + wio_data.angular_vel[2] * dt;

    // position
    wio_data.position = Eigen::Vector3d(wio_data.position[0] + wio_data.linear_vel[0] * cos(yaw_angle) * dt,
                                        wio_data.position[1] + wio_data.linear_vel[0] * sin(yaw_angle) * dt,
                                        0.0);
}

void ROS_Interface::visual_odom_callback(const nav_msgs::OdometryConstPtr &vodom_msg)
{
    // Create a new nav_msgs::Odometry object
    nav_msgs::Odometry vo;

    // Inter-frame Visual Odometry
    data_conversion_vo(vodom_msg, wio_data, vo);

    // ESKF Correction
    eskf.Correct(vo, x);

    // ESKF State_update
    eskf.State_update(x);

    // ESKF Error_State_Reset
    eskf.Error_State_Reset(x);

    // publish odom_path
    geometry_msgs::PoseStamped wo_point;
    wo_point.header.frame_id = "map";
    wo_point.header.stamp = ros::Time::now();
    wo_point.pose.position.x = wo_msg->pose.pose.position.x;
    wo_point.pose.position.y = wo_msg->pose.pose.position.y;
    wo_point.pose.position.z = 0.0;
    wo_point.pose.orientation.w = wo_msg->pose.pose.orientation.w;
    wo_point.pose.orientation.x = wo_msg->pose.pose.orientation.x;
    wo_point.pose.orientation.y = wo_msg->pose.pose.orientation.y;
    wo_point.pose.orientation.z = wo_msg->pose.pose.orientation.z;
    odom_path.poses.push_back(wo_point);
    odom_path_pub.publish(odom_path);

    // publish wio_path
    geometry_msgs::PoseStamped wio_point;
    wio_point.header.frame_id = "map";
    wio_point.header.stamp = ros::Time::now();
    wio_point.pose.position.x = wio_data.position[0];
    wio_point.pose.position.y = wio_data.position[1];
    wio_point.pose.position.z = 0.0;
    wio_point.pose.orientation.w = wio_data.orientation.w();
    wio_point.pose.orientation.x = wio_data.orientation.x();
    wio_point.pose.orientation.y = wio_data.orientation.y();
    wio_point.pose.orientation.z = wio_data.orientation.z();
    wio_path.poses.push_back(wio_point);
    wio_path_pub.publish(wio_path);

    // publish vo_path
    geometry_msgs::PoseStamped vo_point;
    vo_point.header.frame_id = "map";
    vo_point.header.stamp = ros::Time::now();
    vo_point.pose.position.x = vodom_msg->pose.pose.position.x;
    vo_point.pose.position.y = vodom_msg->pose.pose.position.y;
    vo_point.pose.position.z = 0.0;
    vo_point.pose.orientation.w = vodom_msg->pose.pose.orientation.w;
    vo_point.pose.orientation.x = vodom_msg->pose.pose.orientation.x;
    vo_point.pose.orientation.y = vodom_msg->pose.pose.orientation.y;
    vo_point.pose.orientation.z = vodom_msg->pose.pose.orientation.z;
    vo_path.poses.push_back(vo_point);
    vo_path_pub.publish(vo_path);

    // publish estimated_pose
    estimated_pose.pose.pose.position.x = x.position[0];
    estimated_pose.pose.pose.position.y = x.position[1];
    estimated_pose.pose.pose.position.z = 0.0;
    estimated_pose.pose.pose.orientation.x = x.quaternion.x();
    estimated_pose.pose.pose.orientation.y = x.quaternion.y();
    estimated_pose.pose.pose.orientation.z = x.quaternion.z();
    estimated_pose.pose.pose.orientation.w = x.quaternion.w();
    estimated_pose_pub.publish(estimated_pose);

    // /odom to /base_link transform broadcast
    // odom_to_baselink.header.stamp = ros::Time::now();
    // odom_to_baselink.header.frame_id = "odom";
    // odom_to_baselink.child_frame_id = "base_link";
    // odom_to_baselink.transform.translation.x = x.position[0];
    // odom_to_baselink.transform.translation.y = x.position[1];
    // odom_to_baselink.transform.translation.z = 0.0;
    // odom_to_baselink.transform.rotation.x = x.quaternion.x();
    // odom_to_baselink.transform.rotation.y = x.quaternion.y();
    // odom_to_baselink.transform.rotation.z = x.quaternion.z();
    // odom_to_baselink.transform.rotation.w = x.quaternion.w();
    // odom_to_baselink_broadcaster.sendTransform(odom_to_baselink);

    // publish estimated_path
    geometry_msgs::PoseStamped estimated_point;
    estimated_point.header.frame_id = "map";
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
}

void ROS_Interface::data_conversion_vo(const nav_msgs::OdometryConstPtr &vodom_msg, const WIO_Data &wio_data, nav_msgs::Odometry &vo)
{
    // previous visual odometry
    static Eigen::Vector3d prev_vodom;

    // current visual odometry
    Eigen::Vector3d current_vodom(vodom_msg->pose.pose.position.x,
                                  vodom_msg->pose.pose.position.y,
                                  0.0);

    // incremental_position
    Eigen::Vector3d incremental_position = current_vodom - prev_vodom;

    // updata the previous visual odometry
    prev_vodom = current_vodom;

    // visual odometry data updata
    vo.pose.pose.position.x = wio_data.position[0] + incremental_position[0];
    vo.pose.pose.position.y = wio_data.position[1] + incremental_position[1];
    vo.pose.pose.position.z = 0.0;
}

#endif // ROS_INTERFACE