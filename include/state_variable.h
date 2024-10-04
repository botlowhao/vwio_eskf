#ifndef __STATE_VARIABLE__
#define __STATE_VARIABLE__

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>


struct WO_Data
{
    double timestamp;

    Eigen::Vector3d position;
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d linear_vel;
    Eigen::Vector3d angular_vel;
    double yaw_angle;

    WO_Data() : timestamp(0.0),
                position(Eigen::Vector3d(0.0, 0.0, 0.0)),
                quaternion(Eigen::Quaterniond::Identity()),
                linear_vel(Eigen::Vector3d(0.0, 0.0, 0.0)),
                angular_vel(Eigen::Vector3d(0.0, 0.0, 0.0)),
                yaw_angle(0.0)
    {
    }
};

// struct GPS_Data
// {
//     double timestamp;

//     Eigen::Vector3d lla;
//     Eigen::Vector3d ned;
// };

struct WIO_Data
{
    double timestamp;

    Eigen::Vector3d position;
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d linear_vel;
    Eigen::Vector3d angular_vel;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    double yaw_angle;

    WIO_Data() : timestamp(0.0),
                 position(Eigen::Vector3d(0.0, 0.0, 0.0)),
                 quaternion(Eigen::Quaterniond::Identity()),
                 linear_vel(Eigen::Vector3d(0.0, 0.0, 0.0)),
                 angular_vel(Eigen::Vector3d(0.0, 0.0, 0.0)),
                 acc(Eigen::Vector3d(0.0, 0.0, 0.0)),
                 gyro(Eigen::Vector3d(0.0, 0.0, 0.0)),
                 yaw_angle(0.0)
    {
    }
};

struct WVO_Data
{
    double timestamp;

    Eigen::Vector3d position;
    Eigen::Quaterniond quaternion;

    WVO_Data() : timestamp(0.0),
                 position(Eigen::Vector3d(0.0, 0.0, 0.0)),
                 quaternion(Eigen::Quaterniond::Identity())
    {
    }
};


struct State
{
    double timestamp;

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d gravity;
    Eigen::Matrix<double, 18, 18> PEst;
    Eigen::Matrix<double, 18, 1> error;


    State() : timestamp(0.0),
              position(Eigen::Vector3d::Zero()),
              velocity(Eigen::Vector3d::Zero()),
              quaternion(Eigen::Quaterniond::Identity()),
            //   quaternion(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)),
              acc_bias(Eigen::Vector3d::Zero()),
              gyro_bias(Eigen::Vector3d::Zero()),
              gravity(Eigen::Vector3d(0., 0., -9.81007)),
              PEst(Eigen::Matrix<double, 18, 18>::Zero()),
              error(Eigen::Matrix<double, 18, 1>::Zero())
    {

    }

};

struct WOFISData {
    double delta_v;
    double w_z;
    double v_x;

    WOFISData() : delta_v(0.0),
                  w_z(0.0),
                  v_x(0.0)

    {

    }

};

/* lat/lon are in radians */
// struct map_projection_reference {
// 	uint64_t timestamp;
// 	double lat_rad;
// 	double lon_rad;
// 	double sin_lat;
// 	double cos_lat;
// 	bool init_done;
// };

#endif // STATE_VARIABLE