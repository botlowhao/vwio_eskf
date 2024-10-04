#ifndef __FIS_GETDATA__
#define __FIS_GETDATA__

#include <iostream>
#include <fstream>
#include <cmath>
#include "state_variable.h"

using namespace std;

// FIS_GetData class for FIS Data Preparation
class FIS_Getdata 
{

public:
    FIS_Getdata();
    ~FIS_Getdata();

    /**
     * @brief Get the Value For WOFIS object
     * 
     * @param v_x 
     * @param w_z 
     * @return WOFISData containing delta_v and w_z
     */
    WOFISData getValueForWOFIS(double v_x, double w_z);
 
};

FIS_Getdata::FIS_Getdata()
{
    cout << "Fis_GetData Start!" << endl;
}
FIS_Getdata::~FIS_Getdata()
{
    cout << "Fis_GetData Start!" << endl;
}  

WOFISData FIS_Getdata::getValueForWOFIS(double v_x, double w_z) 
{
    // Parameters: wheelbase and wheel radius
    const double b = 0.22; // Wheelbase in meters
    const double r = 0.04; // Wheel radius in meters

    // Define the velocities of the left and right wheels and the wheel speed difference
    double v_l, v_r, delta_v;

    // Calculate the velocities of the left and right wheels respectively (in RPM)
    v_l = (2 * v_x + w_z * b) * 60 / (4 * M_PI * r);
    v_r = (2 * v_x - w_z * b) * 60 / (4 * M_PI * r);

    // Print v_l and v_r to the terminal
    // std::cout << "Left wheel velocity (v_l): " << v_l << " RPM" << std::endl;
    // std::cout << "Right wheel velocity (v_r): " << v_r << " RPM" << std::endl;

    // Calculate the wheel speed difference and take the absolute value (in RPM)
    // delta_v = std::abs(v_l - v_r);
    delta_v = v_r - v_l;
    // std::cout << "Velocity Difference(delta_v): " << delta_v << " RPM" << std::endl;
    // std::cout << "Angular Velocity of Z-Axis (w_z): " << w_z << " rad/s" << std::endl;

    // // Open the CSV file in append mode
    std::ofstream outfile;
    outfile.open("/home/wyatt/catkin_ws/src/fis_wo/ANFIS_dateset/FIS_WO_81.csv", std::ios_base::app);

    // Check if the file was successfully opened
    if (outfile.is_open())
    {
        // If the file is empty, write the column headers
        outfile.seekp(0, std::ios::end);
        if (outfile.tellp() == 0)
        {
            outfile << "delta_v (RPM),w (rad/s),v_x (m/s)\n";
        }

        // Write delta_v and w to the CSV file
        // delta_v in RPM, w in radians/second
        outfile << delta_v << "," << w_z << "," << v_x << "\n";

        // Close the file
        outfile.close();
    }
    else
    {
        // If the file cannot be opened, output an error message
        std::cerr << "Unable to open file" << std::endl;
    }

    // Return the computed values in a WOFISData struct
    WOFISData wofis_value;
    wofis_value.delta_v = delta_v;
    wofis_value.w_z = w_z;
    wofis_value.v_x = v_x;
    return wofis_value;
}

#endif // FIS_GETDATA