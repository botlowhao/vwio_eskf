#ifndef _FILTER_H_
#define _FILTER_H_

#include <vector>
#include <algorithm>
#include <Eigen/Dense>

class Filter
{
private:
    std::vector<std::vector<Eigen::Vector3d>> m_dataList; // List of sensor data
    const int MAX_SENSOR_NUM; // Maximum number of sensors
    const int MAX_DATA_NUM; // Maximum number of sample points
    const int WINDOW_DATA_NUM; // Length of the filter window

public:
    // Constructor, initializes the data list and window length
    Filter(int maxSensorNum, int maxDataNum, int windowSize) 
        : m_dataList(maxSensorNum, std::vector<Eigen::Vector3d>(maxDataNum, Eigen::Vector3d::Zero())), 
          MAX_SENSOR_NUM(maxSensorNum), MAX_DATA_NUM(maxDataNum), WINDOW_DATA_NUM(windowSize) {}

    /**
     * @brief Sliding window average filter function
     * 
     * @param sensorIndex source of data for Sliding window average filter
     * @param data data
     * @return Eigen::Vector3d 
     */
    Eigen::Vector3d slidingWindowAvgFilter(int sensorIndex, const Eigen::Vector3d &data)
    {
        static std::vector<int> dataNum(MAX_SENSOR_NUM, 0); // Records the number of sample points for each sensor
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        Eigen::Vector3d out = Eigen::Vector3d::Zero();

        // Moving the sample points within the window, FIFO operation
        for (int i = MAX_DATA_NUM - 2; i >= 0; --i)
        {
            m_dataList[sensorIndex][i + 1] = m_dataList[sensorIndex][i];
        }
        m_dataList[sensorIndex][0] = data;

        // If the number of sample points is less than the window length, calculate the average after accumulating the sample window data
        if (dataNum[sensorIndex] < MAX_DATA_NUM)
        {
            dataNum[sensorIndex]++;
            for (int i = 0; i < dataNum[sensorIndex]; ++i)
            {
                sum += m_dataList[sensorIndex][i];
            }
            out = sum / dataNum[sensorIndex];
        }
        else
        {
            std::vector<Eigen::Vector3d> array = m_dataList[sensorIndex];
            // Sorting using the sort function from the C++ standard library, using default ascending order
            std::sort(array.begin(), array.end(), [](const Eigen::Vector3d &a, const Eigen::Vector3d &b)
                      { return a.x() < b.x(); });

            int start = (MAX_DATA_NUM - WINDOW_DATA_NUM) / 2;
            for (int i = start; i < start + WINDOW_DATA_NUM; ++i)
            {
                sum += array[i];
            }
            out = sum / WINDOW_DATA_NUM;
        }
        return out;
    }
};

#endif // _FILTER_H_
