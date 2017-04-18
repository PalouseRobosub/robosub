#include "sensors/Bno055.h"
#include "utility/serial.hpp"

#include "ros/ros.h"

#include <cstdint>
#include <iostream>
#include <string>

using namespace rs;
using namespace std;

/**
 * Handles errors returned by the Bno driver.
 *
 * @param x The expression to evaluate.
 * @param s The string representing the function the error occurred in.
 *
 * @return None.
 */
void HandleError(int x, const char * s, bool abort = false)
{
    if (x)
    {
        cout << s << " returned " << x << endl;
        if (abort)
        {
            exit(-1);
        }
    }
}

/**
 * Main entry point into the program.
 *
 * @return Zero upon completion.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_calibration");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    string port_name;

    int id = 0;
    HandleError(np.getParam("id", id) == false, "Failed to get sensor ID.",
            true);

    HandleError(n.getParam("ports/imu_" + std::to_string(id), port_name)
            == false, "ros::param::get(\"port\")", true);

    Bno055 bno;

    HandleError(bno.init(port_name), "Bno055::init()", true);

    HandleError(bno.setOperationMode(Bno055::OperationMode::Ndof),
                "Bno055::setOperationMode()", true);

    /*
     * Perform a calibration of all sensors.
     */
    uint8_t acc_calib = 0, gyr_calib = 0, mag_calib = 0, sys_calib = 0;
    while ((sys_calib < 3 || acc_calib < 3 || mag_calib < 3 || gyr_calib < 3)
            && ros::ok())
    {
        HandleError(bno.getSensorCalibration(Bno055::Sensor::Accelerometer,
                    acc_calib), "Bno055::getSensorCalibration()");
        HandleError(bno.getSensorCalibration(Bno055::Sensor::Gyroscope,
                    gyr_calib), "Bno055::getSensorCalibration()");
        HandleError(bno.getSensorCalibration(Bno055::Sensor::Magnetometer,
                    mag_calib), "Bno055::getSensorCalibration()");
        HandleError(bno.getSystemCalibration(sys_calib),
                "Bno055::getSystemCalibration()");
        cout << "--------" << endl;
        cout << "A: " << static_cast<int>(acc_calib) << "/3" << endl;
        cout << "G: " << static_cast<int>(gyr_calib) << "/3" << endl;
        cout << "M: " << static_cast<int>(mag_calib) << "/3" << endl;
        cout << "S: " << static_cast<int>(sys_calib) << "/3" << endl;
        usleep(500000);
    }

    if (!ros::ok())
    {
        ROS_INFO("Shutting down.");
        return -1;
    }

    HandleError(bno.setOperationMode(Bno055::OperationMode::Config),
                "Bno055::setOperationMode()", true);

    int16_t mag_radius = 0, acc_radius = 0, acc_offset[3] = {0},
             mag_offset[3] = {0}, gyr_offset[3] = {0};
    HandleError(bno.readRadii(acc_radius, mag_radius), "Bno055::readRadii()");
    HandleError(bno.readOffsets(Bno055::Sensor::Accelerometer, acc_offset[0],
                acc_offset[1], acc_offset[2]), "Bno055::readOffsets()");
    HandleError(bno.readOffsets(Bno055::Sensor::Gyroscope, gyr_offset[0],
                gyr_offset[1], gyr_offset[2]), "Bno055::readOffsets()");
    HandleError(bno.readOffsets(Bno055::Sensor::Magnetometer, mag_offset[0],
                mag_offset[1], mag_offset[2]), "Bno055::readOffsets()");

    cout << "sensor:" << endl;
    cout << "    accelerometer:" << endl;
    cout << "        offset:" << endl;
    cout << "            x: " << acc_offset[0] << endl;
    cout << "            y: " << acc_offset[1] << endl;
    cout << "            z: " << acc_offset[2] << endl;
    cout << "        radius:" << acc_radius << endl;
    cout << "    magnetometer:" << endl;
    cout << "        offset:" << endl;
    cout << "            x: " << mag_offset[0] << endl;
    cout << "            y: " << mag_offset[1] << endl;
    cout << "            z: " << mag_offset[2] << endl;
    cout << "        radius:" << mag_radius << endl;
    cout << "    gyroscope:" << endl;
    cout << "        offset:" << endl;
    cout << "            x: " << gyr_offset[0] << endl;
    cout << "            y: " << gyr_offset[1] << endl;
    cout << "            z: " << gyr_offset[2] << endl;

    cout << "=== Calibration Successful! ===" << endl;
}
