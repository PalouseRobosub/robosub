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
    ros::Time::init();
    string port_name;

    HandleError(ros::param::getCached("/ports/sensor", port_name) == false,
            "ros::param::getCached(\"/ports/sensor\")", true);

    Serial port;
    port.Open(port_name.c_str(), 115200);

    Bno055 bno(port);

    HandleError(bno.init(), "Bno055::init()", true);

    HandleError(bno.setOperationMode(Bno055::OperationMode::Ndof),
                "Bno055::setOperationMode()", true);

    /*
     * Perform a calibration of all sensors.
     */
    uint8_t acc_calib = 0, gyr_calib = 0, mag_calib = 0, sys_calib = 0;
    while (sys_calib < 3 || acc_calib < 3 || mag_calib < 3 || gyr_calib < 3)
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

    cout << "A Offsets: X: " << acc_offset[0] << " Y: " << acc_offset[1]
            << " Z: " << acc_offset[2] << endl;
    cout << "G Offsets: X: " << gyr_offset[0] << " Y: " << gyr_offset[1]
            << " Z: " << gyr_offset[2] << endl;
    cout << "M Offsets: X: " << mag_offset[0] << " Y: " << mag_offset[1]
            << " Z: " << mag_offset[2] << endl;
    cout << "A Radius: " << acc_radius << endl;
    cout << "M Radius: " << mag_radius << endl;

    cout << "=== Calibration Successful! ===" << endl;
}
