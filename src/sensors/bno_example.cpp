#include "sensors/Bno055.h"
#include "utility/serial.hpp"
#include <iostream>
#include <cstdint>
#include "ros/ros.h"

using namespace std;

using namespace rs;

/**
 * Handles errors returned by the Bno driver.
 *
 * @param x The expression to evaluate.
 * @param s The string representing the function the error occurred in.
 *
 * @return None.
 */
#define HandleError(x, s) \
{ \
    if ((x)) \
    { \
        cout << s << " returned " << (x) << endl; \
        int ret = bno.getSystemStatus(sys_stat, sys_err); \
        if (ret == 0) \
        { \
            cout << "System status: " << ios::hex << static_cast<int>(sys_stat) << endl; \
            if (sys_stat == 1) \
            { \
                cout << "Error Code: " << ios::hex << static_cast<int>(sys_err) << endl; \
            } \
        } \
        else \
        { \
            cout << "Failed to retrieve system status." << endl; \
        } \
    } \
} \

/**
 * Main entry point into the program.
 *
 * @return Zero upon completion.
 */
int main()
{
    ros::Time::init();
    uint8_t sys_stat, sys_err;

    Serial port;
    port.Open("/dev/ttyUSB0", 115200);

    Bno055 bno(port);

    HandleError(bno.init(), "Bno055::init()");

    HandleError(bno.setOperationMode(Bno055::OperationMode::Ndof), "Bno055::setOperationMode()");

    /*
     * Perform a calibration of all sensors.
     */
    uint8_t acc_calib = 0, gyr_calib = 0, mag_calib = 0, sys_calib = 0;
    do
    {
        HandleError(bno.getSensorCalibration(Bno055::Sensor::Accelerometer, acc_calib), "Bno055::getSensorCalibration()");
        cout << "Accelerometer Calibration: " << static_cast<int>(acc_calib) << endl;
        HandleError(bno.getSensorCalibration(Bno055::Sensor::Gyroscope, gyr_calib), "Bno055::getSensorCalibration()");
        cout << "Gyroscope Calibration: " << static_cast<int>(gyr_calib) << endl;
        HandleError(bno.getSensorCalibration(Bno055::Sensor::Magnometer, mag_calib), "Bno055::getSensorCalibration()");
        cout << "Magnometer Calibration: " << static_cast<int>(mag_calib) << endl;
        HandleError(bno.getSystemCalibration(sys_calib), "Bno055::getSystemCalibration()");
        cout << "System Calibration: " << static_cast<int>(sys_calib) << endl;
        usleep(500000);
    }
    while (sys_calib < 3 || acc_calib < 3 || mag_calib < 3 || gyr_calib < 3);

    double w = 0, x = 0, y = 0, z = 0;
    double roll, pitch, yaw;
    while (1)
    {
        sleep(1);
        HandleError(bno.readQuaternion(w, x, y, z), "Bno055::readQuaternion()");
        cout << "w: " << w << endl;
        cout << "x: " << x << endl;
        cout << "y: " << y << endl;
        cout << "z: " << z << endl;
        HandleError(bno.readEuler(roll, pitch, yaw), "Bno::readEuler()");
        cout << "roll: " << roll << endl;
        cout << "pitch: " << pitch << endl;
        cout << "yaw: " << yaw << endl;
    }

    return 0;
}

