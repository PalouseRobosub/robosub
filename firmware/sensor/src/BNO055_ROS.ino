/*
 * Author: Zachary Pratt
 *
 * Arduino using Adafruit unified sensor library to get data from the
 * BNO055 9-axis imu sensor and publish the data through ros as a quaternion message.
 *
 * Originally created to test effects of outside magnetic fields on
 * the orientation data out BNO055.
 */

#include <Wire.h>
#include <stdint.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <MS5837.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

//#define MUX_2_ADDR_W 0b11100000
//#define MUX_2_ADDR_R 0b11100001
//#define MUX_1_ADDR_W 0b11100010
//#define MUX_1_ADDR_R 0b11100011

Adafruit_BNO055 bno = Adafruit_BNO055(55);
MS5837 ds;

ros::NodeHandle n;

geometry_msgs::Quaternion msg;
geometry_msgs::Vector3 lin_acc;
//geometry_msgs::Vector3 mag;
//geometry_msgs::Vector3 gyro;
//std_msgs::Float32 depth_msg;
//std_msgs::Float32 temp_msg;
std_msgs::Float32 pressure_msg;

ros::Publisher rs_bno_data_pub("rs_bno_data", &msg);
ros::Publisher rs_lin_accel_data_pub("rs_lin_accel_data", &lin_acc);
//ros::Publisher rs_mag_data_pub("rs_mag_data", &mag);
//ros::Publisher rs_gyro_data_pub("rs_gyro_data", &gyro);
//ros::Publisher rs_depth_data_pub("rs_depth_data", &depth_msg);
//ros::Publisher rs_temp_data_pub("rs_temp_data", &temp_msg);
ros::Publisher rs_pressure_data_pub("rs_pressure_data", &pressure_msg);

void setup() {
    while(!bno.begin());

//    Wire.beginTransmission(MUX_1_ADDR_W);
//    Wire.write((uint8_t)0b00000001); // disable channel 1 enable channel 0 (bits 0,1 for control)
//    Wire.endTransmission();

    ds.init();
    ds.setFluidDensity(997.0f); // fluid density of freshwater

    delay(1000);

    /*
     * bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
     * turn off magnetometer to use only the gyroscope and accelerometer
     */

    bno.setExtCrystalUse(true);

    n.initNode();
    n.advertise(rs_bno_data_pub);
    n.advertise(rs_lin_accel_data_pub);
//    n.advertise(rs_depth_data_pub);
//    n.advertise(rs_temp_data_pub);
  n.advertise(rs_pressure_data_pub);
//  n.advertise(rs_gyro_data_pub);
//    n.advertise(rs_mag_data_pub);
}

void loop() {

    /*
     * imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
     * get magnetometer data into a vector
     *
     * imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
     * get orientation data as euler angles into a vector
     *
     * uint8_t sys, gyro, accel, mag;
     * bno.getCalibration(&sys, &gyro, &accel, &mag);
     * get calibration data for sensors. values are either 0, 1, 2, or 3
     * 0 is fully calibrated. 3 is not calibrated.
     */

    imu::Quaternion q = bno.getQuat();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    rs_bno_data_pub.publish(&msg);

    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    lin_acc.x = accel.x();
    lin_acc.y = accel.y();
    lin_acc.z = accel.z();
    rs_lin_accel_data_pub.publish(&lin_acc);

    /*
    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    gyro.x = gyroscope.x();
    gyro.y = gyroscope.y();
    gyro.z = gyroscope.z();
    rs_gyro_data_pub.publish(&gyro);
*/

//    imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
//    mag.x = magnetometer.x();
//    mag.y = magnetometer.y();
//    mag.z = magnetometer.z();
//    rs_mag_data_pub.publish(&mag);

    /*
     * Read depth sensor. The analog depth sensor is an input on pin A1.
     * const float depth_reading = -.025 * (analogRead(2) - 335);

     * depth_msg.data = depth_reading;
     * rs_depth_data_pub.publish(&depth_msg);
     */

//     Wire.beginTransmission(MUX_1_ADDR_W);
//     Wire.write((uint8_t)0b00000001);
//     Wire.endTransmission();

     ds.read();

//     depth_msg.data = ds.depth();
//     rs_depth_data_pub.publish(&depth_msg);
//
//     temp_msg.data = ds.temperature();
//     rs_temp_data_pub.publish(&temp_msg);

     pressure_msg.data = ds.pressure();
     rs_pressure_data_pub.publish(&pressure_msg);

    n.spinOnce();

    delay(25);
}
