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

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ros.h>
#include <geometry_msgs/Quaternion.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

ros::NodeHandle n;

geometry_msgs::Quaternion msg;

ros::Publisher rs_bno_data_pub("rs_bno_data", &msg);

void setup() {
  while(!bno.begin());

  delay(1000);
  
  /*
   * bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
   * turn off magnetometer to use only the gyroscope and accelerometer
   */

  bno.setExtCrystalUse(true);
  
  n.initNode();
  n.advertise(rs_bno_data_pub);
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
  imu::Vector<3> euler = q.toEuler();

  /*
   * x = yaw, y = pitch, z = roll 
   */
  
  msg.x = euler.x();
  msg.y = euler.y();
  msg.z = euler.z();
  rs_bno_data_pub.publish(&msg);
  n.spinOnce();
  delay(200);
}
