#include "robosub/Euler.h"
#include "robosub/QuaternionStampedAccuracy.h"
#include "ros/ros.h"
#include "sensors/Bno055.h"
#include <string>
#include "utility/serial.hpp"
#include <cstdint>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3Stamped.h"

static constexpr double _PI_OVER_180 = 3.14159 / 180.0;
static constexpr double _180_OVER_PI = 180.0 / 3.14159;

using namespace rs;

ros::Publisher euler_publisher;
ros::Publisher quaternion_publisher;
ros::Publisher linear_acceleration_publisher;

/**
 * Fatally exits if expression x evalutes true.
 *
 * @param x The expression to evaluate.
 * @param s The string to list when exitting.
 */
#define FatalAbortIf(x, s) { int ret = (x); if(ret) { ROS_FATAL(s); exit(-1); }}

/**
 * Encodes an axis integral value into the Bno055::Axis type.
 *
 * @param axis A value within [1,3] or [-3,-1] that corresponds to an axis,
 *        where 1 is X, 2 is Y, and 3 is Z. Negative values infer a remapping
 *        into the negative corresponding axis.
 *
 * @return The corresponding Bno055::Axis.
 */
Bno055::Axis encodeAxis(int axis)
{
    switch (axis)
    {
        case 1:
            return Bno055::Axis::X;
            break;
        case 2:
            return Bno055::Axis::Y;
            break;
        case 3:
            return Bno055::Axis::Z;
            break;
        case -1:
            return Bno055::Axis::NegativeX;
            break;
        case -2:
            return Bno055::Axis::NegativeY;
            break;
        case -3:
            return Bno055::Axis::NegativeZ;
            break;
    }

    ROS_FATAL("Sensor node failed to encode axis. Ensure axis mappings are "
              "1, 2, 3, -1, -2, or -3.");
    exit(-1);
}

int main(int argc, char **argv)
{
    /*
     * Initialize ROS for this node.
     */
    ros::init(argc, argv, "sensor");

    /*
     * Construct a node handle for communicating with ROS.
     */
    ros::NodeHandle nh;

    /*
     * Advertise the stamped quaternion and acceleration sensor
     * data to the software.
     */
    quaternion_publisher =
            nh.advertise<robosub::QuaternionStampedAccuracy>("orientation", 1);
    linear_acceleration_publisher =
        nh.advertise<geometry_msgs::Vector3Stamped>("acceleration/linear", 1);
    euler_publisher = nh.advertise<robosub::Euler>("pretty/orientation", 1);

    /*
     * Create the serial port, initialize it, and hand it to the Bno055 sensor
     * class.
     */
    Serial port;
    std::string port_name;
    FatalAbortIf(ros::param::get("ports/sensor", port_name) == false,
            "Failed to get port name parameter.");
    port.Open(port_name.c_str(), B115200);
    Bno055 sensor(port);
    FatalAbortIf(sensor.init() != 0, "Bno055 failed to initialize");
    ROS_INFO("Sensor successfully initialized.");

    /*
     * Load calibration parameters from the param server.
     */
    int accelerometer_radius = -1, magnetometer_radius = -1;
    int accelerometer_offset[3] = {-1, -1, -1},
            magnetometer_offset[3] = {-1, -1, -1},
            gyroscope_offset[3] = {-1, -1, -1};

    /*
     * Use explicit short-circuiting to guarentee that flags are only set when
     * the parameter load fails.
     */
    bool failed_radii_load = false, failed_offset_load = false,
            failed_axis_load = false;
    ROS_WARN_COND(nh.getParamCached("sensor/accelerometer/radius",
            accelerometer_radius) == false && (failed_radii_load = true),
            "Failed to load accelerometer calibration radius.");
    ROS_WARN_COND(nh.getParamCached("sensor/accelerometer/offset/x",
            accelerometer_offset[0]) == false && (failed_offset_load = true),
            "Failed to load accelerometer calibration offset.");
    ROS_WARN_COND(nh.getParamCached("sensor/accelerometer/offset/y",
            accelerometer_offset[1]) == false && (failed_offset_load = true),
            "Failed to load accelerometer calibration offset.");
    ROS_WARN_COND(nh.getParamCached("sensor/accelerometer/offset/z",
            accelerometer_offset[2]) == false && (failed_offset_load = true),
            "Failed to load accelerometer calibration offset.");

    ROS_WARN_COND(nh.getParamCached("sensor/magnetometer/radius",
            magnetometer_radius) == false && (failed_radii_load = true),
            "Failed to load magnetometer calibration radius.");
    ROS_WARN_COND(nh.getParamCached("sensor/magnetometer/offset/x",
            magnetometer_offset[0]) == false && (failed_offset_load = true),
            "Failed to load magnetometer calibration offset.");
    ROS_WARN_COND(nh.getParamCached("sensor/magnetometer/offset/y",
            magnetometer_offset[1]) == false && (failed_offset_load = true),
            "Failed to load magnetometer calibration offset.");
    ROS_WARN_COND(nh.getParamCached("sensor/magnetometer/offset/z",
            magnetometer_offset[2]) == false && (failed_offset_load = true),
            "Failed to load magnetometer calibration offset.");

    ROS_WARN_COND(nh.getParamCached("sensor/gyroscope/offset/x",
            gyroscope_offset[0]) == false && (failed_offset_load = true),
            "Failed to load gyroscope calibration offset.");
    ROS_WARN_COND(nh.getParamCached("sensor/gyroscope/offset/y",
            gyroscope_offset[1]) == false && (failed_offset_load = true),
            "Failed to load gyroscope calibration offset.");
    ROS_WARN_COND(nh.getParamCached("sensor/gyroscope/offset/z",
            gyroscope_offset[2]) == false && (failed_offset_load = true),
            "Failed to load gyroscope calibration offset.");

    /*
     * Remap the sensor orientation. A parameter value of 1 indicates remap the
     * axis to X, 2 indicates to remap to Y, and 3 remaps to Z. By multiplying
     * by negative one to any of the previously mentioned values will remap in
     * the negative direction.
     */
    int axis_x, axis_y, axis_z;
    ROS_WARN_COND(nh.getParamCached("sensor/axis/x", axis_x) == false &&
            (failed_axis_load = true),
            "Failed to load Bno055 remapped X axis.");
    ROS_WARN_COND(nh.getParamCached("sensor/axis/y", axis_y) == false &&
            (failed_axis_load = true),
            "Failed to load Bno055 remapped Y axis.");
    ROS_WARN_COND(nh.getParamCached("sensor/axis/z", axis_z) == false &&
            (failed_axis_load = true),
            "Failed to load Bno055 remapped Z axis.");

    Bno055::Axis xAxis, yAxis, zAxis;
    xAxis = encodeAxis(axis_x);
    yAxis = encodeAxis(axis_y);
    zAxis = encodeAxis(axis_z);
    if (failed_axis_load == false)
    {
        ROS_WARN_COND(sensor.remapAxes(xAxis, yAxis, zAxis) != 0,
                "Bno055 failed to remap axes.");
    }

    /*
     * Write sensor calibrations to the sensor.
     */
    if (failed_radii_load == false)
    {
        ROS_WARN_COND(sensor.writeRadii(static_cast<int16_t>(
                accelerometer_radius),
                static_cast<int16_t>(magnetometer_radius)) != 0,
                "Bno055 failed to write sensor radius calibrations.");
    }
    if (failed_offset_load == false)
    {
        ROS_WARN_COND(sensor.writeOffsets(Bno055::Sensor::Accelerometer,
                static_cast<int16_t>(accelerometer_offset[0]),
                static_cast<int16_t>(accelerometer_offset[1]),
                static_cast<int16_t>(accelerometer_offset[2])) != 0,
                "Bno055 failed to write accelerometer offset calibrations.");
        ROS_WARN_COND(sensor.writeOffsets(Bno055::Sensor::Magnetometer,
                static_cast<int16_t>(magnetometer_offset[0]),
                static_cast<int16_t>(magnetometer_offset[1]),
                static_cast<int16_t>(magnetometer_offset[2])) != 0,
                "Bno055 failed to write magnetometer offset calibrations.");
        ROS_WARN_COND(sensor.writeOffsets(Bno055::Sensor::Gyroscope,
                static_cast<int16_t>(gyroscope_offset[0]),
                static_cast<int16_t>(gyroscope_offset[1]),
                static_cast<int16_t>(gyroscope_offset[2])) != 0,
                "Bno055 failed to write gyroscope offset calibrations.");
    }

    /*
     * Configure the sensor to operate in nine degrees of freedom fusion mode.
     */
    FatalAbortIf(sensor.setOperationMode(Bno055::OperationMode::Ndof),
            "Bno055 failed to enter fusion mode.");

    /*
     * Enter the main ROS loop.
     */
    int rate;
    if (!nh.getParam("rate/sensor", rate))
    {
        ROS_WARN("Failed to load sensor node rate. Falling back to 20Hz.");
        rate = 20;
    }
    ros::Rate r(rate);
    ROS_INFO("BNO055 is now running.");

    while (ros::ok())
    {
        double x, y, z, w, roll, pitch, yaw;
        uint8_t confidence_level = 0;
        robosub::QuaternionStampedAccuracy quaternion_message;
        robosub::Euler euler_message;
        geometry_msgs::Vector3Stamped linear_acceleration_message;

        /*
         * Read a quaternion from the sensor, take the current time, and
         * publish the sensor data. Also read the sensor confidence level in
         * the measurement.
         */
        FatalAbortIf(sensor.readQuaternion(w, x, y, z) != 0,
                "Bno055 failed to read Quaternion");

        quaternion_message.header.stamp = ros::Time::now();

        FatalAbortIf(sensor.getSystemCalibration(confidence_level) != 0,
                "Bno055 failed to read system calibration status.");
        /*
         * Convert the quaternion to human-readable roll, pitch, and yaw.
         */
        tf::Matrix3x3 m(tf::Quaternion(x, y, z, w));
        m.getRPY(roll, pitch, yaw);

        /*
         * Note that we flip the roll and pitch, we do this so that the output
         * is in right-handed rotation notation
         */
        euler_message.roll = (-1) * roll * _180_OVER_PI;
        euler_message.pitch = (-1) * pitch * _180_OVER_PI;
        euler_message.yaw = yaw * _180_OVER_PI;
        euler_publisher.publish(euler_message);

         /*
         * Confidence ranges from [0,3], so normalize the value for
         * transmission.
         */
        quaternion_message.accuracy =
                static_cast<double>(confidence_level)/3.0;
        quaternion_message.quaternion =
                  tf::createQuaternionMsgFromRollPitchYaw(euler_message.roll,
                                                          euler_message.pitch,
                                                          euler_message.yaw);

        quaternion_publisher.publish(quaternion_message);

        /*
         * Read and publish the linear acceleration from the Bno055.
         */
        FatalAbortIf(sensor.readLinearAcceleration(x, y, z),
                "Failed to read linear acceleration.");
        linear_acceleration_message.header.stamp = ros::Time::now();
        linear_acceleration_message.vector.x = x;
        linear_acceleration_message.vector.y = y;
        linear_acceleration_message.vector.z = z;
        linear_acceleration_publisher.publish(linear_acceleration_message);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
