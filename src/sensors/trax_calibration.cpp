#include "sensors/PniTrax.h"
#include "std_srvs/Empty.h"
#include <ros/ros.h>

static volatile int points_taken = 0;

PniTrax trax;

/**
 * Service call to save trax configuration to EEPROM.
 *
 * @param req The service request.
 * @param res The service response.
 *
 * @return True upon success and false upon error.
 */
bool save(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    const int ret = trax.save_configuration();
    if (ret == 0)
    {
        ROS_INFO("Configuration saved to TRAX EEPROM.");
    }
    else
    {
        ROS_INFO("Failed to save configuration.");
    }

    return (ret == 0);
}

/**
 * Service to inform the node to take a TRAX calibration point.
 *
 * @param req The service request.
 * @param res The service response.
 *
 * @return True upon success and false upon error.
 */
bool take_calibration_point(std_srvs::Empty::Request &req,
                            std_srvs::Empty::Response &res)
{
    if (points_taken >= 12)
    {
        ROS_ERROR_STREAM("Calibration is already completed.");
        return false;
    }

    const int ret = trax.takeCalibrationPoint();
    if (ret)
    {
        ROS_INFO_STREAM("Failed to take calibration point " << points_taken + 1);
    }
    else
    {
        ROS_INFO_STREAM("Taking TRAX calibration point " << points_taken++);
    }

    return (ret == 0);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trax_calibration");

    ros::NodeHandle nh;

    /*
     * Load the serial port name from the parameter server.
     */
    string port_name;
    ROS_FATAL_COND(nh.getParam("ports/trax", port_name) == false,
            "Failed to load TRAX serial port.");

    ROS_FATAL_COND(trax.init(port_name), "Failed to initialize TRAX sensor.");

    /*
     * Set up the server for the calibration points.
     */
    ros::ServiceServer point_service = nh.advertiseService("calibrate",
            take_calibration_point);
    ros::ServiceServer save_service = nh.advertiseService("save", save);

    /*
     * Detect what type of calibration should be completed.
     */
    ros::NodeHandle np("~");
    int type;
    ROS_FATAL_COND(np.getParam("type", type) == false,
            "Failed to load ~type for calibration type."
            "\n1 := FullRange,"
            "\n2 := 2-Dimensional"
            "\n3 := HardIron"
            "\n4 := LimitedTilt"
            "\n5 := AccelOnly"
            "\n6 := MagAccel");

    PniTrax::Calibration calib;
    switch (type)
    {
    case 1:
        calib = PniTrax::Calibration::FullRange;
        ROS_INFO("Calibration will be a full range calibration.");
        break;
    case 2:
        calib = PniTrax::Calibration::TwoD;
        ROS_INFO("Calibration will be a two-dimensional calibration.");
        break;
    case 3:
        calib = PniTrax::Calibration::HardIron;
        ROS_INFO("Calibration will be a hard-iron only calibration.");
        break;
    case 4:
        calib = PniTrax::Calibration::LimitedTilt;
        ROS_INFO("Calibration will be limited-tilt calibration.");
        break;
    case 5:
        calib = PniTrax::Calibration::AccelOnly;
        ROS_INFO("Calibration will be for accelerometer only.");
        break;
    case 6:
        calib = PniTrax::Calibration::MagAccel;
        ROS_INFO("Calibration will be magnetometer and accelerometer.");
        break;
    default:
        ROS_FATAL("Invalid calibration type.");
        return -1;
    }

    ROS_FATAL_COND(trax.startCalibration(calib, false),
            "Failed to begin TRAX calibration.");

    /*
     * Loop while the calibration is incomplete and ROS is okay. A calibration
     * defaults to 12 data points. The TRAX driver class does not change this
     * value. Please refer to the TRAX datasheet for specific information about
     * what each calibration point should be.
     */
    ros::Rate r(20);
    while (ros::ok() && points_taken < 12)
    {
        ROS_INFO_THROTTLE(10, "Please use the `calibrate` service call for this "
                "node to take calibration points.");
        ros::spinOnce();
        r.sleep();
    }

    /*
     * If a calibration is completed, read the results and print them to the
     * user.
     */
    if (points_taken >= 12)
    {
        float mag_score, accel_score, distribution_error, tilt_error, tilt_range;
        ROS_ERROR_COND(trax.finishCalibration(mag_score, accel_score, distribution_error, tilt_error, tilt_range),
                "Failed to read the TRAX calibration results.");

        ROS_INFO_STREAM("Calibration score:\n"
                << "Mag: " << mag_score << " [Should be < 1 for full-range and "
                        "< 2 for others]\n"
                << "Acl: " << accel_score << " [Should be < 1]\n"
                << "Dst: " << distribution_error << " [Should be zero. Non-zero "
                        "indicates improper point distribution]\n"
                << "Tilt_error: " << tilt_error << " [Should be zero. Non-zero "
                        "indicates invalid tilt range encountered.]\n"
                << "Tilt_range: " << tilt_range << "[Total dynamic tilt during "
                        "test. For full-range and hard-iron, should be >= 45, "
                        "but for all others, should be <= 2]");

        /*
         * Inform the user to use the service call to save to EEPROM.
         */
        while (ros::ok())
        {
            ROS_INFO_THROTTLE(10, "If you would like to save the calibration to "
                    "EEPROM of device, please use the `save` service call.");
            ROS_INFO_THROTTLE(10, "If you would not like to save results, please"
                    " kill this node now.");
        }
    }

    return 0;
}
