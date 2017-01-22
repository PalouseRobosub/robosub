/**
 * Hydrophone Trilateration (Position Derivation) Node
 *
 * @author Ryan Summers
 * @date 12-12-2016
 *
 * @brief This file contains the calculations of the submarine's position
 *        relative to the pinger from a number of hydrophone time stamps.
 *
 * @note The mathematic equations presented in this file are derived at the
 *       following two links. Please refer to them for all questions relating
 *       to mathematic operations. All variables have been named after the
 *       variables defined in the derivation for clarity.
 *          http://robosub.eecs.wsu.edu/wiki/ee/hydrophones/trilateration/start
 *          http://robosub.eecs.wsu.edu/wiki/cs/hydrophones/start
 */

#include "robosub/HydrophoneDeltas.h"
#include "robosub/PositionArrayStamped.h"
#include "robosub/QuaternionStampedAccuracy.h"
#include "robosub/Float32Stamped.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <vector>

using std::vector;
using Eigen::Vector3d;
using Eigen::Quaterniond;

ros::Publisher position_pub;
ros::Publisher position_all_pub;

/*
 * The speed of sound in water is 1484 m/s.
 */
static constexpr double speed_sound_in_water = 1484;

/*
 * The last read submarine orientation and depth information.
 */
Quaterniond current_orientation;
float current_depth;

/*
 * The depth of the pinger in the pool.
 */
float pinger_depth;

/*
 * The reference hydrophone position with respect to the center of the
 * submarine.
 */
Vector3d reference_hydrophone_position;

/*
 * The hydrophone position arguments. Delta is the position of the hx
 * hydrophone along the x axis, epsilon is hy along the y axis, and zeta is hz
 * along the z axis.
 */
double delta;
double epsilon;
double zeta;

/**
 * Callback function to receive the submarine's current orientation.
 *
 * @param msg A pointer to the received message.
 *
 * @return None.
 */
void orientationCallback(const
        robosub::QuaternionStampedAccuracy::ConstPtr& msg)
{
    current_orientation = Quaterniond(msg->quaternion.w, msg->quaternion.x,
            msg->quaternion.y, msg->quaternion.z);
}

/**
 * Callback function to receive the submarine's current depth.
 *
 * @param msg A pointer to the received message.\
 *
 * @return None.
 */
void depthCallback(const robosub::Float32Stamped::ConstPtr& msg)
{
    current_depth = msg->data;
}

/**
 * Callback funtion for hydrophone time delta message.
 *
 * @param msg A pointer to the received message.
 *
 * @return None.
 */
void deltaCallback(const robosub::HydrophoneDeltas::ConstPtr& msg)
{
    /*
     * The inverting multiplication (by -1) is used to invert the time stamp
     * delta calculation. This was a solution by Brian Moore and Ryan Summers
     * after debug testing.
     */
    const double delta_x = msg->xDelta.toSec() * speed_sound_in_water * -1;
    const double delta_y = msg->yDelta.toSec() * speed_sound_in_water * -1;
    const double delta_z = msg->zDelta.toSec() * speed_sound_in_water * -1;

    /*
     * Calculate the distinct x, y, and z portions of the quadratic
     * coefficients and combine them into a complete quadratic equation.
     */
    const double a_x = pow((delta_x / delta), 2);
    const double a_y = pow((delta_y / epsilon), 2);
    const double a_z = pow((delta_z / zeta), 2);

    const double b_x = delta_x / pow(delta, 2) *
            (pow(delta, 2) - pow(delta_x, 2));
    const double b_y = delta_y / pow(epsilon, 2) *
            (pow(epsilon, 2) - pow(delta_y, 2));
    const double b_z = delta_z / pow(zeta, 2) *
            (pow(zeta, 2) - pow(delta_z, 2));

    const double c_x = pow((pow(delta_x, 2) - pow(delta, 2)) / (2 * delta), 2);
    const double c_y =
            pow((pow(delta_y, 2) - pow(epsilon, 2)) / (2 * epsilon), 2);
    const double c_z = pow((pow(delta_z, 2) - pow(zeta, 2)) / (2 * zeta), 2);

    const double a = a_x + a_y + a_z - 1;
    const double b = b_x + b_y + b_z;
    const double c = c_x + c_y + c_z;

    /*
     * Calculate the distance from the pinger for the two possible solutions of
     * the quadratic equation. The correct result has a valid depth reading and
     * a positive distance. The variable 'distances' represents the p_zero for
     * solutions one and two. If the solution is complex, take the magnitude of
     * the complex number as the distance.
     */
    const double determinant = pow(b, 2) - 4 * a * c;
    double distances[2];
    if (determinant < 0)
    {
        distances[0] = sqrt(pow(-1 * b / 2, 2) + pow(determinant/2, 2));
        distances[1] = distances[0];
    }
    else
    {
        distances[0] = (-1 * b + sqrt(determinant)) / (2 * a);
        distances[1] = (-1 * b - sqrt(determinant)) / (2 * a);
    }

    /*
     * Calculate the two potential position readings.
     */
    const double pos_x_one = (delta_x * (2 * distances[0] - delta_x) +
            pow(delta, 2)) / (2 * delta);
    const double pos_x_two = (delta_x * (2 * distances[1] - delta_x) +
            pow(delta, 2)) / (2 * delta);
    const double pos_y_one = (delta_y * (2 * distances[0]- delta_y) +
            pow(epsilon, 2)) / (2 * epsilon);
    const double pos_y_two = (delta_y * (2 * distances[1]- delta_y) +
            pow(epsilon, 2)) / (2 * epsilon);
    const double pos_z_one = (delta_z * (2 * distances[0]- delta_z) +
            pow(zeta, 2)) / (2 * zeta);
    const double pos_z_two = (delta_z * (2 * distances[1]- delta_z) +
            pow(zeta, 2)) / (2 * zeta);
    const Vector3d position_one(pos_x_one, pos_y_one, pos_z_one),
            position_two(pos_x_two, pos_y_two, pos_z_two);

    /*
     * Note that at the current time, position_one and position_two describe
     * the coordinates of the pinger with respect to the reference hydrophones
     * position and the submarines rotation. This position needs to be
     * rectified to define the pinger at the origin and have no yaw, pitch, or
     * roll. To accomplish this, the coordinates first need to be standardized
     * into the robosub reference frame. This can be thought of like rotating
     * the entire world to be aligned with the robosub's current orientation.
     * Then, the position of the reference hydrophone needs to be rectified so
     * the coordinates instead refer to the submarine's center of mass.
     * Finally, all readings need to be inverted to define the coordinates to
     * the submarine from the pinger.
     */
    Vector3d position_one_rectified = (current_orientation.matrix() *
            position_one - reference_hydrophone_position) * -1;
    Vector3d position_two_rectified = (current_orientation.matrix() *
            position_two - reference_hydrophone_position) * -1;

    /*
     * Filter out invalid solutions. Solutions are considered invalid if their
     * distance is negative or if the z position places the submarine out of
     * the water.
     */
    vector<Vector3d> position_readings;
    vector<double> distance_readings;
    const double real_z_relative_to_pinger = current_depth - pinger_depth;
    const double pos_one_depth_difference = fabs(position_one_rectified[2] -
            real_z_relative_to_pinger);
    const double pos_two_depth_difference = fabs(position_two_rectified[2] -
            real_z_relative_to_pinger);
    if (distances[0] > 0 && position_one_rectified[2] <
            -1 * pinger_depth && pos_one_depth_difference < 2)
    {
        position_readings.push_back(position_one_rectified);
        distance_readings.push_back(distances[0]);
    }
    else
    {
        ROS_INFO_STREAM("Position one invalid:\n" << position_one_rectified <<
                "\nDistance: " << distances[0]);
    }

    if (distances[1] > 0 && position_two_rectified[2] <
            -1 * pinger_depth && pos_two_depth_difference < 2)
    {
        position_readings.push_back(position_two_rectified);
        distance_readings.push_back(distances[1]);
    }
    else
    {
        ROS_DEBUG_STREAM("Position two invalid:\n" << position_two_rectified <<
                "\nDistance: " << distances[1]);
    }

    /*
     * Order them based on how close the depth reading is if there are two
     * valid readings still. If the order swaps, swap the distance measurements
     * as well so that they are properly displayed in the message.
     */
    if (position_readings.size() > 1 && pos_two_depth_difference <
            pos_one_depth_difference)
    {
        position_readings.push_back(position_readings[0]);
        position_readings.erase(position_readings.begin());

        distance_readings.push_back(distance_readings[0]);
        distance_readings.erase(distance_readings.begin());
    }

    /*
     * Publish all of the rectified measurements regardless of validity.
     */
    robosub::PositionArrayStamped all_positions_msg;
    robosub::Position position_msg;
    geometry_msgs::Point position;

    position.x = position_one[0];
    position.y = position_one[1];
    position.z = position_one[2];
    position_msg.position = position;
    position_msg.distance = distances[0];
    all_positions_msg.positions.push_back(position_msg);

    position.x = position_two[0];
    position.y = position_two[1];
    position.z = position_two[2];
    position_msg.position = position;
    position_msg.distance = distances[1];
    all_positions_msg.positions.push_back(position_msg);

    all_positions_msg.header.stamp = ros::Time::now();
    position_all_pub.publish(all_positions_msg);

    /*
     * Publish all validated position measurements ordered by confidence.
     */
    robosub::PositionArrayStamped valid_positions_msg;
    valid_positions_msg.header.stamp = ros::Time::now();
    for (unsigned int i = 0; i < position_readings.size(); ++i)
    {
        position.x = position_readings[i][0];
        position.y = position_readings[i][1];
        position.z = position_readings[i][2];
        position_msg.distance = distance_readings[i];
        position_msg.position = position;

        valid_positions_msg.positions.push_back(position_msg);
    }

    position_pub.publish(valid_positions_msg);
}

/**
 * Main entry point into the program.
 *
 * @param argc The number of arguments passed to ROS.
 * @param argv A pointer to a list of arguments passed to ROS.
 *
 */
int main(int argc, char **argv)
{
    /*
     * Initialize the ROS node as the hydrophones node and load the
     * hydrophone position parameters.
     */
    ros::init(argc, argv, "hydrophones");

    ros::NodeHandle nh;

    if(ros::param::getCached("hydrophones/positions/delta", delta) == false)
    {
        ROS_FATAL("Failed to load X hydrophone position.");
        return -1;
    }

    if (ros::param::getCached("hydrophones/positions/epsilon", epsilon) ==
            false)
    {
        ROS_FATAL("Failed to load Y hydrophone position.");
        return -1;
    }

    if (ros::param::getCached("hydrophones/positions/zeta", zeta) == false)
    {
        ROS_FATAL("Failed to load Y hydrophone position.");
        return -1;
    }

    if (ros::param::getCached("hydrophones/reference/x",
                reference_hydrophone_position[0]) == false)
    {
        ROS_FATAL("Failed to load X reference hydrophone position.");
        return -1;
    }

    if (ros::param::getCached("hydrophones/reference/y",
                reference_hydrophone_position[1]) == false)
    {
        ROS_FATAL("Failed to load Y reference hydrophone position.");
        return -1;
    }

    if (ros::param::getCached("hydrophones/reference/z",
                reference_hydrophone_position[2]) == false)
    {
        ROS_FATAL("Failed to load Z reference hydrophone position.");
        return -1;
    }

    if (ros::param::getCached("hydrophones/pinger/depth", pinger_depth) ==
            false)
    {
        ROS_FATAL("Failed to load the pinger depth.");
        return -1;
    }

    /*
     * Subscribe to the time delta message and publish a derived position
     * topic.
     */
    ros::Subscriber delta_sub = nh.subscribe("hydrophones/30khz/delta", 1,
            deltaCallback);
    ros::Subscriber orientation_sub = nh.subscribe("orientation", 1,
            orientationCallback);
    ros::Subscriber depth_sub = nh.subscribe("depth", 1,
            depthCallback);
    position_pub = nh.advertise<robosub::PositionArrayStamped>(
            "hydrophones/position", 1);
    position_all_pub = nh.advertise<robosub::PositionArrayStamped>(
            "hydrophones/position/all", 1);

    /*
     * Continually handle all ROS message callbacks.
     */
    ros::spin();

    return 0;
}
