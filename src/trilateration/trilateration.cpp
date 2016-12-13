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
#include "robosub/Position.h"
#include "ros/ros.h"

ros::Publisher position_pub;

/*
 * The speed of sound in water is 1484 m/s.
 */
static constexpr double speed_sound_in_water = 1484;

/*
 * The hydrophone position arguments. Delta is the position of the hx
 * hydrophone along the x axis, epsilon is hy along the y axis, and zeta is hz
 * along the z axis.
 */
double delta;
double epsilon;
double zeta;

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

    const double b_x = delta_x / pow(delta, 2) * (pow(delta, 2) -
            pow(delta_x, 2));
    const double b_y = delta_y / pow(epsilon, 2) * (pow(epsilon, 2) -
            pow(delta_y, 2));
    const double b_z = delta_z / pow(zeta, 2) * (pow(zeta, 2) -
            pow(delta_z, 2));

    const double c_x = pow((pow(delta_x, 2) - pow(delta, 2)) / (2 * delta), 2);
    const double c_y = pow((pow(delta_y, 2) - pow(epsilon, 2)) / (2 * epsilon),
            2);
    const double c_z = pow((pow(delta_z, 2) - pow(zeta, 2)) / (2 * zeta), 2);

    const double a = a_x + a_y + a_z - 1;
    const double b = b_x + b_y + b_z;
    const double c = c_x + c_y + c_z;

    /*
     * Find the determinant of the quadratic to ensure that it is not an
     * imaginary solution.
     */
    const double determinant = pow(b, 2) - 4 * a * c;
    if (determinant < 0)
    {
        ROS_INFO_STREAM("dX = " << delta_x);
        ROS_INFO_STREAM("dY = " << delta_y);
        ROS_INFO_STREAM("dZ = " << delta_z);
        ROS_INFO_STREAM("a: " << a);
        ROS_INFO_STREAM("b: " << b);
        ROS_INFO_STREAM("c: " << c);
        ROS_INFO_STREAM("Determinant: " << determinant);
        ROS_ERROR_STREAM("The trilateration encountered an imaginary "
                "solution.");
        return;
    }

    /*
     * Calculate the distance from the pinger for the two possible solutions of
     * the quadratic equation.
     */
    const double p_zero_solution_one = (-1*b + sqrt(determinant)) / (2 * a);
    const double p_zero_solution_two = (-1*b - sqrt(determinant)) / (2 * a);

    /*
     * Calculate the two potential position readings.
     */
    const double pos_x_one = (delta_x * (2 * p_zero_solution_one - delta_x) +
            delta * delta) / (2 * delta);
    const double pos_x_two = (delta_x * (2 * p_zero_solution_two - delta_x) +
            delta * delta) / (2 * delta);
    const double pos_y_one = (delta_y * (2 * p_zero_solution_one - delta_y) +
            epsilon * epsilon) / (2 * epsilon);
    const double pos_y_two = (delta_y * (2 * p_zero_solution_two - delta_y) +
            epsilon * epsilon) / (2 * epsilon);
    const double pos_z_one = (delta_z * (2 * p_zero_solution_one - delta_z) +
            zeta * zeta) / (2 * zeta);
    const double pos_z_two = (delta_z * (2 * p_zero_solution_two - delta_z) +
            zeta * zeta) / (2 * zeta);

    /*
     * Construct and publish the position message.
     */
    robosub::Position position_msg;
    position_msg.header.stamp = ros::Time::now();
    position_msg.distance_position_one = p_zero_solution_one;
    position_msg.position_one.x = pos_x_one;
    position_msg.position_one.y = pos_y_one;
    position_msg.position_one.z = pos_z_one;
    position_msg.distance_position_two = p_zero_solution_two;
    position_msg.position_two.x = pos_x_two;
    position_msg.position_two.y = pos_y_two;
    position_msg.position_two.z = pos_z_two;
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
     * Initialize the ROS node as the trilateration node and load the
     * hydrophone position parameters.
     */
    ros::init(argc, argv, "trilateration");

    ros::NodeHandle nh;

    if(ros::param::getCached("hydrophones/positions/x", delta) == false)
    {
        ROS_FATAL("Failed to load X hydrophone position.");
        return 0;
    }

    if (ros::param::getCached("hydrophones/positions/y", epsilon) == false)
    {
        ROS_FATAL("Failed to load Y hydrophone position.");
        return 0;
    }

    if (ros::param::getCached("hydrophones/positions/z", zeta) == false)
    {
        ROS_FATAL("Failed to load Y hydrophone position.");
        return 0;
    }

    /*
     * Subscribe to the time delta message and publish a derived position
     * topic.
     */
    ros::Subscriber delta_sub = nh.subscribe("hydrophone/30khz/delta", 1,
            deltaCallback);
    position_pub = nh.advertise<robosub::Position>("derived_position", 1);

    /*
     * Continually handle all ROS message callbacks.
     */
    ros::spin();

    return 0;
}
