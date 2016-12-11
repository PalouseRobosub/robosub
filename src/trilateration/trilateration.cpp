#include "ros/ros.h"
#include "robosub/HydrophoneDeltas.h"
#include "robosub/Position.h"

ros::Publisher position_pub;

static constexpr double speed_sound_in_water = 1484;

void deltaCallback(const robosub::HydrophoneDeltas::ConstPtr& msg)
{
    const double delta_x = msg->xDelta.toSec() * speed_sound_in_water;
    const double delta_y = msg->yDelta.toSec() * speed_sound_in_water;
    const double delta_z = msg->zDelta.toSec() * speed_sound_in_water;
    ROS_INFO_STREAM("dX = " << delta_x);
    ROS_INFO_STREAM("dY = " << delta_y);
    ROS_INFO_STREAM("dZ = " << delta_z);

    /*
     * For now, hard-code the hydrophone positions.
     * TODO: Load precise thruster positions from the settings file.
     */
    const double delta = 3;
    const double epsilon = 3;
    const double zeta = 3;

    const double a_x = pow((delta_x / delta), 2);
    const double a_y = pow((delta_y / epsilon), 2);
    const double a_z = pow((delta_z / zeta), 2);

    const double b_x = delta_x / pow(delta, 2) * (pow(delta, 2) + pow(delta_x, 2));
    const double b_y = delta_y / pow(epsilon, 2) * (pow(epsilon, 2) + pow(delta_y, 2));
    const double b_z = delta_z / pow(zeta, 2) * (pow(zeta, 2) + pow(delta_z, 2));

    const double c_x = pow((pow(delta_x, 2) - pow(delta, 2)) / (2 * delta), 2);
    const double c_y = pow((pow(delta_y, 2) - pow(epsilon, 2)) / (2 * epsilon), 2);
    const double c_z = pow((pow(delta_z, 2) - pow(zeta, 2)) / (2 * zeta), 2);

    /*
     * Solve for the quadratic coefficients.
     */
    const double a = a_x + a_y + a_z - 1;
    const double b = b_x + b_y + b_z;
    const double c = c_x + c_y + pow(z, 2);

    /*
     * Find the determinant of the quadratic to ensure that it is not an
     * imaginary solution.
     */
    const double determinant = pow(b, 2) - 4 * a * c;
    ROS_INFO_STREAM("a: " << a);
    ROS_INFO_STREAM("b: " << b);
    ROS_INFO_STREAM("c: " << c);
    ROS_INFO_STREAM("Determinant: " << determinant);
    if (determinant < 0)
    {
        ROS_ERROR_STREAM("The trilateration encountered an imaginary "
                "solution.");
        return;
    }

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
     * Construct the position message.
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

    /*
     * Publish the position message.
     */
    position_pub.publish(position_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trilateration");

    ros::NodeHandle nh;

    ros::Subscriber delta_sub = nh.subscribe("hydrophone/30khz/delta", 1, deltaCallback);
    position_pub = nh.advertise<robosub::Position>("derived_position", 1);

    ros::spin();

    return 0;
}
