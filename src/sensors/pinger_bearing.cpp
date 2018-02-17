#include "robosub_msgs/HydrophoneDeltas.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <vector>

//using std::vector;
using Eigen::Vector3d;
//using Eigen::Quaterniond;

ros::Publisher bearing_pub;
ros::Publisher marker_pub;
Vector3d hydrophone_positions;

/*
 * The speed of sound in water is 1484 m/s.
 */
static constexpr double speed_sound_in_water = 1484.0;


// callback for incoming ping time deltas from the hydrophones
// principle of operation: JFM
// (see http://robosub.eecs.wsu.edu/wiki/cs/hydrophones/pinger_bearing/start
// for details)
void deltaCallback(const robosub_msgs::HydrophoneDeltas::ConstPtr& msg)
{
    Vector3d bearing, d, time_deltas;

    time_deltas[0] = msg->xDelta.toSec();
    time_deltas[1] = msg->yDelta.toSec();
    time_deltas[2] = msg->zDelta.toSec();

    d = time_deltas * speed_sound_in_water;

    bearing = d.cwiseQuotient(hydrophone_positions);

    geometry_msgs::Vector3Stamped bearing_msg;

    bearing_msg.header.stamp = ros::Time::now();
    bearing_msg.vector.x = bearing[0];
    bearing_msg.vector.y = bearing[1];
    bearing_msg.vector.z = bearing[2];

    double mag = std::sqrt(bearing[0] * bearing[0] +
                           bearing[1] * bearing[1] +
                           bearing[2] * bearing[2]);

    ROS_DEBUG("vector magnitude: %lf", mag);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "cobalt";
    marker.header.stamp = ros::Time();
    marker.ns = "pinger_bearing";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point tail, tip;
    tail.x = tail.y = tail.z = 0;
    tip.x = bearing[0];
    tip.y = bearing[1];
    tip.z = bearing[2];
    marker.points.push_back(tail);
    marker.points.push_back(tip);
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;

    bearing_pub.publish(bearing_msg);
    marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pinger_bearing");

    ros::NodeHandle nh;

    if(ros::param::get("hydrophones/positions/x", hydrophone_positions[0]) ==
            false)
    {
        ROS_FATAL("Failed to load X hydrophone position.");
        return -1;
    }

    if (ros::param::get("hydrophones/positions/y", hydrophone_positions[1]) ==
            false)
    {
        ROS_FATAL("Failed to load Y hydrophone position.");
        return -1;
    }

    if (ros::param::get("hydrophones/positions/z", hydrophone_positions[2]) ==
            false)
    {
        ROS_FATAL("Failed to load Z hydrophone position.");
        return -1;
    }


    ros::Subscriber delta_sub = nh.subscribe("hydrophones/30khz/delta", 1,
            deltaCallback);
    bearing_pub = nh.advertise<geometry_msgs::Vector3Stamped>(
            "hydrophones/bearing", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>(
            "rviz/hydrophones/bearing", 1);

    ros::spin();

    return 0;
}
