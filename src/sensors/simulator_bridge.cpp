#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"

ros::Publisher position_pub;
ros::Publisher orientation_pub;
ros::Publisher depth_pub;
//ros::Publisher lin_accel_pub;


// ModelStates msg consists of a name, a pose (position and orientation),
// and a twist (linear and angular velocity) for each object in
// the simulator
// Currently it publishes the position, orientation, and depth
// of the sub
// TODO: Eventually this should be generalized to publish a list
// (defined in params) of objects as well as calulate depth
void modelStatesCallback(const gazebo_msgs::ModelStates& msg)
{
    geometry_msgs::Vector3 position_msg;
    geometry_msgs::Quaternion orientation_msg;
    //geometry_msgs::Vector3 lin_accel_msg;
    std_msgs::Float32 depth_msg;

    // Find top of water and subs indices in modelstates lists
    int sub_index = -1;
    int ceiling_index = -1;
    for(int i=0; i<msg.name.size(); i++)
    {
        if(msg.name[i] == "robosub") sub_index = i;
        if(msg.name[i] == "ceiling_plane") ceiling_index = i;
    }

    if(sub_index >= 0)
    {
        position_msg.x = msg.pose[sub_index].position.x;
        position_msg.y = msg.pose[sub_index].position.y;
        position_msg.z = msg.pose[sub_index].position.z;

        orientation_msg.x = msg.pose[sub_index].orientation.x;
        orientation_msg.y = msg.pose[sub_index].orientation.y;
        orientation_msg.z = msg.pose[sub_index].orientation.z;
        orientation_msg.w = msg.pose[sub_index].orientation.w;

        position_pub.publish(position_msg);
        orientation_pub.publish(orientation_msg);

        if(ceiling_index >= 0)
        {
            depth_msg.data = -(msg.pose[ceiling_index].position.z -
                             msg.pose[sub_index].position.z);
            depth_pub.publish(depth_msg);
        }
    }

    //lin_accel_pub.publish(lin_accel_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simulator_bridge");

    ros::NodeHandle nh;

    position_pub = nh.advertise<geometry_msgs::Vector3>("position", 1);
    orientation_pub = nh.advertise<geometry_msgs::Quaternion>("orientation", 1);
    depth_pub = nh.advertise<std_msgs::Float32>("depth", 1);
    // Info not being published right now
    //lin_accel_pub = nh.advertise<geometry_msgs::Vector3>("lin_accel", 1);

	ros::Subscriber orient_sub = nh.subscribe("gazebo/model_states", 1, modelStatesCallback);

    int rate;
    if(!nh.getParam("control/rate", rate))
        rate = 30;
    ros::Rate r(rate);

    // I use spinOnce and sleeps here because the simulator publishes
    // at a very high rate and we don't need every message
    // You can throttle subscriber input in other ways but
    // doesn't require extra packages or anything
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
