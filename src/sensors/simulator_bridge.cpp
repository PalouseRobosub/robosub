#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "robosub/ObstaclePosArray.h"

ros::Publisher position_pub;
ros::Publisher orientation_pub;
ros::Publisher depth_pub;
ros::Publisher obstacle_pos_pub;

// List of names of objects to publish the position and name of. This will be
// loaded from parameters.
std::vector<std::string> object_names;

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
        // Copy sub pos to position msg
        position_msg.x = msg.pose[sub_index].position.x;
        position_msg.y = msg.pose[sub_index].position.y;
        position_msg.z = msg.pose[sub_index].position.z;

        // Copy sub orientation to orientation msg
        orientation_msg.x = msg.pose[sub_index].orientation.x;
        orientation_msg.y = msg.pose[sub_index].orientation.y;
        orientation_msg.z = msg.pose[sub_index].orientation.z;
        orientation_msg.w = msg.pose[sub_index].orientation.w;

        // Publish sub position and orientation
        position_pub.publish(position_msg);
        orientation_pub.publish(orientation_msg);

        // If the model for the top of the water is found calculate depth from
        // the z positions of the water top and the sub
        if(ceiling_index >= 0)
        {
            depth_msg.data = -(msg.pose[ceiling_index].position.z -
                             msg.pose[sub_index].position.z);
            depth_pub.publish(depth_msg);
        }
    }

    // Iterate through object_names and for each iteration search through the
    // msg.name array and attempt to find object_names[i]. If it is not found
    // std::find returns an iterator equal to msg.name.end(). If it is found
    // std::find returns an iterator pointing to the object with
    // object_name[i]. std::distance is used to find the index of that object
    // wthin the msg.name (and therefore msg.pose array since they contain the
    // same objects in the same order).
    robosub::ObstaclePosArray object_array;
    for(int i=0; i<object_names.size(); i++)
    {
        auto it = std::find(msg.name.begin(), msg.name.end(), object_names[i]);

        // object_name[i] found in msg.name.
        if(it != msg.name.end())
        {
            // Calculate index of object in msg.name from the iterator element.
            int idx = std::distance(msg.name.begin(), it);

            // Extract necessary data from modelstates.
            robosub::ObstaclePos pos;
            pos.x = msg.pose[idx].position.x;
            pos.y = msg.pose[idx].position.y;
            pos.z = msg.pose[idx].position.z;
            pos.name = msg.name[idx];

            object_array.data.push_back(pos);
        }
    }

    obstacle_pos_pub.publish(object_array);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulator_bridge");

    ros::NodeHandle nh;

    position_pub = nh.advertise<geometry_msgs::Vector3>("position", 1);
    orientation_pub = nh.advertise<geometry_msgs::Quaternion>("orientation", 1);
    depth_pub = nh.advertise<std_msgs::Float32>("depth", 1);
    obstacle_pos_pub = nh.advertise<robosub::ObstaclePosArray>("obstacles/positions", 1);

    ros::Subscriber orient_sub = nh.subscribe("gazebo/model_states", 1, modelStatesCallback);

    int rate;
    if(!nh.getParam("control/rate", rate))
    {
        rate = 30;
    }
    ros::Rate r(rate);

    // Put object names in vector.
    if(!nh.getParam("/obstacles", object_names))
    {
        ROS_WARN_STREAM("failed to load obstacle names");
    }

    // I use spinOnce and sleeps here because the simulator publishes
    // at a very high rate and we don't need every message.
    // You can throttle subscriber input in other ways but this method doesn't
    // require extra packages or anything like that.
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
