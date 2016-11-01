#include "localization_system.hpp"

#include "std_srvs/Empty.h"

ros::Publisher loc_pub;
LocalizationSystem *loc_system;

void orientationCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    geometry_msgs::Quaternion q;
    q.w = msg->w;
    q.x = msg->x;
    q.y = msg->y;
    q.z = msg->z;
    loc_system->InputOrientation(q);
}

void accelCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    geometry_msgs::Vector3 v;
    v.x = msg->x;
    v.y = msg->y;
    v.z = msg->z;
    loc_system->InputAccel(v);
}

void depthCallback(const std_msgs::Float32::ConstPtr& msg)
{
    std_msgs::Float32 f;
    f.data = msg->data;
    loc_system->InputDepth(f);
}

bool zeroPosCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep)
{
    loc_system->SetPosition(0.0, 0.0, 0.0);
    loc_system->SetVelocity(0.0, 0.0, 0.0);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "converter");

    ros::NodeHandle n;

    ros::ServiceServer zero_pos_service = n.advertiseService("zero_pos", zeroPosCallback);
    ros::Subscriber orientation_sub = n.subscribe("rs_bno_data", 1, orientationCallback);
    ros::Subscriber accel_sub = n.subscribe("rs_accel_data", 1, accelCallback);
    ros::Subscriber depth_sub = n.subscribe("rs_depth_data", 1, depthCallback);
    loc_pub = n.advertise<geometry_msgs::Vector3>("position", 1);

    loc_system = new LocalizationSystem(1.0/33.0);

    ros::Rate r(40.0);

    while(ros::ok())
    {
        loc_system->Update();

        r.sleep();
        ros::spinOnce();
    }

    delete loc_system;

    return 0;
}
