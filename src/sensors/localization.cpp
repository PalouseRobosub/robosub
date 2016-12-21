#include "localization_system.hpp"
#include <eigen3/Eigen/Dense>

/*
msg/PositionsStamped.msg
std_msgs/Header header
float32[] distances
geometry_msgs/Vector3[] positions
*/

using namespace Eigen;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization");

    ros::NodeHandle n;

    LocalizationSystem loc_system(1.0/33.0, 5);

    // TODO: Change to use localization messages
    ros::Publisher loc_pub = n.advertise<geometry_msgs::Vector3>("pf_position", 1);

    // Service for resetting position and velocity
    ros::ServiceServer reset_filter_service = n.advertiseService("reset_particle_filter", &LocalizationSystem::resetFilterCallback, &loc_system);

    ros::Subscriber depth_sub = n.subscribe("depth", 1, &LocalizationSystem::depthCallback, &loc_system);
    ros::Subscriber hydrophones_position_sub = n.subscribe("hydrophones/position", 1, &LocalizationSystem::hydrophoneCallback, &loc_system);
    ros::Subscriber accel_sub = n.subscribe("rs_lin_accel_data", 1, &LocalizationSystem::linAccelCallback, &loc_system);

    ros::Rate r(40.0);

    while(ros::ok())
    {
        ros::spinOnce();
        loc_system.Update();
        //geometry_msgs::Vector3 pos = loc_system.GetLocalizationMessage();
        //loc_pub.publish(pos);
        r.sleep();


        //ROS_INFO_STREAM(loc_system.randn() << " : " << loc_system.randu());

        //std::vector<double> t = {1.1, 5.6, 1.1, 0.0, 100.0};
        //ROS_INFO_STREAM("t: " << t);
        //ROS_INFO_STREAM("CumSum(t): " << CumSum(t));

        //Matrix<double,6,1> state;
        //Matrix<double,4,1> obs;

        //state(0,0) = 28.0;
        //state(1,0) = 10.0;
        //state(2,0) = -3.0;
        //state(3,0) = state(4,0) = state(5,0) = 0.0;

        //obs = loc_system.state_to_observation(state);
        //ROS_INFO_STREAM("obs: " << obs);

        //obs = loc_system.add_observation_noise(obs);
        //ROS_INFO_STREAM("obs: " << obs);
    }

    return 0;
}
