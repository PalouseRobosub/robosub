#include "StereoProcessor.hpp"
#include "robosub/visionPosArray.h"

#include <algorithm>
#include <iostream>
#include "sensor_msgs/Image.h"
#include <string>
#include <vector>
#include "wfov_camera_msgs/WFOVImage.h"

using std::vector;

ros::Publisher pub;
StereoProcessor *stereoProc;

bool leftOk = false;
bool rightOk = false;

void leftCamCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    stereoProc->updateLeft(*msg);

    if (!leftOk)
    {
        leftOk = true;
    }
}

void rightCamCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    stereoProc->updateRight(*msg);

    if (!rightOk)
    {
        rightOk = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_vision");

    ros::NodeHandle n;

    stereoProc = new StereoProcessor();

    ros::Subscriber leftCamSub = n.subscribe("camera/left/undistorted", 1,
                                             leftCamCallback);
    ros::Subscriber rightCamSub = n.subscribe("camera/right/undistorted", 1,
                                              rightCamCallback);

    /*
     * This output topic should be remapped when launched to avoid conflicts.
     * See vision.launch for examples
     */
    pub = n.advertise<robosub::visionPosArray>("vision/output_default", 1);

    //Create named windows
    namedWindow(ros::this_node::getName() + " Original");
    namedWindow(ros::this_node::getName() + " left_mask");

    ROS_INFO_STREAM("Init done");

    ros::Rate r(10);
    while (ros::ok())
    {
        if (leftOk && rightOk)
        {
            robosub::visionPosArray vpa;
            stereoProc->locateCentroids(vpa);
            pub.publish(vpa);
        }
        ros::spinOnce();
        r.sleep();
    }

    delete stereoProc;
    return 0;
}
