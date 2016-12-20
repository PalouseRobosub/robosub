#include "VisionProcessor.hpp"
#include "robosub/visionPos.h"
#include "robosub/visionPosArray.h"

#include <algorithm>
#include <iostream>
#include "opencv2/highgui.hpp"
#include "sensor_msgs/Image.h"
#include <string>
#include <vector>
#include "wfov_camera_msgs/WFOVImage.h"

using std::vector;

ros::Publisher pub;

int nLargest = 1;

bool compareContourAreas(vector<Point> contour1, vector<Point> contour2)
{
    double i = fabs(contourArea(Mat(contour1)));
    double j = fabs(contourArea(Mat(contour2)));
    return (i > j);
}

void leftCamCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    ///Fetch parameters///
    //Determine if should show images
    ros::NodeHandle nh("~");
    bool doImShow = true;
    if (!nh.getParamCached("processing/doImShow", doImShow))
    {
        ROS_DEBUG_STREAM("Could not get doImShow param, defaulting to true.");
    }

    //Get num of contours to find
    if (nh.getParamCached("nLargest", nLargest))
    {
        ROS_DEBUG_STREAM("Loaded " + ros::this_node::getName() +
                         " nLargest: " << nLargest);
    }
    //Create a vision processor
    VisionProcessor vp;

    //Process the image using the VisionProcessor
    Mat processed = vp.process(*msg);

    //Clone the output image for showing if requested
    Mat procOut;

    if (doImShow)
    {
        procOut = processed.clone();
    }

    //Create a Mat of the original image for showing
    Mat original = cv_bridge::toCvShare(msg,
                                    sensor_msgs::image_encodings::BGR8)->image;

    //Find contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(processed, contours, hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    robosub::visionPosArray arrayOut;

    ROS_DEBUG_STREAM("Contour size: " << contours.size());
    //If there are no contours, no calculations needed
    if (contours.size() >= 1)
    {
        //Sort contours by size
        std::sort(contours.begin(), contours.end(), compareContourAreas);

        for (int i = 0; i < nLargest &&
                        static_cast<unsigned int>(i) < contours.size(); ++i)
        {
            //Find the moments (physical properties) of the contour
            Moments moment;
            moment = moments(contours[i], false);

            int cx = -1;
            int cy = -1;
            int imWidth = msg->width;
            int imHeight = msg->height;
            //Using moments, find the center point of the contour
            if (moment.m00 > 0)
            {
                //Find x and y coordinates with 0,0 being top left corner
                cx = static_cast<int>(moment.m10/moment.m00);
                cy = static_cast<int>(moment.m01/moment.m00);
                if (doImShow)
                {
                    Point2f center = cv::Point2f(cx, cy);
                    ROS_INFO_STREAM("Center at: " << "[" << cx - (imWidth/2) <<
                                 "," << -1*(cy-(imHeight / 2)) << "]");
                    circle(original, center, 5, Scalar(255, 255, 255), -1);
                    //Draw a circle on the original image for location
                    //visualization
                    circle(original, center, 4, Scalar(0, 0, 255), -1);
                }
            }

            ROS_DEBUG_STREAM("Preparing output");
            ///Create the output message
            robosub::visionPos outMsg;

            //Prepare the output message
            outMsg.xPos = cx - (imWidth / 2);
            outMsg.yPos = cy - (imHeight / 2);
            outMsg.magnitude = static_cast<double>(contourArea(contours[i],
                                                   false)) /
                               static_cast<double>(imWidth * imHeight);
            //Add to output
            arrayOut.data.push_back(outMsg);
        }
    }

    //Show images
    if (doImShow)
    {
        namedWindow(ros::this_node::getName() + " Original");
        imshow(ros::this_node::getName() + " Original", original);

        namedWindow(ros::this_node::getName() + " left_mask");
        imshow(ros::this_node::getName() + " left_mask", procOut);
        //Wait for 1 millisecond to show images
        waitKey(1);
    }

    //Publish output message
    pub.publish(arrayOut);
}

/*void rightCamCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    //Currently unused
    robosub::visionPosArray outMsg;


    pub.publish(outMsg);
}*/


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");

    ros::NodeHandle n;

    ros::Subscriber leftCamSub = n.subscribe("camera/left/undistorted", 1,
                                             leftCamCallback);
    //ros::Subscriber rightCamSub = n.subscribe("camera/right/undistorted", 1,
                                              rightCamCallback);

    string topic;

    pub =
      n.advertise<robosub::visionPosArray>("vision/output_default" + topic, 1);


    ros::spin();

    return 0;
}
