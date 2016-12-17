#include "VisionProcessor.hpp"
#include "wfov_camera_msgs/WFOVImage.h"
#include "sensor_msgs/Image.h"
#include "robosub/visionPos.h"
#include "robosub/visionPosArray.h"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

using std::vector;

ros::Publisher pub;

int nLargest = 1;

bool compareContourAreas(vector<Point> contour1, vector<Point> contour2)
{
    double i = fabs(contourArea(Mat(contour1)));
    double j = fabs(contourArea(Mat(contour2)));
    return (i < j);
}

void leftCamCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    //Create a vision processor
    VisionProcessor vp;

    //Process the image using the VisionProcessor
    Mat processed = vp.process(*msg);

    //Clone the output image for showing
    Mat procOut = processed.clone();

    //Create a Mat of the original image for showing
    Mat original = cv_bridge::toCvShare(msg,
                                    sensor_msgs::image_encodings::BGR8)->image;

    //Find contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(processed, contours, hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    //Create the output message
    robosub::visionPos outMsg;

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

    robosub::visionPosArray arrayOut;

    ROS_DEBUG_STREAM("Contour size: " << contours.size());
    //If there are no contours, no calculations needed
    if (contours.size() >= 1)
    {
        std::sort(contours.begin(), contours.end(), compareContourAreas);

        //Find the area of the first contour
        //double largestArea = contourArea(contours[0], false);
        //int largestIndex = 0;

        //Compare the first contour to other areas to find contour with the
        //largest area
        //for (unsigned int i = 1; i < contours.size(); i++)
        //{
        //    double area = contourArea(contours[i], false);
        //    if (area > largestArea)
        //    {
        //        largestArea = area;
        //        largestIndex = i;
        //    }
        //}


        for (int i = 0; i < nLargest; ++i)
        {
            if (static_cast<unsigned int>(i) >= contours.size())
            {
                //There are no more contours to process.
                break;
            }
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
                    std::cout << "Center at: " << "[" << cx - (imWidth/2) <<
                                 "," << -1*(cy-(imHeight / 2)) << "]" <<
                                 std::endl;
                    circle(original, center, 5, Scalar(255, 255, 255), -1);
                    //Draw a circle on the original image for location
                    //visualization
                    circle(original, center, 4, Scalar(0, 0, 255), -1);
                }
            }

            ROS_DEBUG_STREAM("Preparing output");
            //Prepare the output message
            outMsg.xPos = cx - (imWidth / 2);
            ROS_DEBUG_STREAM("X prepared");
            outMsg.yPos = cy - (imHeight / 2);
            ROS_DEBUG_STREAM("Y prepared");
            outMsg.magnitude = static_cast<double>(contourArea(contours[i],
                                                   false)) /
                               static_cast<double>(imWidth * imHeight);
            ROS_DEBUG_STREAM("Magnitude prepared");

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
    }
    //Wait for 1 millisecond to show images
    waitKey(1);

    //Publish output message
    ROS_DEBUG_STREAM("Publishing message");

    pub.publish(arrayOut);
}

void rightCamCallback(const wfov_camera_msgs::WFOVImage::ConstPtr& msg)
{
    robosub::visionPosArray outMsg;


    pub.publish(outMsg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");

    ros::NodeHandle n;

    ros::Subscriber leftCamSub = n.subscribe("camera/left/undistorted", 1,
                                             leftCamCallback);
    ros::Subscriber rightCamSub = n.subscribe("camera/right/undistorted", 1,
                                              rightCamCallback);

    string topic;

    pub =
      n.advertise<robosub::visionPosArray>("vision/output_default" + topic, 1);


    ros::spin();

    return 0;
}
