#include "VisionProcessor.hpp"
#include "StereoProcessor.hpp"
#include "FeatureProcessor.hpp"
#include "robosub/visionPos.h"
#include "robosub/visionPosArray.h"

#include <algorithm>
#include <iostream>
#include "opencv2/highgui.hpp"
#include "sensor_msgs/Image.h"
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


using std::vector;
using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;
using sensor_msgs::Image;

using robosub::visionPos;
using robosub::visionPosArray;

ros::Publisher pub;

int nLargest = 1;

Mat Q(Size(4, 4), CV_64FC1);

void callback(const Image::ConstPtr &left, const Image::ConstPtr &right)
{
    //Create a nodehandle that gets private parameters
    ros::NodeHandle nh("~");

    //Determine if should show images
    bool doImShow = false;
    if (!nh.getParamCached("processing/doImShow", doImShow))
    {
        ROS_WARN_STREAM("Could not get doImShow param, defaulting to false.");
    }

    //Get num of contours to find
    if (nh.getParamCached("nLargest", nLargest))
    {
        ROS_DEBUG_STREAM("Loaded " + ros::this_node::getName() +
                         " nLargest: " << nLargest);
    }

    //Create a vision processor
    VisionProcessor processor;

    //Process the image using the VisionProcessor
    Mat leftProcessed = processor.process(*left);
    Mat rightProcessed = processor.process(*right);

    //Create a stereo processor
    StereoProcessor stereoProc;

    Mat disparity;
    Mat _3dImage;

    //Compute stereo depth map
    stereoProc.process(*left, *right, Q, disparity, _3dImage);

    //Send information to feature processing
    FeatureProcessor fp(nLargest);

    vector<visionPos> messages;
    Mat copy_left = leftProcessed.clone();
    Mat copy_right = rightProcessed.clone();
    Mat original = toCvCopy(left, sensor_msgs::image_encodings::BGR8)->image;
    messages = fp.process(original, copy_left, copy_right,
                          disparity, _3dImage);

    visionPosArray output;
    for (auto it = messages.begin(); it != messages.end(); it++)
    {
        output.data.push_back(*it);
    }

    if (doImShow)
    {
        imshow(ros::this_node::getName() + " Masked", leftProcessed);
        waitKey(1);
    }
    else
    {
        destroyAllWindows();
    }

    pub.publish(output);
}

void threeCamCallback(const Image::ConstPtr &left, const Image::ConstPtr &right,
                      const Image::ConstPtr &bottom)
{
    //Create a nodehandle that gets private parameters
    ros::NodeHandle nh("~");

    //Determine if should show images
    bool doImShow = false;
    if (!nh.getParamCached("processing/doImShow", doImShow))
    {
        ROS_WARN_STREAM("Could not get doImShow param, defaulting to false.");
    }

    //Get num of contours to find
    if (nh.getParamCached("nLargest", nLargest))
    {
        ROS_DEBUG_STREAM("Loaded " + ros::this_node::getName() +
                         " nLargest: " << nLargest);
    }

    //Create a vision processor
    VisionProcessor processor;

    //Process the image using the VisionProcessor
    Mat leftProcessed = processor.process(*left);
    Mat rightProcessed = processor.process(*right);
    Mat bottomProcessed = processor.process(*bottom);

    //Create a stereo processor
    StereoProcessor stereoProc;

    Mat disparity;
    Mat _3dImage;

    //Compute stereo depth map
    stereoProc.process(*left, *right, Q, disparity, _3dImage);

    //Send information to feature processing
    FeatureProcessor fp(nLargest);

    vector<visionPos> messages;
    Mat copy_left = leftProcessed.clone();
    Mat copy_right = rightProcessed.clone();
    Mat original = toCvCopy(left, sensor_msgs::image_encodings::BGR8)->image;
    messages = fp.process(original, copy_left, copy_right,
                          disparity, _3dImage);

    visionPosArray output;
    for (auto it = messages.begin(); it != messages.end(); it++)
    {
        output.data.push_back(*it);
    }

    if (doImShow)
    {
        imshow(ros::this_node::getName() + " Masked", leftProcessed);
        waitKey(1);
    }
    else
    {
        destroyAllWindows();
    }

    pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");

    ros::NodeHandle n;

    //Load Q matrix
    FileStorage fs(argv[1], FileStorage::READ);

    if (!fs.isOpened())
    {
        ROS_FATAL("Invalid filename");
        exit(1);
    }

    fs["Q"] >> Q;

    message_filters::Subscriber<Image> leftCamSub(n, "camera/left/undistorted",
                                                  1);
    message_filters::Subscriber<Image> rightCamSub(n,
                                                   "camera/right/undistorted",
                                                   1);
    message_filters::Subscriber<Image> bottomCamSub(n,
                                                    "camera/bottom/undistorted",
                                                    1);

    Synchronizer<ApproximateTime<Image, Image, Image>> *sync3;
    Synchronizer<ApproximateTime<Image, Image>> *syncStereo;
    
    if (argc >= 3)
    {
        ROS_INFO_STREAM(ros::this_node::getName() <<
                        " creating sync with bottom cam");
    
        sync3 = new Synchronizer<ApproximateTime<Image, Image, Image>>(
                        ApproximateTime<Image, Image, Image>(5), leftCamSub,
                        rightCamSub, bottomCamSub);
        sync3->registerCallback(boost::bind(&threeCamCallback, _1, _2, _3));
    }
    else
    {
        ROS_INFO_STREAM(ros::this_node::getName() <<
                        " creating sync without bottom cam");
        syncStereo = new Synchronizer<ApproximateTime<Image, Image>>(
                        ApproximateTime<Image, Image>(5), leftCamSub,
                        rightCamSub);
        syncStereo->registerCallback(boost::bind(&callback, _1, _2));
    }
    /*
     * This output topic should be remapped when launched to avoid conflicts.
     * See vision.launch for examples
     */
    pub = n.advertise<robosub::visionPosArray>("vision/output_default", 1);

    //Create named windows
    namedWindow(ros::this_node::getName() + " Original");
    namedWindow(ros::this_node::getName() + " left_mask");

    ROS_INFO_STREAM("Init done");

    ros::spin();

    if (sync3)
    {
        delete sync3;
    }

    if (syncStereo)
    {
        delete syncStereo;
    }

    return 0;
}
