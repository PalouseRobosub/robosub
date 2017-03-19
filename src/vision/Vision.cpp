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

VisionProcessor *colorProcessor;
StereoProcessor *stereoProcessor;
FeatureProcessor *featureProcessor;

int nLargest = 1;
bool doImShow = false;

Mat Q(Size(4, 4), CV_64FC1);

// Publish messages
void outputMessages(vector<visionPos> messages)
{
    visionPosArray output;
    for (auto it = messages.begin(); it != messages.end(); it++)
    {
        output.data.push_back(*it);
    }

    pub.publish(output);
}

// Get the parameters from the parameter server
void fetchParams()
{
    ros::NodeHandle nh("~");

    //Determine if should show images
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
}

// Places centroids onto the image and displays it
void displayFinalImage(const Image::ConstPtr &image, vector<visionPos> messages)
{
    Mat finalImg;

    finalImg = toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;

    int imHeight = finalImg.size().width;
    int imWidth = finalImg.size().height;

    for (visionPos point : messages)
    {
        int xPosition = (point.xPos * static_cast<double>(imWidth / 2)) +
                        (imWidth / 2);
        int yPosition = (point.yPos * static_cast<double>(imHeight / 2)) +
                        (imHeight / 2);
        Point2f center = cv::Point2f(xPosition, yPosition);

        ROS_INFO_STREAM("Adding circle to image at [" << xPosition << ", " <<
                        yPosition << "] with normalized: [" << point.xPos <<
                        ", " << point.yPos << "]");

        circle(finalImg, center, 5, Scalar(255, 255, 255), -1);
        circle(finalImg, center, 4, Scalar(0, 0, 255), -1);
    }

    if (doImShow)
    {
        imshow(ros::this_node::getName() + " Original", finalImg);
        waitKey(1);
    }
    else
    {
        destroyAllWindows();
    }
}

// Process vision with only left and right images
void callback(const Image::ConstPtr &left, const Image::ConstPtr &right)
{
    fetchParams();

    /////  Color Processing  /////
    //Process the image using the VisionProcessor
    Mat leftProcessed = colorProcessor->process(*left);
    Mat rightProcessed = colorProcessor->process(*right);

    /////  Stereo Processing  /////
    Mat disparity;
    Mat _3dImage;

    //Compute stereo depth map
    stereoProcessor->process(*left, *right, Q, disparity, _3dImage);

    /////  Feature Processing  /////

    //Send information to feature processing
    featureProcessor->setNLargest(nLargest);

    vector<visionPos> messages;
    Mat copy_left = leftProcessed.clone();
    Mat copy_right = rightProcessed.clone();
    messages = featureProcessor->process(copy_left, copy_right,
                                        disparity, _3dImage);

    displayFinalImage(left, messages);

    if (doImShow)
    {
        imshow(ros::this_node::getName() + " Masked", leftProcessed);
        waitKey(1);
    }
    else
    {
        destroyAllWindows();
    }

    outputMessages(messages);
}

// Process vision with all three images, left, right, and bottom.
void threeCamCallback(const Image::ConstPtr &left, const Image::ConstPtr &right,
                      const Image::ConstPtr &bottom)
{
    fetchParams();
    
    /////  Color Processing  /////

    //Process the image using the VisionProcessor
    Mat leftProcessed = colorProcessor->process(*left);
    Mat rightProcessed = colorProcessor->process(*right);
    Mat bottomProcessed = colorProcessor->process(*bottom);

    /////  Stereo Processing  /////
    Mat disparity;
    Mat _3dImage;

    //Compute stereo depth map
    stereoProcessor->process(*left, *right, Q, disparity, _3dImage);

    /////  Feature Processing  /////
    //Send information to feature processing
    featureProcessor->setNLargest(nLargest);

    vector<visionPos> messages;
    Mat copy_left = leftProcessed.clone();
    Mat copy_right = rightProcessed.clone();
    Mat copy_bottom = bottomProcessed.clone();

    
    Mat original = toCvCopy(left, sensor_msgs::image_encodings::BGR8)->image;
    Mat bottomOrig =
                   toCvCopy(bottom, sensor_msgs::image_encodings::BGR8)->image;
    
    messages = featureProcessor->process(original, bottomOrig, copy_left, copy_right,
                          copy_bottom,
                          disparity, _3dImage);

    if (doImShow)
    {
        imshow(ros::this_node::getName() + " Masked", leftProcessed);
        waitKey(1);
    }
    else
    {
        destroyAllWindows();
    }

    outputMessages(messages);
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

    colorProcessor = new VisionProcessor();
    stereoProcessor = new StereoProcessor();
    featureProcessor = new FeatureProcessor();

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

    //Currently, use command line argument to determine whether or not to use
    // the bottom camera. Another method should probably be devised.
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

    if (colorProcessor)
    {
        delete colorProcessor;
    }

    if (stereoProcessor)
    {
        delete stereoProcessor;
    }

    if (featureProcessor)
    {
        delete featureProcessor;
    }

    return 0;
}
