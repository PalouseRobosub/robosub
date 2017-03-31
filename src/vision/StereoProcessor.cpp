#include "StereoProcessor.hpp"
#include <XmlRpcException.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

StereoProcessor::StereoProcessor() :
    initialized(false)
{
}

StereoProcessor::~StereoProcessor()
{
    delete n;
}

void StereoProcessor::init()
{
    //The NodeHandle is dynamically allocated here to prevent the constructor
    //  from creating it. This is so ros::init() can be called before
    //  the NodeHandle is constructed.
    this->n = new NodeHandle("~processing");
    this->initialized = true;
}

//Processes the image using color filtering with parameters under given subgroup
void StereoProcessor::process(const Image &leftImage, const Image &rightImage,
                              const Mat &Q, Mat &disparityMat, Mat &_3dImageMat)
{
    if (!initialized)
    {
        ROS_FATAL_STREAM("Stereo Processor process called before init.");
        ros::shutdown();
        return;
    }

    Mat leftImg = toOpenCV(leftImage);
    Mat rightImg = toOpenCV(rightImage);

    if (leftImg.empty() || rightImg.empty())
    {
        ROS_WARN_STREAM("Left or right image empty, no processing performed");
        return;
    }
    //Convert to Grayscale and HSV color space
    Mat leftGray, rightGray;
    cvtColor(leftImg, leftGray, cv::COLOR_BGR2GRAY);
    cvtColor(rightImg, rightGray, cv::COLOR_BGR2GRAY);

    //Create disparity image mats
    Mat imgDisparity16S = Mat(leftImg.rows, leftImg.cols, CV_16S);
    Mat imgDisparity8U = Mat(leftImg.rows, leftImg.cols, CV_8UC1);

    ROS_DEBUG_STREAM("Converted images to grayscale");

    bool doImShow = false;
    n->getParamCached("doImShow", doImShow);

    int nDisparities = 16*5; //Range of disparity
    int SADWindowSize = 21; // Size of the block window. Must be odd.

    //Get params
    n->getParamCached("stereo/nDisparities", nDisparities);
    n->getParamCached("stereo/SADWindowSize", SADWindowSize);

    if (nDisparities < 0)
    {
        nDisparities = 0;
    }
    else if (nDisparities % 16 != 0)
    {
        //Must be divisible by 16
        nDisparities += 16 - (nDisparities % 16);
        n->setParam("stereo/nDisparities", nDisparities);
    }

    if (SADWindowSize < 5)
    {
        SADWindowSize = 5;
    }
    else if (SADWindowSize > 255 || SADWindowSize > leftImg.size().width ||
             SADWindowSize > leftImg.size().height)
    {
        SADWindowSize = (255 < leftImg.size().width) ? 255 :
                        ((leftImg.size().width < leftImg.size().height) ?
                          leftImg.size().width : leftImg.size().height);
    }
    if (SADWindowSize % 2 != 1)
    {
        SADWindowSize++;
    }

    //Create a StereoBM
    Ptr<StereoBM> sbm = StereoBM::create(nDisparities, SADWindowSize);

    //Calculate the disparity image
    sbm->compute(leftGray, rightGray, imgDisparity16S);

    //Check extremes
    double minVal, maxVal;
    minMaxLoc(imgDisparity16S, &minVal, &maxVal);

    //Display image as CV8UC1 image
    imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));

    Mat _3dImage;
    reprojectImageTo3D(imgDisparity16S, _3dImage, Q);

    if (doImShow)
    {
        imshow("Disparity 8U", imgDisparity8U);
        imshow("3D Image", _3dImage);
    }

    ROS_DEBUG_STREAM("Returning masks");
    disparityMat = imgDisparity8U;
    _3dImageMat = _3dImage;
}

///////Private functions///////

Mat StereoProcessor::toOpenCV(const Image& image)
{
    ROS_DEBUG_STREAM("Image size in bytes: " << sizeof(image.data));
    //Determine if the image is empty, may be deprecated.
    if (sizeof(image.data) == 0)
    {
        ROS_ERROR_STREAM("Empty image");
        //Return empty mat on empty image
        return Mat();
    }

    //Construct a pointer to contain the converted image
    CvImagePtr cv_ptr;
    try
    {
        ROS_DEBUG_STREAM("Copying/sharing image...");
        //Copy the image using cv_bridge, in future should change to "toCvShare"
        cv_ptr = toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        exit(1);
    }

    ROS_DEBUG_STREAM("Returning Mat");
    return cv_ptr->image;
}
