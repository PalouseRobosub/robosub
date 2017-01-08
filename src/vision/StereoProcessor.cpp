#include "StereoProcessor.hpp"
#include <XmlRpcException.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

StereoProcessor::StereoProcessor()
{
    this->n = NodeHandle("~processing");
}

StereoProcessor::~StereoProcessor()
{
}

//Processes the image using color filtering with parameters under given subgroup
void StereoProcessor::process(Mat &contourMat, Mat &disparityMat)
{
    if (leftImg.empty() || rightImg.empty())
    {
        ROS_WARN_STREAM("Left or right image empty, no processing performed");
        return;
    }
    //Convert to Grayscale and HSV color space
    //Note: Only the left image is used for HSV color detection
    Mat leftGray, rightGray, leftHsv;
    cvtColor(leftImg, leftGray, cv::COLOR_BGR2GRAY);
    cvtColor(rightImg, rightGray, cv::COLOR_BGR2GRAY);
    cvtColor(leftImg, leftHsv, cv::COLOR_BGR2HSV);

    //Create disparity image mats
    Mat imgDisparity16S = Mat(leftImg.rows, leftImg.cols, CV_16S);
    Mat imgDisparity8U = Mat(leftImg.rows, leftImg.cols, CV_8UC1);

    ROS_DEBUG_STREAM("Converted images to hsv and grayscale");

    //Create the lowerbound of thresholding with defaults
    vector<Scalar> lower_bounds;

    getLowerBoundParams(lower_bounds);

    //Create upperbound of thresholding with defaults
    vector<Scalar> upper_bounds;

    getUpperBoundParams(upper_bounds);

    //Initialize blur iteration num with default
    int numBlurIters = 3;

    //Get blur param
    if (!n.getParamCached("blur_iters", numBlurIters))
    {
        ROS_ERROR_STREAM("Node does not contain blur_iters param");
    }

    //Initialize open filter size with default of 3x3
    int openSize[] = {3, 3};

    //Initialize open filter iteration num with default
    int numOpenIters = 1;

    //Get open params
    n.getParamCached("open/width", openSize[0]);
    n.getParamCached("open/height", openSize[1]);
    n.getParamCached("open/iters", numOpenIters);

    //Initialize open filter size with default of 3x3
    int closeSize[] = {3, 3};

    //Initialize open filter iteration num with default
    int numCloseIters = 1;

    //Get open params
    n.getParamCached("close/width", closeSize[0]);
    n.getParamCached("close/height", closeSize[1]);
    n.getParamCached("close/iters", numCloseIters);

    //Sizes must be odd for open filter
    if (openSize[0] % 2 != 1)
    {
        openSize[0]++;
    }

    if (openSize[1] % 2 != 1)
    {
        openSize[1]++;
    }

    if (closeSize[0] % 2 != 1)
    {
        closeSize[0]++;
    }

    if (closeSize[1] % 2 != 1)
    {
        closeSize[1]++;
    }

    ROS_DEBUG_STREAM("Performing Masking and/or other processing");

    //Blur the image for a bit more smoothness
    medianBlur(leftHsv, leftHsv, 3);

    bool doImShow = true;
    n.getParamCached("doImShow", doImShow);

    Mat mask;
    //Mask the image within the threshold ranges
    for (unsigned int i = 0;
         i < lower_bounds.size() && i < upper_bounds.size(); i++)
    {
        ROS_DEBUG_STREAM("Masking color range " << i << " from "
                            << lower_bounds[i] << " to " << upper_bounds[i]);
        Mat temp;
        inRange(leftHsv, lower_bounds[i], upper_bounds[i], temp);
        if (doImShow)
        {
            imshow(ros::this_node::getName() + " [" + std::to_string(i) +
                   "] Found", temp);
        }

        if (!mask.empty())
        {
            bitwise_or(mask, temp, mask);
        }
        else
        {
            temp.copyTo(mask);
        }
    }

    //Add a close filter to remove holes in objects
    morphologyEx(mask, mask, MORPH_CLOSE, getStructuringElement(MORPH_RECT,
              Size(closeSize[0], closeSize[1])), Point(-1, -1), numCloseIters);

    //Add an open filter which reduces small noise.
    morphologyEx(mask, mask, MORPH_OPEN, getStructuringElement(MORPH_RECT,
                 Size(openSize[0], openSize[1])), Point(-1, -1), numOpenIters);

    ROS_DEBUG_STREAM("Image Masked and/or processed otherwise");

    int nDisparities = 16*5; //Range of disparity
    int SADWindowSize = 21; // Size of the block window. Must be odd.

    //Get params
    n.getParamCached("stereo/nDisparities", nDisparities);
    n.getParamCached("stereo/SADWindowSize", SADWindowSize);

    if (nDisparities < 0)
    {
        nDisparities = 0;
    }
    else if (nDisparities % 16 != 0)
    {
        //Must be divisible by 16
        nDisparities += 16 - (nDisparities % 16);
        n.setParam("stereo/nDisparities", nDisparities);
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

    if (doImShow)
    {
        imshow("Disparity 16S", imgDisparity16S);
        imshow("Disparity 8U", imgDisparity8U);
    }

    ROS_DEBUG_STREAM("Returning masks");
    contourMat = mask;
    disparityMat = imgDisparity8U;
}

///////Private functions///////

void StereoProcessor::getScalarParamSet(string mapName,
                                        vector<Scalar> &scalars)
{
    string key;
    //Get lowerbound params
    if (n.searchParam(mapName, key))
    {
        XmlRpc::XmlRpcValue map;
        n.getParamCached(key, map);
        ROS_DEBUG_STREAM("Found: " << map.size() << " sets in key " << key);

        for (int i = 0; i < map.size(); i++)
        {
            std::map<std::string, double> parameters;
            try
            {
                for (auto it = map[i].begin(); it != map[i].end(); it++)
                {
                    double value = 0.0;
                    switch(it->second.getType())
                    {
                        case XmlRpc::XmlRpcValue::TypeDouble:
                            value = it->second;
                            break;
                        case XmlRpc::XmlRpcValue::TypeInt:
                            value = static_cast<double>(
                                                 static_cast<int>(it->second));
                            break;
                        default:
                            ROS_ERROR_STREAM("Invalid parameter type for " <<
                                             mapName << ": " << it->first);
                            return;
                            break;
                    }

                    parameters.emplace(it->first, value);
                }
            }
            catch (XmlRpc::XmlRpcException e)
            {
                ROS_ERROR_STREAM("XmlRpcException code " << e.getCode() <<
                                 ": " << e.getMessage());
                ros::shutdown();
            }

            Scalar s;
            s[0] = parameters["hue"];//map[i]["hue"];
            s[1] = parameters["sat"];
            s[2] = parameters["val"];
            ROS_DEBUG_STREAM("Scalar: " << s);
            scalars.push_back(s);
        }
        ROS_DEBUG_STREAM(mapName << " Params fetched");
    }
    else
    {
        ROS_ERROR_STREAM("Node does not contain " << mapName << " params");
    }
    ROS_DEBUG_STREAM("Size of scalars: " << scalars.size());
}

void StereoProcessor::getLowerBoundParams(vector<Scalar> &lower_bounds)
{
    getScalarParamSet("min", lower_bounds);
}
void StereoProcessor::getUpperBoundParams(vector<Scalar> &upper_bounds)
{
    getScalarParamSet("max", upper_bounds);
}

//Converts a ros image_transport Image to an OpenCV Mat
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

void StereoProcessor::updateLeft(const Image& image)
{
    leftImg = toOpenCV(image);
}

void StereoProcessor::updateRight(const Image& image)
{
    rightImg = toOpenCV(image);
}

void StereoProcessor::locateCentroids(robosub::visionPosArray &vpa)
{
    ///Fetch parameters///

    //Create a nodehandle that gets private parameters
    ros::NodeHandle nh("~");

    //Determine if should show images
    bool doImShow = true;
    if (!nh.getParamCached("processing/doImShow", doImShow))
    {
        ROS_DEBUG_STREAM("Could not get doImShow param, defaulting to true.");
    }

    int nLargest = 1;
    //Get num of contours to find
    if (nh.getParamCached("nLargest", nLargest))
    {
        ROS_DEBUG_STREAM("Loaded " + ros::this_node::getName() +
                         " nLargest: " << nLargest);
    }

    //Process the images stereoscopically
    Mat contourMat;
    Mat disparityMat;
    process(contourMat, disparityMat);

    //Clone the output image for showing if requested
    Mat procOut;

    if (doImShow)
    {
        procOut = contourMat.clone();
    }

    //Create a Mat of the original image for showing
    Mat original = leftImg.clone();

    //Find contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(contourMat, contours, hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


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
            int imWidth = leftImg.size().width;
            int imHeight = leftImg.size().height;
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
            outMsg.magnitude = disparityMat.at<double>(cx, cy);
            //Add to output
            vpa.data.push_back(outMsg);
        }
    }

    //Show images
    if (doImShow)
    {
        imshow(ros::this_node::getName() + " Original", original);

        imshow(ros::this_node::getName() + " left_mask", procOut);
        //Wait for 1 millisecond to show images
        waitKey(1);
    }
    else
    {
        destroyAllWindows();
    }
}

bool compareContourAreas(vector<Point> contour1, vector<Point> contour2)
{
    double i = fabs(contourArea(Mat(contour1)));
    double j = fabs(contourArea(Mat(contour2)));
    return (i > j);
}
