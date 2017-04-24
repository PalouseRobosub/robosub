#include "StereoProcessor.hpp"
#include <XmlRpcException.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include <elas/elas.h>

StereoProcessor::StereoProcessor()
    : initialized(false)
{
}

StereoProcessor::~StereoProcessor()
{
    delete n;
}

void StereoProcessor::init()
{
    if (!initialized)
    {
        //The NodeHandle is dynamically allocated here to prevent the
        //  constructor from creating it. This is so ros::init() can be called
        //  before the NodeHandle is constructed.
        this->n = new NodeHandle("~processing");
        this->initialized = true;
    }
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
    //Mat imgDisparity16S = Mat(leftImg.rows, leftImg.cols, CV_16S);
    //Mat imgDisparity8U = Mat(leftImg.rows, leftImg.cols, CV_8UC1);

    ROS_DEBUG_STREAM("Converted images to grayscale");

    bool doImShow = false;
    n->getParamCached("doImShow", doImShow);

    /*int nDisparities = 16*5; //Range of disparity
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
    reprojectImageTo3D(imgDisparity16S, _3dImage, Q);*/

    ////////ELAS USE//////////

    //Using elas
    Elas::parameters params(Elas::setting::ROBOTICS);
    getParams(params);
    Elas elas(params);

    int width = leftGray.size().width;
    int height = leftGray.size().height;

    Mat leftDpf = Mat::zeros(Size(width, height), CV_32F);
    Mat rightDpf = Mat::zeros(Size(width, height), CV_32F);

    const int32_t dims[3] = {width, height, width};

    elas.process(static_cast<uint8_t *>(leftGray.data),
                 static_cast<uint8_t *>(rightGray.data),
                 leftDpf.ptr<float>(0), rightDpf.ptr<float>(0), dims);

    Mat leftDisp, rightDisp;

    if (doImShow)
    {
        //imshow("Left Dpf", leftDpf);
        //imshow("Right Dpf", rightDpf);
    }

    leftDpf.copyTo(leftDisp);
    rightDpf.copyTo(rightDisp);

    if (doImShow)
    {
        //imshow("Left Disp", leftDisp);
        //imshow("Right Disp", rightDisp);
    }

    double dispMax, dispMin;
    minMaxLoc(leftDisp, &dispMin, &dispMax);

    leftDisp.convertTo(leftDisp, CV_8UC1, 255/(dispMax - dispMin));
    rightDisp.convertTo(rightDisp, CV_8UC1, 255/(dispMax - dispMin));

    Mat elas3dLeft, elas3dRight;

    reprojectImageTo3D(leftDisp, elas3dLeft, Q);
    reprojectImageTo3D(rightDisp, elas3dRight, Q);

    if (doImShow)
    {
        //imshow("Disparity 8U", imgDisparity8U);
        //imshow("3D Image", _3dImage);
        imshow("ELAS left", leftDisp);
        imshow("ELAS right", rightDisp);
        imshow("ELAS left 3D", elas3dLeft);
        imshow("ELAS right 3D", elas3dRight);
    }

    ROS_DEBUG_STREAM("Returning masks");
    disparityMat = leftDisp;//imgDisparity8U;
    _3dImageMat = elas3dLeft;//_3dImage;
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

void StereoProcessor::getParams(Elas::parameters &params)
{
    if (!n->getParamCached("stereo/disp_min", params.disp_min))
    {
        n->setParam("stereo/disp_min", params.disp_min);
    }
    if (!n->getParamCached("stereo/disp_max", params.disp_max))
    {
        n->setParam("stereo/disp_max", params.disp_max);
    }
    if (!n->getParamCached("stereo/support_threshold", params.support_threshold))
    {
        n->setParam("stereo/support_threshold", params.support_threshold);
    }
    if (!n->getParamCached("stereo/support_texture", params.support_texture))
    {
        n->setParam("stereo/support_texture", params.support_texture);
    }
    if (!n->getParamCached("stereo/candidate_stepsize", params.candidate_stepsize))
    {
        n->setParam("stereo/candidate_stepsize", params.candidate_stepsize);
    }
    if (!n->getParamCached("stereo/incon_window_size", params.incon_window_size))
    {
        n->setParam("stereo/incon_window_size", params.incon_window_size);
    }
    if (!n->getParamCached("stereo/incon_threshold", params.incon_threshold))
    {
        n->setParam("stereo/incon_threshold", params.incon_threshold);
    }
    if (!n->getParamCached("stereo/incon_min_support", params.incon_min_support))
    {
        n->setParam("stereo/incon_min_support", params.incon_min_support);
    }
    if (!n->getParamCached("stereo/add_corners", params.add_corners))
    {
        n->setParam("stereo/add_corners", params.add_corners);
    }
    if (!n->getParamCached("stereo/grid_size", params.grid_size))
    {
        n->setParam("stereo/grid_size", params.grid_size);
    }
    if (!n->getParamCached("stereo/beta", params.beta))
    {
        n->setParam("stereo/beta", params.beta);
    }
    if (!n->getParamCached("stereo/gamma", params.gamma))
    {
        n->setParam("stereo/gamma", params.gamma);
    }
    if (!n->getParamCached("stereo/sigma", params.sigma))
    {
        n->setParam("stereo/sigma", params.sigma);
    }
    if (!n->getParamCached("stereo/sradius", params.sradius))
    {
        n->setParam("stereo/sradius", params.sradius);
    }
    if (!n->getParamCached("stereo/match_texture", params.match_texture))
    {
        n->setParam("stereo/match_texture", params.match_texture);
    }
    if (!n->getParamCached("stereo/lr_threshold", params.lr_threshold))
    {
        n->setParam("stereo/lr_threshold", params.lr_threshold);
    }
    if (!n->getParamCached("stereo/speckle_sim_threshold", params.speckle_sim_threshold))
    {
        n->setParam("stereo/speckle_sim_threshold", params.speckle_sim_threshold);
    }
    if (!n->getParamCached("stereo/speckle_size", params.speckle_size))
    {
        n->setParam("stereo/speckle_size", params.speckle_size);
    }
    if (!n->getParamCached("stereo/ipol_gap_width", params.ipol_gap_width))
    {
        n->setParam("stereo/ipol_gap_width", params.ipol_gap_width);
    }
    if (!n->getParamCached("stereo/filter_median", params.filter_median))
    {
        n->setParam("stereo/filter_median", params.filter_median);
    }
    if (!n->getParamCached("stereo/filter_adaptive_mean", params.filter_adaptive_mean))
    {
        n->setParam("stereo/filter_adaptive_mean", params.filter_adaptive_mean);
    }
    if (!n->getParamCached("stereo/postprocess_only_left", params.postprocess_only_left))
    {
        n->setParam("stereo/postprocess_only_left", params.postprocess_only_left);
    }
    if (!n->getParamCached("stereo/subsampling", params.subsampling))
    {
        n->setParam("stereo/subsampling", params.subsampling);
    }
}
