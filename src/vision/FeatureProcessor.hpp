#ifndef FEATUREPROCESSOR_HPP
#define FEATUREPROCESSOR_HPP
#include <ros/ros.h>
#include <robosub/visionPos.h>
#include <algorithm>

#if ROS_VERSION_MINIMUM(1, 12, 0)
    #include <cv_bridge/cv_bridge.h>
#else
    #include <cv3_bridge/cv_bridge.h>
#endif

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

using namespace cv;
using std::vector;
using ros::NodeHandle;
using robosub::visionPos;


class FeatureProcessor
{
    public:
        FeatureProcessor();
        ~FeatureProcessor();

        void init();

        void setNLargest(int nLargest);
        void process(const Mat &leftMask, const Mat &rightMask, const Mat &disp,
                     const Mat &_3dImg, vector<visionPos> &messages);

        void process(const Mat &leftMask, const Mat &rightMask,
                     const Mat &bottomMask, const Mat &disp, const Mat &_3dImg,
                     vector<visionPos> &stereoMessages,
                     vector<visionPos> &bottomMessages);

    private:
        NodeHandle *n;

        bool initialized;

        int nLargest;

        static bool compareContourAreas(vector<Point> contour1,
                                 vector<Point> contour2);
};

#endif //FEATUREPROCESSOR_HPP
