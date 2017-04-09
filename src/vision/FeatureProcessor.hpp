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
#include <string>
#include <map>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#include <boost/algorithm/string.hpp>

#include "feature_processors/ObstacleDetector.hpp"

using namespace cv;
using std::vector;
using std::string;
using std::map;
using ros::NodeHandle;
using robosub::visionPos;
using XmlRpc::XmlRpcValue;
using XmlRpc::XmlRpcException;


class FeatureProcessor
{
    public:
        FeatureProcessor();
        ~FeatureProcessor();

        void init();

        void process(const Mat &leftMask, const Mat &rightMask, const Mat &disp,
                     const Mat &_3dImg, vector<visionPos> &messages);

        void process(const Mat &leftMask, const Mat &rightMask,
                     const Mat &bottomMask, const Mat &disp, const Mat &_3dImg,
                     vector<visionPos> &stereoMessages,
                     vector<visionPos> &bottomMessages);

    private:
        NodeHandle *n;
        bool initialized;

        ObstacleDetector *detector;
        bool updateDetector();

        map<string, void(*)(ObstacleDetector *)> detectors =
        {
            {"CENTROID", CentroidDetector::init},
            {"START GATE", StartGateDetector::init},
            {"BUOY", BuoyDetector::init},
            {"CHANNEL", ChannelDetector::init},
            {"MARKER BIN", MarkerBinDetector::init},
            {"TORPEDO TARGET", TorpedoTargetDetector::init},
            {"OCTAGON", OctagonDetector::init}
        };
};

#endif //FEATUREPROCESSOR_HPP
