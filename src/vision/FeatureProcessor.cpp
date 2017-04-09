#include "FeatureProcessor.hpp"
#include <string>
#include <vector>

FeatureProcessor::FeatureProcessor() :
    initialized(false)
{
}

FeatureProcessor::~FeatureProcessor()
{
    delete n;
    delete detector;
}

void FeatureProcessor::init()
{
    if (!initialized)
    {
        //The NodeHandle is dynamically allocated here to prevent the
        //  constructor from creating it. This is so ros::init() can be called
        //  before the NodeHandle is constructed.
        this->n = new NodeHandle("~processing/features");
        this->initialized = true;
    }
}

void FeatureProcessor::process(const Mat &leftMask, const Mat &rightMask,
                               const Mat &disp, const Mat &_3dImg,
                               vector<visionPos> &messages)
{
    if (!initialized)
    {
        ROS_FATAL_STREAM("Feature Processor process called before init.");
        ros::shutdown();
        return;
    }

    if (!updateDetector())
    {
        return;
    }

    detector->process(leftMask, rightMask, disp, _3dImg, messages);
}

void FeatureProcessor::process(const Mat &leftMask, const Mat &rightMask,
                               const Mat &bottomMask, const Mat &disp,
                               const Mat &_3dImg,
                               vector<visionPos> &stereoMessages,
                               vector<visionPos> &bottomMessages)
{
    if (!initialized)
    {
        ROS_FATAL_STREAM("Feature Processor process called before init.");
        ros::shutdown();
        return;
    }

    if (!updateDetector())
    {
        return;
    }

    detector->process(leftMask, rightMask, disp, _3dImg, stereoMessages);
    detector->process(bottomMask, bottomMessages);
}

bool FeatureProcessor::updateDetector()
{
    string detectorType = "";
    if(!n->getParamCached("detector", detectorType))
    {
        ROS_WARN_ONCE("Could not get feature detector type, defaulting to "
                      "CENTROID. (This prints only once)");
        detectorType = "CENTROID";
    }

    if (boost::iequals(detectorType, "CENTROID"))
    {
        detector = new CentroidDetector();
    }
    else
    {
        ROS_FATAL_STREAM("Invalid process type: " << detectorType << ". "
                         "Consider adding it to updateDetector in "
                         "FeatureProcessor.");
        ros::shutdown();
        return false;
    }

    XmlRpcValue params;
    if (!n->getParamCached("params", params))
    {
        ROS_FATAL("Could not get params for feature detector.");
        ros::shutdown();
        return false;
    }

    detector->setParams(params);
    return true;
}
