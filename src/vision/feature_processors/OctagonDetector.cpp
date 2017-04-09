#include "ObstacleDetector.hpp"
#include <vector>

OctagonDetector::OctagonDetector()
{
}

OctagonDetector::~OctagonDetector()
{
}

void OctagonDetector::init(ObstacleDetector *detector)
{
    detector = new OctagonDetector();
}

void OctagonDetector::setParams(XmlRpcValue &params)
{
}

void OctagonDetector::process(const Mat &leftMask, const Mat &rightMask,
                                const Mat &disp, const Mat &_3dImg,
                                vector<visionPos> &messages)
{
    ROS_ERROR_ONCE("Marker Bin detector currently unimplemented");
}

void OctagonDetector::process(const Mat &bottomMask,
                                vector<visionPos> &bottomMessages)
{
    ROS_ERROR_ONCE("Marker Bin detector bottom cam currently unimplemented");
}
