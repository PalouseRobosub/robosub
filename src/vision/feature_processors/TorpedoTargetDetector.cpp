#include "ObstacleDetector.hpp"
#include <vector>

TorpedoTargetDetector::TorpedoTargetDetector()
{
}

TorpedoTargetDetector::~TorpedoTargetDetector()
{
}

void TorpedoTargetDetector::init(ObstacleDetector *detector)
{
    detector = new TorpedoTargetDetector();
}

void TorpedoTargetDetector::setParams(XmlRpcValue &params)
{
}

void TorpedoTargetDetector::process(const Mat &leftMask, const Mat &rightMask,
                                const Mat &disp, const Mat &_3dImg,
                                vector<visionPos> &messages)
{
    ROS_ERROR_ONCE("Torpedo Target detector currently unimplemented");
}

void TorpedoTargetDetector::process(const Mat &bottomMask,
                                vector<visionPos> &bottomMessages)
{
    ROS_ERROR_ONCE("Torpedo Target detector bottom cam currently "
                   "unimplemented");
}
