#include "ObstacleDetector.hpp"
#include <vector>

StartGateDetector::StartGateDetector()
{
}

StartGateDetector::~StartGateDetector()
{
}

void StartGateDetector::init(ObstacleDetector *&detector)
{
    detector = new StartGateDetector();
}

void StartGateDetector::setParams(XmlRpcValue &params)
{
}

void StartGateDetector::process(const Mat &leftMask, const Mat &rightMask,
                                const Mat &disp, const Mat &_3dImg,
                                vector<visionPos> &messages)
{
    ROS_ERROR_ONCE("Start Gate detector currently unimplemented");
}

void StartGateDetector::process(const Mat &bottomMask,
                                vector<visionPos> &bottomMessages)
{
    ROS_ERROR_ONCE("Start Gate detector bottom cam currently unimplemented");
}
