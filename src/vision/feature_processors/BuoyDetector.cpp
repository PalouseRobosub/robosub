#include "ObstacleDetector.hpp"
#include <vector>

BuoyDetector::BuoyDetector()
{
}

BuoyDetector::~BuoyDetector()
{
}

void BuoyDetector::init(ObstacleDetector *detector)
{
    detector = new BuoyDetector();
}

void BuoyDetector::setParams(XmlRpcValue &params)
{
}

void BuoyDetector::process(const Mat &leftMask, const Mat &rightMask,
                           const Mat &disp, const Mat &_3dImg,
                           vector<visionPos> &messages)
{
    ROS_ERROR_ONCE("Buoy Detector currently unimplemented");
}

void BuoyDetector::process(const Mat &bottomMask,
                           vector<visionPos> &bottomMessages)
{
    ROS_ERROR_ONCE("Buoy Detector bottom cam currently unimplemented");
}
