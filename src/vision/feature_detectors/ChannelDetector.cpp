#include "ObstacleDetector.hpp"
#include <vector>

ChannelDetector::ChannelDetector()
{
}

ChannelDetector::~ChannelDetector()
{
}

void ChannelDetector::init(ObstacleDetector *&detector)
{
    detector = new ChannelDetector();
}

void ChannelDetector::setParams(XmlRpcValue &params)
{
}

void ChannelDetector::process(const Mat &leftMask, const Mat &rightMask,
                                const Mat &disp, const Mat &_3dImg,
                                vector<visionPos> &messages)
{
    ROS_ERROR_ONCE("Channel detector currently unimplemented");
}

void ChannelDetector::process(const Mat &bottomMask,
                                vector<visionPos> &bottomMessages)
{
    ROS_ERROR_ONCE("Channel detector bottom cam currently unimplemented");
}
