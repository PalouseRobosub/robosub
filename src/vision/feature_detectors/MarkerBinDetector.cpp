#include "ObstacleDetector.hpp"
#include <vector>

MarkerBinDetector::MarkerBinDetector()
{
}

MarkerBinDetector::~MarkerBinDetector()
{
}

void MarkerBinDetector::init(ObstacleDetector *&detector)
{
    detector = new MarkerBinDetector();
}

void MarkerBinDetector::setParams(XmlRpcValue &params)
{
}

void MarkerBinDetector::process(const Mat &leftMask, const Mat &rightMask,
                                const Mat &disp, const Mat &_3dImg,
                                vector<visionPos> &messages)
{
    ROS_ERROR_ONCE("Marker Bin detector currently unimplemented");
}

void MarkerBinDetector::process(const Mat &bottomMask,
                                vector<visionPos> &bottomMessages)
{
    ROS_ERROR_ONCE("Marker Bin detector bottom cam currently unimplemented");
}
