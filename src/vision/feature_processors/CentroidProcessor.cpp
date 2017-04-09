#include "ObstacleDetector.hpp"
#include <algorithm>
#include <vector>

CentroidProcessor::CentroidProcessor()
{
}

CentroidProcessor::~CentroidProcessor()
{
}

void CentroidProcessor::setParams(XmlRpcValue &param)
{
    this->nLargest = param["nLargest"];
}

void CentroidProcessor::process(const Mat &leftImg,
                               const Mat &rightImg,
                               const Mat &disp,
                               const Mat &_3dImg,
                               vector<visionPos> &messages)
{
    vector<vector<Point>> lContours, rContours;
    vector<Vec4i> lHierarchy, rHierarchy;

    findContours(leftImg, lContours, lHierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    findContours(rightImg, rContours, rHierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // For now use the left image, in future, use both
    std::sort(lContours.begin(), lContours.end(),
              CentroidProcessor::compareContourAreas);

    for (int i = 0; i < nLargest &&
                    static_cast<unsigned int>(i) < lContours.size(); ++i)
    {
        Moments moment;
        moment = moments(lContours[i], false);

        int cx = -1;
        int cy = -1;
        int imWidth = leftImg.size().width;
        int imHeight = leftImg.size().height;

        if (moment.m00 > 0)
        {
            cx = static_cast<int>(moment.m10 / moment.m00);
            cy = static_cast<int>(moment.m01 / moment.m00);
        }
        else
        {
            ROS_WARN_STREAM("Invalid m00 moment found in "
                             << ros::this_node::getName());
            return;
        }

        robosub::visionPos msg;

        msg.xPos = (cx - (imWidth / 2)) /
                   static_cast<double>(imWidth / 2);
        msg.yPos = (cy - (imHeight / 2)) /
                   static_cast<double>(imHeight / 2);

        msg.magnitude = static_cast<double>(contourArea(lContours[i], false)) /
                        static_cast<double>(imWidth * imHeight);

        messages.push_back(msg);
    }
}

void CentroidProcessor::process(const Mat &bottomImg,
                                vector<visionPos> &bottomMessages)
{
    vector<vector<Point>> bContours;
    vector<Vec4i> bHierarchy;

    findContours(bottomImg, bContours, bHierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Process bottom image
    std::sort(bContours.begin(), bContours.end(),
              CentroidProcessor::compareContourAreas);

    for (int i = 0; i < nLargest &&
                    static_cast<unsigned int>(i) < bContours.size(); ++i)
    {
        Moments moment;
        moment = moments(bContours[i], false);

        int cx = -1;
        int cy = -1;
        int imWidth = bottomImg.size().width;
        int imHeight = bottomImg.size().height;

        if (moment.m00 > 0)
        {
            cx = static_cast<int>(moment.m10 / moment.m00);
            cy = static_cast<int>(moment.m01 / moment.m00);
        }
        else
        {
            ROS_WARN_STREAM("Invalid m00 moment found in "
                             << ros::this_node::getName());
            return;
        }

        robosub::visionPos msg;

        msg.xPos = (cx - (imWidth / 2)) /
                   static_cast<double>(imWidth / 2);
        msg.yPos = (cy - (imHeight / 2)) /
                   static_cast<double>(imHeight / 2);

        msg.magnitude = static_cast<double>(contourArea(bContours[i], false)) /
                        static_cast<double>(imWidth * imHeight);

        bottomMessages.push_back(msg);
    }
}

bool CentroidProcessor::compareContourAreas(vector<Point> contour1,
                                           vector<Point> contour2)
{
    double i = fabs(contourArea(Mat(contour1)));
    double j = fabs(contourArea(Mat(contour2)));
    return (i > j);
}
