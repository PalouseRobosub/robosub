#ifndef OBSTACLEDETECTOR_HPP
#define OBSTACLEDETECTOR_HPP

#include <ros/ros.h>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <vector>

#include <robosub/visionPos.h>

#include <opencv2/imgproc.hpp>

using namespace cv;
using std::vector;
using XmlRpc::XmlRpcValue;
using robosub::visionPos;

class ObstacleDetector
{
   public:
        virtual ~ObstacleDetector() {}

        virtual void setParams(XmlRpcValue &params) = 0;

        virtual void process(const Mat &leftMask, const Mat &rightMask,
                             const Mat &disp, const Mat &_3dImg,
                             vector<visionPos> &messages) = 0;

        virtual void process(const Mat &bottomMask,
                             vector<visionPos> &bottomMessages) = 0;
};


/////////   Centroid Detector  ////////

class CentroidDetector : public ObstacleDetector
{
    public:
        CentroidDetector();
        ~CentroidDetector();

        static void init(ObstacleDetector *&detector);

        void setParams(XmlRpcValue &params);

        void process(const Mat &leftMask, const Mat &rightMask,
                     const Mat &disp, const Mat &_3dImg,
                     vector<visionPos> &messages);

        void process(const Mat &bottomMask, vector<visionPos> &bottomMessages);

    private:
        int nLargest;

        static bool compareContourAreas(vector<Point> contour1,
                                        vector<Point> contour2);
};

//////////  Start Gate Detector  ////////////

class StartGateDetector : public ObstacleDetector
{
    public:
        StartGateDetector();
        ~StartGateDetector();

        static void init(ObstacleDetector *&detector);

        void setParams(XmlRpcValue &params);

        void process(const Mat &leftMask, const Mat &rightMask,
                     const Mat &disp, const Mat &_3dImg,
                     vector<visionPos> &messages);

        void process(const Mat &bottomMask, vector<visionPos> &bottomMessages);
};

//////////  Buoy Detector  ////////////

class BuoyDetector : public ObstacleDetector
{
    public:
        BuoyDetector();
        ~BuoyDetector();

        static void init(ObstacleDetector *&detector);

        void setParams(XmlRpcValue &params);

        void process(const Mat &leftMask, const Mat &rightMask,
                     const Mat &disp, const Mat &_3dImg,
                     vector<visionPos> &messages);

        void process(const Mat &bottomMask, vector<visionPos> &bottomMessages);
};

//////////  Channel Detector  ////////////

class ChannelDetector : public ObstacleDetector
{
    public:
        ChannelDetector();
        ~ChannelDetector();

        static void init(ObstacleDetector *&detector);

        void setParams(XmlRpcValue &params);

        void process(const Mat &leftMask, const Mat &rightMask,
                     const Mat &disp, const Mat &_3dImg,
                     vector<visionPos> &messages);

        void process(const Mat &bottomMask, vector<visionPos> &bottomMessages);
};

//////////  Marker Bin Detector  ////////////

class MarkerBinDetector : public ObstacleDetector
{
    public:
        MarkerBinDetector();
        ~MarkerBinDetector();

        static void init(ObstacleDetector *&detector);

        void setParams(XmlRpcValue &params);

        void process(const Mat &leftMask, const Mat &rightMask,
                     const Mat &disp, const Mat &_3dImg,
                     vector<visionPos> &messages);

        void process(const Mat &bottomMask, vector<visionPos> &bottomMessages);
};

//////////  Torpedo Target Detector  ////////////

class TorpedoTargetDetector : public ObstacleDetector
{
    public:
        TorpedoTargetDetector();
        ~TorpedoTargetDetector();

        static void init(ObstacleDetector *&detector);

        void setParams(XmlRpcValue &params);

        void process(const Mat &leftMask, const Mat &rightMask,
                     const Mat &disp, const Mat &_3dImg,
                     vector<visionPos> &messages);

        void process(const Mat &bottomMask, vector<visionPos> &bottomMessages);
};

//////////  Octagon Detector  ////////////

class OctagonDetector : public ObstacleDetector
{
    public:
        OctagonDetector();
        ~OctagonDetector();

        static void init(ObstacleDetector *&detector);

        void setParams(XmlRpcValue &params);

        void process(const Mat &leftMask, const Mat &rightMask,
                     const Mat &disp, const Mat &_3dImg,
                     vector<visionPos> &messages);

        void process(const Mat &bottomMask, vector<visionPos> &bottomMessages);
};
#endif //OBSTACLEDETECTOR_HPP
