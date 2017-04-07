#ifndef FEATURETYPE_HPP
#define FEATURETYPE_HPP

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

class FeatureType
{
   public:
        virtual ~FeatureType() {}

        virtual void setParams(XmlRpcValue &params) = 0;

        virtual void process(const Mat &leftMask, const Mat &rightMask,
                             const Mat &disp, const Mat &_3dImg,
                             vector<visionPos> &messages) = 0;

        virtual void process(const Mat &bottomMask,
                             vector<visionPos> &bottomMessages) = 0;
};


/////////   Centroid Processor  ////////

class CentroidProcessor : public FeatureType
{
    public:
        CentroidProcessor();
        ~CentroidProcessor();

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
#endif //FEATURETYPE_HPP
