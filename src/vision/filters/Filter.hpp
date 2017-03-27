#ifndef FILTER_HPP
#define FILTER_HPP

#include <ros/ros.h>
#include <string>
#include <map>
#include <vector>

// The following #if is to use the correct version of the cv_bridge
// Kinetic by default uses OpenCV3 so we don't need the custom build
#if ROS_VERSION_MINIMUM(1, 12, 0) // Running Kinetic
    #include <cv_bridge/cv_bridge.h>
#else // Running Indigo
    #include <cv3_bridge/cv_bridge.h>
#endif // if ROS_VERSION_MINIMUM
#include <opencv2/imgproc.hpp>
#include <XmlRpcException.h>
#include <XmlRpcValue.h>

#include <boost/algorithm/string.hpp>

using namespace cv;
using std::vector;
using std::string;
using std::map;
using XmlRpc::XmlRpcValue;

class Filter
{
    public:
        virtual ~Filter() {}

        string getName()
        {
            return this->name;
        }

        // Update the params for this filter
        virtual void setParams(XmlRpcValue &params) = 0;

        // Should apply the filter to the specified image
        virtual void apply(Mat &image) = 0;

        // Should apply the filter to the src image and place the result in
        //  the dst image
        virtual void apply(const Mat &src, Mat &dst) = 0;

    protected:
        string name;
};

// ------------- Close Filter ----------- //

class CloseFilter : public Filter
{
    public:
        CloseFilter();
        ~CloseFilter() {}

        void setParams(XmlRpcValue &params);
        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);

    private:
        int iterations;
        Size size;
};

// ------------ Convert Filter ---------- //

class ConvertFilter : public Filter
{
    public:
        ConvertFilter();
        ~ConvertFilter() {}

        void setParams(XmlRpcValue &params);

        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);

    private:
        int code;

        const std::map<string, int> paramList
                                    {
                                        {"BGR2HSV", cv::COLOR_BGR2HSV},
                                        {"BGR2RGB", cv::COLOR_BGR2RGB},
                                        {"RGB2BGR", cv::COLOR_RGB2BGR},
                                        {"RGB2HSV", cv::COLOR_RGB2HSV},
                                        {"BGR2GRAY", cv::COLOR_BGR2GRAY},
                                        {"RGB2GRAY", cv::COLOR_RGB2GRAY}
                                    };
};

// ------------ In Range Filter ---------- //

class InRangeFilter : public Filter
{
    public:
        InRangeFilter(string &inRangeName);
        ~InRangeFilter() {}

        void setParams(XmlRpcValue &params);

        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);

    private:
        Scalar upperBounds;
        Scalar lowerBounds;
};

// ---------- Median Blur Filter ----------- //

class MedianBlurFilter : public Filter
{
    public:
        MedianBlurFilter();
        ~MedianBlurFilter() {}

        void setParams(XmlRpcValue &params);
        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);

    private:
        int kernelSize;
};

// ------------- Open Filter ------------ //

class OpenFilter : public Filter
{
    public:
        OpenFilter();
        ~OpenFilter() {}

        void setParams(XmlRpcValue &params);

        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);

    private:
        int iterations;
        Size size;
};

// ------------ Or Filter ------------- //

class OrFilter : public Filter
{
    public:
        OrFilter();
        ~OrFilter() {}

        void setParams(XmlRpcValue &params);

        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);

    private:
        vector<Filter *> children;
};

#endif //FILTER_HPP
