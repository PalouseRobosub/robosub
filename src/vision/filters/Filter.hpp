#ifndef FILTER_HPP
#define FILTER_HPP

#include <ros/ros.h>
#include <string>
#include <map>

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
using std::string;
using std::map;
using XmlRpc::XmlRpcValue;

class Filter
{
    public:
        virtual ~Filter() {};

        string getName()
        {
            return this->name;
        }

        virtual void setParams(XmlRpcValue &params) = 0;

        // Should apply the filter to the specified image
        virtual void apply(Mat &image) = 0;

        // Should apply the filter to the src image and place the result in
        //  the dst image
        virtual void apply(const Mat &src, Mat &dst) = 0;

    protected:
        string name;
};

#endif //FILTER_HPP
