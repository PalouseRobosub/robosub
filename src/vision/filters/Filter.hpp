#ifndef FILTER_HPP
#define FILTER_HPP

#include <string>
#include <cv3_bridge/cv_bridge>
#include <opencv2/imgproc.h>

using namespace cv;
using std::string;

class Filter
{
    public:
        virtual Filter() = 0;
        virtual ~Filter();

        string getName()
        {
            return this->name;
        }

        virtual void setParams(XmlRpcValue &params) = 0;

    protected:
        string name;

        // Should apply the filter to the specified image
        virtual void apply(Mat &image) = 0;

        // Should apply the filter to the src image and place the result in
        //  the dst image
        virtual void apply(const Mat &src, Mat &dst) = 0;
};

#endif //FILTER_HPP
