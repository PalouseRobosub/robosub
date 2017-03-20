#ifndef MEDIAN_BLUR_HPP
#define MEDIAN_BLUR_HPP

#include "Filter.hpp"

class MedianBlurFilter : public Filter
{
    public:
        MedianBlurFilter();
        ~MedianBlurFilter() {};

        void setParams(XmlRpcValue &params);
        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);

    private:
        int kernelSize;
};

#endif //MEDIAN_BLUR_HPP
