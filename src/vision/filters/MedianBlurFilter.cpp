#include "MedianBlurFilter.hpp"

MedianBlurFilter::MedianBlurFilter()
{
    this->name = "Median Blur";
}

void MedianBlurFilter::setParams(XmlRpcValue &params)
{
    this->kernelSize = params["kernel_size"];
}

void MedianBlurFilter::apply(Mat &image)
{
    medianBlur(image, image, kernelSize);
}

void MedianBlurFilter::apply(const Mat &src, Mat &dst)
{
    medianBlur(src, dst, kernelSize);
}
