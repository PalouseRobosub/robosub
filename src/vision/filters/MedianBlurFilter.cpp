#include "MedianBlurFilter.hpp"

MedianBlurFilter::MedianBlurFilter()
{
    this->name = "Median Blur";
}

void MedianBlurFilter::setParams(XmlRpcValue &params)
{
    this->kernelSize = params["kernel_size"];

    //Kernel size must be greater than 1
    if (this->kernelSize < 3)
    {
        this->kernelSize = 3;
    }

    //Kernel size must be odd
    if (this->kernelSize % 2 != 1)
    {
        this->kernelSize++;
    }
}

void MedianBlurFilter::apply(Mat &image)
{
    medianBlur(image, image, kernelSize);
}

void MedianBlurFilter::apply(const Mat &src, Mat &dst)
{
    medianBlur(src, dst, kernelSize);
}
