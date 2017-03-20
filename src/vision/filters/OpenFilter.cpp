#include "OpenFilter.hpp"

OpenFilter::OpenFilter()
{
    this->size = Size(0,0);
    this->iterations = 0;
}

void OpenFilter::setParams(XmlRpcValue &params)
{
    int width = params["width"];
    int height = params["height"];

    // Width and height must be greater than 1
    if (width < 3)
    {
        width = 3;
    }
    if (height < 3)
    {
        height = 3;
    }

    // Width and height must be odd
    if (width % 2 != 1)
    {
        width++;
    }
    if (height % 2 != 1)
    {
        height++;
    }

    this->size = Size(width, height);
    this->iterations = params["iters"];
}

void OpenFilter::apply(Mat &image)
{
    morphologyEx(image, image, MORPH_OPEN, getStructuringElement(MORPH_RECT,
                 size, Point(-1, -1), iterations);
}

void OpenFilter::apply(const Mat &src, Mat &dst)
{
    morphologyEx(src, dst, MORPH_OPEN, getStructuringElement(MORPH_RECT,
                 size, Point(-1, -1), iterations);  
}
