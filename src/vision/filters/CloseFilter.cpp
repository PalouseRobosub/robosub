#include "CloseFilter.hpp"

CloseFilter::CloseFilter()
{
    this->size = Size(0,0);
    this->iterations = 0;
}

void CloseFilter::setParams(XmlRpcValue &params)
{
    int width = params["width"];
    int height = params["height"];

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

void CloseFilter::apply(Mat &image)
{
    morphologyEx(image, image, MORPH_CLOSE, getStructuringElement(MORPH_RECT,
                 size, Point(-1, -1), iterations);
}

void CloseFilter::apply(const Mat &src, Mat &dst)
{
    morphologyEx(src, dst, MORPH_CLOSE, getStructuringElement(MORPH_RECT,
                 size, Point(-1, -1), iterations);  
}
