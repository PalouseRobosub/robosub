#include "Filter.hpp"

CloseFilter::CloseFilter()
{
    this->name = "Close";
    this->size = Size(0, 0);
    this->iterations = 0;
}

void CloseFilter::setParams(XmlRpcValue &params)
{
    int width = params["width"];
    int height = params["height"];

    // Width and height must be greater than 1
    if (width < 3)
    {
        ROS_WARN_STREAM("Close Filter width param set to below 3. "
                        << "This is invalid, "
                        << "setting to 3");
        width = 3;
    }
    if (height < 3)
    {
        ROS_WARN_STREAM("Close Filter height param set to below 3. "
                        << "This is invalid, "
                        << "setting to 3");
        height = 3;
    }

    // Width and height must be odd
    if (width % 2 != 1)
    {
        ROS_WARN_STREAM("Close Filter width param is even. This is invalid, "
                        << "adding one to make it odd.");
        width++;
    }
    if (height % 2 != 1)
    {
        ROS_WARN_STREAM("Close Filter height param is even. This is invalid, "
                        << "adding one to make it odd.");
        height++;
    }

    this->size = Size(width, height);
    this->iterations = params["iters"];
}

void CloseFilter::apply(Mat &image)
{
    morphologyEx(image, image, MORPH_CLOSE, getStructuringElement(MORPH_RECT,
                 size), Point(-1, -1), iterations);
}

void CloseFilter::apply(const Mat &src, Mat &dst)
{
    morphologyEx(src, dst, MORPH_CLOSE, getStructuringElement(MORPH_RECT,
                 size), Point(-1, -1), iterations);
}
