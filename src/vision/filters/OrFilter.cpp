#include "OrFilter.hpp"

OrFilter::OrFilter()
{
    this->name = "Bitwise Or";
}

void OrFilter::setParams(XmlRpcValue &params)
{
    if (params.begin() != params.end())
    {
        //Warn user if they are attempting to use parameters for this filter.
        ROS_WARN_STREAM("Bitwise Or does not take any parameters, "
                        << "they do not need to be set.");
    }
}

void OrFilter::apply(const Mat &src1, const Mat &src2, Mat &dst)
{
    bitwise_or(src1, src2, dst);
}
