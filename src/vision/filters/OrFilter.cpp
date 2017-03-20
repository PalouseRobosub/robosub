#include "OrFilter.hpp"

OrFilter::OrFilter()
{
    this->name = "Bitwise Or";
}

void OrFilter::setParams(XmlRpcValue &params)
{
   ROS_WARN_STREAM("Bitwise Or does not take any parameters, "
                   << "they do not need to be set"); 
}

void OrFilter::apply(const Mat &src1, const Mat &src2, Mat &dst)
{
    bitwise_or(src1, src2, dst);
}
