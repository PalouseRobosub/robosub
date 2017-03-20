#ifndef IN_RANGE_FILTER_HPP
#define IN_RANGE_FILTER_HPP

#include "Filter.hpp"

class InRangeFilter : public Filter
{
    public:
        InRangeFilter(string &inRangeName);
        ~InRangeFilter() {};

        void setParams(XmlRpcValue &params);
    
        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);
    
    private:
        Scalar upperBounds;
        Scalar lowerBounds;
};

#endif //IN_RANGE_FILTER_HPP
