#ifndef OR_FILTER_HPP
#define OR_FILTER_HPP

#include "BinaryFilter.hpp"

class OrFilter : public BinaryFilter
{
    public:
        OrFilter();
        ~OrFilter() {};

        void setParams(XmlRpcValue &params);

        void apply(const Mat &src1, const Mat &src2, Mat &dst);
};

#endif //OR_FILTER_HPP
