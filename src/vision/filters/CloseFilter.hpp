#ifndef CLOSE_FILTER_HPP
#define CLOSE_FILTER_HPP

#include "Filter.hpp"

class CloseFilter : public Filter
{
    public:
        CloseFilter();
        ~CloseFilter() {};

        void setParams(XmlRpcValue &params);
    
    private:
        int iterations;
        Size size;

        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);
};

#endif //CLOSE_FILTER_HPP
