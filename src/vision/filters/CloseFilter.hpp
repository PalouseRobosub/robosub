#ifndef CLOSE_FILTER_HPP
#define CLOSE_FILTER_HPP

#include "Filter.hpp"

class CloseFilter : public Filter
{
    public:
        CloseFilter();
        ~CloseFilter() {};

        void setParams(XmlRpcValue &params);
        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);
    
    private:
        int iterations;
        Size size;
};

#endif //CLOSE_FILTER_HPP
