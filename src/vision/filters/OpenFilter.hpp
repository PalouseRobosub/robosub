#ifndef OPEN_FILTER_HPP
#define OPEN_FILTER_HPP

#include "Filter.hpp"

class OpenFilter : public Filter
{
    public:
        OpenFilter();
        ~OpenFilter() {};

        void setParams(XmlRpcValue &params);
    
    private:
        int iterations;
        Size size;

        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);
};

#endif //OPEN_FILTER_HPP
