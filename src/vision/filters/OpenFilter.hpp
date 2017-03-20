#ifndef OPEN_FILTER_HPP
#define OPEN_FILTER_HPP

#include "Filter.hpp"

class OpenFilter : public Filter
{
    public:
        OpenFilter();
        ~OpenFilter() {};

        void setParams(XmlRpcValue &params);
        
        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);
    
    private:
        int iterations;
        Size size;
};

#endif //OPEN_FILTER_HPP
