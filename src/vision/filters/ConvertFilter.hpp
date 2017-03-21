#ifndef CONVERT_FILTER_HPP
#define CONVERT_FILTER_HPP

#include "Filter.hpp"

class ConvertFilter : public Filter
{
    public:
        ConvertFilter();
        ~ConvertFilter() {};

        void setParams(XmlRpcValue &params);

        void apply(Mat &image);
        void apply(const Mat &src, Mat &dst);

    private:
        int code;

        const std::map<string, int> paramList {
                                    {"BGR2HSV", cv::COLOR_BGR2HSV},
                                    {"BGR2RGB", cv::COLOR_BGR2RGB},
                                    {"RGB2BGR", cv::COLOR_RGB2BGR},
                                    {"RGB2HSV", cv::COLOR_RGB2HSV},
                                    {"BGR2GRAY", cv::COLOR_BGR2GRAY},
                                    {"RGB2GRAY", cv::COLOR_RGB2GRAY}
                                    };
};

#endif //CONVERT_FILTER_HPP
