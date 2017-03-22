#ifndef FILTER_SET_HPP
#define FILTER_SET_HPP

#include <string>
#include <vector>
#include <map>

#include <opencv2/highgui.hpp>

#include "CloseFilter.hpp"
#include "ConvertFilter.hpp"
#include "InRangeFilter.hpp"
#include "MedianBlurFilter.hpp"
#include "OpenFilter.hpp"
#include "OrFilter.hpp"

using std::string;
using std::vector;
using std::map;
using cv::imshow;

class FilterSet
{
    public:
        FilterSet();
        ~FilterSet();

        void setImShow(const bool &doImShow);
        void setName(const string& name);
        void setParams(XmlRpcValue &params);

        void apply(Mat &image);
        void apply(Mat &src, Mat &dst);

    private:
        bool doImShow;
        string name;
        XmlRpcValue paramSet;
        vector<Filter *> filters;
};

#endif //FILTER_SET_HPP
