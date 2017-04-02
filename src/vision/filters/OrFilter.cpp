#include "Filter.hpp"
#include <string>

OrFilter::OrFilter()
{
    this->name = "Bitwise Or";
}

void OrFilter::setParams(XmlRpcValue &params)
{
    //An or filter contains two children, set the child filter's params
    try
    {
        int inRangeCount = 1;
        for (int i = 0; i < params.size(); i++)
        {
            Filter *f;
            string memberName;
            if (params[i].hasMember("Close"))
            {
                f = new CloseFilter();
                memberName = "Close";
            }
            else if (params[i].hasMember("Convert"))
            {
                f = new ConvertFilter();
                memberName = "Convert";
            }
            else if (params[i].hasMember("InRange"))
            {
                inRangeCount++;
                string name = std::to_string(inRangeCount);
                f = new InRangeFilter(name);
                memberName = "InRange";
            }
            else if (params[i].hasMember("MedianBlur"))
            {
                f = new MedianBlurFilter();
                memberName = "MedianBlur";
            }
            else if (params[i].hasMember("Open"))
            {
                f = new OpenFilter();
                memberName = "Open";
            }
            else if (params[i].hasMember("Or"))
            {
                f = new OrFilter();
                memberName = "Or";
            }
            else
            {
            ROS_ERROR_STREAM("Invalid Filter: " << params[i] << ". Skipping." <<
                             " Consider" <<
                             " adding it to FilterSet");
                continue;
            }

            f->setParams(params[i][memberName]);
            children.push_back(f);
        }
    }
    catch(XmlRpc::XmlRpcException e)
    {
        ROS_FATAL_STREAM("Or filter params malformed");
        ROS_FATAL_STREAM("" << ros::this_node::getName() << " threw XmlRpc"
                         << " exception " << e.getCode() << ": "
                         << e.getMessage());
        ros::shutdown();
        return;
    }
}

void OrFilter::apply(Mat &image)
{
    apply(image, image);
}

void OrFilter::apply(const Mat &src, Mat &dst)
{
    Mat result;

    children[0]->apply(src, result);

    for (auto it = children.begin() + 1; it != children.end(); it++)
    {
        Mat temp;

        (*it)->apply(src, temp);

        bitwise_or(result, temp, dst);
    }
}
