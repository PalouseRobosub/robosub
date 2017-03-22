#include "FilterSet.hpp"
#include <boost/algorithm/string.hpp>

FilterSet::FilterSet()
{
    //Set default name
    this->name = "Default filter name";
    this->doImShow = false;
}

FilterSet::~FilterSet()
{
    for (auto it = filters.begin(); it != filters.end(); it++)
    {
        if (*it)
        {
            delete(*it);
        }
    }
}

void FilterSet::setImShow(const bool &doImShow)
{
    this->doImShow = doImShow;
}

void FilterSet::setName(const string &name)
{
    this->name = name;
}

void FilterSet::setParams(XmlRpcValue &params)
{
    this->paramSet = params;
    filters.clear();

    int inRangeCount = 0;
    try
    {
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
            filters.push_back(f);
        }
    } catch(XmlRpc::XmlRpcException e)
    {
        ROS_FATAL_STREAM("" << ros::this_node::getName() << " threw XmlRpc"
                         << " exception " << e.getCode() << ": "
                         << e.getMessage());
        ros::shutdown();
        return;
    }
}

void FilterSet::apply(Mat &image)
{
    apply(image, image);
}

void FilterSet::apply(Mat &src, Mat &dst)
{
    Mat tempImg;
    Mat previousImg, secondPreviousImg;
    src.copyTo(previousImg);
    for (auto it = filters.begin(); it != filters.end(); it++)
    {
        if (dynamic_cast<BinaryFilter *>(*it) == nullptr)
        {
            //Not binary filter, do not need multiple images
            (*it)->apply(previousImg, tempImg);
        }
        else
        {
            //Binary filter, need multiple images
            if (secondPreviousImg.empty())
            {
                ROS_FATAL_STREAM("Invalid filter ordering, a binary filter " <<
                                " cannot be the first or second filter.");
                ros::shutdown();
                return;
            }

            dynamic_cast<BinaryFilter *>(*it)->apply(previousImg,
                                                     secondPreviousImg,
                                                     tempImg);
        }

        if (doImShow)
        {
            imshow("" + ros::this_node::getName() + " " + (*it)->getName(),
                   tempImg);
        }

        previousImg.copyTo(secondPreviousImg);
        tempImg.copyTo(previousImg);
    }

    if (doImShow)
    {
        imshow("" + ros::this_node::getName() + " Filter Output", tempImg);
    }

    tempImg.copyTo(dst);
}
