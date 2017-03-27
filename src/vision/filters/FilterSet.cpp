#include "FilterSet.hpp"
#include <boost/algorithm/string.hpp>
#include <string>

FilterSet::FilterSet()
{
    //Set default name
    this->name = "Default filter name";
    this->doImShow = false;
}

FilterSet::~FilterSet()
{
    // Delete all filters in the vector
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
    // Reset params and filters
    this->paramSet = params;
    filters.clear();

    // In Range count to separate InRange filters
    int inRangeCount = 0;
    try
    {
        // Iterate over every parameter in the set
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

            // Set the params on the filter using this set's parameters
            f->setParams(params[i][memberName]);
            // Add the filter to the vector
            filters.push_back(f);
        }
    }
    catch(XmlRpc::XmlRpcException e)
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
    Mat tempImg, previousImg;
    // Copy the source image to the previous image
    src.copyTo(previousImg);

    // Apply each filter in sequence
    for (auto it = filters.begin(); it != filters.end(); it++)
    {
        // Apply to the previoius image and place the result in tempImg
        (*it)->apply(previousImg, tempImg);

        // Show the individual filter results
        if (doImShow)
        {
            imshow("" + ros::this_node::getName() + " " + (*it)->getName(),
                   tempImg);
        }

        // Set the previous image to this filter's result.
        tempImg.copyTo(previousImg);
    }

    // Show the final output
    if (doImShow)
    {
        imshow("" + ros::this_node::getName() + " Filter Output", tempImg);
    }

    // Copy the final image to the destination Mat
    tempImg.copyTo(dst);
}
