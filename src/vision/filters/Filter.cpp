#include "Filter.hpp"
#include <map>
#include <string>

///// ------------ Close Filter ---------- ////

CloseFilter::CloseFilter()
{
    this->name = "Close";
    this->size = Size(0, 0);
    this->iterations = 0;
}

void CloseFilter::setParams(XmlRpcValue &params)
{
    int width = params["width"];
    int height = params["height"];

    // Width and height must be greater than 1
    if (width < 3)
    {
        ROS_WARN("Close Filter width param set to below 3. This is invalid, "
                 << "setting to 3");
        width = 3;
    }
    if (height < 3)
    {
        ROS_WARN("Close Filter height param set to below 3. This is invalid, "
                 << "setting to 3");
        height = 3;
    }

    // Width and height must be odd
    if (width % 2 != 1)
    {
        ROS_WARN("Close Filter width param is even. This is invalid, "
                 << "adding one to make it odd.");
        width++;
    }
    if (height % 2 != 1)
    {
        ROS_WARN("Close Filter height param is even. This is invalid, "
                 << "adding one to make it odd.");
        height++;
    }

    this->size = Size(width, height);
    this->iterations = params["iters"];
}

void CloseFilter::apply(Mat &image)
{
    morphologyEx(image, image, MORPH_CLOSE, getStructuringElement(MORPH_RECT,
                 size), Point(-1, -1), iterations);
}

void CloseFilter::apply(const Mat &src, Mat &dst)
{
    morphologyEx(src, dst, MORPH_CLOSE, getStructuringElement(MORPH_RECT,
                 size), Point(-1, -1), iterations);
}

///// ----------- Convert Filter ----------- /////

ConvertFilter::ConvertFilter()
{
    this->name = "Convert";
    this->code = cv::COLOR_BGR2HSV;
}

void ConvertFilter::setParams(XmlRpcValue &params)
{
    string strCode;
    try
    {
        XmlRpc::XmlRpcValue xrv = params["code"];

        if (xrv.getType() != XmlRpc::XmlRpcValue::Type::TypeString)
        {
            ROS_FATAL_STREAM("Convert filter param invalid type");
            ros::shutdown();
            return;
        }

        strCode = static_cast<string>(xrv);
    }
    catch (XmlRpc::XmlRpcException e)
    {
        ROS_FATAL_STREAM("Convert filter params malformed");
        ros::shutdown();
        return;
    }
    auto it = paramList.find(strCode);

    if (it != paramList.end())
    {
        code = it->second;
    }
    else
    {
        ROS_FATAL_STREAM("Unknown convert code for ConvertFilter. "
                         << "Consider adding it in Filter.hpp");
        ros::shutdown();
    }
}

void ConvertFilter::apply(Mat &image)
{
    cvtColor(image, image, code);
}

void ConvertFilter::apply(const Mat &src, Mat &dst)
{
    cvtColor(src, dst, code);
}

///// ------------- In Range Filter ----------- /////

InRangeFilter::InRangeFilter(string &inRangeName)
{
    this->name = "In Range " + inRangeName;
}

void InRangeFilter::setParams(XmlRpcValue &params)
{
    try
    {
        for (auto i = params.begin(); i != params.end(); i++)
        {
            std::map<std::string, double> parameters;
            for (auto it = i->second.begin(); it != i->second.end(); it++)
            {
                double value = 0.0;
                switch(it->second.getType())
                {
                    case XmlRpc::XmlRpcValue::TypeDouble:
                        value = it->second;
                        break;
                    case XmlRpc::XmlRpcValue::TypeInt:
                        value = static_cast<double>(
                                                  static_cast<int>(it->second));
                        break;
                    default:
                        ROS_ERROR_STREAM("Invalid parameter type for " <<
                                         i->first << ": " << it->first);
                        return;
                }

                //Values must be between 0 and 255
                if (value < 0)
                {
                    ROS_WARN_STREAM("InRange " << it->first
                                    << " value should not be below zero."
                                    << " Setting to zero.");
                    value = 0;
                }
                if (value > 255)
                {
                    ROS_WARN_STREAM("InRange " << it->first
                                    << " value should not be above 255."
                                    << " Setting to 255.");
                    value = 255;
                }

                parameters.emplace(it->first, value);
            }

            if (i->first == "min")
            {
                if (parameters.find("hue") == parameters.end() ||
                    parameters.find("sat") == parameters.end() ||
                    parameters.find("val") == parameters.end())
                {
                    ROS_FATAL_STREAM("Min value for " << this->name <<
                                     " is missing hue, saturation or value.");
                }
                lowerBounds[0] = parameters["hue"];
                lowerBounds[1] = parameters["sat"];
                lowerBounds[2] = parameters["val"];
            }
            else if (i->first == "max")
            {
                if (parameters.find("hue") == parameters.end() ||
                    parameters.find("sat") == parameters.end() ||
                    parameters.find("val") == parameters.end())
                {
                    ROS_FATAL_STREAM("Max value for " << this->name <<
                                     " is missing hue, saturation or value.");
                }
                upperBounds[0] = parameters["hue"];
                upperBounds[1] = parameters["sat"];
                upperBounds[2] = parameters["val"];
            }
            else
            {
                ROS_WARN_STREAM("Invalid param name found for in range filter: "
                                << i->first);
            }
        }
    }
    catch (XmlRpc::XmlRpcException e)
    {
       ROS_ERROR_STREAM("XmlRpcException code " << e.getCode() <<
                         ": " << e.getMessage());
       ros::shutdown();
    }
}

void InRangeFilter::apply(Mat &image)
{
    inRange(image, lowerBounds, upperBounds, image);
}

void InRangeFilter::apply(const Mat &src, Mat &dst)
{
    inRange(src, lowerBounds, upperBounds, dst);
}

////// ------------ Median Blur Filter ---------- /////

MedianBlurFilter::MedianBlurFilter()
{
    this->name = "Median Blur";
}

void MedianBlurFilter::setParams(XmlRpcValue &params)
{
    this->kernelSize = params["kernel_size"];

    //Kernel size must be greater than 1
    if (this->kernelSize < 3)
    {
        ROS_WARN("Median Blur kernel_size param is less than 3. This is "
                 << "invalid, setting to 3.");
        this->kernelSize = 3;
    }

    //Kernel size must be odd
    if (this->kernelSize % 2 != 1)
    {
        ROS_WARN("Median Blur kernel_size param is even. This is invalid, "
                 << "adding one to make it odd.");
        this->kernelSize++;
    }
}

void MedianBlurFilter::apply(Mat &image)
{
    medianBlur(image, image, kernelSize);
}

void MedianBlurFilter::apply(const Mat &src, Mat &dst)
{
    medianBlur(src, dst, kernelSize);
}

///// ----------- Open Filter --------- /////

OpenFilter::OpenFilter()
{
    this->name = "Open";
    this->size = Size(0, 0);
    this->iterations = 0;
}

void OpenFilter::setParams(XmlRpcValue &params)
{
    int width = params["width"];
    int height = params["height"];

    // Width and height must be greater than 1
    if (width < 3)
    {
        ROS_WARN("Open Filter width param set to below 3. This is invalid, "
                 << "setting to 3");
        width = 3;
    }
    if (height < 3)
    {
        ROS_WARN("Open Filter height param set to below 3. This is invalid, "
                 << "setting to 3");
        height = 3;
    }

    // Width and height must be odd
    if (width % 2 != 1)
    {
        ROS_WARN("Open Filter width param is even. This is invalid, "
                 << "adding one to make it odd.");
        width++;
    }
    if (height % 2 != 1)
    {
        ROS_WARN("Open Filter height param is even. This is invalid, "
                 << "adding one to make it odd.");
        height++;
    }

    this->size = Size(width, height);
    this->iterations = params["iters"];
}

void OpenFilter::apply(Mat &image)
{
    morphologyEx(image, image, MORPH_OPEN, getStructuringElement(MORPH_RECT,
                 size), Point(-1, -1), iterations);
}

void OpenFilter::apply(const Mat &src, Mat &dst)
{
    morphologyEx(src, dst, MORPH_OPEN, getStructuringElement(MORPH_RECT,
                 size), Point(-1, -1), iterations);
}


/////  ------------- Or Filter ------------- /////

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
