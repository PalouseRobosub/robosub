#include "Filter.hpp"

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
        ROS_FATAL_STREAM("" << ros::this_node::getName() << " threw XmlRpc"
                         << " exception " << e.getCode() << ": "
                         << e.getMessage());
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
