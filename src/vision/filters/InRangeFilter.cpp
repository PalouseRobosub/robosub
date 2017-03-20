#include "InRangeFilter.hpp"

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
                        value = static_cast<double>(static_cast<int>(it->second));
                        break;
                    default:
                        ROS_ERROR_STREAM("Invalid parameter type for " <<
                                         i->first << ": " << it->first);
                        return;
                        break;
                }
            
                //Values must be between 0 and 255
                if (value < 0)
                {
                    value = 0;
                }
                if (value > 255)
                {
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
