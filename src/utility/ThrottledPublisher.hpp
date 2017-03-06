#ifndef THROTTLEDPUBLISHER_HPP
#define THROTTLEDPUBLISHER_HPP

#include <ros/ros.h>
#include <string>

namespace rs
{
template <class msg> class ThrottledPublisher
{
    private:
        ros::Time nextPubTime;
        ros::Duration rate;
        ros::Publisher pub;

        std::string paramName;
        double paramHz;
        double lastParamHz;

    public:
        ThrottledPublisher() {}

        ThrottledPublisher(std::string topicName, int queueSize, float hz,
                std::string _paramName = "")
        {
            ros::NodeHandle nh;
            pub = nh.advertise<msg>(topicName, queueSize);
            setRate(hz);

            paramName = _paramName;
            getRateParam(paramName);
        }

        ~ThrottledPublisher()
        {}

        void publish(msg message)
        {
            getRateParam(paramName);

            if (nextPubTime <= ros::Time::now())
            {
                pub.publish(message);
                nextPubTime = ros::Time::now() + rate;
            }
        }

        void getRateParam(std::string paramName)
        {
            if(paramName != "")
            {
                if (!ros::param::getCached(paramName, paramHz))
                {
                    ROS_ERROR_STREAM("Failed to load param %s." << paramName);
                    return;
                }

                if (paramHz != lastParamHz)
                {
                    ROS_INFO_STREAM(paramName << " parameter changed!");
                    setRate(paramHz);
                    lastParamHz = paramHz;
                }
            }
        }

        void setRate(float hz)
        {
            if(hz != 0.0)
            {
                this->rate = ros::Duration(1.0/hz);
            }
            else
            {
                this->rate = ros::Duration(0.0);
            }
            nextPubTime = ros::Time::now();
        }
};
};

#endif // THROTTLEDPUBLISHER_HPP
