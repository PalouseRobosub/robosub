#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "SharedImageReader.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    rs::SharedImageReader image_reader_left("left");
    rs::SharedImageReader image_reader_right("right");

    image_transport::ImageTransport it(nh);

    image_transport::Publisher pub_left = it.advertise("camera/image_left", 1);
    image_transport::Publisher pub_right = it.advertise("camera/image_right", 1);

    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        cv::Mat l = image_reader_left.Read();
        cv::Mat r = image_reader_right.Read();

        sensor_msgs::ImagePtr l_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", l).toImageMsg();
        sensor_msgs::ImagePtr r_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", r).toImageMsg();

        pub_left.publish(l_msg);
        pub_right.publish(r_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

