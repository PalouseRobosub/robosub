#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
// The following #if is to use the correct version of the cv_bridge
// Kinetic by default uses OpenCV3 so we don't need the custom build
#if ROS_VERSION_MINIMUM(1, 12, 0)  // Running Kinetic
    #include <cv_bridge/cv_bridge.h>
#else  // Running Indigo
    #include <cv3_bridge/cv_bridge.h>
#endif
#include "SharedImageReader.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    rs::SharedImageReader image_reader_left("left");
    rs::SharedImageReader image_reader_right("right");

    image_transport::ImageTransport it(nh);

    image_transport::Publisher pub_left = it.advertise("camera/left/image", 1);
    image_transport::Publisher pub_right =
                                         it.advertise("camera/right/image", 1);

    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        cv::Mat l = image_reader_left.Read();
        cv::Mat r = image_reader_right.Read();

        sensor_msgs::ImagePtr l_msg =
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", l).toImageMsg();
        sensor_msgs::ImagePtr r_msg =
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", r).toImageMsg();

        pub_left.publish(l_msg);
        pub_right.publish(r_msg);

        // This is ugly but it works
        delete [] l.data;
        delete [] r.data;

        ros::spinOnce();
        loop_rate.sleep();
    }
}
