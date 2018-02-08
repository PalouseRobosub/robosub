/*
 * Author: Lance Hoffmann
 * Date: 2018-02-07
 * Usage: Captures images from both the left and right cameras as closely
 *        as possible for use in stereo calibration. Saves images in
 *        current directory.
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sstream>

using namespace sensor_msgs;
using namespace message_filters;

// Counter for image names
int ct = 00;

void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
{
    std::cout << "Capturing images..." << ct << std::endl;

    // Create cvImage pointers for left and right cameras
    cv_bridge::CvImagePtr cv_ptr1;
    cv_bridge::CvImagePtr cv_ptr2;

    // All for consistent naming for images
    std::stringstream ss_left, ss_right;
    std::string cleft = "left_", cright = "right_", type = ".jpg";
    ss_left<<cleft<<(ct)<<type;
    ss_right<<cright<<(ct)<<type;
    std::string fileLeft = ss_left.str();
    std::string fileRight = ss_right.str();
    ss_left.str("");
    ss_right.str("");

    // Converts image messages to cvImages
    try
    {
        cv_ptr1 = cv_bridge::toCvCopy(image1,
                             sensor_msgs::image_encodings::BGR8);
        cv_ptr2 = cv_bridge::toCvCopy(image2,
                             sensor_msgs::image_encodings::BGR8);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Write images to current directory
    cv::imwrite(fileLeft, cv_ptr1->image);
    cv::imwrite(fileRight, cv_ptr2->image);

    ct++;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_capture");

    ros::NodeHandle nh;
    message_filters::Subscriber<Image> image1_sub(nh,
                                        "camera/left/undistorted", 1);
    message_filters::Subscriber<Image> image2_sub(nh,
                                        "camera/right/undistorted", 1);

    // Set up time syncronizer between left and right image messages
    typedef sync_policies::ApproximateTime<Image, Image> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(10), image1_sub, image2_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;

}

