/**
 * @author Ryan Summers
 * @date 1-29-2018
 *
 * @brief Applies fisheye undistortion to a warped image to rectify it.
 */
#include <image_transport/image_transport.h>
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <XmlRpcException.h>

#if ROS_VERSION_MINIMUM(1, 12, 0)  // Running Kinetic
    #include <cv_bridge/cv_bridge.h>
#else  // Running Indigo
    #include <cv3_bridge/cv_bridge.h>
#endif

using namespace cv_bridge;
using namespace cv;
using std::string;

/**
 * The OpenCV camera matrix.
 */
Mat camera_matrix(Size(3, 3), CV_64FC1);

/**
 * The OpenCV distortion coefficients.
 */
Mat distortion_coefficients(Size(1, 4), CV_64FC1);

/**
 * The rectification maps used for remapping.
 */
Mat rectification_map[2];

/**
 * The undistorted image publisher.
 */
image_transport::Publisher undistorted_publisher;

/**
 * ROS image callback.
 *
 * @param msg A pointer to the image message.
 *
 * @return None.
 */
void image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    /*
     * Convert the Image message into an OpenCV image.
     */
    cv_bridge::CvImagePtr image_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    /*
     * Undistort the image by using the remap matrices.
     */
    Mat distorted = image_ptr->image.clone();
    remap(distorted, image_ptr->image, rectification_map[0],
            rectification_map[1], INTER_LINEAR);

    /*
     * Publish the undistorted image.
     */
    sensor_msgs::Image out_image;
    image_ptr->toImageMsg(out_image);
    out_image.header = msg->header;

    undistorted_publisher.publish(out_image);
}

/**
 * Main entry point.
 *
 * @return Exit status.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "undistortion");

    ros::NodeHandle n("~");
    ros::NodeHandle n_public;

    /*
     * Load the camera matrix coefficients.
     */
    XmlRpc::XmlRpcValue param;
    if (n.getParam("camera_matrix", param) == false)
    {
        ROS_FATAL_STREAM("Failed to load camera matrix.");
        exit(-1);
    }

    if (param.size() != 3)
    {
        ROS_FATAL_STREAM("Camera matrix rows dimensions invalid.");
        exit(-1);
    }

    for (int i = 0; i < param.size(); ++i)
    {
        XmlRpc::XmlRpcValue row = param[i];

        if (row.size() != 3)
        {
            ROS_FATAL_STREAM("Camera matrix columns dimension invalid.");
        }

        for (int j = 0; j < row.size(); ++j)
        {
            camera_matrix.at<double>(i, j) = static_cast<double>(row[j]);
        }
    }

    /*
     * Load the distortion coefficients.
     */
    if (n.getParam("distortion_coefficients", param) == false)
    {
        ROS_FATAL_STREAM("Failed to load camera matrix.");
        exit(-1);
    }

    if (param.size() != 4)
    {
        ROS_FATAL_STREAM("Distortion coefficients dimensions invalid.");
        exit(-1);
    }

    for (int i = 0; i < param.size(); ++i)
    {
        distortion_coefficients.at<double>(0, i) =
            static_cast<double>(param[i]);
    }

    /*
     * Load the camera parameter.
     */
    string camera;
    if (n.getParam("camera", camera) == false)
    {
        ROS_FATAL_STREAM("Failed to get camera parameter");
        exit(-1);
    }

    /*
     * Load the image size.
     */
    int rows, cols;
    if (n.getParam("size/rows", rows) == false ||
        n.getParam("size/cols", cols) == false)
    {
        ROS_FATAL_STREAM("Failed to load dimensions of image.");
        exit(-1);
    }

    /*
     * Set up image publisher and subscriber.
     */
    image_transport::ImageTransport it(n_public);

    image_transport::Subscriber sub = it.subscribe(
            "camera/" + camera + "/image_raw", 1, image_callback);

    undistorted_publisher = it.advertise("camera/" + camera + "/undistorted",
            1);

    /*
     * Calculate the rectification matrices for unmapping fisheye distortion.
     */
    fisheye::initUndistortRectifyMap(camera_matrix,
            distortion_coefficients, Mat::eye(3, 3, CV_32FC1),
            camera_matrix, Size(rows, cols), CV_16SC2,
            rectification_map[0], rectification_map[1]);

    ros::spin();
}
