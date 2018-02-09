#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
// The following #if is to use the correct version of the cv_bridge
// Kinetic by default uses OpenCV3 so we don't need the custom build
#if ROS_VERSION_MINIMUM(1, 12, 0)  // Running Kinetic
    #include <cv_bridge/cv_bridge.h>
#else  // Running Indigo
    #include <cv3_bridge/cv_bridge.h>
#endif
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d.hpp>
#include <string>

#include <opencv2/opencv.hpp>

#include <sys/stat.h>
#include <sys/types.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>


using namespace cv_bridge;
using namespace cv;
using std::string;
using sensor_msgs::Image;

void image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    //TODO complete and adjust for main.
    // save as a png output
    ros::Time t = m.getTime();

    double secs = t.toSec();
    string topic_name = m.getTopic();

    int start = topic_name.find("/", 1);
    int end = topic_name.find("/", start+1);

    int len = end - start - 1;

    string prefix = topic_name.substr(start+1, len);

    string message_name = std::to_string(secs);

    string output_file = output_dir + "/" + prefix + "_" + message_name + ".png";
    ROS_DEBUG_STREAM("writing to: " << output_file);
    imwrite(output_file, image_ptr->image);

}


int main (int argc, char** argv)
{
    if(argc != 3){
        std::cout << "usage: " << argv[0] << " sourceDir/ destinationDir/\n";
    }

    else {


        ros::init(argc, argv, "undistMono");

        ros::NodeHandle n("~");
        ros::NodeHandle n_public;

        ROS_INFO_STREAM("Init done");
        // paths
        string bagPath;
        string output_dir;

        if (!ros::param::get("~bagPath", bagPath))
        {
            ROS_FATAL("could not load \"~bagPath\" parameter");
            return 1;
        }


        if (!ros::param::get("~output_dir", output_dir))
        {
            ROS_FATAL("could not load \"~output_dir\" parameter");
            return 1;
        }

        // mkdir returns a 0 on success and -1 on faliure

        if(mkdir(output_dir.c_str(), 0755))
        {
            int error = errno;
            ROS_ERROR_STREAM("Failed to create path: " << output_dir <<
                    "\nreason: " << strerror(error));

            exit(1);
        }

        ROS_INFO_STREAM("Reading: " << bagPath);
        ROS_INFO_STREAM("Saving in: " << output_dir);


        // load up the ros bag
        rosbag::Bag bag;
        bag.open(bagPath, rosbag::bagmode::Read);

        std::vector<string> topics;
        topics.push_back("/camera/left/image_raw");
        topics.push_back("/camera/right/image_raw");

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        image_transport::ImageTransport it(n_public);

        undistorted_publisher = it.advertise("camera/left/undistorted",
                1);
        undistorted_publisher = it.advertise("camera/right/undistorted",
                1);

        image_transport::Subscriber sub = it.subscribe(
                "camera/left/image_raw", 1, image_callback);
        image_transport::Subscriber sub = it.subscribe(
                "camera/right/image_raw", 1, image_callback)

        for(rosbag::MessageInstance const m : view)
        {

            if (m.instantiate<Image>() != NULL)
            {
                //TODO To complete requires to publish images to both cameras.
            }
        }

        bag.close();
    }
}
