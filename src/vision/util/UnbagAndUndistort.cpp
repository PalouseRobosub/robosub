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

Mat rectifyMap[2][2];

int stereoCropRadius = -1;


int main (int argc, char** argv)
{
    if(argc != 3){
        std::cout << "usage: " << argv[0] << " sourceDir/ destinationDir/\n";
    }

    else {


        ros::init(argc, argv, "undistMono");

        ROS_INFO_STREAM("Init done");

        ros::NodeHandle n;
        image_transport::ImageTransport it(n);

        string stereoCalibFile = "";
        n.getParam("stereo_calib_file", stereoCalibFile);

        ROS_INFO_STREAM("Fetched params");
        ROS_INFO_STREAM("Reading stereo calibration file: " << stereoCalibFile);

        FileStorage fs(stereoCalibFile, FileStorage::READ);

        Mat K1, K2, D1, D2, R1, R2, P1, P2;
        Size imageSize;

        if (!fs.isOpened())
        {
            ROS_FATAL_STREAM("Stereo calibration file could not be opened from: " <<
                    stereoCalibFile);
            exit(1);
        }
        else
        {
            ROS_INFO_STREAM("Valid calibration file");
            string calibTime;
            fs["calibration_time"] >> calibTime;
            fs["crop_radius"] >> stereoCropRadius;
            ROS_INFO_STREAM("Calibration was performed on: " << calibTime);
            fs["K1"] >> K1;
            fs["D1"] >> D1;
            fs["K2"] >> K2;
            fs["D2"] >> D2;
            fs["R1"] >> R1;
            fs["R2"] >> R2;
            fs["P1"] >> P1;
            fs["P2"] >> P2;
            fs["image_size"] >> imageSize;
        }

        ROS_INFO_STREAM("Stereo calibration file read");

        ROS_INFO_STREAM("Initializing stereo undistortion maps");

        fisheye::initUndistortRectifyMap(K1, D1, R1, P1,
                imageSize, CV_16SC2, rectifyMap[0][0],
                rectifyMap[0][1]);

        fisheye::initUndistortRectifyMap(K2, D2, R2, P2,
                imageSize, CV_16SC2, rectifyMap[1][0],
                rectifyMap[1][1]);

        ROS_INFO_STREAM("Completed initialization");

        // paths
        const string bagPath;
        const string output_dir;
        
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
        
        if(!mkdir(output_dir.c_str(), 0755))
        {
            ROS_ERROR_STREAM("Failed to create path: " << output_dir);

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

        for(rosbag::MessageInstance const m : view)
        {
            Image::ConstPtr s = m.instantiate<Image>();

            if (s != NULL)
            {
                // do convert stuff
                cv_bridge::CvImagePtr image_ptr;
                image_ptr = toCvCopy(s, sensor_msgs::image_encodings::BGR8);

                Mat temp = image_ptr->image.clone();
                Mat cropped;

                if (stereoCropRadius != -1)
                {
                    Mat cropMask = Mat::zeros(temp.rows, temp.cols, CV_8UC1);
                    circle(cropMask, Point(temp.size().width / 2,
                                temp.size().height / 2), stereoCropRadius,
                            Scalar(255, 255, 255), -1, 8, 0);

                    temp.copyTo(cropped, cropMask);
                }
                else
                {
                    temp.copyTo(cropped);
                }

                remap(temp, image_ptr->image, rectifyMap[0][0], rectifyMap[0][1],
                        INTER_LINEAR);

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
        }

        bag.close();
    }
}
