#include "VisionProcessor.hpp"

VisionProcessor::VisionProcessor()
{
    
}

VisionProcessor::~VisionProcessor()
{

}


cv::Mat VisionProcessor::process(sensor_msgs::Image& image)
{
    cv::Mat toProcess = toOpenCV(image);//manConvert(image);//toOpenCV(image);
    
    std::cout << "Processed" << std::endl;

    cv::Mat hsv;
    cv::cvtColor(toProcess, hsv, cv::COLOR_BGR2HSV);

    std::cout << "Converted to hsv" << std::endl;

    cv::Scalar lower_bound(0, 10, 10);
    cv::Scalar upper_bound(10,255,255);
    
    std::cout << "Masking" << std::endl;
    cv::Mat mask;
    cv::inRange(hsv, lower_bound, upper_bound, mask);

    std::cout << "Masked" << std::endl;
    return mask;
}

cv::Mat VisionProcessor::manConvert(const sensor_msgs::Image& source)
{
    std::cout << "Manually converting" << std::endl;
    std::vector<unsigned char> imData = source.data;
    std::cout << "imData: " << imData.size() << std::endl;

    cv::Mat jpegData((uint32_t)source.height, (uint32_t)source.width, CV_8UC1, &imData, (uint32_t)source.step);
   
    std::cout << "Created Mat: " << jpegData.checkVector(1) << std::endl;
    std::cout << "\tsize: " << jpegData.size() << std::endl;
    std::cout << "\tchannels: " << jpegData.channels() << std::endl;
    //std::cout << jpegData << std::endl;
    
    //for (int i = 9500; i < 10000; i++)
    //{
        //std::cout << "At " << i << ": ";
        //std::cout << imData[i] << std::endl;
    //}

    //for (int i = 0; i < jpegData.rows; i++)
    //{
        //std::cout << "Mat row " << i << ": " << std::endl;
        //for (int j = 0; j < jpegData.cols; j++)
        //{
            //std::cout << "j: " << j;
            //std::cout << jpegData.at<uchar>(i,j) << std::endl;
        //}
        //std::cout << std::endl;
    //}

    std::cout << "Converting color" << std::endl;
    
    cv::Mat hsvMat;
    cv::cvtColor(hsvMat, hsvMat, cv::COLOR_BGR2HSV);
    std::cout << "BGR Mat created and returning" << std::endl;
    return hsvMat;
}

///////Private functions///////

cv::Mat VisionProcessor::toOpenCV(sensor_msgs::Image& image)
{
    std::cout << "Image size: " << sizeof(image.data) << std::endl;
    if (sizeof(image.data) == 0)
    {
        std::cout << "Empty image" << std::endl;
        return cv::Mat();
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        std::cout << "Copying..." << std::endl;
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        exit(1);
    }
    
    std::cout << "Returning Mat" << std::endl;
    return cv_ptr->image;
}
