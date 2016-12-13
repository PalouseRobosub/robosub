#ifndef SHAREDIMAGEWRITER_HPP
#define SHAREDIMAGEWRITER_HPP

#include <opencv2/core.hpp>
#include <iostream>
#include <string>
#include "SharedImageHeader.hpp"
#include "ros/ros.h"
#include "ros/console.h"

using namespace std;
using namespace cv;

namespace rs
{
class SharedImageWriter
{
private:
    string header_name;
    Ptr<SharedImageHeader> header;

public:
    SharedImageWriter(string name, Mat image);
    ~SharedImageWriter();

    void Write(Mat image);
    void operator<<(Mat image);
};
};

#endif // SHAREDIMAGEWRITER_HPP
