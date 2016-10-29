#ifndef SHAREDIMAGEREADER_HPP
#define SHAREDIMAGEREADER_HPP

#include <opencv2/core.hpp>
#include <iostream>
#include "SharedImageHeader.hpp"
#include "ros/ros.h"
#include "ros/console.h"

using namespace std;
using namespace cv;

namespace rs
{
    class SharedImageReader
    {
    private:
        string header_name;
        Ptr<SharedImageHeader> header;

    public:
        SharedImageReader(string name);
        ~SharedImageReader();

        Mat Read();
        void operator>>(Mat image);
    };
};

#endif
