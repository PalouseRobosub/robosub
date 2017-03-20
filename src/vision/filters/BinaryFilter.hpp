#ifndef BINARY_FILTER_HPP
#define BINARY_FILTER_HPP

#include "Filter.hpp"

class BinaryFilter : public Filter
{
    public:
        // Applies a binary filter to two src images and outputs it to the dst
        //  image.
        virtual void apply(const Mat &src1, const Mat &src2, Mat &dst) = 0;

        // Override unnecessary functions from Filter
        void apply(Mat &image) {};
        void apply(const Mat &src, Mat &dst) {}; 
};

#endif //BINARY_FILTER_HPP
