#ifndef CALIBRATIONSETTINGS_HPP
#define CALIBRATIONSETTINGS_HPP

#include <ros/ros.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>

using namespace cv;
using std::string;
using std::vector;

class Settings
{
    public:
        Settings() : goodInput(false) {}
        enum class Pattern
        {
            NON_EXISTANT,
            CHESSBOARD,
            CIRCLES_GRID,
            ASYMMETRIC_CIRCLES_GRID
        };

        void write(FileStorage& fs) const;
        void read(const FileNode& node);

        void validate();

        static bool readStringList(const string& filename, vector<string>& l);

    //private:
        Size boardSize;
        Pattern calibrationPattern;
        float squareSize;
        int nrFrames;
        float aspectRatio;
        int delay;
        bool writePoints;
        bool writeExtrinsics;
        bool calibZeroTangentDist;
        bool calibFixPrincipalPoint;
        bool flipVertical;
        string outputFileName;
        bool showUndistorted;
        string input;
        bool useFisheye;
        bool fixK1;
        bool fixK2;
        bool fixK3;
        bool fixK4;
        bool fixK5;

        bool goodInput;
        int flag;
    private:
        string patternToUse;
};

void read(const FileNode& node, Settings& x,
          const Settings& default_value = Settings());

void write(FileStorage& fs, const Settings& s);



#endif //CALIBRATIONSETTINGS_HPP
