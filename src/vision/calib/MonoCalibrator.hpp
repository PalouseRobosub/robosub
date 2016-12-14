#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>
#include <ctime>
#include <iostream>

#include <opencv2/core.hpp>
//#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

enum Mode
{
    DETECTION = 0,
    CAPTURING = 1,
    CALIBRATED = 2
};

/*static std::stringstream & operator<<(std::stringstream & out, Mode &mode)
{
    if (mode == DETECTION)
    {
        out << "DETECTION";
    }
    else if (mode == CAPTURING)
    {
        out << "CAPTURING";
    }
    else
    {
        out << "CALIBRATED";
    }
    return out;
}*/

class Settings
{
    public:
        Settings() : goodInput(false){}
        enum Pattern {NON_EXISTANT, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
        
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

static inline void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if (node.empty())
    {
        x = default_value;
    }
    else
    {
        x.read(node);
    }
}

static inline void write(FileStorage& fs, const Settings& s)
{
    s.write(fs);
}

bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>> imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs, double& totalAvgErr);

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>> imagePoints);

void saveCameraParams(Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const vector<float>& reprojErrs, const vector<vector<Point2f>>& imagePoints, double totalAvgErr);

void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners, Settings::Pattern patternType);

double computeReprojectionErrors( const vector<vector<Point3f>>& objectPoints, const vector<vector<Point2f>>& imagePoints, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const Mat& cameraMatrix, const Mat& distCoeffs, vector<float>& perViewErrors, bool fisheye);

