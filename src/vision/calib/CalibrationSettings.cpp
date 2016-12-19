#include "CalibrationSettings.hpp"
#include <string>
#include <vector>

using std::string;
using std::vector;

void Settings::write(FileStorage& fs) const
{
    fs << "{"
                << "BoardSize_Width"    << boardSize.width
                << "BoardSize_Height"   << boardSize.height
                << "Square_Size"        << squareSize
                << "Calibrate_Pattern"  << patternToUse
                << "Calibrate_NrOfFrameToUse" << nrFrames
                << "Calibrate_FixAspectRatio" << aspectRatio
                << "Calibrate_AssumeZeroTangentialDistortion" <<
                    calibZeroTangentDist
                << "Calibrate_FixPrincipalPointAtTheCenter" <<
                    calibFixPrincipalPoint
                << "Write_DetectedFeaturePoints" << writePoints
                << "Write_extrinsicParameters" << writeExtrinsics
                << "Write_outputFileName" << outputFileName
                << "Show_UndistortedImage" << showUndistorted
                << "Input_FlipAroundHorizontalAxis" << flipVertical
                << "Input_Delay"        << delay
       << "}";
}

void Settings::read(const FileNode& node)
{
    node["BoardSize_Width" ] >> boardSize.width;
    node["BoardSize_Height"] >> boardSize.height;
    node["Calibrate_Pattern"] >> patternToUse;
    node["Square_Size"]  >> squareSize;
    node["Calibrate_NrOfFrameToUse"] >> nrFrames;
    node["Calibrate_FixAspectRatio"] >> aspectRatio;
    node["Write_DetectedFeaturePoints"] >> writePoints;
    node["Write_extrinsicParameters"] >> writeExtrinsics;
    node["Write_outputFileName"] >> outputFileName;
    node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
    node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
    node["Calibrate_UseFisheyeModel"] >> useFisheye;
    node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
    node["Show_UndistortedImage"] >> showUndistorted;
    node["Input_Delay"] >> delay;
    node["Fix_K1"] >> fixK1;
    node["Fix_K2"] >> fixK2;
    node["Fix_K3"] >> fixK3;
    node["Fix_K4"] >> fixK4;
    node["Fix_K5"] >> fixK5;

    validate();
}

void Settings::validate()
{
    goodInput = true;
    if (boardSize.width <= 0 || boardSize.height <= 0)
    {
        ROS_FATAL_STREAM("Invalid Board size: " << boardSize.width << " " <<
                         boardSize.height);
        goodInput = false;
    }

    if (squareSize <= 10e-6)
    {
        ROS_FATAL_STREAM("Invalid square size " << squareSize);
        goodInput = false;
    }

    if (nrFrames <= 0)
    {
        ROS_FATAL_STREAM("Invalid number of frames " << nrFrames);
        goodInput = false;
    }

    flag = 0;
    if (calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
    if (calibZeroTangentDist) flag |= CALIB_ZERO_TANGENT_DIST;
    if (aspectRatio) flag |= CALIB_FIX_ASPECT_RATIO;
    if (fixK1) flag |= CALIB_FIX_K1;
    if (fixK2) flag |= CALIB_FIX_K2;
    if (fixK3) flag |= CALIB_FIX_K3;
    if (fixK4) flag |= CALIB_FIX_K4;
    if (fixK5) flag |= CALIB_FIX_K5;

    if (useFisheye)
    {
        flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        if (fixK1) flag |= fisheye::CALIB_FIX_K1;
        if (fixK2) flag |= fisheye::CALIB_FIX_K2;
        if (fixK3) flag |= fisheye::CALIB_FIX_K3;
        if (fixK4) flag |= fisheye::CALIB_FIX_K4;
    }

    calibrationPattern = Pattern::NON_EXISTANT;
    if (!patternToUse.compare("CHESSBOARD"))
    {
        calibrationPattern = Pattern::CHESSBOARD;
    }
    if (!patternToUse.compare("CIRCLES_GRID"))
    {
        calibrationPattern = Pattern::CIRCLES_GRID;
    }

    if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID"))
    {
        calibrationPattern = Pattern::ASYMMETRIC_CIRCLES_GRID;
    }

    if (calibrationPattern == Pattern::NON_EXISTANT)
    {
        ROS_FATAL_STREAM("Camera calibration mode does not exist: " <<
                         patternToUse);
        goodInput = false;
    }

    if (!goodInput)
    {
        exit(-1);
    }
}


bool Settings::readStringList(const string& filename, vector<string>& l)
{
    l.clear();
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
    {
        return false;
    }

    FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != FileNode::SEQ)
    {
        return false;
    }

    for (FileNodeIterator it = n.begin(); it != n.end(); it++)
    {
        l.push_back((string)*it);
    }

    return true;
}
