#include "MonoCalibrator.hpp"

void Settings::write(FileStorage& fs) const
{
    fs << "{"
                << "BoardSize_Width"    << boardSize.width
                << "BoardSize_Height"   << boardSize.height
                << "Square_Size"        << squareSize
                << "Calibrate_Pattern"  << patternToUse
                << "Calibrate_NrOfFrameToUse" << nrFrames
                << "Calibrate_FixAspectRatio" << aspectRatio
                << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint
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
        ROS_FATAL_STREAM("Invalid Board size: " << boardSize.width << " " << boardSize.height);
        goodInput = false;
    }

    if (squareSize <= 10e-6)
    {
        ROS_ERROR_STREAM("Invalid square size " << squareSize);
        goodInput = false;
    }

    if (nrFrames <= 0)
    {
        ROS_ERROR_STREAM("Invalid number of frames " << nrFrames);
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

    calibrationPattern = NON_EXISTANT;
    if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
    if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
    if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;

    if (calibrationPattern == NON_EXISTANT)
    {
        ROS_ERROR_STREAM("Camera calibration mode does not exist: " << patternToUse);
        goodInput = false;
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

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>> imagePoints)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs, totalAvgErr);

    ROS_INFO_STREAM((ok ? "Calibration succeeded" : "Calibration failed") << ". avg re-projection errror = " << totalAvgErr);

    if (ok)
    {
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints, totalAvgErr);
    }

    return ok;
}

bool runCalibration(Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>> imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs, double& totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if (s.flag & CALIB_FIX_ASPECT_RATIO)
    {
        cameraMatrix.at<double>(0,0) = s.aspectRatio;
    }

    if (s.useFisheye)
    {
        distCoeffs = Mat::zeros(4,1,CV_64F);
    }
    else
    {
        distCoeffs = Mat::zeros(8,1,CV_64F);
    }

    vector<vector<Point3f>> objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    double rms;

    if (s.useFisheye)
    {
        Mat _rvecs, _tvecs;
        rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs, _tvecs, s.flag);

        rvecs.reserve(_rvecs.rows);
        rvecs.reserve(_tvecs.rows);

        for (int i = 0; i < (int)(objectPoints.size()); i++)
        {
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }
    }
    else
    {
        rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, s.flag);
    }

    ROS_INFO_STREAM("Re-projection error reported by calibrateCamera: " << rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs, s.useFisheye);

    return ok;
}

void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints, double totalAvgErr)
{
    FileStorage fs(s.outputFileName, FileStorage::WRITE);

    time_t tm;
    time (&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);

    fs << "calibration_time" << buf;

    if (!rvecs.empty() || !reprojErrs.empty())
    {
        fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
    }
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;

    if (s.flag & CALIB_FIX_ASPECT_RATIO)
    {
        fs << "fix_aspect_ratio" << s.aspectRatio;
    }

    if (s.flag)
    {
        std::stringstream flagsStringStream;
        if (s.useFisheye)
        {
            flagsStringStream << "flags:"
                << (s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
                << (s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
                << (s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
                << (s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
                << (s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
                << (s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        }
        else
        {
            flagsStringStream << "flags:"
                << (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
                << (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                << (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
                << (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
                << (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
                << (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
                << (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
                << (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
                << (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
        }
        //fs.writeComment(flagsStringStream.str());
    }

    fs << "flags" << s.flag;
    fs << "fisheye_model" << s.useFisheye;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "avg_reprojection_error" << totalAvgErr;

    if (s.writeExtrinsics && !reprojErrs.empty())
    {
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);
    }

    if (s.writeExtrinsics && !rvecs.empty() && !tvecs.empty())
    {
        if (rvecs[0].type() != tvecs[0].type()) ROS_FATAL("rvecs and tvecs type mismatch");
        
        Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
        bool needReshapeR = rvecs[0].depth() != 1;
        bool needReshapeT = tvecs[0].depth() != 1;

        for (size_t i = 0; i < rvecs.size(); i++)
        {
            Mat r = bigmat(Range(int(i), int(i+1)), Range(0,3));
            Mat t = bigmat(Range(int(i), int(i+1)), Range(3,6));

            if (needReshapeR)
            {
                rvecs[i].reshape(1,1).copyTo(r);
            }
            else
            {
                if (!(rvecs[i].rows == 3 && rvecs[i].cols == 1)) ROS_FATAL("rvecs invalid row and column sizes");
                r = rvecs[i].t();
            }

            if (needReshapeT)
            {
                tvecs[i].reshape(1,1).copyTo(t);
            }
            else
            {
                if (!(tvecs[i].rows == 3 && tvecs[i].cols == 1)) ROS_FATAL("tvecs invalid row and column sizes");
                t = tvecs[i].t();
            }
        }

        //fs.writeComment("a set of 6-tuples (rotation vector + translation vecotr) for each view");
        fs << "extrinsic_parameters" << bigmat;
    }

    if (s.writePoints && !imagePoints.empty())
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for(size_t i = 0; i < imagePoints.size(); i++)
        {
            Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners, Settings::Pattern patternType)
{
    corners.clear();
    
    for (int i = 0; i < boardSize.height; i++)
    {
        for (int j = 0; j < boardSize.width; j++)
        {
            switch(patternType)
            {
                case Settings::CHESSBOARD:
                case Settings::CIRCLES_GRID:
                    corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
                    break;
                case Settings::ASYMMETRIC_CIRCLES_GRID:
                    corners.push_back(Point3f((2*j + i % 2)*squareSize, i*squareSize, 0));
                    break;
                default:
                    break;
            }
        }
    }
}

double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints, const vector<vector<Point2f> >& imagePoints, const vector<Mat>& rvecs, const vector<Mat>& tvecs, const Mat& cameraMatrix , const Mat& distCoeffs, vector<float>& perViewErrors, bool fisheye)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (size_t i = 0; i < objectPoints.size(); i++)
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix, distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }

        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

