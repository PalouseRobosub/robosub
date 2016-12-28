#include "MonoCalibrator.hpp"
#include <string>
#include <algorithm>
#include <vector>

std::string modeToString(CalibrationMode & mode)
{
    switch(mode)
    {
        case CalibrationMode::DETECTION:
            return "DETECTION";
        case CalibrationMode::CAPTURING:
            return "CAPTURING";
        case CalibrationMode::CALIBRATED:
            return "CALIBRATED";
        default:
            return "UNKNOWN";
    }
}

void read(const FileNode& node, Settings& x,
          const Settings& default_value)
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

void write(FileStorage& fs, const Settings& s)
{
    s.write(fs);
}

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix,
                           Mat& distCoeffs, vector<vector<Point2f>> imagePoints)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs,
                             imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);

    ROS_INFO_STREAM((ok ? "Calibration succeeded" : "Calibration failed") <<
                    ". avg re-projection errror = " << totalAvgErr);

    if (ok)
    {
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
                         reprojErrs, imagePoints, totalAvgErr);
    }

    return ok;
}

bool runCalibration(Settings& s, Size& imageSize, Mat& cameraMatrix,
                    Mat& distCoeffs, vector<vector<Point2f>> imagePoints,
                    vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs, double& totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if (s.flag & CALIB_FIX_ASPECT_RATIO)
    {
        cameraMatrix.at<double>(0, 0) = s.aspectRatio;
    }

    if (s.useFisheye)
    {
        distCoeffs = Mat::zeros(4, 1, CV_64F);
    }
    else
    {
        distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    vector<vector<Point3f>> objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0],
                             s.calibrationPattern);

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    double rms;

    if (s.useFisheye)
    {
        Mat _rvecs, _tvecs;
        rms = fisheye::calibrate(objectPoints, imagePoints, imageSize,
                                 cameraMatrix, distCoeffs, _rvecs, _tvecs,
                                 s.flag);

        rvecs.reserve(_rvecs.rows);
        rvecs.reserve(_tvecs.rows);

        for (int i = 0; i < static_cast<int>(objectPoints.size()); i++)
        {
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }
    }
    else
    {
        rms = calibrateCamera(objectPoints, imagePoints, imageSize,
                              cameraMatrix, distCoeffs, rvecs, tvecs, s.flag);
    }

    ROS_INFO_STREAM("Re-projection error reported by calibrateCamera: " << rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs,
                                            tvecs, cameraMatrix, distCoeffs,
                                            reprojErrs, s.useFisheye);

    return ok;
}

void saveCameraParams(Settings& s, Size& imageSize, Mat& cameraMatrix,
                      Mat& distCoeffs, const vector<Mat>& rvecs,
                      const vector<Mat>& tvecs, const vector<float>& reprojErrs,
                      const vector<vector<Point2f> >& imagePoints,
                      double totalAvgErr)
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
        fs << "nr_of_frames" <<
              static_cast<int>(std::max(rvecs.size(), reprojErrs.size()));
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
                << (s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ?
                             " +recompute_extrinsic" : "");
        }
        else
        {
            flagsStringStream << "flags:"
                << (s.flag & CALIB_USE_INTRINSIC_GUESS ?
                             " +use_intrinsic_guess" : "")
                << (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                << (s.flag & CALIB_FIX_PRINCIPAL_POINT ?
                             " +fix_principal_point" : "")
                << (s.flag & CALIB_ZERO_TANGENT_DIST ?
                             " +zero_tangent_dist" : "")
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
        if (rvecs[0].type() != tvecs[0].type())
            ROS_FATAL("rvecs and tvecs type mismatch");

        Mat bigmat(static_cast<int>(rvecs.size()), 6,
                   CV_MAKETYPE(rvecs[0].type(), 1));
        bool needReshapeR = rvecs[0].depth() != 1;
        bool needReshapeT = tvecs[0].depth() != 1;

        for (size_t i = 0; i < rvecs.size(); i++)
        {
            Mat r = bigmat(Range(static_cast<int>(i), static_cast<int>(i + 1)),
                           Range(0, 3));
            Mat t = bigmat(Range(static_cast<int>(i), static_cast<int>(i + 1)),
                           Range(3, 6));

            if (needReshapeR)
            {
                rvecs[i].reshape(1, 1).copyTo(r);
            }
            else
            {
                if (!(rvecs[i].rows == 3 && rvecs[i].cols == 1))
                    ROS_FATAL("rvecs invalid row and column sizes");
                r = rvecs[i].t();
            }

            if (needReshapeT)
            {
                tvecs[i].reshape(1, 1).copyTo(t);
            }
            else
            {
                if (!(tvecs[i].rows == 3 && tvecs[i].cols == 1))
                    ROS_FATAL("tvecs invalid row and column sizes");
                t = tvecs[i].t();
            }
        }

        fs << "extrinsic_parameters" << bigmat;
    }

    if (s.writePoints && !imagePoints.empty())
    {
        Mat imagePtMat(static_cast<int>(imagePoints.size()),
                       static_cast<int>(imagePoints[0].size()), CV_32FC2);
        for(size_t i = 0; i < imagePoints.size(); i++)
        {
            Mat r = imagePtMat.row(static_cast<int>(i)).reshape(2,
                                                               imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

void calcBoardCornerPositions(Size boardSize, float squareSize,
                              vector<Point3f>& corners,
                              Settings::Pattern patternType)
{
    corners.clear();

    for (int i = 0; i < boardSize.height; i++)
    {
        for (int j = 0; j < boardSize.width; j++)
        {
            switch(patternType)
            {
                case Settings::Pattern::CHESSBOARD:
                case Settings::Pattern::CIRCLES_GRID:
                    corners.push_back(Point3f(j * squareSize, i * squareSize,
                                              0));
                    break;
                case Settings::Pattern::ASYMMETRIC_CIRCLES_GRID:
                    corners.push_back(Point3f((2 * j + i % 2) * squareSize,
                                              i * squareSize, 0));
                    break;
                default:
                    break;
            }
        }
    }
}

double computeReprojectionErrors(const vector<vector<Point3f>>& objectPoints,
                                 const vector<vector<Point2f>>& imagePoints,
                                 const vector<Mat>& rvecs,
                                 const vector<Mat>& tvecs,
                                 const Mat& cameraMatrix,
                                 const Mat& distCoeffs,
                                 vector<float>& perViewErrors, bool fisheye)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (size_t i = 0; i < objectPoints.size(); i++)
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i],
                                   tvecs[i], cameraMatrix, distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix,
                          distCoeffs, imagePoints2);
        }

        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = static_cast<float>(std::sqrt(err * err / n));
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

