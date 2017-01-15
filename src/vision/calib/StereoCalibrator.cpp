#include "StereoCalibrator.hpp"
#include <utility>
#include <string>
#include <algorithm>
#include <vector>

StereoCalibrator::StereoCalibrator(Size boardSize, float squareSize,
                         string outputFilename, bool displayCorners,
                         bool useCalibrated, bool showRectified)
{
    this->boardSize = boardSize;
    this->squareSize = squareSize;
    this->displayCorners = displayCorners;
    this->useCalibrated = useCalibrated;
    this->showRectified = showRectified;

    outputFile.open(outputFilename, FileStorage::WRITE);
}

StereoCalibrator::~StereoCalibrator()
{
    outputFile.release();
}

void StereoCalibrator::submitLeftImg(const Mat &leftImg)
{
    vector<Point2f> corners;
    if (findChessboardCorners(leftImg, boardSize, corners,
                              CALIB_CB_ADAPTIVE_THRESH |
                              CALIB_CB_NORMALIZE_IMAGE |
                              CALIB_CB_FAST_CHECK))
    {
        hasLeft = true;
    }

    if (hasLeft && hasRight)
    {
        validPairs.push_back(pair<Mat, Mat>(leftImg.clone(), imgTemp.clone()));
        hasLeft = hasRight = false;
        imgTemp.release();
    }
}

void StereoCalibrator::submitRightImg(const Mat &rightImg)
{
    vector<Point2f> corners;
    if (findChessboardCorners(rightImg, boardSize, corners,
                              CALIB_CB_ADAPTIVE_THRESH |
                              CALIB_CB_NORMALIZE_IMAGE |
                              CALIB_CB_FAST_CHECK))
    {
        hasRight = true;
    }

    if (hasLeft && hasRight)
    {
        validPairs.push_back(pair<Mat, Mat>(imgTemp.clone(), rightImg.clone()));
        hasLeft = hasRight = false;
        imgTemp.release();
    }
}

void StereoCalibrator::calibrate()
{
    if (validPairs.size() < 2)
    {
        ROS_ERROR_STREAM("StereoCalibrator calibrate: Too few valid pairs " <<
                         "to calibrate");
        return;
    }

    Size imageSize = validPairs[1].first.size();

    vector<vector<Point2f>> imagePoints[2];
    vector<vector<Point3f>> objectPoints;

    imagePoints[0].resize(validPairs.size());
    imagePoints[1].resize(validPairs.size());
    objectPoints.resize(validPairs.size());

    for (unsigned int i = 0; i < validPairs.size(); i++)
    {
        for (int j = 0; j < boardSize.height; j++)
        {
            for (int k = 0; k < boardSize.width; k++)
            {
                objectPoints[i].push_back(Point3f(k*squareSize,
                                                 j*squareSize, 0));
            }
        }
    }

    ROS_INFO_STREAM("Running stereo calibration");

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0],
                                         imageSize, 0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1],
                                         imageSize, 0);

    Mat R, T, E, F;

    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imageSize, R, T, E, F,
                                 CALIB_FIX_ASPECT_RATIO +
                                 CALIB_ZERO_TANGENT_DIST +
                                 CALIB_USE_INTRINSIC_GUESS +
                                 CALIB_SAME_FOCAL_LENGTH +
                                 CALIB_RATIONAL_MODEL +
                                 CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                                 TermCriteria(TermCriteria::COUNT +
                                             TermCriteria::EPS, 100, 1e-5));

    ROS_INFO_STREAM("Calibrated with RMS error of: " << rms);


    //Calibration Quality Check
    double err = 0;
    int nPoints = 0;
    vector<Vec3f> lines[2];

    for (unsigned int i = 0; i < validPairs.size(); i++)
    {
        int npt = static_cast<int>(imagePoints[0][i].size());
        Mat imgpt[2];

        for (int k = 0; k < 2; k++)
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k],
                            Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }

        for (int j = 0; j < npt; j++)
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].x*lines[1][j][1] +
                                lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].x*lines[0][j][1] +
                                lines[0][j][2]);
            err += errij;
        }

        nPoints += npt;
    }

    ROS_INFO_STREAM("Average Epipolar Error: " << err/nPoints);

    saveIntrinsics(cameraMatrix[0], distCoeffs[0], cameraMatrix[1],
                   distCoeffs[1]);


    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize,
                  &validRoi[0], &validRoi[1]);

    saveExtrinsics(R, T, R1, R2, P1, P2, Q);

    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) >
                            fabs(P2.at<double>(0, 3));

    if (!showRectified)
        return;


    Mat rmap[2][2];

    //Determine if should use Hartley's Method
    if (!useCalibrated)
    {
        vector<Point2f> allimgpt[2];
        for (int k = 0; k < 2; k++)
        {
            for (unsigned int i = 0; i < validPairs.size(); i++)
            {
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(),
                          back_inserter(allimgpt[k]));
            }
        }
        Mat F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]),
                                   FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F,
                                  imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H1*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize,
                            CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize,
                            CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    if (!isVerticalStereo)
    {
        sf = 600.0/MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width * sf);
        h = cvRound(imageSize.height * sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300.0/MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width * sf);
        h = cvRound(imageSize.height * sf);
        canvas.create(h*2, w, CV_8UC3);
    }

    for (unsigned int i = 0; i < validPairs.size(); i++)
    {
        for (int k = 0; k < 2; k++)
        {
            Mat img = (k == 0 ? validPairs[i].first : validPairs[i].second);
            Mat rimg, cimg;

            remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
            cvtColor(rimg, cimg, COLOR_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) :
                                                 canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
            if (useCalibrated)
            {
                Rect vroi(cvRound(validRoi[k].x * sf),
                          cvRound(validRoi[k].y * sf),
                          cvRound(validRoi[k].width * sf),
                          cvRound(validRoi[k].height * sf));
                rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
            }
        }

        if (!isVerticalStereo)
        {
            for (int j = 0; j < canvas.rows; j += 16)
            {
                line(canvas, Point(0, j), Point(canvas.cols, j),
                     Scalar(0, 255, 0), 1, 8);
            }
        }
        else
        {
            for (int j = 0; j < canvas.cols; j += 16)
            {
                line(canvas, Point(j, 0), Point(j, canvas.rows),
                     Scalar(0, 255, 0), 1, 8);
            }
        }

        imshow("Rectified", canvas);
        char c = static_cast<char>(waitKey());
        if (c == 27 || c == 'q' || c == 'Q')
        {
            break;
        }
    }
}

void StereoCalibrator::saveIntrinsics(Mat camMat1, Mat distCoeffs1,
                                      Mat camMat2, Mat distCoeffs2)
{
    outputFile << "M1" << camMat1;
    outputFile << "D1" << distCoeffs1;
    outputFile << "M2" << camMat2;
    outputFile << "D2" << distCoeffs2;
}

void StereoCalibrator::saveExtrinsics(Mat R, Mat T, Mat R1, Mat R2, Mat P1,
                                      Mat P2, Mat Q)
{
    outputFile << "R" << R;
    outputFile << "T" << T;
    outputFile << "R1" << R1;
    outputFile << "R2" << R2;
    outputFile << "P1" << P1;
    outputFile << "P2" << P2;
    outputFile << "Q" << Q;
}


int StereoCalibrator::getNumValidPairs()
{
    return validPairs.size();
}
