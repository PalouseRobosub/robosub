#include "StereoStandaloneCalibrator.hpp"
#include <utility>
#include <string>
#include <algorithm>
#include <vector>

StereoStandaloneCalibrator::StereoStandaloneCalibrator( Mat &K1, Mat &D1, Mat &K2,
                         Mat &D2, Size boardSize, float squareSize,
                         string outputFilename, bool displayCorners,
                         bool useCalibrated, bool showRectified)
{
    this->boardSize = boardSize;
    this->squareSize = squareSize;
    this->displayCorners = displayCorners;
    this->useCalibrated = useCalibrated;
    this->showRectified = showRectified;
    this->K1 = K1;
    this->D1 = D1;
    this->K2 = K2;
    this->D2 = D2;

    outputFile.open(outputFilename, FileStorage::WRITE);
}

StereoStandaloneCalibrator::~StereoStandaloneCalibrator()
{
    if (outputFile.isOpened())
    {
        outputFile.release();
    }
}

void StereoStandaloneCalibrator::submitImgs(const Mat &rightImg, const Mat &leftImg)
{
    vector<Point2f> rightCorners;
    vector<Point2f> leftCorners;
    if (findChessboardCorners(rightImg, boardSize, rightCorners,
                              CALIB_CB_ADAPTIVE_THRESH |
                              CALIB_CB_NORMALIZE_IMAGE |
                              CALIB_CB_FAST_CHECK)
        && findChessboardCorners(leftImg, boardSize, leftCorners,
                                 CALIB_CB_ADAPTIVE_THRESH |
                                 CALIB_CB_NORMALIZE_IMAGE |
                                 CALIB_CB_FAST_CHECK))
    {
        validPairs.push_back(pair<Mat, Mat>(leftImg.clone(), rightImg.clone()));

        Mat leftGray, rightGray;
        cvtColor(leftImg, leftGray, CV_BGR2GRAY);
        cvtColor(rightImg, rightGray, CV_BGR2GRAY);

        cornerSubPix(leftGray, leftCorners, cv::Size(5, 5), cv::Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        cornerSubPix(rightGray, rightCorners, cv::Size(5, 5), cv::Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

        drawChessboardCorners(rightImg, boardSize, rightCorners, true);
        drawChessboardCorners(leftImg, boardSize, leftCorners, true);

        imagePoints1.push_back(leftCorners);
        imagePoints2.push_back(rightCorners);

        vector<Point3d> points;
        vector<Point3f> fPoints;
        for (int j = 0; j < boardSize.height; j++)
        {
            for (int k = 0; k < boardSize.width; k++)
            {
                points.push_back(Point3d(j*squareSize, k*squareSize, 0));
                fPoints.push_back(Point3f(j*squareSize, k*squareSize, 0));
            }
        }
        objectPointsf.push_back(fPoints);
        objectPoints.push_back(points);
    }

    imshow("right", rightImg);
    imshow("left", leftImg);
    waitKey(1);
}

void StereoStandaloneCalibrator::calibrate()
{
    if (validPairs.size() < 2)
    {
        ROS_ERROR_STREAM("StereoStandaloneCalibrator calibrate: Too few valid pairs " <<
                         "to calibrate");
        return;
    }

    Size imageSize = validPairs[1].first.size();

    for(unsigned int i = 0; i < imagePoints1.size(); i++)
    {
        vector<Point2d> v1, v2;
        for (unsigned int j = 0; j < imagePoints1[i].size(); j++)
        {
            v1.push_back(Point2d(static_cast<double>(imagePoints1[i][j].x),
                                 static_cast<double>(imagePoints1[i][j].y))
                        );
            v2.push_back(Point2d(static_cast<double>(imagePoints2[i][j].x),
                                 static_cast<double>(imagePoints2[i][j].y))
                        );
        }
        leftImgPoints.push_back(v1);
        rightImgPoints.push_back(v2);
    }

    ROS_INFO_STREAM("Running stereo calibration");

    ROS_INFO_STREAM("Using image size of: " << imageSize);
    /*double rms = stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imageSize, R, T, E, F,
                                 CALIB_FIX_ASPECT_RATIO +
                                 CALIB_ZERO_TANGENT_DIST +
                                 CALIB_USE_INTRINSIC_GUESS +
                                 CALIB_SAME_FOCAL_LENGTH +
                                 CALIB_RATIONAL_MODEL +
                                 CALIB_FIX_K3 + CALIB_FIX_K4,
                                 TermCriteria(TermCriteria::COUNT +
                                             TermCriteria::EPS, 100, 1e-5));*/

    int flag = 0;
    flag |= fisheye::CALIB_FIX_INTRINSIC;
    flag |= fisheye::CALIB_USE_INTRINSIC_GUESS;
    flag |= fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flag |= fisheye::CALIB_CHECK_COND;
    flag |= fisheye::CALIB_FIX_SKEW;
    //flag |= fisheye::CALIB_FIX_K1;
    //flag |= fisheye::CALIB_FIX_K2;
    //flag |= fisheye::CALIB_FIX_K3;
    //flag |= fisheye::CALIB_FIX_K4;


    Mat R, T, E, F;
    double rms = fisheye::stereoCalibrate(objectPoints, leftImgPoints,
                                          rightImgPoints, K1, D1, K2,
                                          D2, imageSize, R, T,
                                          flag,
                                          TermCriteria(TermCriteria::COUNT +
                                                       TermCriteria::EPS, 12,
                                                       0));

    ROS_INFO_STREAM("Calibrated with RMS error of: " << rms);

    F = findFundamentalMat(imagePoints1[0], imagePoints1[1]);
    E = findEssentialMat(imagePoints1[0], imagePoints1[1]);

    //Calibration Quality Check
    double err = 0;
    int nPoints = 0;
    vector<Vec3f> lines[2];

    for (unsigned int i = 0; i < validPairs.size(); i++)
    {
        int npt = static_cast<int>(imagePoints1[i].size());
        Mat imgpt1, imgpt2;

        imgpt1 = Mat(imagePoints1[i]);
        imgpt2 = Mat(imagePoints2[i]);
        fisheye::undistortPoints(imgpt1, imgpt1, K1, D1, Mat(), K1);
        fisheye::undistortPoints(imgpt2, imgpt2, K2, D2, Mat(), K2);
        computeCorrespondEpilines(imgpt1, 1, F, lines[0]);
        computeCorrespondEpilines(imgpt2, 2, F, lines[1]);

        for (int j = 0; j < npt; j++)
        {
            double errij = fabs(imagePoints1[i][j].x*lines[1][j][0] +
                                imagePoints1[i][j].x*lines[1][j][1] +
                                lines[1][j][2]) +
                           fabs(imagePoints2[i][j].x*lines[0][j][0] +
                                imagePoints2[i][j].x*lines[0][j][1] +
                                lines[0][j][2]);
            err += errij;
        }

        nPoints += npt;
    }

    ROS_INFO_STREAM("Average Epipolar Error: " << err/nPoints);

    saveIntrinsics(K1, D1, K2, D2, imageSize);


    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    /*stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize,
                  &validRoi[0], &validRoi[1]);*/

    fisheye::stereoRectify(K1, D1, K2, D2, imageSize, R, T, R1, R2,
                           P1, P2, Q, CALIB_ZERO_DISPARITY,
                           imageSize, 1);

    ROS_INFO_STREAM("Completed rectification.");

    saveExtrinsics(R, T, R1, R2, P1, P2, F, E, Q);

    outputFile.release();

    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) >
                            fabs(P2.at<double>(0, 3));

    if (!showRectified)
        return;


    Mat rmap[2][2];

    //Determine if should use Hartley's Method
    if (!useCalibrated)
    {
        vector<Point2f> allimgpt[2];
        for (unsigned int i = 0; i < validPairs.size(); i++)
        {
            std::copy(imagePoints1[i].begin(), imagePoints1[i].end(),
                      back_inserter(allimgpt[0]));
            std::copy(imagePoints2[i].begin(), imagePoints2[i].end(),
                      back_inserter(allimgpt[1]));
        }

        Mat F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]),
                                   FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F,
                                  imageSize, H1, H2, 3);

        R1 = K1.inv()*H1*K1;
        R2 = K2.inv()*H1*K2;
        P1 = K1;
        P2 = K2;
    }

    fisheye::initUndistortRectifyMap(K1, D1, R1, P1, imageSize, CV_16SC2,
                                     rmap[0][0], rmap[0][1]);
    fisheye::initUndistortRectifyMap(K2, D2, R2, P2,
                                     imageSize, CV_16SC2, rmap[1][0],
                                     rmap[1][1]);

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
            ROS_INFO_STREAM("Rimg type: " << rimg.type());
            cimg = rimg.clone();
            //cvtColor(rimg, cimg, COLOR_GRAY2BGR);
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

void StereoStandaloneCalibrator::saveIntrinsics(Mat camMat1, Mat distCoeffs1,
                                      Mat camMat2, Mat distCoeffs2,
                                      Size imageSize)
{
    ROS_INFO_STREAM("Saving Intrinsics");
    time_t tm;
    time (&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);

    outputFile << "calibration_time" << buf;
    outputFile << "K1" << camMat1;
    outputFile << "D1" << distCoeffs1;
    outputFile << "K2" << camMat2;
    outputFile << "D2" << distCoeffs2;
    outputFile << "image_size" << imageSize;
    ROS_INFO_STREAM("Intrinsics saved");
}

void StereoStandaloneCalibrator::saveExtrinsics(Mat R, Mat T, Mat R1, Mat R2, Mat P1,
                                      Mat P2, Mat F, Mat E, Mat Q)
{
    ROS_INFO_STREAM("Saving Extrinsics");
    outputFile << "R" << R;
    outputFile << "T" << T;
    outputFile << "R1" << R1;
    outputFile << "R2" << R2;
    outputFile << "P1" << P1;
    outputFile << "P2" << P2;
    outputFile << "F" << F;
    outputFile << "E" << E;
    outputFile << "Q" << Q;
    ROS_INFO_STREAM("Extrinsics saved");
}


int StereoStandaloneCalibrator::getNumValidPairs()
{
    return validPairs.size();
}
