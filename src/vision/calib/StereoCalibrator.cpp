#include "StereoCalibrator.hpp"
#include <utility>
#include <string>
#include <algorithm>
#include <vector>

StereoCalibrator::StereoCalibrator(Size boardSize, float squareSize,
                         string outputFilename, bool displayCorners,
                         bool useCalibrated, bool showRectified,
			 int cropRadius)
{
    this->boardSize = boardSize;
    this->squareSize = squareSize;
    this->displayCorners = displayCorners;
    this->useCalibrated = useCalibrated;
    this->showRectified = showRectified;
    this->cropRadius = cropRadius;

    outputFile.open(outputFilename, FileStorage::WRITE);
}

StereoCalibrator::~StereoCalibrator()
{
    if (outputFile.isOpened())
    {
        outputFile.release();
    }
}

void StereoCalibrator::submitImgs(const Mat &rightImg, const Mat &leftImg)
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

void StereoCalibrator::calibrate()
{
    if (validPairs.size() < 2)
    {
        ROS_ERROR_STREAM("StereoCalibrator calibrate: Too few valid pairs " <<
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

    Mat cameraMatrix[2], distCoeffs[2];

    ROS_INFO_STREAM("Image points 1 size: " << imagePoints1.size());
    //ROS_INFO_STREAM("Image points 1 total: " << imagePoints1.total());
    ROS_INFO_STREAM("Image points 2 size: " << imagePoints2.size());
    //ROS_INFO_STREAM("Image points 2 total: " << imagePoints2.total());
    ROS_INFO_STREAM("Left image points size: " << leftImgPoints.size());
    ROS_INFO_STREAM("Right image points size: " << rightImgPoints.size());

    for (unsigned int i = 0; i < imagePoints1.size(); i++)
    {
        ROS_INFO_STREAM("Image points 1 [" << i << "] size: " <<
                        imagePoints1[i].size());
    }
    ROS_INFO_STREAM("Object points size: " << objectPoints.size());
    ROS_INFO_STREAM("Object points type: " << Mat(objectPoints).type());
    ROS_INFO_STREAM("CV_64FC3: " << CV_64FC3);
    //ROS_INFO_STREAM("Object points total: " << objectPoints.total());
    for (unsigned int i = 0; i < objectPoints.size(); i++)
    {
        ROS_INFO_STREAM("Object points [" << i << "] size: " <<
                        objectPoints[i].size());
    }
    //ROS_INFO_STREAM("Image points 2 check vector: " <<
    //                imagePoints2.getMat(0).checkVector(2, CV_32F));
    //cameraMatrix[0] = initCameraMatrix2D(objectPointsf, imagePoints1,
    //                                     imageSize, 0);
    ROS_INFO_STREAM("Created cameraMatrix 0? " << !cameraMatrix[0].empty());
    //cameraMatrix[1] = initCameraMatrix2D(objectPointsf, imagePoints2,
    //                                     imageSize, 0);

    ROS_INFO_STREAM("Distortion coeffs 1 empty? " << distCoeffs[0].empty());

    ROS_INFO_STREAM("Created cameraMatrix 1? " << !cameraMatrix[1].empty());

    ROS_INFO_STREAM("Distortion coeffs 2 empty? " << distCoeffs[1].empty());

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

    Mat lR, rR, lT, rT;

    int monoFlag = 0;
    monoFlag |= fisheye::CALIB_FIX_SKEW;
    monoFlag |= fisheye::CALIB_CHECK_COND;
    monoFlag |= fisheye::CALIB_RECOMPUTE_EXTRINSIC;

    double leftRms = fisheye::calibrate(objectPoints, leftImgPoints, imageSize,
                                        cameraMatrix[0], distCoeffs[0], lR,
                                        lT, monoFlag);

    ROS_INFO_STREAM("Calibrated left cam with rms: " << leftRms);

    double rightRms = fisheye::calibrate(objectPoints, rightImgPoints,
                                        imageSize, cameraMatrix[1],
                                        distCoeffs[1], rR, rT, monoFlag);

    ROS_INFO_STREAM("Calibrated right cam with rms: " << rightRms);


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
                                          rightImgPoints, cameraMatrix[0],
                                          distCoeffs[0], cameraMatrix[1],
                                          distCoeffs[1], imageSize, R, T,
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
        fisheye::undistortPoints(imgpt1, imgpt1, cameraMatrix[0], distCoeffs[0],
                                 Mat(), cameraMatrix[0]);
        fisheye::undistortPoints(imgpt2, imgpt2, cameraMatrix[1], distCoeffs[1],
                                 Mat(), cameraMatrix[1]);
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

    saveIntrinsics(cameraMatrix[0], distCoeffs[0], cameraMatrix[1],
                   distCoeffs[1], imageSize, cropRadius);


    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    /*stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize,
                  &validRoi[0], &validRoi[1]);*/

    fisheye::stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, imageSize, 1);

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

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H1*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    fisheye::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1,
                                     imageSize, CV_16SC2, rmap[0][0],
                                     rmap[0][1]);
    fisheye::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2,
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

void StereoCalibrator::saveIntrinsics(Mat camMat1, Mat distCoeffs1,
                                      Mat camMat2, Mat distCoeffs2,
                                      Size imageSize, int cropRadius)
{
    ROS_INFO_STREAM("Saving Intrinsics");
    time_t tm;
    time (&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);

    outputFile << "calibration_time" << buf;
    outputFile << "crop_radius" << cropRadius;
    outputFile << "K1" << camMat1;
    outputFile << "D1" << distCoeffs1;
    outputFile << "K2" << camMat2;
    outputFile << "D2" << distCoeffs2;
    outputFile << "image_size" << imageSize;
    ROS_INFO_STREAM("Intrinsics saved");
}

void StereoCalibrator::saveExtrinsics(Mat R, Mat T, Mat R1, Mat R2, Mat P1,
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


int StereoCalibrator::getNumValidPairs()
{
    return validPairs.size();
}
