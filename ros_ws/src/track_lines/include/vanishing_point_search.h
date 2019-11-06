#ifndef VANISHING_POINT_SEARCH_H
#define VANISHING_POINT_SEARCH_H

#include <iostream>
#include <stdio.h>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "datatypes.h"
#include "utils.h"

using namespace std;
using namespace cv;
using namespace vanishing_point_search;

#define PI 3.14

class VanishingPointSearch
{
    private:
        Mat current_image_;


        cv::Matx33f warp_matrix_;

        const int kLowThreshold_ = 100;
        const int kHighThreshold_ = 200;
        const int kKernelSize_ = 3;


        /*
        image – 8-bit, single-channel binary source image. The image may be modified by the function.
        lines – Output vector of lines. Each line is represented by a 4-element vector (x_1, y_1, x_2, y_2) , where (x_1,y_1) and (x_2, y_2) are the ending points of each detected line segment.
        rho – Distance resolution of the accumulator in pixels.
        theta – Angle resolution of the accumulator in radians.
        threshold – Accumulator threshold parameter. Only those lines are returned that get enough votes ( >\texttt{threshold} ).
        minLineLength – Minimum line length. Line segments shorter than that are rejected.
        maxLineGap – Maximum allowed gap between points on the same line to link them.
        */

        const int kRho_ = 1;
        const float kTheta_ = CV_PI/180;
        const int kMinIntersections = 20;
        const int kMinLineLength = 10;
        const int kMaxLineGap = 50;

        const int kXROIStart_ =    0;
        const int kYROIStart_ =  350;
        const int kROIWidth_  = 1280;
        const int kROIHeight_ =   67;

        const int kMinLeftLineAngle =  20;
        const int kMaxLeftLineAngle = 90;

        const int kMinRightLineAngle =  90;
        const int kMaxRightLineAngle = 160;


        const int kXMinLeftLine = 150;
        const int kXMaxLeftLine = 300;
        const int kXMinRightLine = 650;
        const int kXMaxRightLine = 850;

        const int kCarMidPositionInFrame = 640;

        Mat canny_image_;
        Mat hough_image_;
        Mat current_image_roi_;
        vector<Vec4i> hough_lines_;

        vector<HoughLinesInDriveDirection> hough_lines_in_drive_direction_;
        vector<HoughLinesInDriveDirection> left_hough_lines_in_drive_direction_;
        vector<HoughLinesInDriveDirection> right_hough_lines_in_drive_direction_;

        vector<HoughLinesPointsAndAngle> left_hough_lines_points_and_angle_;
        vector<HoughLinesPointsAndAngle> right_hough_lines_points_and_angle_;

        vector<HoughLinesWarpedPerspektive> left_hough_lines_warped_perspektive_;
        vector<HoughLinesWarpedPerspektive> right_hough_lines_warped_perspektive_;




        void ClearMemory();
        void SetImage(Mat image);
        void CropToRegionOfInterest();
        void ApplyCannyEdge();
        void ApplyHoughLines();
        void ChangeLinePointsToDriveDirection();
        void GatherTrueRangeLeftAndRightLines();
        void RejectFalseLeftAndRightLineAngles();
        void WarpPerspektiveOfHoughLines(int _line);

        void ComputeIntersections();

    public:
        VanishingPointSearch();

        void FindVanishingPoint(Mat image, Mat warp_matrix);

        void ShowCannyEdgeImage();
        void DrawHoughLines(Mat &image, int _line);
        void DrawWarpedPerspektiveHoughLines(Mat &rgb, int _line);
        void CoutHoughLines();
};

#endif // VANISHING_POINT_SEARCH_H
