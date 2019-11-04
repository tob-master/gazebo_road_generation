#ifndef HOUGHLINE_TRANSFORM_H
#define HOUGHLINE_TRANSFORM_H

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

using namespace std;
using namespace cv;

#define PI 3.14

class HoughLineTransform
{

    Mat current_image_;


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
    const int kMinIntersections = 10;
    const int kMinLineLength = 50;
    const int kMaxLineGap = 10;



    Mat canny_image_;
    Mat hough_image_;
    vector<Vec4i> found_lines_;



    public:
        HoughLineTransform();

        void FindVanashingPoint(Mat image);

        void ApplyCannyEdge();
    void ComputeIntersections();
        void ApplyHoughLines();
        void ShowCannyEdgeImage();
        void ShowHoughLines();
};

#endif // HOUGHLINE_TRANSFORM_H
