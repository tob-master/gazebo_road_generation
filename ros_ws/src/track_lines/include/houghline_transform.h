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

    Mat canny_image_;
    Mat hough_image_;
    vector<Vec4i> found_lines_;



    public:
        HoughLineTransform();
        void ApplyCannyEdge(Mat image, int low_threshold, int high_threshold, int kernel_size);
        void ApplyHoughLines(int rho, float theta, int min_intersection, int min_line_length, int max_line_gap);
        void ShowCannyEdgeImage();
        void ShowHoughLines();
};

#endif // HOUGHLINE_TRANSFORM_H
