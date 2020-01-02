#ifndef PERCEPTU_ALGROUPING_H
#define PERCEPTU_ALGROUPING_H

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
#include "line_validation_table.h"
#include "utils.h"
#include "defines.h"



class PerceptualGrouping
{
private:
        Mat current_image_;
        Mat canny_image_;

      vector<Vec4i> hough_lines_;
      vector<Point> hough_mid_points_;

      vector<float> hough_line_lengths_;
      vector<float> hough_line_proximities_;
      vector<pair<int,int>> hough_line_proximity_thresholed_;

      vector<string> used_permutations_;

      const int kHoghLinesRho_ = 1;
      const float kHoughLinesTheta_ = 0.01745329251;
      const int kHoughLinesMinintersections_ = 20;
      const int kHoughLinesMinLineLength_ = 10;
      const int kHoughLinesMaxLineGap_ = 50;


      const int kCannyLowThreshold_ = 100;
      const int kCannyHighThreshold_ = 200;
      const int kCannyKernelSize_ = 3;


      Mat labeled_image_;
      Mat components_stats_;
      Mat components_centroids_;

      const int kConnectionCount_ = 8;

      int components_count_;

      bool IsPermuted(int i, int j, vector<string> &used_permutations);

public:

    PerceptualGrouping();

    void SetImage(Mat image);
    void ApplyHoughLines();
    void ApplyCannyEdge();
    void ApplyConnectedComponents();
    void ClearMemory();
    void ChangeLinePointsToDriveDirection();
    void ComputeHoughLineProximity();
    void ComputeGroupingParameters();
    void ComputeHoughLineMidPoints();
    void ComputeHoughLineLengths();
    void DrawHoughLineMidPoints(Mat &rgb);
    void DrawHoughLines(Mat &rgb);
    void DrawHoughLineProximityThresholded(Mat &rgb);




};

#endif // PERCEPTU_ALGROUPING_H
