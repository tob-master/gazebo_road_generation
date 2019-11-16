#ifndef VALID_LINE_POINT_SEARCH_H
#define VALID_LINE_POINT_SEARCH_H

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
using namespace line_points_reducer;

class ValidLinePointSearch
{


private:
    Mat current_image_;

    const int kMinMidLineDistance_ =  45;
    const int kMaxMidLineDistance_ =  70;

    const int kMinLeftLineDistance_ = 120;
    const int kMaxLeftLineDistance_ = 140;

    const int kMinRightLineDistance_ = 120;
    const int kMaxRightLineDistance_ = 140;


    const int kImageHeight_ = 417;
    const int kImageWidth_ = 1280;


    const int kMinMidLineIntensity_ = 99;

    const int kMinMidLineWidth_ = 2;
    const int kMaxMidLineWidth_ = 5;

    vector<ReducedPointDirection> left_line_directions_;
    vector<ReducedPointDirection> right_line_directions_;

    vector<Point> left_line_follow_mid_line_points_;



    void SetImage(Mat image);
    void ClearMemory();
    void SearchOrthogonalValues(int point_in_search_direction_x,
                                 int point_in_search_direction_y,
                                 float orthogonal_angle,
                                  vector<int>& orthogonal_line_activations,
                                  vector<Point>& orthogonal_line_points);

    float GetOrthogonalAngle(float angle, int line);
    int GetPixelValue(int x, int y);
    int GetPixelValue(Point point);

    bool SearchMidLineMatch(vector<int> orthogonal_line_activations,pair<int,int>& mid_line_match);


public:
    ValidLinePointSearch();
    void FindValidPointsFromLeftLineFollow(Mat image, vector<ReducedPointDirection> left_line_directions);
    void DrawMidLinePoints(Mat &rgb);
};

#endif // VALID_LINE_POINT_SEARCH_H
