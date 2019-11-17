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
using namespace valid_line_point_search;
using namespace mid_line_search;

class ValidLinePointSearch
{


private:



    Mat current_image_;



    const int kMinLeftToRightLineDistance_ =  120;
    const int kMaxLeftToRightLineDistance_ =  140;
    const int kMinLeftToMidLineDistance_ =  45;
    const int kMaxLeftToMidLineDistance_ =  70;

    const int kMinRightToLeftLineDistance_ =  120;
    const int kMaxRightToLeftLineDistance_ =  140;
    const int kMinRightToMidLineDistance_ =  45;
    const int kMaxRightToMidLineDistance_ =  70;

    const int kMinLeftToRightPixelIntensity_ = 99;
    const int kMinLeftToMidPixelIntensity_ = 99;
    const int kMinRightToLeftPixelIntensity_ = 99;
    const int kMinRightToMidPixelIntensity_ = 99;


    const int kMinLeftToRightLineWidth_ =  2;
    const int kMaxLeftToRightLineWidth_ =  5;
    const int kMinLeftToMidLineWidth_ =  2;
    const int kMaxLeftToMidLineWidth_ =  5;

    const int kMinRightToLeftLineWidth_ =  2;
    const int kMaxRightToLeftLineWidth_ =  5;
    const int kMinRightToMidLineWidth_ =  2;
    const int kMaxRightToMidLineWidth_ =  5;

    const int kImageWidth_ = 1280;
    const int kImageHeight_ = 417;

    vector<PointInDirection> left_line_directions_;
    vector<PointInDirection> right_line_directions_;

    vector<Point> left_line_follow_mid_line_points_;
    vector<Point> left_line_follow_right_line_points_;

    vector<Point> right_line_follow_mid_line_points_;
    vector<Point> right_line_follow_left_line_points_;


    void ClearMemory(int SEARCH_LINE_CODE);
    void SearchOrthogonalValues(int point_in_search_direction_x,
                                 int point_in_search_direction_y,
                                 float orthogonal_angle,
                                  vector<int>& orthogonal_line_activations,
                                  vector<Point>& orthogonal_line_points,
                                  int SEARCH_LINE_CODE);

    float GetOrthogonalAngle(float angle, int SEARCH_LINE_CODE);
    int GetPixelValue(int x, int y);
    int GetPixelValue(Point point);

    bool CheckLineMatch(vector<int> orthogonal_line_activations,SegmentStartIDAndWidth& line_match, int SEARCH_LINE_CODE);
    void SafeLinePoint(SegmentStartIDAndWidth mid_line_match, vector<Point> orthogonal_line_points, int SEARCH_LINE_CODE);

      SearchLineDistanceThresholds GetSearchLineDistanceThresholds(int SEARCH_LINE_CODE);
    int GetMinPixelIntensityThreshold(int SEARCH_LINE_CODE);

     SearchLineWidthThresholds GetSearchLineWidthThresholds(int SEARCH_LINE_CODE);

     vector<PointInDirection> GetLineDirections(int SEARCH_LINE_CODE);



public:
    ValidLinePointSearch();
    void SetImage(Mat image);
    void SetLine(vector<PointInDirection> line_directions, int START_LINE_CODE);
    void FindValidPointsFromLineFollow(int SEARCH_LINE_CODE);
    void DrawLinePoints(Mat &rgb, int SEARCH_LINE_CODE);
};

#endif // VALID_LINE_POINT_SEARCH_H
