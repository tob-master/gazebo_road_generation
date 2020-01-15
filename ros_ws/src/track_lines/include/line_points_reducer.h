#ifndef LINE_POINTS_REDUCER_H
#define LINE_POINTS_REDUCER_H

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


#include "utils.h"
#include "datatypes.h"

//using namespace std;
//using namespace cv;



using namespace line_points_reducer;

class LinePointsReduce
{
    private:

    vector<RamerDouglasPeuckerTypePoint> left_line_points_;
    vector<RamerDouglasPeuckerTypePoint> right_line_points_;

    vector<RamerDouglasPeuckerTypePoint> left_line_points_reduced_;
    vector<RamerDouglasPeuckerTypePoint> right_line_points_reduced_;

    vector<PointInDirection> left_line_points_reduced_length_direction_;
    vector<PointInDirection> right_line_points_reduced_length_direction_;

    double max_distance_;

    bool left_line_is_reduced_;
    bool right_line_is_reduced_;

    void ClearMemory();

    void SetContainers(
    vector<line_follower::PointAndDirection> left_line,
    vector<line_follower::PointAndDirection> right_line);

    void SetMaxDistance(
    double max_distance);

    void ApplyRamerDouglasPeucker(
    const vector<RamerDouglasPeuckerTypePoint> &pointList,
    double epsilon, vector<RamerDouglasPeuckerTypePoint> &out);

    double GetPerpendicularDistance(
    const RamerDouglasPeuckerTypePoint &pt,
    const RamerDouglasPeuckerTypePoint &lineStart,
    const RamerDouglasPeuckerTypePoint &lineEnd);

    void ComputeLengthAndDirectionFromConsecutiveReducedLinePoints(
    int line);

    LinePointsReducerReturnInfo GetReturnInfo();

    public:

    LinePointsReduce(LinePointsReduceInitializationParameters init);

    LinePointsReducerReturnInfo ReduceLinePoints(
    vector<line_follower::PointAndDirection> left_line,
    vector<line_follower::PointAndDirection> right_line);

    void GetLengthAndDirectionFromConsecutiveReducedLinePoints(
    vector<PointInDirection> &line_points_reduced_length_direction,
    int line);

    void GetReducedLinePoints(
    vector<ReducedPoints> &line_points_reduced,
    int line);

    void DrawReducedLinePoints(
    Mat &rgb,
    int line);

    void CoutLengthAndDirectionFromConsecutiveReducedLinePoints();
};

#endif // LINE_POINTS_REDUCER_H
