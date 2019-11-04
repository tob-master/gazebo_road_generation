#ifndef LINE_POINT_REDUCER_H
#define LINE_POINT_REDUCER_H

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


#include "own_utils.h"
#include "own_datatypes.h"

using namespace std;
using namespace cv;



using namespace line_points_reducer;

class LinePointsReducer
{
    private:
        vector<RamerDouglasPeuckerTypePoint> left_line_points_;
        vector<RamerDouglasPeuckerTypePoint> right_line_points_;

        vector<RamerDouglasPeuckerTypePoint> left_line_points_reduced_;
        vector<RamerDouglasPeuckerTypePoint> right_line_points_reduced_;

        vector<LengthAndDirectionFromConsecutiveReducedLinePoints> left_line_points_reduced_length_direction_;
        vector<LengthAndDirectionFromConsecutiveReducedLinePoints> right_line_points_reduced_length_direction_;


        double max_distance_;


        void ClearMemory();
        void SetContainers(vector<line_follower::PointAndDirection> left_line, vector<line_follower::PointAndDirection> right_line);
        void SetMaxDistance(double max_distance);
        void ApplyRamerDouglasPeucker(const vector<RamerDouglasPeuckerTypePoint> &pointList, double epsilon, vector<RamerDouglasPeuckerTypePoint> &out);
        double GetPerpendicularDistance(const RamerDouglasPeuckerTypePoint &pt, const RamerDouglasPeuckerTypePoint &lineStart, const RamerDouglasPeuckerTypePoint &lineEnd);
        void ComputeLengthAndDirectionFromConsecutiveReducedLinePoints();

    public:
        LinePointsReducer();
        void ReduceLinePoints(vector<line_follower::PointAndDirection> left_line, vector<line_follower::PointAndDirection> right_line, double max_distance);
        void GetLengthAndDirectionFromConsecutiveReducedLinePoints(vector<LengthAndDirectionFromConsecutiveReducedLinePoints> &left_line_points_reduced_length_direction,
                                                                   vector<LengthAndDirectionFromConsecutiveReducedLinePoints> &right_line_points_reduced_length_direction);
        void GetReducedLinePoints(vector<ReducedPoints> &left_line_points_reduced, vector<ReducedPoints> &right_line_points_reduced);
        void DrawReducedLinePoints(Mat &rgb);

        void CoutLengthAndDirectionFromConsecutiveReducedLinePoints();
};

#endif // LINE_POINT_REDUCER_H
