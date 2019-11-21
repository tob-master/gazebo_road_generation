#ifndef LINE_VALIDATION_TABLE_H
#define LINE_VALIDATION_TABLE_H

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



class LineValidationTable
{
    private:
        Point origin_;
        int line_code_;
        int label_;

        float   search_direction_;
        int     next_direction_distance_;

        Point left_point_;
        Point mid_point_;
        Point right_point_;

        int left_id_;
        int mid_id_;
        int right_id_;

        bool left_prediction_;
        bool mid_prediction_;
        bool right_prediction_;

        bool left_mid_directions_in_range_;
        bool left_right_directions_in_range_;
        bool mid_left_directions_in_range_;
        bool mid_right_directions_in_range_;
        bool right_left_directions_in_range_;
        bool right_mid_directions_in_range_;

        bool left_to_origin_prediction_;
        bool mid_to_origin_prediction_;
        bool right_to_origin_prediction_;

        bool found_left_point_;
        bool found_mid_point_;
        bool found_right_point_;

        int score_;


    public:

        LineValidationTable(int line_code,Point origin,float search_direction, int next_direction_distance, Point adjacent_line_point_1, Point adjacent_line_point_2, int label);
        Point GetAdjacentPointPrediction(int SEARCH_LINE_CODE);
        void SetAdjacentPointPredictionFound(int SEARCH_LINE_CODE, bool flag);
        Point GetOriginPoint();
        int GetDirection();
        void SetAdjacentPointDirectionInRange(int SEARCH_LINE_CODE, bool flag);
        Point GetLeftLinePointPrediction();
        Point GetMidLinePointPrediction();
        Point GetRightLinePointPrediction();

        void SetAdjacentPointPrediction(int SEARCH_LINE_CODE, bool flag);
        void SetAdjacentPointId(int SEARCH_LINE_CODE,int min_distance_adjacent_point_id);
        void SetAdjacentPointOriginPrediction(int SEARCH_LINE_CODE, bool flag);

        bool GetLeftPrediction(){return left_prediction_;};
        bool GetMidPrediction(){return mid_prediction_;};
        bool GetRightPrediction(){return right_prediction_;};

        int GetLeftPointId(){ return left_id_;};
        int GetMidPointId(){ return mid_id_;};
        int GetRightPointId(){ return right_id_;};

        int GetAdjacent1Id(int LINE_CODE);
        int GetAdjacent2Id(int LINE_CODE);

        int GetAdjacent1Prediction(int LINE_CODE);
        int GetAdjacent2Prediction(int LINE_CODE);
};

#endif // LINE_VALIDATION_TABLE_H
