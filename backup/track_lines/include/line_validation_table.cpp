#include "line_validation_table.h"

LineValidationTable::LineValidationTable(int line_code,Point origin,float search_direction, int next_direction_distance, Point adjacent_point_1, Point adjacent_point_2, int label)
{
    origin_    = origin;
    line_code_ = line_code;
    label_     = label;

    search_direction_        = search_direction;
    next_direction_distance_ = next_direction_distance;


    switch(line_code)
    {
        case LEFT_LINE:
                            mid_point_   =  adjacent_point_1;
                            right_point_ =  adjacent_point_2;
                            break;
        case MID_LINE:
                            left_point_  =  adjacent_point_1;
                            right_point_ =  adjacent_point_2;
                            break;
        case RIGHT_LINE:
                            left_point_ =  adjacent_point_1;
                            mid_point_  =  adjacent_point_2;
                            break;
    }

    left_id_ = -1;
    mid_id_ = -1;
    right_id_ = -1;


    left_prediction_ = false;
    mid_prediction_ = false;
    right_prediction_ = false;

    left_to_mid_directions_in_range_ = false;
    left_to_right_directions_in_range_ = false;
    mid_to_left_directions_in_range_ = false;
    mid_to_right_directions_in_range_ = false;
    right_to_left_directions_in_range_ = false;
    right_to_mid_directions_in_range_ = false;


    left_to_origin_prediction_ = false;
    mid_to_origin_prediction_ = false;
    right_to_origin_prediction_ = false;


    found_left_point_ = false;
    found_mid_point_ = false;
    found_right_point_ = false;

    score_ = 0;
}

Point LineValidationTable::GetAdjacentPointPrediction(int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return mid_point_;

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) return right_point_;

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return left_point_;

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return right_point_;

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return left_point_;

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return mid_point_;
 }


void LineValidationTable::SetAdjacentPointPredictionFound(int SEARCH_LINE_CODE, bool flag)
{
   if(SEARCH_LINE_CODE == LEFT_TO_MID) found_mid_point_ = flag;

   if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) found_right_point_ = flag;

   if(SEARCH_LINE_CODE == MID_TO_LEFT) found_left_point_ = flag;

   if(SEARCH_LINE_CODE == MID_TO_RIGHT) found_right_point_ = flag;

   if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) found_left_point_ = flag;

   if(SEARCH_LINE_CODE == RIGHT_TO_MID) found_mid_point_ = flag;
}

Point LineValidationTable::GetOriginPoint()
{
    return origin_;
}


Point LineValidationTable::GetLeftLinePointPrediction()
{
    return left_point_;
}

Point LineValidationTable::GetMidLinePointPrediction()
{
    return mid_point_;
}

Point LineValidationTable::GetRightLinePointPrediction()
{
    return right_point_;
}

void LineValidationTable::SetAdjacentPointPrediction(int SEARCH_LINE_CODE, bool flag)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) mid_prediction_ = flag;

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) right_prediction_ = flag;

    if(SEARCH_LINE_CODE == MID_TO_LEFT) left_prediction_ = flag;

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) right_prediction_ = flag;

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) left_prediction_ = flag;

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) mid_prediction_ = flag;
}



void LineValidationTable::SetAdjacentPointId(int SEARCH_LINE_CODE,int min_distance_adjacent_point_id)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) mid_id_ = min_distance_adjacent_point_id;

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) right_id_ = min_distance_adjacent_point_id;

    if(SEARCH_LINE_CODE == MID_TO_LEFT) left_id_ = min_distance_adjacent_point_id;

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) right_id_ = min_distance_adjacent_point_id;

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) left_id_ = min_distance_adjacent_point_id;

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) mid_id_ = min_distance_adjacent_point_id;
}

int LineValidationTable::GetDirection()
{
    return search_direction_;
}


void LineValidationTable::SetAdjacentPointDirectionInRange(int SEARCH_LINE_CODE, bool flag)
{

    if(SEARCH_LINE_CODE == LEFT_TO_MID) left_to_mid_directions_in_range_ = flag;

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) left_to_right_directions_in_range_ = flag;

    if(SEARCH_LINE_CODE == MID_TO_LEFT) mid_to_left_directions_in_range_ = flag;

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) mid_to_right_directions_in_range_ = flag;

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) right_to_left_directions_in_range_ = flag;

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) right_to_mid_directions_in_range_ = flag;
}


int LineValidationTable::GetAdjacent1Id(int LINE_CODE)
{
    if(LINE_CODE == LEFT_LINE) return mid_id_;
    if(LINE_CODE == MID_LINE) return left_id_;
    if(LINE_CODE == RIGHT_LINE) return left_id_;
}



int LineValidationTable::GetAdjacent2Id(int LINE_CODE)
{
    if(LINE_CODE == LEFT_LINE) return right_id_;
    if(LINE_CODE == MID_LINE) return right_id_;
    if(LINE_CODE == RIGHT_LINE) return mid_id_;
}

int LineValidationTable::GetAdjacent1Prediction(int LINE_CODE)
{
    if(LINE_CODE == LEFT_LINE) return mid_prediction_;
    if(LINE_CODE == MID_LINE) return left_prediction_;
    if(LINE_CODE == RIGHT_LINE) return left_prediction_;
}

int LineValidationTable::GetAdjacent2Prediction(int LINE_CODE)
{
    if(LINE_CODE == LEFT_LINE) return right_prediction_;
    if(LINE_CODE == MID_LINE) return right_prediction_;
    if(LINE_CODE == RIGHT_LINE) return mid_prediction_;
}

void LineValidationTable::SetAdjacentPointOriginPrediction(int SEARCH_LINE_CODE, bool flag)
{



    if(SEARCH_LINE_CODE == LEFT_TO_MID) mid_to_origin_prediction_ = flag;

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) right_to_origin_prediction_ = flag;

    if(SEARCH_LINE_CODE == MID_TO_LEFT) left_to_origin_prediction_ = flag;

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) right_to_origin_prediction_ = flag;

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) left_to_origin_prediction_ = flag;

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) mid_to_origin_prediction_ = flag;
}
