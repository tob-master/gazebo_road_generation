#include "safe_drive_area_evaluation.h"



SafeDriveAreaEvaluation::SafeDriveAreaEvaluation(
int image_height,
int image_width,
SafeDriveAreaEvaluationInitializationParameters init):
kImageHeight_(image_height),
kImageWidth_(image_width),
kSearchRectWidth_(init.search_rect_width),
kSearchRectHeight_(init.search_rect_height),
kStartMidPointOfRectSafety_(init.start_of_rect_safety_x,init.start_of_rect_safety_y),
kStartSearchDirectionOfRectSafety_(init.start_search_direction_of_rect_safety),
kRectBorderDistanceThresholdForContinousLine_(init.rect_border_distance_threshold_for_continous_line),
kRectStepLength_(init.rect_step_length)
{

}

void SafeDriveAreaEvaluation::LoadLinePointsInDriveDirection(
vector<LineValidationTable> left_line_in_drive_direction_table,
vector<LineValidationTable>mid_line_in_drive_direction_table,
vector<LineValidationTable>right_line_in_drive_direction_table)
{
    left_line_in_drive_direction_table_ = left_line_in_drive_direction_table;
    mid_line_in_drive_direction_table_ = mid_line_in_drive_direction_table;
    right_line_in_drive_direction_table_ = right_line_in_drive_direction_table;
}

vector<SafeDriveAreaEvaluationReturnInfo> SafeDriveAreaEvaluation::EvaluateTrackInDriveDirection()
{
    FollowTrack(
    kStartSearchDirectionOfRectSafety_,
    kStartMidPointOfRectSafety_,
    kSearchRectWidth_,
    kSearchRectHeight_);

    return safe_drive_area_evaluation_return_info_vector_;
}

void SafeDriveAreaEvaluation::FollowTrack(
float search_direction,
Point rect_mid_point,
const int kSearchRectLength,
const int kSearchRectHeight)
{

    ClearAllFollowTrackTables(
    left_line_points_in_rect_ids_,
    mid_line_points_in_rect_ids_,
    right_line_points_in_rect_ids_,
    left_line_points_in_rect_,
    mid_line_points_in_rect_,
    right_line_points_in_rect_,
    left_priority_ids_,
    mid_priority_ids_,
    right_priority_ids_,
    left_priority_table_,
    mid_priority_table_,
    right_priority_table_,
    left_line_rect_safety_,
    mid_line_rect_safety_,
    right_line_rect_safety_);

    vector<vector<Point>> search_rect = GetSearchRect(
                                        rect_mid_point,
                                        search_direction,
                                        kSearchRectLength,
                                        kSearchRectHeight);

    GetLinesPointsInRect(
    left_line_in_drive_direction_table_,
    search_rect,
    left_line_points_in_rect_ids_,
    left_line_points_in_rect_);

    GetLinesPointsInRect(
    mid_line_in_drive_direction_table_,
    search_rect,
    mid_line_points_in_rect_ids_,
    mid_line_points_in_rect_);

    GetLinesPointsInRect(
    right_line_in_drive_direction_table_,
    search_rect,
    right_line_points_in_rect_ids_,
    right_line_points_in_rect_);

    FillPriorityTables(
    left_line_in_drive_direction_table_,
    left_line_points_in_rect_ids_,
    mid_line_in_drive_direction_table_,
    mid_line_points_in_rect_ids_,
    right_line_in_drive_direction_table_,
    right_line_points_in_rect_ids_,
    left_priority_table_,
    mid_priority_table_,
    right_priority_table_,
    left_priority_ids_,
    mid_priority_ids_,
    right_priority_ids_);

    CheckRectSafety(
    search_rect,
    left_line_points_in_rect_,
    left_priority_table_,
    left_line_rect_safety_,
    kSearchRectHeight,
    kRectBorderDistanceThresholdForContinousLine_);

    CheckRectSafety(
    search_rect,
    mid_line_points_in_rect_,
    mid_priority_table_,
    mid_line_rect_safety_,
    kSearchRectHeight,
    kRectBorderDistanceThresholdForContinousLine_);

    CheckRectSafety(
    search_rect,
    right_line_points_in_rect_,
    right_priority_table_,
    right_line_rect_safety_,
    kSearchRectHeight,
    kRectBorderDistanceThresholdForContinousLine_);

    GatherSafeDriveAreaEvaluationTableReturInfo(left_priority_table_,
    mid_priority_table_,
    right_priority_table_,
    left_line_rect_safety_,
    mid_line_rect_safety_,
    right_line_rect_safety_,
    safe_drive_area_evaluation_return_info_vector_,
    search_direction,
    rect_mid_point,
    priority_0_multiplier_,
    priority_1_2_multiplier_,
    priority_3_4_multiplier_,
    priority_5_multiplier_,
    priority_6_7_multiplier_);

    float new_search_direction = -1;

    GetNewSearchDirection(
    safe_drive_area_evaluation_return_info_vector_,
    new_search_direction);

    if(new_search_direction == -1)
    {new_search_direction = search_direction;}

    Point new_rect_mid_point;

    GetNewRectMidPoint(
    new_search_direction,
    rect_mid_point,
    new_rect_mid_point,
    kRectStepLength_);

    if(RectMidPointOutOfImage(
      new_rect_mid_point,
      kImageWidth_,
      kImageHeight_))
    {return;}

    FollowTrack(
    new_search_direction,
    new_rect_mid_point,
    kSearchRectLength,
    kSearchRectHeight);

}



void SafeDriveAreaEvaluation::ClearAllFollowTrackTables(
vector<int> &left_line_points_in_rect_ids,
vector<int> &mid_line_points_in_rect_ids,
vector<int> &right_line_points_in_rect_ids,
vector<LineValidationTable> &left_line_points_in_rect,
vector<LineValidationTable> &mid_line_points_in_rect,
vector<LineValidationTable> &right_line_points_in_rect,
vector<vector<int>> &left_priority_ids,
vector<vector<int>> &mid_priority_ids,
vector<vector<int>> &right_priority_ids,
vector<vector<LineValidationTable>> &left_priority_table,
vector<vector<LineValidationTable>> &mid_priority_table,
vector<vector<LineValidationTable>> &right_priority_table,
RectSafetyTable &left_line_rect_safety,
RectSafetyTable &mid_line_rect_safety,
RectSafetyTable &right_line_rect_safety)
{
    left_line_points_in_rect_ids.clear();
    mid_line_points_in_rect_ids.clear();
    right_line_points_in_rect_ids.clear();

    left_line_points_in_rect.clear();
    mid_line_points_in_rect.clear();
    right_line_points_in_rect.clear();

    for(auto &it: left_priority_ids) it.clear();
    for(auto &it: mid_priority_ids) it.clear();
    for(auto &it: right_priority_ids) it.clear();

    for(auto &it: left_priority_table) it.clear();
    for(auto &it: mid_priority_table) it.clear();
    for(auto &it: right_priority_table) it.clear();

    EmtpySafetyTable(left_line_rect_safety);
    EmtpySafetyTable(mid_line_rect_safety);
    EmtpySafetyTable(right_line_rect_safety);
}


vector<vector<Point>> SafeDriveAreaEvaluation::GetSearchRect(
Point rect_mid,
float search_direction,
const int kSearchRectLength,
const int kSearchRectHeight)
{

    float rect_length_radius = kSearchRectLength / 2;
    float rect_height_radius = kSearchRectHeight / 2;

    int length_to_corner = sqrt(pow(rect_length_radius,2)+pow(rect_height_radius,2));
    int corner_angle = 90 - atan(rect_height_radius/rect_length_radius) * 180/PI;

    int rect_top_right_angle =  search_direction - corner_angle;
    int rect_top_left_angle =  search_direction + corner_angle;
    int rect_bottom_left_angle = search_direction + 180 - corner_angle;
    int rect_bottom_right_angle = search_direction + 180 + corner_angle;

    if(rect_bottom_right_angle > 359) rect_top_right_angle %= 360;
    if(rect_top_right_angle < 0)   rect_top_right_angle = 360 - abs(rect_top_right_angle);

    if(rect_top_left_angle > 359) rect_top_left_angle %= 360;
    if(rect_top_left_angle < 0)   rect_top_left_angle = 360 - abs(rect_top_left_angle);

    if(rect_bottom_left_angle > 359) rect_bottom_left_angle %= 360;
    if(rect_bottom_left_angle < 0)   rect_bottom_left_angle = 360 - abs(rect_bottom_left_angle);

    if(rect_bottom_right_angle > 359) rect_bottom_right_angle %= 360;
    if(rect_bottom_right_angle < 0)   rect_bottom_right_angle = 360 - abs(rect_bottom_right_angle);

    float rect_top_right_angle_f    = rect_top_right_angle * (PI/180);
    float rect_top_left_angle_f     = rect_top_left_angle * (PI/180);
    float rect_bottom_left_angle_f  = rect_bottom_left_angle * (PI/180);
    float rect_bottom_right_angle_f = rect_bottom_right_angle * (PI/180);

    int rect_top_left_x_offset = length_to_corner * cos(rect_top_right_angle_f);
    int rect_top_left_y_offset = -length_to_corner * sin(rect_top_right_angle_f);
    Point rect_top_left(rect_mid.x + rect_top_left_x_offset, rect_mid.y + rect_top_left_y_offset);

    int rect_top_right_x_offset = length_to_corner * cos(rect_top_left_angle_f);
    int rect_top_right_y_offset = -length_to_corner * sin(rect_top_left_angle_f);
    Point rect_top_right(rect_mid.x + rect_top_right_x_offset, rect_mid.y + rect_top_right_y_offset);

    int rect_bottom_left_x_offset = length_to_corner * cos(rect_bottom_left_angle_f);
    int rect_bottom_left_y_offset = -length_to_corner * sin(rect_bottom_left_angle_f);
    Point rect_bottom_left(rect_mid.x + rect_bottom_left_x_offset, rect_mid.y + rect_bottom_left_y_offset);

    int rect_bottom_right_x_offset = length_to_corner * cos(rect_bottom_right_angle_f);
    int rect_bottom_right_y_offset = -length_to_corner * sin(rect_bottom_right_angle_f);
    Point rect_bottom_right(rect_mid.x + rect_bottom_right_x_offset, rect_mid.y + rect_bottom_right_y_offset);

    vector<vector<Point>> contours(1);

    contours[0].push_back(rect_top_left);
    contours[0].push_back(rect_top_right);
    contours[0].push_back(rect_bottom_left);
    contours[0].push_back(rect_bottom_right);

    return contours;
}

void SafeDriveAreaEvaluation::GetLinesPointsInRect(
vector<LineValidationTable> line_direction_in_range,
vector<vector<Point>> contours,
vector<int>& line_points_in_rect_id,
vector<LineValidationTable>& line_points_in_rect)
{
    for(int i=0; i<line_direction_in_range.size(); i++)
    {
        Point origin = line_direction_in_range[i].GetOriginPoint();
        double res = pointPolygonTest(contours[0], origin, false);
        if(res>=0)
        {
            line_points_in_rect_id.push_back(i);


            line_points_in_rect.push_back(line_direction_in_range[i]);

        }
    }
}


void SafeDriveAreaEvaluation::FillPriorityTables(
vector<LineValidationTable> left_line_in_drive_direction_table,
vector<int> left_line_points_in_rect_ids,
vector<LineValidationTable> mid_line_in_drive_direction_table,
vector<int> mid_line_points_in_rect_ids,
vector<LineValidationTable> right_line_in_drive_direction_table,
vector<int> right_line_points_in_rect_ids,
vector<vector<LineValidationTable>> &left_priority_table,
vector<vector<LineValidationTable>> &mid_priority_table,
vector<vector<LineValidationTable>> &right_priority_table,
vector<vector<int>> &left_priority_ids,
vector<vector<int>> &mid_priority_ids,
vector<vector<int>> &right_priority_ids)
{


    for(int i=0; i<left_line_points_in_rect_ids.size(); i++)
    {
        LineValidationTable left_table = left_line_in_drive_direction_table[left_line_points_in_rect_ids[i]];

        bool found_mid_point = left_table.GetFoundMidPoint();
        bool mid_prediction  = left_table.GetMidPrediction();
        bool found_right_point = left_table.GetFoundRightPoint();
        bool right_prediction = left_table.GetRightPrediction();
        bool left_to_mid_directions_in_range = left_table.GetLeftToMidDirectionsInRange();
        bool left_to_right_directions_in_range = left_table.GetLeftToRightDirectionsInRange();

        FillPriorityTable(
        left_table,
        i,
        found_mid_point,
        found_right_point,
        mid_prediction,
        right_prediction,
        left_to_mid_directions_in_range,
        left_to_right_directions_in_range,
        left_priority_ids,
        left_priority_table);

    }

    for(int i=0; i<mid_line_points_in_rect_ids.size(); i++)
    {

        LineValidationTable mid_table = mid_line_in_drive_direction_table[mid_line_points_in_rect_ids[i]];

        bool found_left_point = mid_table.GetFoundLeftPoint();
        bool left_prediction  = mid_table.GetLeftPrediction();
        bool found_right_point = mid_table.GetFoundRightPoint();
        bool right_prediction = mid_table.GetRightPrediction();
        bool mid_to_left_directions_in_range = mid_table.GetMidToLeftDirectionsInRange();
        bool mid_to_right_directions_in_range = mid_table.GetMidToRightDirectionsInRange();

        FillPriorityTable(
        mid_table,
        i,
        found_left_point,
        found_right_point,
        left_prediction,right_prediction,
        mid_to_left_directions_in_range,
        mid_to_right_directions_in_range,
        mid_priority_ids,
        mid_priority_table);
    }

    for(int i=0; i<right_line_points_in_rect_ids.size(); i++)
    {
       LineValidationTable right_table =  right_line_in_drive_direction_table[right_line_points_in_rect_ids[i]];

       bool found_left_point = right_table.GetFoundLeftPoint();
       bool left_prediction  = right_table.GetLeftPrediction();
       bool found_mid_point = right_table.GetFoundMidPoint();
       bool mid_prediction = right_table.GetMidPrediction();
       bool right_to_left_directions_in_range = right_table.GetRightToLeftDirectionsInRange();
       bool right_to_mid_directions_in_range = right_table.GetRightToMidDirectionsInRange();

       FillPriorityTable(
        right_table,
        i,
        found_left_point,found_mid_point,
        left_prediction,
        mid_prediction,
        right_to_left_directions_in_range,
        right_to_mid_directions_in_range,
        right_priority_ids,
        right_priority_table);

    }
}


void SafeDriveAreaEvaluation::FillPriorityTable(
LineValidationTable table,
int i,
bool found_point1,
bool found_point2,
bool prediction1,
bool prediction2,
bool directions_in_range1,
bool directions_in_range2,
vector<vector<int>>& priority_ids,
vector<vector<LineValidationTable>>& priority_table)
{

    if(prediction1 && prediction2)
    {
        priority_ids[0].push_back(i);
        priority_table[0].push_back(table);
        return;
    }


    if(prediction1 && found_point2 || prediction2 && found_point1)
    {
        if(prediction1 && found_point2 )
        {
            priority_ids[1].push_back(i);
            priority_table[1].push_back(table);
        }

        if (prediction2 && found_point1)
        {
            priority_ids[2].push_back(i);
            priority_table[2].push_back(table);
        }

        return;
    }


    if(prediction1 || prediction2)
    {
        if(prediction1)
        {
            priority_ids[3].push_back(i);
            priority_table[3].push_back(table);
        }

        if(prediction2)
        {
            priority_ids[4].push_back(i);
            priority_table[4].push_back(table);
        }

        return;
    }


    if(found_point1 && found_point2)
    {
        priority_ids[5].push_back(i);
        priority_table[5].push_back(table);
        return;
    }


    if(found_point1 || found_point2)
    {
        if(found_point1)
        {
            priority_ids[6].push_back(i);
            priority_table[6].push_back(table);
        }

        if(found_point2)
        {
            priority_ids[7].push_back(i);
            priority_table[7].push_back(table);
        }

        return;
    }
    return;
}


void SafeDriveAreaEvaluation::CheckRectSafety(
vector<vector<Point>> search_rect,
vector<LineValidationTable>line_points_in_rect,
vector<vector<LineValidationTable>> priority_table,
RectSafetyTable& rect_safety,
const int kSearchRectHeight,
const int kRectBorderDistanceThreshold)
{
    if(line_points_in_rect.size()>0)
    {
        float all_percent_found = (line_points_in_rect.size() / kSearchRectHeight) * 100;

        float line_points_count = 0;

        if(line_points_in_rect.size() > kSearchRectHeight)
        {
            line_points_count = line_points_in_rect.size();
        }
        else {

            line_points_count = kSearchRectHeight;
        }

        float prio_0_percent_found = (priority_table[0].size() / line_points_count) * 100;
        float prio_1_percent_found = (priority_table[1].size() / line_points_count) * 100;
        float prio_2_percent_found = (priority_table[2].size() / line_points_count) * 100;
        float prio_3_percent_found = (priority_table[3].size() / line_points_count) * 100;
        float prio_4_percent_found = (priority_table[4].size() / line_points_count) * 100;
        float prio_5_percent_found = (priority_table[5].size() / line_points_count) * 100;
        float prio_6_percent_found = (priority_table[6].size() / line_points_count) * 100;
        float prio_7_percent_found = (priority_table[7].size() / line_points_count) * 100;

        MinMaxLineElements rect_min_max;

        ExtractMinMaxLineElements(
        line_points_in_rect,
        rect_min_max );

        Point x_min = rect_min_max.x_min;
        Point y_min = rect_min_max.y_min;
        Point x_max = rect_min_max.x_max;
        Point y_max = rect_min_max.y_max;


        double res = 0;

        bool y_min_in_rect_border_range = false;
        bool y_max_in_rect_border_range = false;

        res = pointPolygonTest(
              search_rect[0],
              y_min,
              true);

        if(res>=0 && res < kRectBorderDistanceThreshold)
        {
            y_min_in_rect_border_range = true;
        }

        res = pointPolygonTest(
              search_rect[0],
              y_max,
              true);

        if(res>=0 && res < kRectBorderDistanceThreshold)
        {
            y_max_in_rect_border_range = true;

        }

        rect_safety.percent_points_with_priority_0 = prio_0_percent_found;
        rect_safety.percent_points_with_priority_1 = prio_1_percent_found;
        rect_safety.percent_points_with_priority_2 = prio_2_percent_found;
        rect_safety.percent_points_with_priority_3 = prio_3_percent_found;
        rect_safety.percent_points_with_priority_4 = prio_4_percent_found;
        rect_safety.percent_points_with_priority_5 = prio_5_percent_found;
        rect_safety.percent_points_with_priority_6 = prio_6_percent_found;
        rect_safety.percent_points_with_priority_7 = prio_7_percent_found;

        rect_safety.percent_points_in_rect = all_percent_found;

        rect_safety.y_min_in_rect_border_range = y_min_in_rect_border_range;
        rect_safety.y_max_in_rect_border_range = y_max_in_rect_border_range;
    }
    else{
        EmtpySafetyTable(
        rect_safety);
    }

}

void SafeDriveAreaEvaluation::GatherSafeDriveAreaEvaluationTableReturInfo(
vector<vector<LineValidationTable>> left_priority_table,
vector<vector<LineValidationTable>> mid_priority_table,
vector<vector<LineValidationTable>> right_priority_table,
RectSafetyTable left_line_rect_safety,
RectSafetyTable mid_line_rect_safety,
RectSafetyTable right_line_rect_safety,
vector<SafeDriveAreaEvaluationReturnInfo>& SafeDriveAreaEvaluationReturnInfoVector_,
const float search_direction,
const Point rect_mid_point,
const long int priority_0_multiplier,
const long int priority_1_2_multiplier,
const int priority_3_4_multiplier,
const int priority_5_multiplier,
const int priority_6_7_multiplier)
{


    bool left_line_is_safe = false;
    bool mid_line_is_safe = false;
    bool right_line_is_safe = false;

    if(!left_line_rect_safety.too_few_points_in_rect &&
       left_line_rect_safety.y_min_in_rect_border_range &&
       left_line_rect_safety.y_max_in_rect_border_range)
    {
        left_line_is_safe = true;
    }

    if(!mid_line_rect_safety.too_few_points_in_rect &&
       mid_line_rect_safety.y_min_in_rect_border_range &&
       mid_line_rect_safety.y_max_in_rect_border_range)
    {
        mid_line_is_safe = true;
    }

    if(!right_line_rect_safety.too_few_points_in_rect &&
       right_line_rect_safety.y_min_in_rect_border_range &&
       right_line_rect_safety.y_max_in_rect_border_range)
    {
        right_line_is_safe = true;
    }

    int left_score_priority_0 = ceil(left_line_rect_safety.percent_points_with_priority_0);
    int mid_score_priority_0 = ceil(mid_line_rect_safety.percent_points_with_priority_0);
    int right_score_priority_0 = ceil(right_line_rect_safety.percent_points_with_priority_0);

    int left_score_priority_1 = ceil(left_line_rect_safety.percent_points_with_priority_1);
    int mid_score_priority_1 = ceil(mid_line_rect_safety.percent_points_with_priority_1);
    int right_score_priority_1 = ceil(right_line_rect_safety.percent_points_with_priority_1);

    int left_score_priority_2 = ceil(left_line_rect_safety.percent_points_with_priority_2);
    int mid_score_priority_2 = ceil(mid_line_rect_safety.percent_points_with_priority_2);
    int right_score_priority_2 = ceil(right_line_rect_safety.percent_points_with_priority_2);

    int left_score_priority_3 =ceil( left_line_rect_safety.percent_points_with_priority_3);
    int mid_score_priority_3 = ceil(mid_line_rect_safety.percent_points_with_priority_3);
    int right_score_priority_3 = ceil(right_line_rect_safety.percent_points_with_priority_3);

    int left_score_priority_4 = ceil(left_line_rect_safety.percent_points_with_priority_4);
    int mid_score_priority_4 = ceil(mid_line_rect_safety.percent_points_with_priority_4);
    int right_score_priority_4 = ceil(right_line_rect_safety.percent_points_with_priority_4);

    int left_score_priority_5 = ceil(left_line_rect_safety.percent_points_with_priority_5);
    int mid_score_priority_5 = ceil(mid_line_rect_safety.percent_points_with_priority_5);
    int right_score_priority_5 = ceil(left_line_rect_safety.percent_points_with_priority_5);

    int left_score_priority_6 = ceil(left_line_rect_safety.percent_points_with_priority_6);
    int mid_score_priority_6 = ceil(mid_line_rect_safety.percent_points_with_priority_6);
    int right_score_priority_6 = ceil(right_line_rect_safety.percent_points_with_priority_6);

    int left_score_priority_7 = ceil(left_line_rect_safety.percent_points_with_priority_7);
    int mid_score_priority_7 = ceil(mid_line_rect_safety.percent_points_with_priority_7);
    int right_score_priority_7 = ceil(right_line_rect_safety.percent_points_with_priority_7);

    unsigned long long int left_line_score = 0;
    unsigned long long int mid_line_score = 0;
    unsigned long long int right_line_score = 0;
    unsigned long long int trackscore = 0;

    left_line_score += (left_score_priority_0<100)  ?  left_score_priority_0* priority_0_multiplier : 99* priority_0_multiplier;
    left_line_score += (left_score_priority_1<100)  ?  left_score_priority_1* priority_1_2_multiplier : 99* priority_1_2_multiplier;
    left_line_score += (left_score_priority_2<100)  ?  left_score_priority_2* priority_1_2_multiplier : 99* priority_1_2_multiplier;
    left_line_score += (left_score_priority_3<100)  ?  left_score_priority_3* priority_3_4_multiplier : 99* priority_3_4_multiplier;
    left_line_score += (left_score_priority_4<100) ? left_score_priority_4*priority_3_4_multiplier : 99*priority_3_4_multiplier;
    left_line_score += (left_score_priority_5<100) ? left_score_priority_5*priority_5_multiplier : 99*priority_5_multiplier;
    left_line_score += (left_score_priority_6<100) ? left_score_priority_6*priority_6_7_multiplier : 99*priority_6_7_multiplier;
    left_line_score += (left_score_priority_7<100) ? left_score_priority_7*priority_6_7_multiplier : 99*priority_6_7_multiplier;

    mid_line_score += (mid_score_priority_0<100)  ?  mid_score_priority_0* priority_0_multiplier : 99* priority_0_multiplier;
    mid_line_score += (mid_score_priority_1<100)  ?  mid_score_priority_1* priority_1_2_multiplier : 99* priority_1_2_multiplier;
    mid_line_score += (mid_score_priority_2<100)  ?  mid_score_priority_2* priority_1_2_multiplier : 99* priority_1_2_multiplier;
    mid_line_score += (mid_score_priority_3<100)  ?  mid_score_priority_3* priority_3_4_multiplier : 99* priority_3_4_multiplier;
    mid_line_score += (mid_score_priority_4<100) ? mid_score_priority_4*priority_3_4_multiplier : 99*priority_3_4_multiplier;
    mid_line_score += (mid_score_priority_5<100) ? mid_score_priority_5*priority_5_multiplier : 99*priority_5_multiplier;
    mid_line_score += (mid_score_priority_6<100) ? mid_score_priority_6*priority_6_7_multiplier : 99*priority_6_7_multiplier;
    mid_line_score += (mid_score_priority_7<100) ? mid_score_priority_7*priority_6_7_multiplier : 99*priority_6_7_multiplier;

    right_line_score += (right_score_priority_0<100)  ?  right_score_priority_0* priority_0_multiplier : 99* priority_0_multiplier;
    right_line_score += (right_score_priority_1<100)  ?  right_score_priority_1* priority_1_2_multiplier : 99* priority_1_2_multiplier;
    right_line_score += (right_score_priority_2<100)  ?  right_score_priority_2* priority_1_2_multiplier : 99* priority_1_2_multiplier;
    right_line_score += (right_score_priority_3<100)  ?  right_score_priority_3* priority_3_4_multiplier : 99* priority_3_4_multiplier;
    right_line_score += (right_score_priority_4<100) ? right_score_priority_4*priority_3_4_multiplier : 99*priority_3_4_multiplier;
    right_line_score += (right_score_priority_5<100) ? right_score_priority_5*priority_5_multiplier : 99*priority_5_multiplier;
    right_line_score += (right_score_priority_6<100) ? right_score_priority_6*priority_6_7_multiplier : 99*priority_6_7_multiplier;
    right_line_score += (right_score_priority_7<100) ? right_score_priority_7*priority_6_7_multiplier : 99*priority_6_7_multiplier;

    unsigned long long int tmp_score = 0;

    tmp_score = (left_score_priority_0+mid_score_priority_0+right_score_priority_0)/3;
    trackscore += (tmp_score<100) ? tmp_score*priority_0_multiplier : 99*priority_0_multiplier;

    tmp_score = (left_score_priority_1+mid_score_priority_1+right_score_priority_1)/3;
    trackscore += (tmp_score<100) ? tmp_score*priority_1_2_multiplier : 99*priority_1_2_multiplier;

    tmp_score = (left_score_priority_2+mid_score_priority_2+right_score_priority_2)/3;
    trackscore += (tmp_score<100) ? tmp_score*priority_1_2_multiplier : 99*priority_1_2_multiplier;

    tmp_score = (left_score_priority_3+mid_score_priority_3+right_score_priority_3)/3;
    trackscore += (tmp_score<100) ? tmp_score*priority_3_4_multiplier : 99*priority_3_4_multiplier;

    tmp_score = (left_score_priority_4+mid_score_priority_4+right_score_priority_4)/3;
    trackscore += (tmp_score<100) ? tmp_score*priority_3_4_multiplier : 99*priority_3_4_multiplier;

    tmp_score = (left_score_priority_5+mid_score_priority_5+right_score_priority_5)/3;
    trackscore += (tmp_score<100) ? tmp_score*priority_5_multiplier : 99*priority_5_multiplier;

    tmp_score = (left_score_priority_6+mid_score_priority_6+right_score_priority_6)/3;
    trackscore += (tmp_score<100) ? tmp_score*priority_6_7_multiplier : 99*priority_6_7_multiplier;

    tmp_score = (left_score_priority_7+mid_score_priority_7+right_score_priority_7)/3;
    trackscore += (tmp_score<100) ? tmp_score*priority_6_7_multiplier : 99*priority_6_7_multiplier;


    //if(left_line_is_safe){ left_line_score += METRIC_CONTINOUS_VAL; trackscore += METRIC_CONTINOUS_VAL;}
    //if(mid_line_is_safe) { mid_line_score += METRIC_CONTINOUS_VAL; trackscore += METRIC_CONTINOUS_VAL;}
    //if(right_line_is_safe) { right_line_score += METRIC_CONTINOUS_VAL; trackscore += METRIC_CONTINOUS_VAL;}

/*
    cout <<"left_line_score: "   << left_line_score     << " " << CountDigits(left_line_score) <<endl;
    cout <<"mid_line_score: "   << mid_line_score     << " " << CountDigits(mid_line_score) <<endl;
    cout <<"right_line_score: "   << right_line_score     << " " << CountDigits(right_line_score) <<endl;
    cout <<"TRACKS: "   << trackscore << " " << CountDigits(trackscore) <<endl;
*/

    vector<unsigned long long int> line_scores{left_line_score,mid_line_score,right_line_score};

    auto max_score_it = std::max_element(line_scores.begin(),line_scores.end());
    int max_score_id = std::distance(line_scores.begin(), max_score_it);

    int max_line_type = -1;

    int max_line_score = CountDigits(line_scores[max_score_id]);
    bool max_scored_line_is_continous = false;

    switch(max_score_id)
    {
        case LEFT_LINE:{
                            max_scored_line_is_continous = left_line_is_safe;
                            max_line_type = LEFT_LINE;
                            break;}
        case MID_LINE:{
                            max_scored_line_is_continous = mid_line_is_safe;
                            max_line_type = MID_LINE;
                            break;}
        case RIGHT_LINE:{
                            max_scored_line_is_continous = right_line_is_safe;
                            max_line_type = RIGHT_LINE;
                            break;}
        default:
                            cout << "NO MAX ???" << endl;
                            exit(0);
    }

    vector<LineValidationTable> left_safest_table1, left_safest_table2,
                                mid_safest_table1, mid_safest_table2,
                                right_safest_table1, right_safest_table2;

    GetSafestTables(left_safest_table1,left_safest_table2,left_priority_table,left_line_score);
    GetSafestTables(mid_safest_table1,mid_safest_table2,mid_priority_table,mid_line_score);
    GetSafestTables(right_safest_table1,right_safest_table2,right_priority_table,right_line_score);


    SafeDriveAreaEvaluationReturnInfoVector_.push_back(SafeDriveAreaEvaluationReturnInfo{
                                                      left_line_score,
                                                      mid_line_score,
                                                      right_line_score,
                                                      trackscore,
                                                      max_line_type,
                                                      max_line_score,
                                                      max_scored_line_is_continous,
                                                      left_line_is_safe,
                                                      mid_line_is_safe,
                                                      right_line_is_safe,
                                                      left_safest_table1,
                                                      left_safest_table2,
                                                      mid_safest_table1,
                                                      mid_safest_table2,
                                                      right_safest_table1,
                                                      right_safest_table2,
                                                      (int)search_direction,
                                                      rect_mid_point,
                                                      left_priority_table,
                                                      mid_priority_table,
                                                      right_priority_table,});


}

int SafeDriveAreaEvaluation::CountDigits(
unsigned long long int n)
{
    if(n<=0) return 1;
    else     return floor(log10(n) + 1);

}


void SafeDriveAreaEvaluation::GetNewSearchDirection(
vector<SafeDriveAreaEvaluationReturnInfo> safe_drive_area_evaluation_return_info_vector_,
float& search_direction)
{

    int max_line_type = safe_drive_area_evaluation_return_info_vector_[safe_drive_area_evaluation_return_info_vector_.size() - 1].max_line_type;

    vector<pair<int,int>> left_safest_directions1, left_safest_directions2,
                            mid_safest_directions1,  mid_safest_directions2,
                            right_safest_directions1, right_safest_directions2;



    GetSafestDirections(safe_drive_area_evaluation_return_info_vector_[safe_drive_area_evaluation_return_info_vector_.size() - 1].left_safest_table1,
                        safe_drive_area_evaluation_return_info_vector_[safe_drive_area_evaluation_return_info_vector_.size() - 1].left_safest_table2,
                        left_safest_directions1,left_safest_directions2);


    GetSafestDirections(safe_drive_area_evaluation_return_info_vector_[safe_drive_area_evaluation_return_info_vector_.size() - 1].mid_safest_table1,
                        safe_drive_area_evaluation_return_info_vector_[safe_drive_area_evaluation_return_info_vector_.size() - 1].mid_safest_table2,
                        mid_safest_directions1,mid_safest_directions2);


    GetSafestDirections(safe_drive_area_evaluation_return_info_vector_[safe_drive_area_evaluation_return_info_vector_.size() - 1].right_safest_table1,
                        safe_drive_area_evaluation_return_info_vector_[safe_drive_area_evaluation_return_info_vector_.size() - 1].right_safest_table2,
                        right_safest_directions1,right_safest_directions2);
/*
    cout << "LEFT DIRS" << endl;
    for(auto it:left_safest_directions1) cout << it.first << " " << it.second << endl;
    for(auto it:left_safest_directions2) cout << it.first << " " << it.second << endl;

    cout << "MID DIRS" << endl;
    for(auto it:mid_safest_directions1) cout << it.first << " " << it.second << endl;
    for(auto it:mid_safest_directions2) cout << it.first << " " << it.second << endl;

    cout << "RIGHT DIRS" << endl;
    for(auto it:right_safest_directions1) cout << it.first << " " << it.second << endl;
    for(auto it:right_safest_directions2) cout << it.first << " " << it.second << endl;
*/
    vector<pair<int,int>> safest_directions1, safest_directions2;

    if(max_line_type==LEFT_LINE)
    {
        safest_directions1=left_safest_directions1;
        safest_directions2=left_safest_directions2;
    }
    else if(max_line_type==MID_LINE)
    {
        safest_directions1=mid_safest_directions1 ;
        safest_directions2=mid_safest_directions2;
    }
    else if(max_line_type==RIGHT_LINE)
    {
        safest_directions1=right_safest_directions1;
        safest_directions2=right_safest_directions2;
    }
    else{ cout << "unbelievable value!" << endl; exit(0);}



    if(safest_directions1.size()>0 && safest_directions2.size()>0)
    {
        search_direction = (safest_directions1[safest_directions1.size()-1].first + safest_directions2[safest_directions2.size()-1].first) / 2;
    }
    else if(safest_directions1.size()>0)
    {
        search_direction = safest_directions1[safest_directions1.size()-1].first;
    }
    else if(safest_directions2.size()>0)
    {
        search_direction = safest_directions2[safest_directions2.size()-1].first;
    }
    else
    {
        search_direction = -1;
    }

}

void SafeDriveAreaEvaluation::GetNewRectMidPoint(
float new_search_direction,
Point rect_mid_point,
Point& new_rect_mid_point,
const int kRectStepLength)
{
    int new_search_direction_i = new_search_direction;

    if(new_search_direction_i > 359) new_search_direction_i %= 360;
    if(new_search_direction_i < 0)   new_search_direction_i = 360 - abs(new_search_direction_i);

    new_search_direction  = new_search_direction_i * (PI/180);
    int x_offset = kRectStepLength * cos(new_search_direction);
    int y_offset = -kRectStepLength * sin(new_search_direction);

    Point point(rect_mid_point.x + x_offset, rect_mid_point.y + y_offset);

    new_rect_mid_point = point;

}


bool SafeDriveAreaEvaluation::RectMidPointOutOfImage(
Point rect_mid_point,
const int kImageWidth,
const int kImageHeight)
{
    if(rect_mid_point.x >= kImageWidth || rect_mid_point.x < 0){ return true;}

    else if(rect_mid_point.y >= kImageHeight || rect_mid_point.y < 0){ return true;}

    else{ return false;}

}





void SafeDriveAreaEvaluation::GetSafestDirections(
vector<LineValidationTable> safest_table1,
vector<LineValidationTable> safest_table2,
vector<pair<int,int>>& safest_line_directions1,
vector<pair<int,int>>& safest_line_directions2)
{
    vector<float> safest_directions1, safest_directions2;
    vector<int> safest_dircetions1_count;
    vector<int> safest_dircetions2_count;


    for(auto it : safest_table1)  safest_directions1.push_back(it.GetDirection());
    for(auto it : safest_table2) safest_directions2.push_back(it.GetDirection());

    auto it_1 = std::unique(safest_directions1.begin(),safest_directions1.end());
    auto it_2 = std::unique(safest_directions2.begin(),safest_directions2.end());

    int unique1_distance = std::distance(safest_directions1.begin(), it_1);
    int unique2_distance = std::distance(safest_directions2.begin(), it_2);

    for(auto it = safest_directions1.begin(); it != it_1; ++it)
    {
        int unique_direction_count = std::count(safest_directions1.begin(), safest_directions1.end(), *it);
        safest_dircetions1_count.push_back(unique_direction_count);
    }

    for(auto it = safest_directions2.begin(); it != it_2; ++it)
    {
        int unique_direction_count = std::count(safest_directions2.begin(), safest_directions2.end(), *it);
        safest_dircetions2_count.push_back(unique_direction_count);

    }

    safest_directions1.resize(unique1_distance);
    safest_directions2.resize(unique2_distance);


    for(int i=0; i<safest_directions1.size();i++) safest_line_directions1.push_back(make_pair(safest_directions1[i],safest_dircetions1_count[i]));
    for(int i=0; i<safest_directions2.size();i++) safest_line_directions2.push_back(make_pair(safest_directions2[i],safest_dircetions2_count[i]));
}



void SafeDriveAreaEvaluation::ClearMemory()
{

    left_lane_drive_points_.clear();
    right_lane_drive_points_.clear();

    safe_drive_area_evaluation_return_info_vector_.clear();

    left_line_points_in_rect_ids_.clear();
    mid_line_points_in_rect_ids_.clear();
    right_line_points_in_rect_ids_.clear();


    for(auto &it: left_priority_ids_) it.clear();
    for(auto &it: mid_priority_ids_) it.clear();
    for(auto &it: right_priority_ids_) it.clear();
}

void SafeDriveAreaEvaluation::DrawEvaluatedSafetyAreasInDriveDirection(
Mat& rgb)
{

    for(auto it: safe_drive_area_evaluation_return_info_vector_)
    {
        Point rect_mid_point   = it.rect_mid_point;
        float search_direction = it.search_direction;

        vector<vector<Point>> search_rect = GetSearchRect(rect_mid_point,
                                                          search_direction,
                                                          kSearchRectWidth_,
                                                          kSearchRectHeight_);

        string str_score = to_string(it.max_line_score);


        drawContours(rgb, search_rect, -1, Scalar(0,255,0), 2, LINE_8);
        putText(rgb, str_score, rect_mid_point,FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,0), 1, CV_AA);

    }
}


void SafeDriveAreaEvaluation::GetSafestTables(
vector<LineValidationTable>& safest_table1,
vector<LineValidationTable>& safest_table2,
vector<vector<LineValidationTable>> priority_table,
unsigned long long int SCORE)
{
    switch(CountDigits(SCORE))
    {

        case 1:
        case 2: safest_table1 = priority_table[PRIO_7_FP2];
                safest_table2 = priority_table[PRIO_6_FP1];
                break;
        case 3:
        case 4: safest_table1 = priority_table[PRIO_5_FP1_AND_FP2];
                safest_table2 = priority_table[PRIO_5_FP1_AND_FP2];
                break;
        case 5:
        case 6: safest_table1 = priority_table[PRIO_4_P2];
                safest_table2 = priority_table[PRIO_3_P1];
                break;
        case 7:
        case 8: safest_table1 = priority_table[PRIO_2_P2_AND_FP1];
                safest_table2 = priority_table[PRIO_1_P1_AND_FP2];
                break;
        case 9:
        case 10: safest_table1 = priority_table[PRIO_0_P1_AND_P2];
                 safest_table2 = priority_table[PRIO_0_P1_AND_P2];
                 break;
        default: cout << "unpossible ???" << endl; exit(0); break;
    }
}


void SafeDriveAreaEvaluation::EmtpySafetyTable(
RectSafetyTable& rect_safety)
{
    rect_safety.percent_points_with_priority_0 = 0;
    rect_safety.percent_points_with_priority_1 = 0;
    rect_safety.percent_points_with_priority_2 = 0;
    rect_safety.percent_points_with_priority_3 = 0;
    rect_safety.percent_points_with_priority_4 = 0;
    rect_safety.percent_points_with_priority_5 = 0;
    rect_safety.percent_points_with_priority_6 = 0;
    rect_safety.percent_points_with_priority_7 = 0;
    rect_safety.percent_points_in_rect = 0;

    rect_safety.too_few_points_in_rect = false;
    rect_safety.rect_straight = false;
    rect_safety.rect_left_curve = false;
    rect_safety.rect_right_curve = false;

    rect_safety.y_min_in_rect_border_range = false;
    rect_safety.y_max_in_rect_border_range = false;
}

void SafeDriveAreaEvaluation::ExtractMinMaxLineElements(
vector<LineValidationTable> line,
MinMaxLineElements& line_minmax_elements )
{

    if(line.size() > 0)
    {
        auto x_it = minmax_element(begin(line),end(line),
                                   [&](LineValidationTable& line_point1, LineValidationTable& line_point2)
                                   {
                                       return line_point1.GetOriginPoint().x < line_point2.GetOriginPoint().x;
                                   });

        auto y_it = minmax_element(begin(line),end(line),
                                   [&](LineValidationTable& line_point1, LineValidationTable& line_point2)
                                   {
                                       return line_point1.GetOriginPoint().y < line_point2.GetOriginPoint().y;
                                   });

        int x_min_id = std::distance(line.begin(), x_it.first);
        int x_max_id = std::distance(line.begin(), x_it.second);
        int y_min_id = std::distance(line.begin(), y_it.first);
        int y_max_id = std::distance(line.begin(), y_it.second);

        line_minmax_elements = MinMaxLineElements{line[x_min_id].GetOriginPoint(),
                                                  line[x_max_id].GetOriginPoint(),
                                                  line[y_min_id].GetOriginPoint(),
                                                  line[y_max_id].GetOriginPoint(),
                                                  true};
    }
}





/*
 *
 *float SafeDriveAreaEvaluation::GetOrthogonalAngle(float angle, int SEARCH_LINE_CODE)
{
    int  angle_ = 0;

    if(SEARCH_LINE_CODE == LEFT_TO_MID || SEARCH_LINE_CODE == LEFT_TO_RIGHT || SEARCH_LINE_CODE == MID_TO_RIGHT) angle_ = (angle - 90);

    if(SEARCH_LINE_CODE == RIGHT_TO_MID || SEARCH_LINE_CODE == RIGHT_TO_LEFT || SEARCH_LINE_CODE == MID_TO_LEFT) angle_ = (angle + 90);

    if(angle_ > 359) angle_ = angle_ % 360;

    if(angle_ < 0) angle_ =  360 - abs(angle_);

    return float(angle_);
}

 *
void SafeDriveAreaEvaluation::FindSafePointsForSpline()
{


    for(auto it: safe_drive_area_evaluation_return_info_vector_)
    {
        int  MAX_LINE_SCORE = it.MAX_LINE_SCORE;
        int  max_line_type = it.max_line_type;

        if(MAX_LINE_SCORE >= 0)
        {
            if(max_line_type == LEFT_LINE)
            {
                for(auto itt : it.left_safest_table1)
                {
                    Point safe_point = itt.GetOriginPoint();
                    float direction  = itt.GetDirection();

                    float right_lane_orthogonal_angle = GetOrthogonalAngle(direction, LEFT_TO_RIGHT);
                    float left_lane_orthogonal_angle = GetOrthogonalAngle(direction, LEFT_TO_RIGHT);


                    int right_lane_x  = safe_point.x + kLeftToRightDriveLaneOffset_ * cos(right_lane_orthogonal_angle*PI/180);
                    int right_lane_y  = safe_point.y - kLeftToRightDriveLaneOffset_ * sin(right_lane_orthogonal_angle*PI/180);

                    int left_lane_x  = safe_point.x + kLeftToLeftDriveLaneOffset_ * cos(left_lane_orthogonal_angle*PI/180);
                    int left_lane_y  = safe_point.y - kLeftToLeftDriveLaneOffset_ * sin(left_lane_orthogonal_angle*PI/180);


                    right_lane_drive_points_.push_back(Point(right_lane_x,right_lane_y));
                    left_lane_drive_points_.push_back(Point(left_lane_x,left_lane_y));
                }

                for(auto itt : it.left_safest_table2)
                {
                    Point safe_point = itt.GetOriginPoint();
                    float direction  = itt.GetDirection();

                    float right_lane_orthogonal_angle = GetOrthogonalAngle(direction, LEFT_TO_RIGHT);
                    float left_lane_orthogonal_angle = GetOrthogonalAngle(direction, LEFT_TO_RIGHT);


                    int right_lane_x  = safe_point.x + kLeftToRightDriveLaneOffset_ * cos(right_lane_orthogonal_angle*PI/180);
                    int right_lane_y  = safe_point.y - kLeftToRightDriveLaneOffset_ * sin(right_lane_orthogonal_angle*PI/180);

                    int left_lane_x  = safe_point.x + kLeftToLeftDriveLaneOffset_ * cos(left_lane_orthogonal_angle*PI/180);
                    int left_lane_y  = safe_point.y - kLeftToLeftDriveLaneOffset_ * sin(left_lane_orthogonal_angle*PI/180);

                    right_lane_drive_points_.push_back(Point(right_lane_x,right_lane_y));
                    left_lane_drive_points_.push_back(Point(left_lane_x,left_lane_y));
                }

            }
            else if(max_line_type == MID_LINE)
            {

                for(auto itt : it.mid_safest_table1)
                {
                    Point safe_point = itt.GetOriginPoint();
                    float direction  = itt.GetDirection();

                    float right_lane_orthogonal_angle = GetOrthogonalAngle(direction, MID_TO_RIGHT);
                    float left_lane_orthogonal_angle = GetOrthogonalAngle(direction, MID_TO_LEFT);

                    int right_lane_x  = safe_point.x + kMidToLeftDriveLaneOffset_ * cos(right_lane_orthogonal_angle*PI/180);
                    int right_lane_y  = safe_point.y - kMidToLeftDriveLaneOffset_ * sin(right_lane_orthogonal_angle*PI/180);

                    int left_lane_x  = safe_point.x + kMidToLeftDriveLaneOffset_ * cos(left_lane_orthogonal_angle*PI/180);
                    int left_lane_y  = safe_point.y - kMidToLeftDriveLaneOffset_ * sin(left_lane_orthogonal_angle*PI/180);

                    right_lane_drive_points_.push_back(Point(right_lane_x,right_lane_y));
                    left_lane_drive_points_.push_back(Point(left_lane_x,left_lane_y));
                }

                for(auto itt : it.mid_safest_table2)
                {
                    Point safe_point = itt.GetOriginPoint();
                    float direction  = itt.GetDirection();

                    float right_lane_orthogonal_angle = GetOrthogonalAngle(direction, MID_TO_RIGHT);
                    float left_lane_orthogonal_angle = GetOrthogonalAngle(direction, MID_TO_LEFT);

                    int right_lane_x  = safe_point.x + kMidToLeftDriveLaneOffset_ * cos(right_lane_orthogonal_angle*PI/180);
                    int right_lane_y  = safe_point.y - kMidToLeftDriveLaneOffset_ * sin(right_lane_orthogonal_angle*PI/180);

                    int left_lane_x  = safe_point.x + kMidToLeftDriveLaneOffset_ * cos(left_lane_orthogonal_angle*PI/180);
                    int left_lane_y  = safe_point.y - kMidToLeftDriveLaneOffset_ * sin(left_lane_orthogonal_angle*PI/180);

                    right_lane_drive_points_.push_back(Point(right_lane_x,right_lane_y));
                    left_lane_drive_points_.push_back(Point(left_lane_x,left_lane_y));
                }
            }
            else if(max_line_type == RIGHT_LINE)
            {

                for(auto itt : it.right_safest_table1)
                {
                    Point safe_point = itt.GetOriginPoint();
                    float direction  = itt.GetDirection();

                    float right_lane_orthogonal_angle = GetOrthogonalAngle(direction, RIGHT_TO_LEFT);
                    float left_lane_orthogonal_angle = GetOrthogonalAngle(direction, RIGHT_TO_LEFT);

                    int right_lane_x  = safe_point.x + kRightToRightDriveLaneOffset_ * cos(right_lane_orthogonal_angle*PI/180);
                    int right_lane_y  = safe_point.y - kRightToRightDriveLaneOffset_ * sin(right_lane_orthogonal_angle*PI/180);

                    int left_lane_x  = safe_point.x + kRightToLeftDriveLaneOffset_ * cos(left_lane_orthogonal_angle*PI/180);
                    int left_lane_y  = safe_point.y - kRightToLeftDriveLaneOffset_ * sin(left_lane_orthogonal_angle*PI/180);

                    right_lane_drive_points_.push_back(Point(right_lane_x,right_lane_y));
                    left_lane_drive_points_.push_back(Point(left_lane_x,left_lane_y));

                }

                for(auto itt : it.right_safest_table2)
                {
                    Point safe_point = itt.GetOriginPoint();
                    float direction  = itt.GetDirection();

                    float right_lane_orthogonal_angle = GetOrthogonalAngle(direction, RIGHT_TO_LEFT);
                    float left_lane_orthogonal_angle = GetOrthogonalAngle(direction, RIGHT_TO_LEFT);

                    int right_lane_x  = safe_point.x + kRightToRightDriveLaneOffset_ * cos(right_lane_orthogonal_angle*PI/180);
                    int right_lane_y  = safe_point.y - kRightToRightDriveLaneOffset_ * sin(right_lane_orthogonal_angle*PI/180);

                    int left_lane_x  = safe_point.x + kRightToLeftDriveLaneOffset_ * cos(left_lane_orthogonal_angle*PI/180);
                    int left_lane_y  = safe_point.y - kRightToLeftDriveLaneOffset_ * sin(left_lane_orthogonal_angle*PI/180);

                    right_lane_drive_points_.push_back(Point(right_lane_x,right_lane_y));
                    left_lane_drive_points_.push_back(Point(left_lane_x,left_lane_y));
                }
            }
        }


    }



}
*/
