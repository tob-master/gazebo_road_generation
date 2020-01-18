#ifndef SAFE_DRIVE_AREA_EVALUATION_H
#define SAFE_DRIVE_AREA_EVALUATION_H

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


using namespace std;
using namespace cv;
using namespace valid_line_point_search;

class SafeDriveAreaEvaluation
{

private:


const Point kStartMidPointOfRectSafety_;
const float kStartSearchDirectionOfRectSafety_;

const float kSearchRectWidth_;
const float kSearchRectHeight_;

const int kImageWidth_;
const int kImageHeight_;

const int kRectBorderDistanceThresholdForContinousLine_;
const int kRectStepLength_;

long int priority_0_multiplier_ =  100000000;
long int priority_1_2_multiplier_ =  1000000;
int priority_3_4_multiplier_ =  10000;
int priority_5_multiplier_ = 100;
int priority_6_7_multiplier_ = 1;

vector<vector<int>> left_priority_ids_{8};
vector<vector<int>> mid_priority_ids_{8};
vector<vector<int>> right_priority_ids_{8};
vector<vector<LineValidationTable>> left_priority_table_{8};
vector<vector<LineValidationTable>> mid_priority_table_{8};
vector<vector<LineValidationTable>> right_priority_table_{8};

vector<int> left_line_points_in_rect_ids_;
vector<int> mid_line_points_in_rect_ids_;
vector<int> right_line_points_in_rect_ids_;

vector<LineValidationTable> left_line_points_in_rect_;
vector<LineValidationTable> mid_line_points_in_rect_;
vector<LineValidationTable> right_line_points_in_rect_;

vector<Point> right_lane_drive_points_;
vector<Point> left_lane_drive_points_;

vector<LineValidationTable> left_line_in_drive_direction_table_;
vector<LineValidationTable> mid_line_in_drive_direction_table_;
vector<LineValidationTable> right_line_in_drive_direction_table_;


vector<SafeDriveAreaEvaluationReturnInfo> safe_drive_area_evaluation_return_info_vector_;

void FollowTrack(
float search_direction,
Point rect_mid_point,
const int kSearchRectLength,
const int kSearchRectHeight);

void ClearAllFollowTrackTables(
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
RectSafetyTable &right_line_rect_safety);

void GetLinesPointsInRect(
vector<LineValidationTable> line_direction_in_range_,
vector<vector<Point>> contours,
vector<int>& line_points_in_rect_id,
vector<LineValidationTable>& line_points_in_rect_);

void FillPriorityTables(
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
vector<vector<int>> &right_priority_ids);

void FillPriorityTable(
LineValidationTable table,
int i,
bool found_point1,
bool found_point2,
bool prediction1,
bool prediction2,
bool directions_in_range1,
bool directions_in_range2,
vector<vector<int>>& priority_ids,
vector<vector<LineValidationTable>>& priority_table);


void CheckRectSafety(
vector<vector<Point>> search_rect,
vector<LineValidationTable>line_points_in_rect,
vector<vector<LineValidationTable>> priority_table,
RectSafetyTable& rect_safety,
const int kSearchRectHeight,
const int kRectBorderDistanceThreshold);
RectSafetyTable left_line_rect_safety_;
RectSafetyTable mid_line_rect_safety_;
RectSafetyTable right_line_rect_safety_;

void GatherSafeDriveAreaEvaluationTableReturInfo(
vector<vector<Point>> search_rect,
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
const int priority_6_7_multiplier);

void GetNewSearchDirection(
vector<SafeDriveAreaEvaluationReturnInfo> safe_drive_area_evaluation_return_info_vector_,
float& search_direction);


void GetNewRectMidPoint(
float new_search_direction,
Point rect_mid_point,
Point& new_rect_mid_point,
const int kRectStepLength);


vector<vector<Point>> GetSearchRect(
Point rect_mid,
float search_direction,
const int kSearchRectLength,
const int kSearchRectHeight);

void EmtpySafetyTable(
RectSafetyTable& rect_safety);

int CountDigits(
unsigned long long int n);

void GetSafestTables(
vector<LineValidationTable>& safest_table1 ,
vector<LineValidationTable>& safest_table2,
vector<vector<LineValidationTable>> priority_table,
unsigned long long int SCORE);


void GetSafestDirections(
vector<LineValidationTable> safest_table1,
vector<LineValidationTable> safest_table2,
vector<pair<int,int>>& safest_line_directions1,
vector<pair<int,int>>& safest_line_directions2);

bool RectMidPointOutOfImage(
Point rect_mid_point,
const int kImageWidth,
const int kImageHeight);

void FindSafePointsForSpline();

float GetOrthogonalAngle(
float angle,
int SEARCH_LINE_CODE);

void ExtractMinMaxLineElements(
vector<LineValidationTable>  line,
MinMaxLineElements& line_minmax_elements );


public:

SafeDriveAreaEvaluation(
int image_height,
int image_width,
SafeDriveAreaEvaluationInitializationParameters init);

void LoadLinePointsInDriveDirection(
vector<LineValidationTable> left_line_in_drive_direction_table,
vector<LineValidationTable>mid_line_in_drive_direction_table,
vector<LineValidationTable>right_line_in_drive_direction_table);

vector<SafeDriveAreaEvaluationReturnInfo> EvaluateTrackInDriveDirection();
void DrawEvaluatedSafetyAreasInDriveDirection(
Mat& rgb);

void DrawPriorityPoints(
Mat &rgb,
vector<int> priority_ids);

void ClearMemory();

};

#endif // SAFE_DRIVE_AREA_EVALUATION_H
