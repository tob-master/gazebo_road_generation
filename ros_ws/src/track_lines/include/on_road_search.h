#ifndef ON_ROAD_SEARCHER_H
#define ON_ROAD_SEARCHER_H


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

#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <numeric>
#include "line_validation_table.h"

using namespace std;
using namespace cv;

class OnRoadSearch
{

      enum{LEFT_IN_LEFT_LANE, LEFT_IN_RIGHT_LANE, RIGHT_IN_LEFT_LANE, RIGHT_IN_RIGHT_LANE};

private:

    Mat image_;

    const int kVelocitySignTemplateHeight_ = 65;
    const int kVelocitySignTemplateWidth_ = 30;
    const int kGoalLineFieldOfView_ = 10;

    const int GoalLineIntesitiyThreshold_ = 100;

    const int kMinGoalSegmentWidth_ = 2;
    const int kMaxGoalSegmentWidth_ = 4;
    const int kMinGoalSegmentsToFind_ = 20;

    const int kMaxCrossWalkForsightDistance_ = 150;
    const int kMaxCrossWalkForsightStepSize_ = 5;


    const int kMinCrossWalkSegmentWidth_ = 6;
    const int kMaxCrossWalkSegmentWidth_ = 8;
    const int kMinCrossWalkSegmentsToFind_ = 15;

    bool found_goal_line_ = false;
    bool found_cross_walk_ = false;

    const int kMaxRoadSignForsightDistance_ = 125;

    const int kLeftToLeftMidLaneDistance = 30;

    const int kLeftInLeftLaneDistance  = 30;
    const int kLeftInRightLaneDistance = 95;
    const int kRightInLeftLaneDistance = 95;
    const int kRightInRightLaneDistance = 30;
    const int kLaneObjectRadialScanStepSize = 2;
    const int kLaneObjectRadialScanRadius1 = 20;
    const int kLaneObjectRadialScanRadius2 =  5;
    const int kLaneObjectForsightStepSize = 40;
    const int kMaxPxCountForLaneMarkingRadialScanRadius1 = 120;
    const int kMaxPxCountForLaneMarkingRadialScanRadius2 = 120;
    const int kMinPxCountForLaneMarkingRadialScanRadius1 = 30;
    const int kMinPxCountForLaneMarkingRadialScanRadius2 = 20;

    const int kMaxLaneObjectForsightDistance_ = 125;

    const int kMinPxCountForBoxRadialScanRadius1  = 150;
    const int kMinPxCountForBoxRadialScanRadius2 = 150;

    Mat image_template_10;
    Mat image_template_20;
    Mat image_template_30;
    Mat image_template_40;
    Mat image_template_50;
    Mat image_template_60;
    Mat image_template_70;
    Mat image_template_80;
    Mat image_template_90;

    int start_left_x_ = 0;
    int start_right_x_ = 0;
    int start_left_y_ = 0;
    int start_right_y_ = 0;
    float start_angle_left_= 0;
    float start_angle_right_ =0;


    bool found_mid_crossing_ = false;
       bool found_left_crossing_ = false;
       bool found_right_crossing_ = false;



    const int kMinToLeftMidLineDirectionForCrossing_ = 340;
    const int kMaxToLeftMidLineDirectionForCrossing_ = 20;
    const int kMinToRightMidLineDirectionForCrossing_ = 160;
    const int kMaxToRightMidLineDirectionForCrossing_ = 200;


    const int kMinOutLineDirectionDifferenceForCrossing_ = 80;
    const int kMaxCrossingForsightY_ = 220;

    const int kMinToLeftLeftLineDirectionForCrossing_ = 140;
    const int kMaxToLeftLeftLineDirectionForCrossing_ = 220;
    const int kMinToRightRightLineDirectionForCrossing_ = 320;
    const int kMaxToRightRightLineDirectionForCrossing_ = 40;

    const int MaxLeftLineYHeightForCrossing = 250;
    const int MaxRightLineYHeightForCrossing = 250;
    const int MaxLeftLineSizetForCrossing = 150;
    const int MaxRightLineSizeForCrossing = 150;


    vector<LineValidationTable> left_line_validation_table_;
    vector<LineValidationTable> mid_line_validation_table_;
     vector<LineValidationTable> right_line_validation_table_;

     vector<LineValidationTable> left_line_points_in_drive_direction_;
     vector<LineValidationTable> right_line_points_in_drive_direction_;


     bool left_line_in_crossing_height_;
     bool right_line_in_crossing_height_;
     bool left_line_in_crossing_size_;
     bool right_line_in_crossing_size_;

     bool found_crossing_ = false;


     const int kTemplateRoiSizeX_ = 50;
     const int kTemplateRoiSizeY_ = 50;

public:
    void LoadImage(Mat image){image_ = image;};
    OnRoadSearch();
    void SearchOnRoad(vector<LineValidationTable> left_table,vector<LineValidationTable> right_table);
    vector<pair<string,int>> GatherBlackAndWhiteRoadSegments(Mat scanned_line_mat);
    void SearchRoadObject(vector<LineValidationTable> current_table, vector<PointInDirection>& markings, vector<PointInDirection>& boxes,  int SEARCH_DIRECTION, int SEARCH_DISTANCE_CODE);
    double MatchTemplateCCOEFF(Mat image, Mat template_image);
    double MatchTemplateSQDIFF(Mat image, Mat template_image);
    float GetOrthogonalAngle(float angle, int SEARCH_LINE_CODE);
    Point GetPolarCoordinate(int x, int y, float angle, int radius);
    bool SearchGoalLine();
    bool SearchCrossWalk();
    bool SearchRoadSigns();
    void SetStartParameters(StartParameters start_parameters);
    void ExtractMinMaxLineElements( vector<LineValidationTable>  line,  MinMaxLineElements& line_minmax_elements );
    void LoadValidationTables(vector<LineValidationTable> left_line_validation_table,
                              vector<LineValidationTable> mid_line_validation_table,
                              vector<LineValidationTable> right_line_validation_table);

    void SearchForStrongDirectionDifferencesInValidationtables();
    void LoadInDriveDirectionTables(vector<LineValidationTable> left_line_points_in_drive_direction,vector<LineValidationTable> right_line_points_in_drive_direction);
    void SearchCrossRoad();
    void TemplateMatchMarking(vector<PointInDirection> markings);
};

#endif // ON_ROAD_SEARCHER_H
