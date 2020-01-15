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

    image_transport::Publisher road_sign_image_publisher_;
    image_transport::ImageTransport it_;

    Mat image_;

    int start_left_x_ = 0;
    int start_right_x_ = 0;
    int start_left_y_ = 0;
    int start_right_y_ = 0;
    float start_angle_left_= 0;
    float start_angle_right_ =0;
/*
    const int kGoalLineIntensityThreshold_ = 100;
    const int kMinGoalSegmentWidth_ = 2;
    const int kMaxGoalSegmentWidth_ = 4;
    const int kMinGoalSegmentsToFind_ = 20;

    const int kMaxCrossWalkForsightDistance_ = 150;
    const int kMaxCrossWalkForsightStepSize_ = 5;
    const int kMinCrossWalkSegmentWidth_ = 6;
    const int kMaxCrossWalkSegmentWidth_ = 8;
    const int kMinCrossWalkSegmentsToFind_ = 15;

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
    const int MaxLeftLineYHeightForCrossing_ = 250;
    const int MaxRightLineYHeightForCrossing_ = 250;
    const int MaxLeftLineSizeForCrossing_ = 150;
    const int MaxRightLineSizeForCrossing_ = 150;

    bool left_line_in_crossing_height_;
    bool right_line_in_crossing_height_;
    bool left_line_in_crossing_size_;
    bool right_line_in_crossing_size_;

    const int kMaxLaneObjectForsightDistance_ = 125;
    const int kLaneObjectForsightStepSize_ = 40;

    const int kLeftInLeftLaneLineIteratorEndOffset_ = 60;
    const int kLeftInRightLaneLineIteratorStartOffset_ = 70;
    const int kLeftInRightLaneLineIteratorEndOffset_ = 125;
    const int kRightInLeftLaneLineIteratorStartOffset_ = 70;
    const int kRightInLeftLaneLineIteratorEndOffset_ = 125;
    const int kRightInRightLaneLineIteratorEndOffset_ = 60;

    const int kLeftInLeftLaneRadialOuterLineOffset_  = 30;
    const int kLeftInRightLaneRadialOuterLineOffset_ = 95;
    const int kRightInLeftLaneRadialOuterLineOffset_ = 95;
    const int kRightInRightLaneRadialOuterLineOffset_ = 30;

    const int kLaneObjectRadialScanStepSize_ = 1;
    const int kLaneObjectRadialScanRadius_ = 20;

    const int kMinMarkingSegmentsThreshold_ = 4;

    const int kMinBoxSegmentsThreshold_ = 1;
    const int kMinWhitePixelsForBox_ = 40;

    bool found_mid_crossing_pattern_ = false;
    bool found_left_crossing_pattern_ = false;
    bool found_right_crossing_pattern_ = false;

    bool found_cross_walk_ = false;
    bool found_goal_line_ = false;
    bool found_crossing_ = false;
    bool found_box_ = false;
    bool found_marking_ = false;

    Point cross_walk_mid_point_ = Point(-1,-1);
    Point crossing_mid_point_ = Point(-1,-1);
    Point road_object_mid_point_ = Point(-1,-1);
    Point goal_line_mid_point_ = Point(-1,-1);

    const int kVelocitySignTemplateHeight_ = 65;
    const int kVelocitySignTemplateWidth_ = 30;
    const int kGoalLineFieldOfView_ = 10;

    Mat image_template_10;
    Mat image_template_20;
    Mat image_template_30;
    Mat image_template_40;
    Mat image_template_50;
    Mat image_template_60;
    Mat image_template_70;
    Mat image_template_80;
    Mat image_template_90;

    Point kRectTopLeftPointForClassifierRoi_ = Point(619,233);
    Point kRectBottomRightPointForClassifierRoi_ = Point(656,310);
    const int kResizeHeightForClassifierRoi_ = 56;
    const int kResizeWidthForClassifierRoi_ = 28;

    const int kTemplateRoiSizeX_ = 50;
    const int kTemplateRoiSizeY_ = 50;
    const int kRoadSignIntensityThreshold_ = 100;

*/
    const int kGoalLineIntensityThreshold_;
    const int kMinGoalSegmentWidth_;
    const int kMaxGoalSegmentWidth_;
    const int kMinGoalSegmentsToFind_;

    const int kMaxCrossWalkForsightDistance_;
    const int kMaxCrossWalkForsightStepSize_;
    const int kMinCrossWalkSegmentWidth_;
    const int kMaxCrossWalkSegmentWidth_;
    const int kMinCrossWalkSegmentsToFind_;

    const int kMinToLeftMidLineDirectionForCrossing_;
    const int kMaxToLeftMidLineDirectionForCrossing_;
    const int kMinToRightMidLineDirectionForCrossing_;
    const int kMaxToRightMidLineDirectionForCrossing_;
    const int kMinOutLineDirectionDifferenceForCrossing_;
    const int kMaxCrossingForsightY_;
    const int kMinToLeftLeftLineDirectionForCrossing_;
    const int kMaxToLeftLeftLineDirectionForCrossing_;
    const int kMinToRightRightLineDirectionForCrossing_;
    const int kMaxToRightRightLineDirectionForCrossing_;
    const int MaxLeftLineYHeightForCrossing_;
    const int MaxRightLineYHeightForCrossing_;
    const int MaxLeftLineSizeForCrossing_;
    const int MaxRightLineSizeForCrossing_;

    bool left_line_in_crossing_height_ = false;
    bool right_line_in_crossing_height_ = false;
    bool left_line_in_crossing_size_ = false;
    bool right_line_in_crossing_size_ = false;

    const int kMaxLaneObjectForsightDistance_;
    const int kLaneObjectForsightStepSize_;

    const int kLeftInLeftLaneLineIteratorEndOffset_;
    const int kLeftInRightLaneLineIteratorStartOffset_;
    const int kLeftInRightLaneLineIteratorEndOffset_;
    const int kRightInLeftLaneLineIteratorStartOffset_;
    const int kRightInLeftLaneLineIteratorEndOffset_;
    const int kRightInRightLaneLineIteratorEndOffset_;

    const int kLeftInLeftLaneRadialOuterLineOffset_;
    const int kLeftInRightLaneRadialOuterLineOffset_;
    const int kRightInLeftLaneRadialOuterLineOffset_;
    const int kRightInRightLaneRadialOuterLineOffset_;

    const int kLaneObjectRadialScanStepSize_;
    const int kLaneObjectRadialScanRadius_;

    const int kMinMarkingSegmentsThreshold_;

    const int kMinBoxSegmentsThreshold_;
    const int kMinWhitePixelsForBox_;

    bool found_mid_crossing_pattern_ = false;
    bool found_left_crossing_pattern_ = false;
    bool found_right_crossing_pattern_ = false;

    bool found_cross_walk_ = false;
    bool found_goal_line_ = false;
    bool found_crossing_ = false;
    bool found_box_ = false;
    bool found_marking_ = false;

    Point cross_walk_mid_point_ = Point(-1,-1);
    Point crossing_mid_point_ = Point(-1,-1);
    Point road_object_mid_point_ = Point(-1,-1);
    Point goal_line_mid_point_ = Point(-1,-1);

    const int kVelocitySignTemplateHeight_;
    const int kVelocitySignTemplateWidth_;
    const int kGoalLineFieldOfView_;

    Mat image_template_10;
    Mat image_template_20;
    Mat image_template_30;
    Mat image_template_40;
    Mat image_template_50;
    Mat image_template_60;
    Mat image_template_70;
    Mat image_template_80;
    Mat image_template_90;

    Point kRectTopLeftPointForClassifierRoi_;
    Point kRectBottomRightPointForClassifierRoi_;
    const int kResizeHeightForClassifierRoi_;
    const int kResizeWidthForClassifierRoi_;

    const int kTemplateRoiSizeX_;
    const int kTemplateRoiSizeY_;
    const int kRoadSignIntensityThreshold_;
    vector<LineValidationTable> left_line_validation_table_;
    vector<LineValidationTable> mid_line_validation_table_;
    vector<LineValidationTable> right_line_validation_table_;

    vector<LineValidationTable> left_line_points_in_drive_direction_;
    vector<LineValidationTable> right_line_points_in_drive_direction_;

    vector<SafeDriveAreaEvaluationReturnInfo> safe_drive_area_evaluation_;


    bool SearchGoalLine(
    Mat image,
    const int start_left_x,
    const int start_left_y,
    const float start_angle_left,
    const int kGoalLineFieldOfView,
    const int kGoalLineIntensityThreshold,
    const int kMinGoalSegmentWidth,
    const int kMaxGoalSegmentWidth,
    const int kMinGoalSegmentsToFind,
    bool &found_goal_line,
    Point &goal_line_mid_point);

    float GetOrthogonalAngle(
    float angle,
    int SEARCH_LINE_CODE);

    Point GetPolarCoordinate(
    int x,
    int y,
    float angle,
    int radius);

    vector<pair<string,int>> GatherBlackAndWhiteRoadSegments(
    Mat scanned_line_mat);

    bool SearchCrossWalk(
    Mat image,
    bool &found_cross_walk,
    Point &cross_walk_mid_point,
    const int start_left_x,
    const int start_left_y,
    const float start_angle_left,
    const int start_right_x,
    const int start_right_y,
    const float start_angle_right,
    const int kMinCrossWalkSegmentWidth,
    const int kMaxCrossWalkSegmentWidth,
    const int kMinCrossWalkSegmentsToFind,
    const int kMaxCrossWalkForsightDistance,
    const int kMaxCrossWalkForsightStepSize);

    void SearchCrossRoad(
    vector<LineValidationTable> left_line_validation_table,
    vector<LineValidationTable>mid_line_validation_table,
    vector<LineValidationTable>right_line_validation_table,
    vector<LineValidationTable> left_line_points_in_drive_direction,
    vector<LineValidationTable> right_line_points_in_drive_direction,
    Point &crossing_mid_point,
    bool &found_left_crossing_pattern,
    bool &found_mid_crossing_pattern,
    bool &found_right_crossing_pattern,
    bool &left_line_in_crossing_height,
    bool &right_line_in_crossing_height,
    bool &left_line_in_crossing_size,
    bool &right_line_in_crossing_size,
    bool &found_crossing,
    const int kMinToLeftMidLineDirectionForCrossing,
    const int kMaxToLeftMidLineDirectionForCrossing,
    const int kMaxToRightMidLineDirectionForCrossing,
    const int kMinToRightMidLineDirectionForCrossing,
    const int kMinOutLineDirectionDifferenceForCrossing,
    const int kMaxCrossingForsightY,
    const int kMinToLeftLeftLineDirectionForCrossing,
    const int kMaxToLeftLeftLineDirectionForCrossing,
    const int kMinToRightRightLineDirectionForCrossing,
    const int kMaxToRightRightLineDirectionForCrossing,
    const int MaxLeftLineYHeightForCrossing,
    const int MaxRightLineYHeightForCrossing,
    const int MaxLeftLineSizeForCrossing,
    const int MaxRightLineSizeForCrossing);

    void SearchForStrongDirectionDifferencesInValidationtables(
    vector<LineValidationTable> left_line_validation_table,
    vector<LineValidationTable>mid_line_validation_table,
    vector<LineValidationTable>right_line_validation_table,
    Point & crossing_left_line_point,
    Point & crossing_right_line_point,
    bool &found_mid_crossing_pattern,
    bool &found_left_crossing_pattern,
    bool &found_right_crossing_pattern,
    const int kMinToLeftMidLineDirectionForCrossing,
    const int kMaxToLeftMidLineDirectionForCrossing,
    const int kMaxToRightMidLineDirectionForCrossing,
    const int kMinToRightMidLineDirectionForCrossing,
    const int kMinOutLineDirectionDifferenceForCrossing,
    const int kMaxCrossingForsightY,
    const int kMinToLeftLeftLineDirectionForCrossing,
    const int kMaxToLeftLeftLineDirectionForCrossing,
    const int kMinToRightRightLineDirectionForCrossing,
    const int kMaxToRightRightLineDirectionForCrossing);

    void ExtractMinMaxLineElements(
    vector<LineValidationTable> line,
    MinMaxLineElements& line_minmax_elements);

    void SearchRoadObject(
    Mat image,
    vector<LineValidationTable> current_table,
    int search_direction,
    int search_distance_code,
    Point &road_object_mid_point,
    bool &found_box,
    bool &found_marking,
    const int kLeftInLeftLaneRadialOuterLineOffset,
    const int kLeftInRightLaneRadialOuterLineOffset,
    const int kRightInLeftLaneRadialOuterLineOffset,
    const int kRightInRightLaneRadialOuterLineOffset,
    const int kMaxLaneObjectForsightDistance,
    const int kLeftInLeftLaneLineIteratorEndOffset,
    const int kLeftInRightLaneLineIteratorStartOffset,
    const int kLeftInRightLaneLineIteratorEndOffset,
    const int kRightInLeftLaneLineIteratorStartOffset,
    const int kRightInLeftLaneLineIteratorEndOffset,
    const int kRightInRightLaneLineIteratorEndOffset,
    const int kLaneObjectRadialScanStepSize,
    const int kLaneObjectRadialScanRadius1,
    const int kMinWhitePixelsForBox,
    const int kRoadSignIntensityThreshold,
    const int kMinBoxSegmentsThreshold,
    const int kMinMarkingSegmentsThreshold,
    const int kLaneObjectForsightStepSize);

    void SendMarkingImageToClassifier(
    Mat image,
    Point kRectTopLeftPointForClassifierRoi,
    Point kRectBottomRightPointForClassifierRoi,
    const int kResizeHeightForClassifierRoi,
    const int kResizeWidthForClassifierRoi);

    void TemplateMatchMarking(
    vector<PointInDirection> markings);

    double MatchTemplateCCOEFF(
    Mat image,
    Mat template_image);

    double MatchTemplateSQDIFF(
    Mat image,
    Mat template_image);


public:


    OnRoadSearch(
    ros::NodeHandle* nh_,
    OnRoadSearchInitializationParameters init);

    void SetImage(
    Mat image)
    {image_ = image;}

    void LoadSafeDriveAreaEvaluation(
    vector<SafeDriveAreaEvaluationReturnInfo> safe_drive_area_evaluation);

    void LoadValidationTables(
    vector<LineValidationTable> left_line_validation_table,
    vector<LineValidationTable> mid_line_validation_table,
    vector<LineValidationTable> right_line_validation_table);

    void LoadInDriveDirectionTables(
    vector<LineValidationTable> left_line_points_in_drive_direction,
    vector<LineValidationTable> right_line_points_in_drive_direction);

    void SetStartParameters(
    StartParameters start_parameters);

    void SearchOnRoad();

    void DrawEvaluatedSafetyAreas(
    Mat& rgb);

};


#endif // ON_ROAD_SEARCHER_H
