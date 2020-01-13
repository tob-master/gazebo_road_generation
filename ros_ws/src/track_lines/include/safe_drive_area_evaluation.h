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
    void FollowTrack(float search_direction, Point rect_mid_point);
        const Point kStartMidPointOfRectSafety_ = Point(612,360);
        const float kStartSearchDirectionOfRectSafety_ = 90;

        void ClearAllFollowTrackTables();



        vector<vector<Point>> GetSearchRect(Point rect_mid, float search_direction);
                const float kSearchRectLength_ = 300;
        void GetLinesPointsInRect( vector<LineValidationTable> line_direction_in_range_, vector<vector<Point>> contours, vector<int>& line_points_in_rect_id,vector<LineValidationTable>& line_points_in_rect_);
            vector<int> left_line_points_in_rect_ids_;
            vector<int> mid_line_points_in_rect_ids_;
            vector<int> right_line_points_in_rect_ids_;

            vector<LineValidationTable> left_line_points_in_rect_;
            vector<LineValidationTable> mid_line_points_in_rect_;
            vector<LineValidationTable> right_line_points_in_rect_;

        void FillPriorityTables(vector<LineValidationTable>& left_line_in_drive_direction_table_,vector<int> left_line_points_in_rect_ids_, vector<LineValidationTable>& mid_line_in_drive_direction_table_,vector<int> mid_line_points_in_rect_ids_,vector<LineValidationTable>& right_line_in_drive_direction_table_,vector<int> right_line_points_in_rect_ids_);
            void FillPriorityTable(LineValidationTable table,int i, bool found_point1,bool found_point2,bool prediction1,bool prediction2, bool directions_in_range1, bool directions_in_range2, vector<vector<int>>& priority_ids,vector<vector<LineValidationTable>>& priority_table);
                vector<vector<int>> left_priority_ids_{14};
                vector<vector<int>> mid_priority_ids_{14};
                vector<vector<int>> right_priority_ids_{14};
                vector<vector<LineValidationTable>> left_priority_table_{14};
                vector<vector<LineValidationTable>> mid_priority_table_{14};
                vector<vector<LineValidationTable>> right_priority_table_{14};

        void CheckRectSafety(vector<vector<Point>> search_rect,vector<LineValidationTable>line_points_in_rect_,vector<vector<LineValidationTable>> priority_table_,RectSafetyTable& rect_safety);
            RectSafetyTable left_line_rect_safety_;
            RectSafetyTable mid_line_rect_safety_;
            RectSafetyTable right_line_rect_safety_;
            const float kSearchRectHeight_ = 80;
            //void ExtractMinMaxLineElements( vector<SafeDriveAreaEvaluation>  line,  MinMaxLineElements& line_minmax_elements );
            const int kMinYDistanceInRect_ = 15;
            const int kMinStraightDifferenceForStraightLineInRect_ = 15;
            const int kRectBorderDistanceThreshold_ = 10;
            void EmtpySafetyTable(RectSafetyTable& rect_safety);

        void GatherRectSafetyInfo(vector<TrackSafetyRect>& track_safety_rects_ ,float search_direction, Point rect_mid_point );
            //Include in class
            int CountDigits(unsigned long long int n);
            void GetSafestTables(vector<LineValidationTable>& safest_table1 , vector<LineValidationTable>& safest_table2, vector<vector<LineValidationTable>> priority_table, unsigned long long int SCORE);
            vector<TrackSafetyRect> track_safety_rects_;

        void FindNewSearchDirection(vector<TrackSafetyRect> track_safety_rects_,float& search_direction);
            //safest direction questionable
            void GetSafestDirections(vector<LineValidationTable> safest_table1,vector<LineValidationTable> safest_table2, vector<pair<int,float>>& safest_line_directions1, vector<pair<int,float>>& safest_line_directions2);

        void GetNewRectMidPoint(float new_search_direction,Point rect_mid_point, Point& new_rect_mid_point);
            const int kRectStepLength_ = 80;

        bool RectMidPointOutOfImage(Point rect_mid_point);

            const int kImageWidth_ = 1280;
            const int kImageHeight_ = 417;



    void FindSafePointsForSpline();

        vector<Point> right_lane_drive_points_;
        vector<Point> left_lane_drive_points_;
        const int kLeftToRightDriveLaneOffset_ = 93;
        const int kMidToRightDriveLaneOffset_ = 31;
        const int kRightToRightDriveLaneOffset_ = 31;

        const int kLeftToLeftDriveLaneOffset_ = 31;
        const int kMidToLeftDriveLaneOffset_ = 31;
        const int kRightToLeftDriveLaneOffset_ = 93;


        vector<LineValidationTable> left_line_in_drive_direction_table_;
        vector<LineValidationTable> mid_line_in_drive_direction_table_;
        vector<LineValidationTable> right_line_in_drive_direction_table_;

float GetOrthogonalAngle(float angle, int SEARCH_LINE_CODE);
void ExtractMinMaxLineElements( vector<LineValidationTable>  line,  MinMaxLineElements& line_minmax_elements );




public:
        void LoadLinePointsInDriveDirection(vector<LineValidationTable> left_line_in_drive_direction_table,vector<LineValidationTable>mid_line_in_drive_direction_table,vector<LineValidationTable>right_line_in_drive_direction_table);
    void EvaluateTrackInDriveDirection();
    void DrawEvaluatedSafetyAreasInDriveDirection(Mat& rgb);

    void ClearMemory();
    //SafeDriveAreaEvaluation();
};

#endif // SAFE_DRIVE_AREA_EVALUATION_H
