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


#include "depth_first_search.h"
#include "datatypes.h"
#include "line_validation_table.h"
#include "utils.h"
#include "spline.h"
#include "defines.h"




using namespace std;
using namespace cv;
using namespace line_points_reducer;
using namespace valid_line_point_search;
using namespace mid_line_search;

class LineValidationTableCreation
{


private:



    Mat current_image_;


// FindValidPointsFromLineFollow (public)
    void SetOuterLineDirections(vector<PointInDirection> line_directions, int SEARCH_LINE_CODE);
    vector<PointInDirection> left_line_directions_;
    vector<PointInDirection> right_line_directions_;

    void FindValidPoints(vector<PointInDirection> line_directions, int SEARCH_LINE_CODE);
        float GetOrthogonalAngle(float angle, int SEARCH_LINE_CODE);
        void SearchOrthogonalValues(int point_in_search_direction_x,int point_in_search_direction_y,float orthogonal_angle,vector<int>& orthogonal_line_activations,vector<Point>& orthogonal_line_points,int SEARCH_LINE_CODE);
        int GetPixelValue(int x, int y);
        int GetPixelValue(Point point);
        SearchLineDistanceThresholds GetSearchLineDistanceThresholds(int SEARCH_LINE_CODE);

            const int kMinLeftToRightLineDistance_ =  100;
            const int kMaxLeftToRightLineDistance_ =  130;
            const int kMinLeftToMidLineDistance_ =  45;
            const int kMaxLeftToMidLineDistance_ =  70;

            const int kMinRightToLeftLineDistance_ =  100;
            const int kMaxRightToLeftLineDistance_ =  130;
            const int kMinRightToMidLineDistance_ =  45;
            const int kMaxRightToMidLineDistance_ =  70;

            const int kMinMidToLeftLineDistance_ = 45;
            const int kMaxMidToLeftLineDistance_ = 70;
            const int kMinMidToRightLineDistance_ = 45;
            const int kMaxMidToRightLineDistance_ = 70;

       int GetMinPixelIntensityThreshold(int SEARCH_LINE_CODE);
            const int kMinLeftToRightPixelIntensity_ = 99;
            const int kMinLeftToMidPixelIntensity_ = 99;
            const int kMinRightToLeftPixelIntensity_ = 99;
            const int kMinRightToMidPixelIntensity_ = 99;
            const int kMinMidToLeftPixelIntensity_ = 99;
            const int kMinMidToRightPixelIntensity_ = 99;

        bool CheckLineMatch(vector<int> orthogonal_line_activations,SegmentStartIDAndWidth& line_match, int SEARCH_LINE_CODE);
            SearchLineWidthThresholds GetSearchLineWidthThresholds(int SEARCH_LINE_CODE);
                const int kMinLeftToRightLineWidth_ =  2;
                const int kMaxLeftToRightLineWidth_ =  5;
                const int kMinLeftToMidLineWidth_ =  2;
                const int kMaxLeftToMidLineWidth_ =  5;

                const int kMinRightToLeftLineWidth_ =  2;
                const int kMaxRightToLeftLineWidth_ =  5;
                const int kMinRightToMidLineWidth_ =  2;
                const int kMaxRightToMidLineWidth_ =  5;

                const int kMinMidToLeftLineWidth_ = 2;
                const int kMaxMidToLeftLineWidth_ = 5;
                const int kMinMidToRightLineWidth_ = 2;
                const int kMaxMidToRightLineWidth_ = 5;



        void SafeLinePoint(SegmentStartIDAndWidth line_match, vector<Point> orthogonal_line_points, bool is_matched,int point_in_search_direction_x,int point_in_search_direction_y, int current_to_next_point_distance,float angle_to_next_point,int SEARCH_LINE_CODE);
         /*   vector<ValidLinePointSearchInfo> left_to_mid_search_info_;
            vector<ValidLinePointSearchInfo> left_to_right_search_info_;

            vector<ValidLinePointSearchInfo> right_to_mid_search_info_;
            vector<ValidLinePointSearchInfo> right_to_left_search_info_;

            vector<ValidLinePointSearchInfo> mid_to_right_search_info_;
            vector<ValidLinePointSearchInfo> mid_to_left_search_info_;

            vector<LineValidationTableCreation> left_line_validation_table_;
            vector<LineValidationTableCreation> mid_line_validation_table_;
            vector<LineValidationTableCreation> right_line_validation_table_;*/


// FindValidPointsFromMidLineSearch (public)
    vector<vector<PointInDirection>> mid_line_directions_clusters_;

    // FindValidPoints

    vector<vector<ValidLinePointSearchInfo>> mid_to_right_search_info_clusters_;
    vector<vector<ValidLinePointSearchInfo>> mid_to_left_search_info_clusters_;




void CreateValidationTables();
vector<ValidLinePointSearchInfo> left_to_mid_search_info_;
vector<ValidLinePointSearchInfo> left_to_right_search_info_;

vector<ValidLinePointSearchInfo> right_to_mid_search_info_;
vector<ValidLinePointSearchInfo> right_to_left_search_info_;

vector<ValidLinePointSearchInfo> mid_to_right_search_info_;
vector<ValidLinePointSearchInfo> mid_to_left_search_info_;

vector<LineValidationTable> left_line_validation_table_;
vector<LineValidationTable> mid_line_validation_table_;
vector<LineValidationTable> right_line_validation_table_;



void SearchValidPoints();

    void ExamineValidationTable(int SEARCH_LINE_CODE, vector<LineValidationTable>&table);
        const Point EMPTY_POINT_ = Point(-1,-1);
        const int kMaxPointDistance_ = 5;
        const int kMaxDirectionDifference_ = 15;


        bool AdjacentValidationTableIsEmpty(int SEARCH_LINE_CODE);
        void FindMinDistanceFromPredictionToAdjacentPoint(int SEARCH_LINE_CODE,Point adjacent_point_prediction, Point &min_distance_adjacent_point,int &min_distance_adjacent_point_id, int &min_distance);
            double Distance2d(const Point& p, LineValidationTable  hs);
        int GetAdjacentPointDirection(int SEARCH_LINE_CODE, int min_distance_adjacent_point_id);
        Point GetAdjacentPointOriginPrediction(int SEARCH_LINE_CODE, int min_distance_adjacent_point_id);
        double Distance2d(const Point p1, const Point p2);

void ExtractValidPoints();

    void ExtractDirectionsInRange(vector<LineValidationTable>& line_validation_table, vector<LineValidationTable>& line_direction_in_range );
        vector<LineValidationTable> left_line_direction_in_range_;
        vector<LineValidationTable> mid_line_direction_in_range_;
        vector<LineValidationTable> right_line_direction_in_range_;
        const int kMinStartDirectionOnSameLine_ =  30;
        const int kMaxStartDirectionOnSameLine_ = 150;
        const int kMaxDirectionDifferenceOnSameLine_ = 30;

    void ExtractMinMaxLineElements( vector<LineValidationTable>  line,  MinMaxLineElements& line_minmax_elements );
        MinMaxLineElements left_line_minmax_elements_;
        MinMaxLineElements mid_line_minmax_elements_;
        MinMaxLineElements right_line_minmax_elements_;

    void ExtractLastMatchingPoints(vector<LineValidationTable> line_validation_table_,vector<LineValidationTable> line_direction_in_range_,int LINE_CODE);
        LastAdjacentPointMatch left_line_last_adjacent_point_match_;
        LastAdjacentPointMatch mid_line_last_adjacent_point_match_;
        LastAdjacentPointMatch right_line_last_adjacent_point_match_;


void FollowTrack(float search_direction, Point rect_mid_point, Mat &rgb);
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

    void FillPriorityTables(vector<LineValidationTable>& left_line_direction_in_range_,vector<int> left_line_points_in_rect_ids_, vector<LineValidationTable>& mid_line_direction_in_range_,vector<int> mid_line_points_in_rect_ids_,vector<LineValidationTable>& right_line_direction_in_range_,vector<int> right_line_points_in_rect_ids_);
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
        //void ExtractMinMaxLineElements( vector<LineValidationTableCreation>  line,  MinMaxLineElements& line_minmax_elements );
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







vector<float> GetUniqueDirectionsInRect(vector<LineValidationTable> table, vector<int> rect_ids, vector<vector<int>> priority_ids, int priority);


public:
    LineValidationTableCreation();
    void SetImage(Mat image);

    void FindValidPointsFromLineFollow(vector<PointInDirection> line_directions, int SEARCH_LINE_CODE);
    void FindValidPointsFromMidLineSearch(vector<vector<PointInDirection>> mid_line_directions_clusters, int SEARCH_LINE_CODE);
    void ClearValidationTables();

    void GatherLineValidationTables(vector<LineValidationTable> &left_line_validation_table, vector<LineValidationTable> &mid_line_validation_table, vector<LineValidationTable>& right_line_validation_table);


    void ValidateTrack(Mat &rgb);

    void CoutRectSafetyTables();

    void DrawDirectionInRangeTable(Mat &rgb);
    void DrawSpline(Mat &rgb);
    void DrawSafetyRects(Mat& rgb_spline);
    void DrawValidatedPointsOnDriveWay(Mat &rgb);


};

#endif // VALID_LINE_POINT_SEARCH_H
