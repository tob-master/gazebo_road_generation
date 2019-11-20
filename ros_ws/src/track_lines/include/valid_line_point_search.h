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

#include "line_validation_table.h"
#include "depth_first_search.h"
#include "datatypes.h"
#include "utils.h"

using namespace std;
using namespace cv;
using namespace line_points_reducer;
using namespace valid_line_point_search;
using namespace mid_line_search;

class ValidLinePointSearch
{


private:



    Mat current_image_;



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

    const int kMinLeftToRightPixelIntensity_ = 99;
    const int kMinLeftToMidPixelIntensity_ = 99;
    const int kMinRightToLeftPixelIntensity_ = 99;
    const int kMinRightToMidPixelIntensity_ = 99;
    const int kMinMidToLeftPixelIntensity_ = 99;
    const int kMinMidToRightPixelIntensity_ = 99;


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

    const int kImageWidth_ = 1280;
    const int kImageHeight_ = 417;


    const int kMaxPointDistance_ = 10;
    const int kMaxDirectionDifference_ = 15;

        const Point EMPTY_POINT_ = Point(-1,-1);

    vector<PointInDirection> left_line_directions_;
    vector<PointInDirection> right_line_directions_;
    vector<vector<PointInDirection>> mid_line_directions_clusters_;

    vector<ValidLinePointSearchInfo> left_to_mid_search_info_;
    vector<ValidLinePointSearchInfo> left_to_right_search_info_;

    vector<ValidLinePointSearchInfo> right_to_mid_search_info_;
    vector<ValidLinePointSearchInfo> right_to_left_search_info_;


    vector<ValidLinePointSearchInfo> mid_to_right_search_info_;
    vector<ValidLinePointSearchInfo> mid_to_left_search_info_;


    vector<vector<ValidLinePointSearchInfo>> mid_to_right_search_info_clusters_;
    vector<vector<ValidLinePointSearchInfo>> mid_to_left_search_info_clusters_;

    vector<RightValidationTable> right_points_validation_table_;
    vector<MidValidationTable> mid_points_validation_table_;
    vector<LeftValidationTable> left_points_validation_table_;





    vector<LineValidationTable*> left_line_validation_table_;
    vector<LineValidationTable*> mid_line_validation_table_;
    vector<LineValidationTable*> right_line_validation_table_;



    vector<Point> l;
    vector<Point> m;
    vector<Point> r;


    vector<ValidPoints> l_info;
    vector<ValidPoints> m_info;
    vector<ValidPoints> r_info;



    void ClearMemory(int SEARCH_LINE_CODE);
    void SearchOrthogonalValues(int point_in_search_direction_x,
                                 int point_in_search_direction_y,
                                 float orthogonal_angle,
                                  vector<int>& orthogonal_line_activations,
                                  vector<Point>& orthogonal_line_points,
                                  int SEARCH_LINE_CODE);

    float GetOrthogonalAngle(float angle, int SEARCH_LINE_CODE);
    int GetPixelValue(int x, int y);
    int GetPixelValue(Point point);

    bool CheckLineMatch(vector<int> orthogonal_line_activations,SegmentStartIDAndWidth& line_match, int SEARCH_LINE_CODE);
    void SafeLinePoint(SegmentStartIDAndWidth line_match, vector<Point> orthogonal_line_points, bool is_matched,
                       int point_in_search_direction_x,int point_in_search_direction_y, int current_to_next_point_distance,
                       float angle_to_next_point,int SEARCH_LINE_CODE);

      SearchLineDistanceThresholds GetSearchLineDistanceThresholds(int SEARCH_LINE_CODE);
    int GetMinPixelIntensityThreshold(int SEARCH_LINE_CODE);

     SearchLineWidthThresholds GetSearchLineWidthThresholds(int SEARCH_LINE_CODE);

     vector<PointInDirection> GetLineDirections(int SEARCH_LINE_CODE);

     void FindValidPoints(vector<PointInDirection> line_directions, int SEARCH_LINE_CODE);

     void SetOuterLineDirections(vector<PointInDirection> line_directions, int SEARCH_LINE_CODE);


        MinMaxLineElements GetLinesMinMaxElements(vector<Point> line);


        void FindLinePointConnections(vector<Point> line, vector<pair<int,int>> &line_point_connections);
        bool IsPermuted(int i, int j, vector<string> &used_permutations);

        void FindMinDistanceFromPredictionToAdjacentPoint(int SEARCH_LINE_CODE,
                                                        Point adjacent_point_prediction,
                                                        Point &min_distance_adjacent_point,
                                                        int &min_distance_adjacent_point_id,
                                                        int &min_distance);


        bool AdjacentValidationTableIsEmpty(int SEARCH_LINE_CODE);

        void ExamineValidationTable(int SEARCH_LINE_CODE, vector<LineValidationTable*>&table);

        int GetAdjacentPointDirection(int SEARCH_LINE_CODE, int min_distance_adjacent_point_id);

        Point GetAdjacentPointOriginPrediction(int SEARCH_LINE_CODE, int min_distance_adjacent_point_id);

public:
    ValidLinePointSearch();
    void SetImage(Mat image);
    void FindValidPointsFromMidLineSearch(vector<vector<PointInDirection>> mid_line_directions_clusters, int SEARCH_LINE_CODE);
    void FindValidPointsFromLineFollow(vector<PointInDirection> line_directions, int SEARCH_LINE_CODE);
    void DrawLinePoints(Mat &rgb, int SEARCH_LINE_CODE);
    void CreateValidationTables();
    void ClearValidationTables();
    void SearchValidPoints();
    void ComputePointScores();
    void DrawValidScorePoints(Mat &rgb);
     void DrawMergedPoints(Mat &rgb);
    void MergePoints();
    void SearchMinMax();
    void DrawTables(Mat &rgb);


    void JJ();

};

#endif // VALID_LINE_POINT_SEARCH_H
