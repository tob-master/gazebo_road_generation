#ifndef LINE_VALIDATION_TABLE_FEATURE_EXTRACTION_H
#define LINE_VALIDATION_TABLE_FEATURE_EXTRACTION_H

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
class LineValidationTableFeatureExtraction
{


private:

    enum {ROI_X_COLUMN, ROI_Y_COLUMN, ROI_WIDTH_COLUMN, ROI_HEIGHT_COLUMN, COMPONENT_SIZE_COLUMN};
    enum {CENTROIDS_X_COLUMN, CENTROIDS_Y_COLUMN};

    Mat current_image_;

    Mat labeled_image_;
    Mat components_stats_;
    Mat components_centroids_;
    int components_count_ = 0;

    const int kConnectionCount_ = 8;

    vector<LineValidationTable> left_line_validation_table_;
    vector<LineValidationTable> mid_line_validation_table_;
    vector<LineValidationTable> right_line_validation_table_;

    void ExtractValidPoints();

        void ExtractLinePointsInDriveDirection(vector<LineValidationTable>& line_validation_table, vector<LineValidationTable>& line_points_in_drive_direction );
            vector<LineValidationTable> left_line_points_in_drive_direction_;
            vector<LineValidationTable> mid_line_points_in_drive_direction_;
            vector<LineValidationTable> right_line_points_in_drive_direction_;
            const int kMinStartDirection_ =  30;
            const int kMaxStartDirection_ = 150;
            const int kMaxDirectionDifference_ = 30;
/*
        void ExtractMinMaxLineElements( vector<LineValidationTable>  line,  MinMaxLineElements& line_minmax_elements );
            MinMaxLineElements left_line_minmax_elements_;
            MinMaxLineElements mid_line_minmax_elements_;
            MinMaxLineElements right_line_minmax_elements_;

        void ExtractLastMatchingPoints(vector<LineValidationTable> line_validation_table_,vector<LineValidationTable> line_direction_in_range_,int LINE_CODE);
            LastAdjacentPointMatch left_line_last_adjacent_point_match_;
            LastAdjacentPointMatch mid_line_last_adjacent_point_match_;
            LastAdjacentPointMatch right_line_last_adjacent_point_match_;
*/
        void ExtractMinMaxLineElements( vector<LineValidationTable>  line,  MinMaxLineElements& line_minmax_elements );

public:
        void SearchParkingArea(Mat &rgb);
    //LineValidationTableFeatureExtraction();
        void LoadImage(Mat image){current_image_ = image;};
    void LoadLineValidationTables(vector<LineValidationTable> left_line_validation_table,vector<LineValidationTable>mid_line_validation_table,vector<LineValidationTable>right_line_validation_table);
    void GetLinePointsInDriveDirection(vector<LineValidationTable> &l,vector<LineValidationTable> &m,vector<LineValidationTable> &r);
    void DrawLinePointsInDriveDirection(Mat &rgb);
    void SearchCrossRoad(Mat &rgb);
    void ClearMemory();

};

#endif // LINE_VALIDATION_TABLE_FEATURE_EXTRACTION_H
