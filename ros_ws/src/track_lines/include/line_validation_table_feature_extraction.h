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

class LineValidationTableFeatureExtraction
{


private:

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

public:
    //LineValidationTableFeatureExtraction();
    void LoadLineValidationTables(vector<LineValidationTable> left_line_validation_table,vector<LineValidationTable>mid_line_validation_table,vector<LineValidationTable>right_line_validation_table);
    void GetLinePointsInDriveDirection(vector<LineValidationTable> &l,vector<LineValidationTable> &m,vector<LineValidationTable> &r);
    void DrawLinePointsInDriveDirection(Mat &rgb);
    void ClearMemory();
};

#endif // LINE_VALIDATION_TABLE_FEATURE_EXTRACTION_H
