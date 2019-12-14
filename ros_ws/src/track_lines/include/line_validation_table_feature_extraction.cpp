#include "line_validation_table_feature_extraction.h"


void LineValidationTableFeatureExtraction::LoadLineValidationTables(vector<LineValidationTable> left_line_validation_table,vector<LineValidationTable>mid_line_validation_table,vector<LineValidationTable>right_line_validation_table)
{

    left_line_validation_table_  = left_line_validation_table;
    mid_line_validation_table_   = mid_line_validation_table;
    right_line_validation_table_ = right_line_validation_table;

}

/*
void LineValidationTableFeatureExtraction::ExtractValidPoints()
{

ExtractLinePointsInDriveDirection(left_line_validation_table_, left_line_points_in_drive_direction_ );
ExtractLinePointsInDriveDirection(mid_line_validation_table_, mid_line_points_in_drive_direction_ );
ExtractLinePointsInDriveDirection(right_line_validation_table_,right_line_points_in_drive_direction_ );

ExtractMinMaxLineElements(left_line_points_in_drive_direction_ , left_line_minmax_elements_ );
ExtractMinMaxLineElements(mid_line_points_in_drive_direction_  , mid_line_minmax_elements_ );
ExtractMinMaxLineElements(right_line_points_in_drive_direction_, right_line_minmax_elements_ );

ExtractLastMatchingPoints(left_line_points_in_drive_direction_,left_line_points_in_drive_direction_,LEFT_LINE);
ExtractLastMatchingPoints(mid_line_points_in_drive_direction_,mid_line_points_in_drive_direction_,MID_LINE);
ExtractLastMatchingPoints(right_line_points_in_drive_direction_,right_line_points_in_drive_direction_,RIGHT_LINE);


}
*/

void LineValidationTableFeatureExtraction::GetLinePointsInDriveDirection(vector<LineValidationTable> &l,vector<LineValidationTable> &m,vector<LineValidationTable> &r)
{

    left_line_points_in_drive_direction_.clear();
    mid_line_points_in_drive_direction_.clear();
    right_line_points_in_drive_direction_.clear();

    ExtractLinePointsInDriveDirection(left_line_validation_table_, left_line_points_in_drive_direction_ );
    ExtractLinePointsInDriveDirection(mid_line_validation_table_, mid_line_points_in_drive_direction_ );
    ExtractLinePointsInDriveDirection(right_line_validation_table_,right_line_points_in_drive_direction_ );


    l = left_line_points_in_drive_direction_;
    m = mid_line_points_in_drive_direction_;
    r = right_line_points_in_drive_direction_;
}

void LineValidationTableFeatureExtraction::ClearMemory()
{
    cout << "e.what()" << endl;
}


void LineValidationTableFeatureExtraction::ExtractLinePointsInDriveDirection(vector<LineValidationTable>& line_validation_table, vector<LineValidationTable>& line_points_in_drive_direction )
{

    // only take line points which have low change in direction
    // if they have then theres an issue like a cross section

    if(line_validation_table.size()>0)
    {
        int cut_off_id = 0;
        bool cut_off = false;

        int tmp_direction = line_validation_table[0].GetDirection();

        if(tmp_direction > kMinStartDirection_ && tmp_direction < kMaxStartDirection_)
        {
            for(int i=0; i<line_validation_table.size(); i++)
            {
                int current_direction =  line_validation_table[i].GetDirection();
                int direction_difference = abs(current_direction - tmp_direction);

                if(direction_difference > kMaxDirectionDifference_ || (360 - kMaxDirectionDifference_) < direction_difference)
                {
                    cut_off_id =  i;
                    cut_off = true;
                    break;
                }
                tmp_direction = current_direction;
            }

            if(cut_off)
            {
                for(int i=0; i<cut_off_id; i++) line_points_in_drive_direction.emplace_back(line_validation_table[i]);
            }
            else{
                    copy(line_validation_table.begin(), line_validation_table.end(), back_inserter(line_points_in_drive_direction));
            }
        }
    }
}


void LineValidationTableFeatureExtraction::DrawLinePointsInDriveDirection(Mat &rgb)
{


    for(int i=0; i<left_line_points_in_drive_direction_.size(); i++)
    {

            circle(rgb, left_line_points_in_drive_direction_[i].GetOriginPoint(), 1, Scalar(0, 0, 255),CV_FILLED );

    }

    for(int i=0; i<mid_line_points_in_drive_direction_.size(); i++)
    {

            circle(rgb, mid_line_points_in_drive_direction_[i].GetOriginPoint(), 1, Scalar(255, 0, 0),CV_FILLED );

    }

    for(int i=0; i<right_line_points_in_drive_direction_.size(); i++)
    {

            circle(rgb, right_line_points_in_drive_direction_[i].GetOriginPoint(), 1, Scalar(0, 255, 0),CV_FILLED );

    }

}


/*
void LineValidationTableFeatureExtraction::ExtractMinMaxLineElements( vector<LineValidationTable>  line,  MinMaxLineElements& line_minmax_elements )
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


void LineValidationTableFeatureExtraction::ExtractLastMatchingPoints(vector<LineValidationTable> line_validation_table_,
                                                     vector<LineValidationTable> line_direction_in_range_,
                                                     int LINE_CODE)
{
    int last_id_adjacent1 = 0;
    int last_id_adjacent2 = 0;
    bool changed_adjacent1 = false;
    bool changed_adjacent2 = false;


    for(int i=0; i<line_direction_in_range_.size(); i++)
    {
        if(line_direction_in_range_[i].GetAdjacent1Prediction(LINE_CODE))
        {
            last_id_adjacent1 = line_direction_in_range_[i].GetAdjacent1Id(LINE_CODE);
            changed_adjacent1 = true;
        }

        if(line_direction_in_range_[i].GetAdjacent2Prediction(LINE_CODE))
        {
            last_id_adjacent2 = line_direction_in_range_[i].GetAdjacent2Id(LINE_CODE);
            changed_adjacent2 = true;
        }
    }

    if(LINE_CODE == LEFT_LINE)
    {
        LastAdjacentPointMatch last_adjacent_point_match;

        if(changed_adjacent1)
        {
            int last_mid_id    = last_id_adjacent1;
            Point last_mid_point = mid_line_validation_table_[last_mid_id].GetOriginPoint();

            last_adjacent_point_match.mid_set        = true;
            last_adjacent_point_match.last_mid_point = last_mid_point;
            last_adjacent_point_match.last_mid_id    = last_mid_id;
        }

        if(changed_adjacent2)
        {
            int last_right_id    = last_id_adjacent2;
            Point last_right_point = right_line_validation_table_[last_right_id].GetOriginPoint();

            last_adjacent_point_match.right_set        = true;
            last_adjacent_point_match.last_right_point = last_right_point;
            last_adjacent_point_match.last_right_id    = last_right_id;
        }

        left_line_last_adjacent_point_match_ = last_adjacent_point_match;

    }


    if(LINE_CODE == MID_LINE)
    {
        LastAdjacentPointMatch last_adjacent_point_match;

        if(changed_adjacent1)
        {
            int last_left_id    = last_id_adjacent1;
            Point last_left_point = left_line_validation_table_[last_left_id].GetOriginPoint();

            last_adjacent_point_match.left_set        = true;
            last_adjacent_point_match.last_left_point = last_left_point;
            last_adjacent_point_match.last_left_id    = last_left_id;
        }

        if(changed_adjacent2)
        {
            int last_right_id    = last_id_adjacent2;
            Point last_right_point = right_line_validation_table_[last_right_id].GetOriginPoint();

            last_adjacent_point_match.right_set        = true;
            last_adjacent_point_match.last_right_point = last_right_point;
            last_adjacent_point_match.last_right_id    = last_right_id;
        }

        mid_line_last_adjacent_point_match_ = last_adjacent_point_match;

    }

    if(LINE_CODE == RIGHT_LINE)
    {
        LastAdjacentPointMatch last_adjacent_point_match;

        if(changed_adjacent1)
        {
            int last_left_id    = last_id_adjacent1;
            Point last_left_point = left_line_validation_table_[last_left_id].GetOriginPoint();

            last_adjacent_point_match.left_set        = true;
            last_adjacent_point_match.last_left_point = last_left_point;
            last_adjacent_point_match.last_left_id    = last_left_id;
        }

        if(changed_adjacent2)
        {
            int last_mid_id    = last_id_adjacent2;
            Point last_mid_point = mid_line_validation_table_[last_mid_id].GetOriginPoint();

            last_adjacent_point_match.mid_set        = true;
            last_adjacent_point_match.last_mid_point = last_mid_point;
            last_adjacent_point_match.last_mid_id    = last_mid_id;
        }

        right_line_last_adjacent_point_match_ = last_adjacent_point_match;

    }

}
*/
