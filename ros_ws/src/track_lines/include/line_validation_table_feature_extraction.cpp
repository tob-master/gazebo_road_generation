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


void LineValidationTableFeatureExtraction::SearchParkingArea(Mat &rgb)
{
    components_stats_.release();
    components_centroids_.release();
    labeled_image_.release();
    components_count_ = connectedComponentsWithStats(current_image_, labeled_image_, components_stats_, components_centroids_, kConnectionCount_, CV_32S);

    Mat under_max_size_components_truth_table = components_stats_.col(COMPONENT_SIZE_COLUMN)<1280*100;
    Mat over_min_size_components_truth_table  = components_stats_.col(COMPONENT_SIZE_COLUMN)>30000;


        under_max_size_components_truth_table.convertTo(under_max_size_components_truth_table, CV_8UC1);
        over_min_size_components_truth_table.convertTo(over_min_size_components_truth_table, CV_8UC1);

        for (int id = 0; id < components_count_; id++)
        {
            if (under_max_size_components_truth_table.at<uchar>(id, 0)&&
                over_min_size_components_truth_table.at<uchar>(id, 0))
            {

                int component_centroid_x = components_centroids_.at<float>(id,CENTROIDS_X_COLUMN);
                int component_centroid_y = components_centroids_.at<float>(id,CENTROIDS_Y_COLUMN);
                int component_roi_x      = components_stats_.at<int>(id,ROI_X_COLUMN);
                int component_roi_y      = components_stats_.at<int>(id,ROI_Y_COLUMN);
                int component_roi_width  = components_stats_.at<int>(id,ROI_WIDTH_COLUMN);
                int component_roi_height = components_stats_.at<int>(id,ROI_HEIGHT_COLUMN);
                int component_size       = components_stats_.at<int>(id,COMPONENT_SIZE_COLUMN);


                cout <<component_centroid_x << endl;
                cout <<component_centroid_y << endl;
                cout <<component_roi_x << endl;
                cout <<component_roi_y << endl;
                cout <<component_roi_width << endl;
                cout <<component_roi_height << endl;
                cout <<component_size << endl;
                cout <<"#######################" << endl;


                rectangle(rgb, Rect(component_roi_x,component_roi_y,component_roi_width,component_roi_height), Scalar(0, 0, 255), 1, 8 );

            }
        }


    //cout << components_stats_ << endl;

    //components_centroids_.at<float>(id,CENTROIDS_X_COLUMN);

   //int component_roi_x      = components_stats_.at<int>(id,ROI_X_COLUMN);

}

void LineValidationTableFeatureExtraction::SearchCrossRoad(Mat &rgb)
{

 bool mid_crossing = false;
 if(mid_line_validation_table_[mid_line_validation_table_.size()-1].GetLabel() > 0)
 {
  for(auto it: mid_line_validation_table_)
    {
        if( it.GetDirection()>340 || it.GetDirection() < 20 || (it.GetDirection() < 200 && it.GetDirection() > 160))
        {
                mid_crossing = true;
                //crossing_score++;
        }
    }
 }
    //cout << "Cscore: " << crossing_score << endl;


        int crossing_score = 0;

    bool left_crossing = false;
    if(left_line_validation_table_.size()>0)
    {

        int tmp_direction = left_line_validation_table_[0].GetDirection();


            for(int i=0; i<left_line_validation_table_.size(); i++)
            {
                if( left_line_validation_table_[i].GetOriginPoint().y < 220) break;
                int current_direction =  left_line_validation_table_[i].GetDirection();
                int direction_difference = abs(current_direction - tmp_direction);

                if(direction_difference > 80)// && (360 - 80) < direction_difference)
                {
                    if(current_direction > 140 && current_direction < 220)
                    {    left_crossing = true;
                         break;
                    }
                }
                tmp_direction = current_direction;
            }

    }


    bool right_crossing = false;
    if(right_line_validation_table_.size()>0)
    {

        int tmp_direction = right_line_validation_table_[0].GetDirection();


            for(int i=0; i<right_line_validation_table_.size(); i++)
            {
                if( right_line_validation_table_[i].GetOriginPoint().y < 220) break;
                int current_direction =  right_line_validation_table_[i].GetDirection();
                int direction_difference = abs(current_direction - tmp_direction);

                if(direction_difference > 80)// || (360 - 80) < direction_difference)
                {
                    if(current_direction > 320 || current_direction < 40)
                    {    right_crossing = true;
                         break;
                    }
                }
                tmp_direction = current_direction;
            }

    }

    MinMaxLineElements left_line_minmax_elements_;
    MinMaxLineElements right_line_minmax_elements_;

    ExtractMinMaxLineElements(left_line_points_in_drive_direction_ , left_line_minmax_elements_ );
    ExtractMinMaxLineElements(right_line_points_in_drive_direction_, right_line_minmax_elements_ );

    bool left_ymin_is_small = left_line_minmax_elements_.y_min.y  > 250;
    bool right_y_min_is_small = right_line_minmax_elements_.y_min.y  > 250;
    bool left_size_is_small  = left_line_points_in_drive_direction_.size() < 150;
    bool right_size_is_small = right_line_points_in_drive_direction_.size()< 150 ;

    //if (left_line_minmax_elements_.y_min.y  > 250) cout << "lmin" << endl;
    //if (right_line_minmax_elements_.y_min.y > 250) cout << "rmin" << endl;




    bool crossing = false;

    if(left_crossing && right_crossing && mid_crossing && left_ymin_is_small && right_y_min_is_small && left_size_is_small && right_size_is_small){ cout << "lrm_ls_rs crossing" << endl;
        cout << left_line_points_in_drive_direction_.size() << endl;
        cout << right_line_points_in_drive_direction_.size() << endl;
        crossing= true;}
    else if(left_crossing && mid_crossing && left_ymin_is_small && right_y_min_is_small && left_size_is_small && right_size_is_small){ cout << "lm_ls_rs crossing" << endl;
        cout << left_line_points_in_drive_direction_.size() << endl;
        cout << right_line_points_in_drive_direction_.size() << endl;
        crossing= true;}
    else if(right_crossing && mid_crossing && left_ymin_is_small && right_y_min_is_small && left_size_is_small && right_size_is_small){ cout << "rm_ls_rs crossing" << endl;
        cout << left_line_points_in_drive_direction_.size() << endl;
        cout << right_line_points_in_drive_direction_.size() << endl;
        crossing= true;}
    else if (left_crossing && right_crossing && left_ymin_is_small && right_y_min_is_small && left_size_is_small && right_size_is_small){ cout << "lr_ls_rs crossing" << endl;
        cout << left_line_points_in_drive_direction_.size() << endl;
        cout << right_line_points_in_drive_direction_.size() << endl;
        crossing= true;}
    else if (right_crossing && mid_crossing && right_y_min_is_small && left_size_is_small && right_size_is_small){ cout << "rm_rs crossing" << endl;
        cout << left_line_validation_table_.size() << endl;
        cout << right_line_points_in_drive_direction_.size() << endl;
        crossing= true;}
    else if(mid_crossing && right_y_min_is_small && left_ymin_is_small && left_size_is_small && right_size_is_small){ cout << "m_ls_rs crossing" << endl;
        cout << left_line_points_in_drive_direction_.size() << endl;
        cout << right_line_points_in_drive_direction_.size() << endl;
        crossing= true;}
    else if(mid_crossing && left_ymin_is_small && left_size_is_small && right_size_is_small){ cout << "m_ls crossing" << endl;
        cout << left_line_points_in_drive_direction_.size() << endl;
        cout << right_line_points_in_drive_direction_.size() << endl;
        crossing= true;}
    else if(mid_crossing && right_y_min_is_small && left_size_is_small && right_size_is_small){ cout << "m_rs crossing" << endl;
        cout << left_line_points_in_drive_direction_.size() << endl;
        cout << right_line_points_in_drive_direction_.size() << endl;
        crossing= true;}



    if(crossing)
    {
        circle(rgb,  right_line_minmax_elements_.y_min, 10, Scalar(0, 0, 255),CV_FILLED );
         //imshow("bird_rdgb", rgb);
        //waitKey(0);
    }

    //else if(left_crossing) cout << "l crossing" << endl;
    //else if(right_crossing) cout << "r crossing" << endl;

    //cout << left_crossing << " " << right_crossing << endl;
//cout << "Cscore: " << crossing_score << endl;

    /*
    for(auto it: mid_line_validation_table_)



        cout << it.GetLabel()<< " " << it.GetNextDirectionDistance() << " ";


    vector<vector<int>> cluster_directions;


    int num_clusters = mid_line_validation_table_.size();

    for(int i=0;i<num_clusters;i++)
    {



        int distance_id = 0;
        vector<int> directions;
        while(distance_id < mid_line_validation_table_[i].size())
        {
            directions.push_back(mid_line_validation_table_[distance_id].GetDirection());

            int next_direction_distance = mid_line_validation_table_[distance_id].GetNextDirectionDistance();

            distance_id += next_direction_distance;
        }
        cluster_directions.push_back(directions);


    }
*/




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

        if(tmp_direction > kMinStartDirectionOfLinePointsInDriveDirection_ && tmp_direction < kMaxStartDirectionOfLinePointsInDriveDirection_)
        {
            for(int i=0; i<line_validation_table.size(); i++)
            {
                int current_direction =  line_validation_table[i].GetDirection();
                int direction_difference = abs(current_direction - tmp_direction);

                if(direction_difference > kMaxDirectionDifferenceOfLinePointsInDriveDirection_ || (360 - kMaxDirectionDifferenceOfLinePointsInDriveDirection_) < direction_difference)
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


//void LineValidationTableFeatureExtraction::FindSafestPointsOnTrack

void LineValidationTableFeatureExtraction::DrawLinePointsInDriveDirection(Mat &rgb)
{


    for(int i=0; i<left_line_points_in_drive_direction_.size(); i++)
    {

            circle(rgb, left_line_points_in_drive_direction_[i].GetOriginPoint(), 1, Scalar(0, 255, 255),CV_FILLED );

    }

    for(int i=0; i<mid_line_points_in_drive_direction_.size(); i++)
    {

            circle(rgb, mid_line_points_in_drive_direction_[i].GetOriginPoint(), 1, Scalar(255, 0, 255),CV_FILLED );

    }

    for(int i=0; i<right_line_points_in_drive_direction_.size(); i++)
    {

            circle(rgb, right_line_points_in_drive_direction_[i].GetOriginPoint(), 1, Scalar(255, 255, 0),CV_FILLED );

    }

}



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

/*
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
