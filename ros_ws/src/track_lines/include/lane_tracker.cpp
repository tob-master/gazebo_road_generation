#include "lane_tracker.h"


void LaneTracker::CheckStartParameters()
{
    found_start_of_lines_    = start_of_lines_search_return_info_.has_found_start_parameters;
    found_left_hough_line_   = vanishing_point_search_return_info_.has_found_left_hough_line;
    found_right_hough_line_  = vanishing_point_search_return_info_.has_found_right_hough_line;
    found_vanishing_point_   = vanishing_point_search_return_info_.has_found_vanishing_point;

    StartParameters start_of_lines_start_parameters = StartOfLinesSearcher_->GetStartParameters();
    StartParameters vanishing_point_start_parameters = VanishingPointSearcher_->GetLineFollowerStartParameters();


    /*
    int   sol_left_x = start_of_lines_start_parameters.left_x;
    int   sol_left_y = start_of_lines_start_parameters.left_y;
    float sol_left_angle = start_of_lines_start_parameters.left_angle;
    int   sol_right_x = start_of_lines_start_parameters.right_x;
    int   sol_right_y = start_of_lines_start_parameters.right_y;
    float sol_right_angle = start_of_lines_start_parameters.right_angle;

    int   vp_left_x = vanishing_point_start_parameters.left_x;
    int   vp_left_y = vanishing_point_start_parameters.left_y;
    float vp_left_angle = vanishing_point_start_parameters.left_angle;
    int   vp_right_x = vanishing_point_start_parameters.right_x;
    int   vp_right_y = vanishing_point_start_parameters.right_y;
    float vp_right_angle = vanishing_point_start_parameters.right_angle;
    */



    if(found_start_of_lines_ && found_vanishing_point_)
    {
       start_parameters_for_line_follower_ = StartOfLinesSearcher_->GetStartParameters();
       START_PARAMETERS_SCORE_ = 5;
       found_start_of_left_line_ = true;
       found_start_of_mid_line_ = true;
       found_start_of_right_line_ = true;
    }
    else if (found_start_of_lines_)
    {
        start_parameters_for_line_follower_ = StartOfLinesSearcher_->GetStartParameters();
        START_PARAMETERS_SCORE_ = 4;
        found_start_of_left_line_ = true;
        found_start_of_mid_line_ = true;
        found_start_of_right_line_ = true;
    }
    else if (found_vanishing_point_)
    {
        start_parameters_for_line_follower_ = VanishingPointSearcher_->GetLineFollowerStartParameters();
        START_PARAMETERS_SCORE_ = 3;
        found_start_of_left_line_ = true;
        found_start_of_mid_line_ = false;
        found_start_of_right_line_ = true;
    }
    else if (found_left_hough_line_ && found_right_hough_line_)
    {
        start_parameters_for_line_follower_ = VanishingPointSearcher_->GetLineFollowerStartParameters();
        START_PARAMETERS_SCORE_ = 2;
        found_start_of_left_line_ = true;
        found_start_of_mid_line_ = false;
        found_start_of_right_line_ = true;
    }
    else if (found_left_hough_line_)
    {
        start_parameters_for_line_follower_ = VanishingPointSearcher_->GetLineFollowerStartParameters();
        START_PARAMETERS_SCORE_ = 1;
        found_start_of_left_line_ = true;
        found_start_of_mid_line_ = false;
        found_start_of_right_line_ = false;
    }
    else if (found_right_hough_line_)
    {
        start_parameters_for_line_follower_ = VanishingPointSearcher_->GetLineFollowerStartParameters();
        START_PARAMETERS_SCORE_ = 1;
        found_start_of_left_line_ = false;
        found_start_of_mid_line_ = false;
        found_start_of_right_line_ = true;
    }
    else
    {
        START_PARAMETERS_SCORE_ = 0;
        found_start_of_left_line_ = false;
        found_start_of_mid_line_ = false;
        found_start_of_right_line_ = false;
    }



}

float GetOrthogonalAngle(float angle, int SEARCH_LINE_CODE)
{
    int  angle_ = 0;

    if(SEARCH_LINE_CODE == LEFT_TO_MID || SEARCH_LINE_CODE == LEFT_TO_RIGHT || SEARCH_LINE_CODE == MID_TO_RIGHT) angle_ = (angle - 90);

    if(SEARCH_LINE_CODE == RIGHT_TO_MID || SEARCH_LINE_CODE == RIGHT_TO_LEFT || SEARCH_LINE_CODE == MID_TO_LEFT) angle_ = (angle + 90);

    if(angle_ > 359) angle_ = angle_ % 360;

    if(angle_ < 0) angle_ =  360 - abs(angle_);

    return float(angle_);
}



Point GetPolarCoordinate(int x, int y, float angle, int radius)
{
    x = x + radius * cos(angle) + 0.5;
    y = y - radius * sin(angle) + 0.5;

    return Point(x,y);
}





void LaneTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");

        // blackout wheels
        circle(cv_ptr->image, Point(709,454),35, Scalar(0,0,0),CV_FILLED, 8,0);
        circle(cv_ptr->image, Point(574,454),35, Scalar(0,0,0),CV_FILLED, 8,0);

        image_mono_ = cv_ptr->image;

        cvtColor(image_mono_, image_rgb_, CV_GRAY2BGR);

        warpPerspective(image_mono_, image_mono_bird_, birdseye_transformation_matrix_, kInputImageSize_, INTER_CUBIC | WARP_INVERSE_MAP);
        image_mono_bird_ = image_mono_bird_(Rect(0,0,image_width_,image_height_));
        threshold(image_mono_bird_, image_mono_bird_otsu_, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

        cvtColor(image_mono_bird_, image_rgb_bird_, CV_GRAY2BGR);


        VanishingPointSearcher_->ClearMemory();
        VanishingPointSearcher_->SetImage(image_mono_);
        vanishing_point_search_return_info_ = VanishingPointSearcher_->FindVanishingPoint();




        StartOfLinesSearcher_->ClearMemory();
        StartOfLinesSearcher_->SetImage(image_mono_bird_otsu_);
        start_of_lines_search_return_info_ = StartOfLinesSearcher_->FindStartParameters();

        StartOfLinesSearcher_->DrawStartParameters(image_rgb_bird_);
        //VanishingPointSearcher_->ShowCannyEdgeImage();
        VanishingPointSearcher_->DrawHoughLines(image_rgb_,LEFT_LINE);
        VanishingPointSearcher_->DrawHoughLines(image_rgb_,RIGHT_LINE);
        VanishingPointSearcher_->DrawVanishingPoint(image_rgb_);

        CheckStartParameters();

/*
        if(found_vanishing_point_)
        {

            StartParameters vanishing_point_start_parameters = VanishingPointSearcher_->GetLineFollowerStartParameters();


            }





        if(found_vanishing_point_)
        {

            StartParameters vanishing_point_start_parameters = VanishingPointSearcher_->GetLineFollowerStartParameters();



        }
*/


         if(START_PARAMETERS_SCORE_ != 0)
          {
                LineFollower_->ClearMemory();
                LineFollower_->SetImage(image_mono_bird_);
                LineFollower_->SetStartParameters(start_parameters_for_line_follower_);
                line_follower_return_info_ = LineFollower_->FollowLines();

                left_line_follower_min_iterations_reached_ = line_follower_return_info_.left_line_iterations_counter  >= kMinLineFollowerIterationsCount;
                right_line_follower_min_iterations_reached_ = line_follower_return_info_.right_line_iterations_counter >= kMinLineFollowerIterationsCount;

                if(left_line_follower_min_iterations_reached_) LineFollower_->GetLine(left_line_from_line_follower_, LEFT_LINE);
                if(right_line_follower_min_iterations_reached_)LineFollower_->GetLine(right_line_from_line_follower_, RIGHT_LINE);


                //LineFollower_->DrawLinePoints(image_rgb_bird_,LEFT_LINE);
                //LineFollower_->DrawLinePoints(image_rgb_bird_,RIGHT_LINE);
                line_points_reducer_return_info_ = LinePointsReducer_->ReduceLinePoints(left_line_from_line_follower_,right_line_from_line_follower_);

                left_line_is_reduced_  = line_points_reducer_return_info_.left_line_is_reduced;
                right_line_is_reduced_ = line_points_reducer_return_info_.right_line_is_reduced;

                if(left_line_is_reduced_)  LinePointsReducer_->GetLengthAndDirectionFromConsecutiveReducedLinePoints(left_line_lengths_and_directions_from_line_points_reducer_, LEFT_LINE);
                if(right_line_is_reduced_) LinePointsReducer_->GetLengthAndDirectionFromConsecutiveReducedLinePoints(right_line_lengths_and_directions_from_line_points_reducer_,RIGHT_LINE);


                //LinePointsReducer_->DrawReducedLinePoints(image_rgb_bird_,LEFT_LINE);
                //LinePointsReducer_->DrawReducedLinePoints(image_rgb_bird_,RIGHT_LINE);
         }

        MidLineSearcher_->ClearMemory();
        MidLineSearcher_->SetImage(image_mono_bird_);
        mid_line_search_return_info_ = MidLineSearcher_->FindMidLineClusters();

        mid_lines_found_ = mid_line_search_return_info_.has_found_mid_line_clusters;
        mid_line_groups_found_ = mid_line_search_return_info_.has_found_group;

        if(mid_line_groups_found_)
        {
             mid_line_groups_from_mid_line_searcher_ = MidLineSearcher_->GetGroupedMidLineClustersLengthAndDirection();

             //MidLineSearcher_->DrawGroupedMidLineClustersDirections(image_rgb_bird_);
        }


        //connected_component_search_return_info_ = ConnectedComponentsSearcher_->FindConnectedComponents(image_mono_bird_otsu_);

        LineValidationTableCreator_->ClearMemory();
        LineValidationTableCreator_->SetImage(image_mono_bird_);

        if(line_points_reducer_return_info_.left_line_is_reduced)
        {
            LineValidationTableCreator_->FindValidPointsFromLineFollow(left_line_lengths_and_directions_from_line_points_reducer_, LEFT_TO_RIGHT);
            LineValidationTableCreator_->FindValidPointsFromLineFollow(left_line_lengths_and_directions_from_line_points_reducer_, LEFT_TO_MID);
        }

        if(line_points_reducer_return_info_.right_line_is_reduced)
        {
            LineValidationTableCreator_->FindValidPointsFromLineFollow(right_line_lengths_and_directions_from_line_points_reducer_,RIGHT_TO_LEFT);
            LineValidationTableCreator_->FindValidPointsFromLineFollow(right_line_lengths_and_directions_from_line_points_reducer_,RIGHT_TO_MID);
        }

        if(mid_line_search_return_info_.has_found_group)
        {

            LineValidationTableCreator_->FindValidPointsFromMidLineSearch(mid_line_groups_from_mid_line_searcher_, MID_TO_LEFT);
            LineValidationTableCreator_->FindValidPointsFromMidLineSearch(mid_line_groups_from_mid_line_searcher_, MID_TO_RIGHT);
        }


        line_validatiohn_table_creation_return_info_ = LineValidationTableCreator_->CreateLineValidationTables();

        //LineValidationTableCreator_->DrawReturnInfo(image_rgb_bird_);
        //cout << line_validatiohn_table_creation_return_info_.left_found_both_points_and_predictions << endl;
        //cout << line_validatiohn_table_creation_return_info_.mid_found_both_points_and_predictions << endl;
        //cout << line_validatiohn_table_creation_return_info_.right_found_both_points_and_predictions << endl;
        //cout << "###" << endl;

        LineValidationTableCreator_->GetLineValidationTables(left_line_validation_table_,mid_line_validation_table_,right_line_validation_table_);

        LineValidationTableCreator_->GetLinePointsInDriveDirection(left_line_in_drive_direction_table_,mid_line_in_drive_direction_table_,right_line_in_drive_direction_table_);

        //LineValidationTableCreator_->DrawLinePointsInDriveDirection(image_rgb_bird_);

        SafeDriveAreaEvaluator_->ClearMemory();

        SafeDriveAreaEvaluator_->LoadLinePointsInDriveDirection(left_line_in_drive_direction_table_,mid_line_in_drive_direction_table_,right_line_in_drive_direction_table_);


        vector<SafeDriveAreaEvaluationReturnInfo> safe_drive_area_evaluation_return_info_ =  SafeDriveAreaEvaluator_->EvaluateTrackInDriveDirection();



    /*

        for(auto it:safe_drive_area_evaluation_return_info_)
        {
            //cout << it.max_line_type << " " <<  it.max_line_score << " " << it.max_line_is_continous << endl;

            int line_type = it.max_line_type;

            vector<LineValidationTable> safest_table1;
            vector<LineValidationTable> safest_table2;

            if(line_type == LEFT_LINE)
            {
                safest_table1 = it.left_safest_table1;
                safest_table2 = it.left_safest_table2;
            }
            else if(line_type == MID_LINE)
            {
                safest_table1 = it.mid_safest_table1;
                safest_table2 = it.mid_safest_table2;
            }
            else if(line_type == RIGHT_LINE)
            {
                safest_table1 = it.right_safest_table1;
                safest_table2 = it.right_safest_table2;
            }

            //for(auto hh: safest_table1) cout << hh.GetOriginPoint() << endl;


        }
        cout << "######" << endl;
*/

        OnRoadSearcher_->SetImage(image_mono_bird_);
        OnRoadSearcher_->LoadSafeDriveAreaEvaluation(safe_drive_area_evaluation_return_info_);

        OnRoadSearcher_->LoadValidationTables(left_line_validation_table_,
                                              mid_line_in_drive_direction_table_,
                                              right_line_validation_table_);


        OnRoadSearcher_->LoadInDriveDirectionTables(left_line_in_drive_direction_table_,right_line_in_drive_direction_table_);


        if(found_start_of_lines_)
        {

            OnRoadSearcher_->SetStartParameters(StartOfLinesSearcher_->GetStartParameters());
            OnRoadSearcher_->SearchOnRoad();
        }
        else if (found_vanishing_point_)
        {

            OnRoadSearcher_->SetStartParameters(VanishingPointSearcher_->GetLineFollowerStartParameters());
            OnRoadSearcher_->SearchOnRoad();
        }



        OnRoadSearcher_->DrawEvaluatedSafetyAreas(image_rgb_bird_);

        //vector<TrackSafetyRects> SafeDriveAreaEvaluationReturnInfoVector_;

        //LineValidationTableCreator_.ValidateTrack(image_rgb_bird_);





        //PerceputalGrouper_.SetImage(image_mono_bird_);
        //PerceputalGrouper_.ComputeGroupingParameters();


        //PerceputalGrouper_.DrawHoughLines(image_rgb_bird_);
        //PerceputalGrouper_.DrawHoughLineMidPoints(image_rgb_bird_);
        //PerceputalGrouper_.DrawHoughLineProximityThresholded(image_rgb_bird_);
        //PerceputalGrouper_.ClearMemory();

        //StartParameters GetStartParameters();

        //DrawStartParameterScore(image_rgb_bird_);
        //DrawStartOfLinesBird(image_rgb_bird_);



        //DrawHoughLinesFront(image_rgb_);



       //DrawHoughLinesBird(image_rgb_bird_);


        //DrawLineFollowerBird(image_rgb_bird_);
        //DrawReducedLinePointsBird(image_rgb_bird_);

        //DrawAllMidLineClustersBird(image_rgb_bird_);
        //DrawSingleMidLinesBird(image_rgb_bird_);
        //DrawMidLineGroupsBird(image_rgb_bird_);

        //DrawCCLMidLineRectComponentsBird(image_rgb_bird_);
        //DrawCCLMidLineComponentsGroupsBird(image_rgb_bird_);

        //DrawValidatedLines(image_rgb_bird_);


        //DrawLinePointsInDriveDirection(image_rgb_bird_);


        //SafeDriveAreaEvaluator_.DrawEvaluatedSafetyAreasInDriveDirection(image_rgb_bird_);

        //DrawValidatedPointsOnDriveWay(image_rgb_bird_);
        //DrawValidatedSplines(image_rgb_bird_);
        //DrawValidatedSafetyAreas(image_rgb_bird_);


        //warpPerspective(image_rgb_bird_, image_rgb_warped_back_, birdseye_transformation_matrix_.inv(), Size(image_width_,image_height_), INTER_CUBIC | WARP_INVERSE_MAP);


        //imshow("ostu bird",image_mono_bird_otsu_);
        imshow("normal",image_rgb_);
        imshow("bird_rgb", image_rgb_bird_);
        //imshow("warped_back",image_rgb_warped_back_);
        waitKey(1);

                ClearTrackingData();

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.",
        msg->encoding.c_str());
    }
};



void LaneTracker::DrawLinePointsInDriveDirection(Mat &rgb)
{
    LineValidationTableFeatureExtractor_.DrawLinePointsInDriveDirection(image_rgb_bird_);
}

void LaneTracker::DrawStartParameterScore(Mat& rgb)
{
    string text = to_string(START_PARAMETERS_SCORE_);
    putText(rgb, text, Point(640, 390),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,255), 1, CV_AA);
}



void LaneTracker::DrawStartOfLinesBird(Mat& rgb)
{
    if(found_start_of_lines_) StartOfLinesSearcher_->DrawStartParameters(image_rgb_bird_);
}

void LaneTracker::ReduceLinePoints()
{
    LineFollower_->GetLine(left_line_from_line_follower_, LEFT_LINE);
    LineFollower_->GetLine(right_line_from_line_follower_, RIGHT_LINE);

    line_points_reducer_return_info_ = LinePointsReducer_->ReduceLinePoints(left_line_from_line_follower_,right_line_from_line_follower_);

    left_line_is_reduced_  = line_points_reducer_return_info_.left_line_is_reduced;
    right_line_is_reduced_ = line_points_reducer_return_info_.right_line_is_reduced;

    LinePointsReducer_->GetLengthAndDirectionFromConsecutiveReducedLinePoints(left_line_lengths_and_directions_from_line_points_reducer_, LEFT_LINE);
    LinePointsReducer_->GetLengthAndDirectionFromConsecutiveReducedLinePoints(right_line_lengths_and_directions_from_line_points_reducer_,RIGHT_LINE);

}






void LaneTracker::DrawCCLMidLineRectComponentsBird(Mat &rgb)
{
    if(connected_component_search_return_info_.has_found_mid_line_components)
    {
        ConnectedComponentsSearcher_->DrawMidLineComponentsRect(rgb);
    }
}



void LaneTracker::DrawCCLMidLineComponentsGroupsBird(Mat &rgb)
{
    if(connected_component_search_return_info_.has_found_mid_line_group)
    {
        ConnectedComponentsSearcher_->DrawGroupedMidLineComponentsDirections(rgb);
    }
}


void LaneTracker::DrawMidLineGroupsBird(Mat &rgb)
{
    if(mid_line_search_return_info_.has_found_group)
    {
        MidLineSearcher_->DrawGroupedMidLineClustersDirections(image_rgb_bird_);
    }
}


void LaneTracker::DrawAllMidLineClustersBird(Mat &rgb)
{
    if(mid_line_search_return_info_.has_found_mid_line_clusters)
    {
        MidLineSearcher_->DrawClusters(rgb);
    }
}

void LaneTracker::DrawSingleMidLinesBird(Mat &rgb)
{
    if(mid_line_search_return_info_.has_found_mid_line_clusters)
    {
        MidLineSearcher_->DrawSingleClusters(rgb);
    }
}

void LaneTracker::DrawReducedLinePointsBird(Mat &rgb)
{
    if(line_points_reducer_return_info_.left_line_is_reduced)
    {
        LinePointsReducer_->DrawReducedLinePoints(rgb,LEFT_LINE);
    }

    if(line_points_reducer_return_info_.right_line_is_reduced)
    {
        LinePointsReducer_->DrawReducedLinePoints(rgb,RIGHT_LINE);
    }
}



void LaneTracker::DrawLineFollowerBird(Mat& rgb)
{
    if(line_follower_return_info_.left_line_iterations_counter >= kMinLineFollowerIterationsCount)
    {
        LineFollower_->DrawLinePoints(rgb,LEFT_LINE);
    }

    if(line_follower_return_info_.right_line_iterations_counter >= kMinLineFollowerIterationsCount)
    {
        LineFollower_->DrawLinePoints(rgb,RIGHT_LINE);
    }
}

void LaneTracker::DrawHoughLinesFront(Mat& rgb)
{
    if(vanishing_point_search_return_info_.has_found_left_hough_line)
    {
       VanishingPointSearcher_->DrawHoughLines(rgb, LEFT_LINE);
    }

    if(vanishing_point_search_return_info_.has_found_right_hough_line)
    {
        VanishingPointSearcher_->DrawHoughLines(rgb, RIGHT_LINE);
    }

    if(vanishing_point_search_return_info_.has_found_intersections)
    {

        VanishingPointSearcher_->DrawLineIntersections(rgb);
    }

    if(vanishing_point_search_return_info_.has_found_vanishing_point)
    {
        VanishingPointSearcher_->DrawVanishingPoint(rgb);
    }
}

void LaneTracker::DrawHoughLinesBird(Mat& rgb)
{
    if(vanishing_point_search_return_info_.has_found_left_hough_line)
    {
       VanishingPointSearcher_->DrawWarpedPerspektiveHoughLines(rgb, LEFT_LINE);
    }

    if(vanishing_point_search_return_info_.has_found_right_hough_line)
    {
        VanishingPointSearcher_->DrawWarpedPerspektiveHoughLines(rgb, RIGHT_LINE);
    }

    //if(vanishing_point_search_return_info_.has_found_vanishing_point)
    //{
    //    VanishingPointSearcher_->DrawWarpedVanishingPointDirection(rgb);
    //}
}


void LaneTracker::ClearTrackingData()
{
    vanishing_point_search_return_info_.reset();
    start_parameters_for_line_follower_.reset();
    line_follower_return_info_.reset();
    line_points_reducer_return_info_.reset();
    mid_line_search_return_info_.reset();
    connected_component_search_return_info_.reset();
    start_of_lines_search_return_info_.reset();

    left_line_from_line_follower_.clear();
    right_line_from_line_follower_.clear();

    left_line_from_line_points_reducer_.clear();
    right_line_from_line_points_reducer_.clear();

    left_line_lengths_and_directions_from_line_points_reducer_.clear();
    right_line_lengths_and_directions_from_line_points_reducer_.clear();


    mid_line_groups_from_mid_line_searcher_;
}


void LaneTracker::LoadStartOfLinesSearchInitializationParameters()
{
    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/start_of_lines_search_init.yaml";
    const char *command = str.c_str();
    system(command);


    n.getParam("/start_of_lines_search_init/top_row", start_of_lines_search_init.top_row);
    n.getParam("/start_of_lines_search_init/mid_row", start_of_lines_search_init.mid_row);
    n.getParam("/start_of_lines_search_init/bottom_row", start_of_lines_search_init.bottom_row);
    n.getParam("/start_of_lines_search_init/min_line_width", start_of_lines_search_init.min_line_width);
    n.getParam("/start_of_lines_search_init/max_line_width", start_of_lines_search_init.max_line_width);
    n.getParam("/start_of_lines_search_init/min_track_width", start_of_lines_search_init.min_track_width);
    n.getParam("/start_of_lines_search_init/max_track_width", start_of_lines_search_init.max_track_width);
    n.getParam("/start_of_lines_search_init/window_size_for_line_search", start_of_lines_search_init.window_size_for_line_search);
    n.getParam("/start_of_lines_search_init/line_threshold", start_of_lines_search_init.line_threshold);
    n.getParam("/start_of_lines_search_init/mid_line_threshold", start_of_lines_search_init.mid_line_threshold);
    n.getParam("/start_of_lines_search_init/window_size_for_mid_line_search", start_of_lines_search_init.window_size_for_mid_line_search);
    n.getParam("/start_of_lines_search_init/max_distance_between_adjacent_row_pairs", start_of_lines_search_init.max_distance_between_adjacent_row_pairs);
    n.getParam("/start_of_lines_search_init/car_poCreateValidationTablessition_in_frame", start_of_lines_search_init.car_position_in_frame);
    n.getParam("/start_of_lines_search_init/road_model_left_line", start_of_lines_search_init.road_model_left_line);
    n.getParam("/start_of_lines_search_init/road_model_right_line", start_of_lines_search_init.road_model_right_line);
    n.getParam("/start_of_lines_search_init/line_to_car_distance_threshold", start_of_lines_search_init.line_to_car_distance_threshold);

}

void LaneTracker::LoadLineFollowerInitializationParameters()
{

    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/line_follower_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/line_follower_init/max_iterations", line_follower_init.max_iterations);
    n.getParam("/line_follower_init/search_radius", line_follower_init.search_radius);
    n.getParam("/line_follower_init/max_weight_direction_scaler", line_follower_init.max_weight_direction_scaler);
    n.getParam("/line_follower_init/field_of_view", line_follower_init.field_of_view);
    n.getParam("/line_follower_init/max_consecutive_back_steps", line_follower_init.max_consecutive_back_steps);
    n.getParam("/line_follower_init/min_travel_distance_to_not_got_stuck", line_follower_init.min_travel_distance_to_not_got_stuck);
    n.getParam("/line_follower_init/max_got_stuck_counts", line_follower_init.max_got_stuck_counts);

}

void LaneTracker::LoadVanishingPointSearchInitializationParameters()
{

    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/vanishing_point_search_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/vanishing_point_search_init/canny_low_threshold",vanishing_point_search_init.canny_low_threshold);
    n.getParam("/vanishing_point_search_init/canny_high_threshold",vanishing_point_search_init.canny_high_threshold);
    n.getParam("/vanishing_point_search_init/canny_kernel_size",vanishing_point_search_init.canny_kernel_size);
    n.getParam("/vanishing_point_search_init/hough_lines_rho",vanishing_point_search_init.hough_lines_rho);
    n.getParam("/vanishing_point_search_init/hough_lines_theta",vanishing_point_search_init.hough_lines_theta);
    n.getParam("/vanishing_point_search_init/hough_lines_min_intersections",vanishing_point_search_init.hough_lines_min_intersections);
    n.getParam("/vanishing_point_search_init/hough_lines_min_line_length",vanishing_point_search_init.hough_lines_min_line_length);
    n.getParam("/vanishing_point_search_init/hough_lines_min_line_gap",vanishing_point_search_init.hough_lines_min_line_gap);
    n.getParam("/vanishing_point_search_init/x_roi_start",vanishing_point_search_init.x_roi_start);
    n.getParam("/vanishing_point_search_init/y_roi_start",vanishing_point_search_init.y_roi_start);
    n.getParam("/vanishing_point_search_init/roi_width",vanishing_point_search_init.roi_width);
    n.getParam("/vanishing_point_search_init/roi_height",vanishing_point_search_init.roi_height);
    n.getParam("/vanishing_point_search_init/min_left_line_angle",vanishing_point_search_init.min_left_line_angle);
    n.getParam("/vanishing_point_search_init/max_left_line_angle",vanishing_point_search_init.max_left_line_angle);
    n.getParam("/vanishing_point_search_init/min_right_line_angle",vanishing_point_search_init.min_right_line_angle);
    n.getParam("/vanishing_point_search_init/max_right_line_angle",vanishing_point_search_init.max_right_line_angle);
    n.getParam("/vanishing_point_search_init/x_min_left_line",vanishing_point_search_init.x_min_left_line);
    n.getParam("/vanishing_point_search_init/x_max_left_line",vanishing_point_search_init.x_max_left_line);
    n.getParam("/vanishing_point_search_init/x_min_right_line",vanishing_point_search_init.x_min_right_line);
    n.getParam("/vanishing_point_search_init/x_max_right_line",vanishing_point_search_init.x_max_right_line);
    n.getParam("/vanishing_point_search_init/car_mid_position_x",vanishing_point_search_init.car_mid_position_x);
    n.getParam("/vanishing_point_search_init/car_mid_position_y",vanishing_point_search_init.car_mid_position_y);
    n.getParam("/vanishing_point_search_init/max_standard_deviation_for_valid_vanishing_point",vanishing_point_search_init.max_standard_deviation_for_valid_vanishing_point);

}

void LaneTracker::LoadBirdseyeInitializationParameters()
{
    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/birdseye_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/birdseye_init/alpha", birdseye_init.alpha);
    n.getParam("/birdseye_init/beta", birdseye_init.beta);
    n.getParam("/birdseye_init/gamma", birdseye_init.gamma);
    n.getParam("/birdseye_init/fov", birdseye_init.fov);
    n.getParam("/birdseye_init/distance", birdseye_init.distance);

};

void LaneTracker::LoadMidLineSearchInitializationParameters()
{
    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/mid_line_search_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/mid_line_search_init/min_pixel_value_for_clustering", mid_line_search_init.min_pixel_value_for_clustering);
    n.getParam("/mid_line_search_init/max_radial_scan_out_of_cluster_value", mid_line_search_init.max_radial_scan_out_of_cluster_value);
    n.getParam("/mid_line_search_init/radial_scan_scaling_factor", mid_line_search_init.radial_scan_scaling_factor);
    n.getParam("/mid_line_search_init/mid_line_length", mid_line_search_init.mid_line_length);
    n.getParam("/mid_line_search_init/min_valuable_cluster_size", mid_line_search_init.min_valuable_cluster_size);
    n.getParam("/mid_line_search_init/min_cluster_distance", mid_line_search_init.min_cluster_distance);
    n.getParam("/mid_line_search_init/max_cluster_distance", mid_line_search_init.max_cluster_distance);
    n.getParam("/mid_line_search_init/car_position_x", mid_line_search_init.car_position_x);
    n.getParam("/mid_line_search_init/car_position_y", mid_line_search_init.car_position_y);
}





void LaneTracker::LoadConnectedComponentsSearchInitializationParameters()
{

    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/connected_components_search_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/connected_components_search_init/connection_count", connected_components_search_init.connection_count);
    n.getParam("/connected_components_search_init/max_mid_line_component_size", connected_components_search_init.max_mid_line_component_size);
    n.getParam("/connected_components_search_init/min_mid_line_component_size", connected_components_search_init.min_mid_line_component_size);
    n.getParam("/connected_components_search_init/max_mid_line_component_volume", connected_components_search_init.max_mid_line_component_volume);
    n.getParam("/connected_components_search_init/min_mid_line_component_volume", connected_components_search_init.min_mid_line_component_volume);
    n.getParam("/connected_components_search_init/min_mid_line_component_distance", connected_components_search_init.min_mid_line_component_distance);
    n.getParam("/connected_components_search_init/max_mid_line_component_distance", connected_components_search_init.max_mid_line_component_distance);
    n.getParam("/connected_components_search_init/end_of_linkage_marker", connected_components_search_init.end_of_linkage_marker);
    n.getParam("/connected_components_search_init/max_roi_center_to_centroid_distance", connected_components_search_init.max_roi_center_to_centroid_distance);
    n.getParam("/connected_components_search_init/car_position_x", connected_components_search_init.car_position_x);
    n.getParam("/connected_components_search_init/car_position_y", connected_components_search_init.car_position_y);


}

void LaneTracker::LoadLineValidationTableCreationInitializationParameters()
{

    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/line_validation_table_creation_init.yaml";
    const char *command = str.c_str();
    system(command);


     n.getParam("/line_validation_table_creation_init/min_left_to_mid_line_distance",line_validation_table_creation_init.min_left_to_mid_line_distance);
     n.getParam("/line_validation_table_creation_init/max_left_to_mid_line_distance",line_validation_table_creation_init.max_left_to_mid_line_distance);
     n.getParam("/line_validation_table_creation_init/min_left_to_right_line_distance",line_validation_table_creation_init.min_left_to_right_line_distance);
     n.getParam("/line_validation_table_creation_init/max_left_to_right_line_distance",line_validation_table_creation_init.max_left_to_right_line_distance);
     n.getParam("/line_validation_table_creation_init/min_right_to_mid_line_distance",line_validation_table_creation_init.min_right_to_mid_line_distance);
     n.getParam("/line_validation_table_creation_init/max_right_to_mid_line_distance",line_validation_table_creation_init.max_right_to_mid_line_distance);
     n.getParam("/line_validation_table_creation_init/min_right_to_left_line_distance",line_validation_table_creation_init.min_right_to_left_line_distance);
     n.getParam("/line_validation_table_creation_init/max_right_to_left_line_distance",line_validation_table_creation_init.max_right_to_left_line_distance);
     n.getParam("/line_validation_table_creation_init/min_mid_to_left_line_distance",line_validation_table_creation_init.min_mid_to_left_line_distance);
     n.getParam("/line_validation_table_creation_init/max_mid_to_left_line_distance",line_validation_table_creation_init.max_mid_to_left_line_distance);
     n.getParam("/line_validation_table_creation_init/min_mid_to_right_line_distance",line_validation_table_creation_init.min_mid_to_right_line_distance);
     n.getParam("/line_validation_table_creation_init/max_mid_to_right_line_distance",line_validation_table_creation_init.max_mid_to_right_line_distance);

     n.getParam("/line_validation_table_creation_init/min_left_to_mid_pixel_intensity",line_validation_table_creation_init.min_left_to_mid_pixel_intensity);
     n.getParam("/line_validation_table_creation_init/min_left_to_right_pixel_intensity",line_validation_table_creation_init.min_left_to_right_pixel_intensity);
     n.getParam("/line_validation_table_creation_init/min_right_to_mid_pixel_intensity",line_validation_table_creation_init.min_right_to_mid_pixel_intensity);
     n.getParam("/line_validation_table_creation_init/min_right_to_left_pixel_intensity",line_validation_table_creation_init.min_right_to_left_pixel_intensity);
     n.getParam("/line_validation_table_creation_init/min_mid_to_right_pixel_intensity",line_validation_table_creation_init.min_mid_to_right_pixel_intensity);
     n.getParam("/line_validation_table_creation_init/min_mid_to_left_pixel_intensity",line_validation_table_creation_init.min_mid_to_left_pixel_intensity);

     n.getParam("/line_validation_table_creation_init/min_left_to_mid_line_width",line_validation_table_creation_init.min_left_to_mid_line_width);
     n.getParam("/line_validation_table_creation_init/max_left_to_mid_line_width",line_validation_table_creation_init.max_left_to_mid_line_width);
     n.getParam("/line_validation_table_creation_init/min_left_to_right_line_width",line_validation_table_creation_init.min_left_to_right_line_width);
     n.getParam("/line_validation_table_creation_init/max_left_to_right_line_width",line_validation_table_creation_init.max_left_to_right_line_width);
     n.getParam("/line_validation_table_creation_init/min_right_to_mid_line_width",line_validation_table_creation_init.min_right_to_mid_line_width);
     n.getParam("/line_validation_table_creation_init/max_right_to_mid_line_width",line_validation_table_creation_init.max_right_to_mid_line_width);
     n.getParam("/line_validation_table_creation_init/min_right_to_left_line_width",line_validation_table_creation_init.min_right_to_left_line_width);
     n.getParam("/line_validation_table_creation_init/max_right_to_left_line_width",line_validation_table_creation_init.max_right_to_left_line_width);
     n.getParam("/line_validation_table_creation_init/min_mid_to_left_line_width",line_validation_table_creation_init.min_mid_to_left_line_width);
     n.getParam("/line_validation_table_creation_init/max_mid_to_left_line_width",line_validation_table_creation_init.max_mid_to_left_line_width);
     n.getParam("/line_validation_table_creation_init/min_mid_to_right_line_width",line_validation_table_creation_init.min_mid_to_right_line_width);
     n.getParam("/line_validation_table_creation_init/max_mid_to_right_line_width",line_validation_table_creation_init.max_mid_to_right_line_width);

}

void LaneTracker::LoadLinePointsReduceInitializationParameters()
{

    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/line_points_reduce_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/line_points_reduce_init/max_distance_to_reduce_points", line_points_reduce_init.max_distance_to_reduce_points);



}

void LaneTracker::LoadSafeDriveAreaEvaluationInitializationParameters()
{

    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/safe_drive_area_evaluation_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/safe_drive_area_evaluation_init/start_of_rect_safety_x", safe_drive_area_evaluation_init.start_of_rect_safety_x);
    n.getParam("/safe_drive_area_evaluation_init/start_of_rect_safety_y", safe_drive_area_evaluation_init.start_of_rect_safety_y);
    n.getParam("/safe_drive_area_evaluation_init/start_search_direction_of_rect_safety", safe_drive_area_evaluation_init.start_search_direction_of_rect_safety);
    n.getParam("/safe_drive_area_evaluation_init/search_rect_width", safe_drive_area_evaluation_init.search_rect_width);
    n.getParam("/safe_drive_area_evaluation_init/search_rect_height", safe_drive_area_evaluation_init.search_rect_height);
    n.getParam("/safe_drive_area_evaluation_init/rect_border_distance_threshold_for_continous_line", safe_drive_area_evaluation_init.rect_border_distance_threshold_for_continous_line);
    n.getParam("/safe_drive_area_evaluation_init/rect_step_length", safe_drive_area_evaluation_init.rect_step_length);

}




void LaneTracker::LoadAllInitializationParameters()
{

    LoadStartOfLinesSearchInitializationParameters();
    LoadLineFollowerInitializationParameters();
    LoadBirdseyeInitializationParameters();
    LoadLinePointsReduceInitializationParameters();
    LoadMidLineSearchInitializationParameters();
    LoadLineValidationTableCreationInitializationParameters();
    LoadVanishingPointSearchInitializationParameters();
    LoadConnectedComponentsSearchInitializationParameters();
    LoadSafeDriveAreaEvaluationInitializationParameters();
}



void LaneTracker::InitializeBirdseyeTransformationMatrix()
{
  double alpha = ((double)birdseye_init.alpha - 90.)*PI/180;
  double beta = ((double)birdseye_init.beta - 90.)*PI/180;
  double gammma = ((double)birdseye_init.gamma - 90.)*PI/180;
  double f = (double) birdseye_init.fov;
  double dist = (double) birdseye_init.distance;


 double w = kInputImageWidth_, h = kInputImageHeight_;

  // Projection 2D -> 3D matrix
  Mat A1 = (Mat_<double>(4,3) <<
    1, 0, -w/2,
    0, 1, -h/2,
    0, 0,    0,
    0, 0,    1);

  // Rotation matrices around the X,Y,Z axis
  Mat RX = (Mat_<double>(4, 4) <<
    1,          0,           0, 0,
    0, cos(alpha), -sin(alpha), 0,
    0, sin(alpha),  cos(alpha), 0,
    0,          0,           0, 1);

  Mat RY = (Mat_<double>(4, 4) <<
    cos(beta), 0, -sin(beta), 0,
            0, 1,          0, 0,
    sin(beta), 0,  cos(beta), 0,
            0, 0,          0, 1);

  Mat RZ = (Mat_<double>(4, 4) <<
    cos(gammma), -sin(gammma), 0, 0,
    sin(gammma),  cos(gammma), 0, 0,
    0,          0,           1, 0,
    0,          0,           0, 1);

  // Composed rotation matrix with (RX,RY,RZ)
  Mat R = RX * RY * RZ;

  // Translation matrix on the Z axis change dist will change the height
  Mat T = (Mat_<double>(4, 4) <<           1, 0, 0, 0,           0, 1, 0, 0,           0, 0, 1, dist,           0, 0, 0, 1);       // Camera Intrisecs matrix 3D -> 2D
  Mat A2 = (Mat_<double>(3,4) <<
    f, 0, w/2, 0,
    0, f, h/2, 0,
    0, 0,   1, 0);

  // Final and overall birdseye_transformation_matrix_rmation matrix
  birdseye_transformation_matrix_ = A2 * (T * (R * A1));



}


LaneTracker::LaneTracker(ros::NodeHandle* nh_):n(*nh_),it(*nh_)
{
  camera_subscriber_ = it.subscribe("/rrbot/camera1/image_raw", 1, &LaneTracker::imageCallback, this);



  namedWindow("bird_rgb", WINDOW_NORMAL);
  setMouseCallback("bird_rgb", mouse_callback);

  LoadAllInitializationParameters();
  InitializeBirdseyeTransformationMatrix();

  StartOfLinesSearcher_ = new StartOfLinesSearch(image_height_,image_width_,start_of_lines_search_init);
  LineFollower_         = new LineFollow(image_height_,image_width_,line_follower_init);
  LinePointsReducer_    = new LinePointsReduce(line_points_reduce_init);
  MidLineSearcher_       = new MidLineSearch(image_height_,image_width_,mid_line_search_init);
  LineValidationTableCreator_ = new LineValidationTableCreation(image_height_,image_width_,line_validation_table_creation_init);
  VanishingPointSearcher_ = new VanishingPointSearch(birdseye_transformation_matrix_,vanishing_point_search_init);
  ConnectedComponentsSearcher_ = new ConnectedComponentsSearch(image_height_, image_width_, connected_components_search_init);
  SafeDriveAreaEvaluator_ = new SafeDriveAreaEvaluation(image_height_, image_width_, safe_drive_area_evaluation_init);
  OnRoadSearcher_ =  new OnRoadSearch(nh_);

};

/*
void LaneTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");



        circle(cv_ptr->image, Point(709,454),35, Scalar(0,0,0),CV_FILLED, 8,0);
        circle(cv_ptr->image, Point(574,454),35, Scalar(0,0,0),CV_FILLED, 8,0);


        image_mono_ = cv_ptr->image;



        cv::cvtColor(image_mono_, image_rgb_, CV_GRAY2BGR);

        warpPerspective(image_mono_, image_mono_bird_, birdseye_transformation_matrix_, kInputImageSize_, INTER_CUBIC | WARP_INVERSE_MAP);
        image_mono_bird_ = image_mono_bird_(Rect(0,0,image_width_,image_height_));
        threshold(image_mono_bird_, image_mono_bird_otsu_, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

         Mat bird2;
         Mat rgb_spline;
        cv::cvtColor(image_mono_bird_, image_rgb_bird_, CV_GRAY2BGR);
        cv::cvtColor(image_mono_bird_, rgb_spline, CV_GRAY2BGR);
        cv::cvtColor(image_mono_bird_, bird2, CV_GRAY2BGR);




        //cv::Mat image_rgb2_;

        //cv::cvtColor(image_mono_bird_, image_rgb2_, CV_GRAY2BGR);
        clock_t begin = clock();



        vanishing_point_search_return_info_ = VanishingPointSearcher_->FindVanishingPoint(image_mono_);

        //cout << info.vanishing_point << endl;
        //cout << info.car_mid_point_to_vanishing_point_angle << endl;

        if(vanishing_point_search_return_info_.has_found_left_hough_line)
        {
           VanishingPointSearcher_->DrawHoughLines(image_rgb_, LEFT_LINE);
           VanishingPointSearcher_->DrawWarpedPerspektiveHoughLines(image_rgb_bird_, LEFT_LINE);

           //VanishingPointSearcher_->GetWarpedPerspektiveHoughLin
        }

        if(vanishing_point_search_return_info_.has_found_right_hough_line)
        {
            VanishingPointSearcher_->DrawHoughLines(image_rgb_, RIGHT_LINE);
            VanishingPointSearcher_->DrawWarpedPerspektiveHoughLines(image_rgb_bird_, RIGHT_LINE);
        }

        if(vanishing_point_search_return_info_.has_found_intersections)
        {

            VanishingPointSearcher_->DrawLineIntersections(image_rgb_);
        }

        if(vanishing_point_search_return_info_.has_found_vanishing_point)
        {
            VanishingPointSearcher_->DrawVanishingPoint(image_rgb_);
            VanishingPointSearcher_->DrawWarpedVanishingPointDirection(image_rgb_bird_);
        }





        //auto start_of_lines_search_return_info_ = StartOfLinesSearcher_->FindStartParameters(image_mono_bird_otsu_);

        //if(start_of_lines_search_return_info_.has_found_start_parameters)
        if(vanishing_point_search_return_info_.has_found_left_hough_line || vanishing_point_search_return_info_.has_found_right_hough_line )
        {

            start_parameters_for_line_follower_ = VanishingPointSearcher_->GetLineFollowerStartParameters();

            //StartOfLinesSearcher_->DrawStartParameters(image_rgb_bird_);

            //LineFollowerReturnInfo line_follower_return_info_ = LineFollower_->FollowLines(image_mono_bird_,StartOfLinesSearcher_->GetStartParameters());
            line_follower_return_info_ = LineFollower_->FollowLines(image_mono_bird_,start_parameters_for_line_follower_);


            //cout << start_parameters_for_line_follower_.right_x << " " << start_parameters_for_line_follower_.right_y << " " << start_parameters_for_line_follower_.right_angle << endl;

            //LineFollower_->CoutReturnInfo();



            if(line_follower_return_info_.left_line_iterations_counter >= 2)
            {
                LineFollower_->GetLine(left_line_from_line_follower_, LEFT_LINE);
                //LineFollower_->DrawLinePoints(image_rgb_bird_,LEFT_LINE);
            }

            if(line_follower_return_info_.right_line_iterations_counter >= 2)
            {
                LineFollower_->GetLine(right_line_from_line_follower_, RIGHT_LINE);
                //LineFollower_->DrawLinePoints(image_rgb_bird_,RIGHT_LINE);
            }

            double max_distance = 5;

            line_points_reducer_return_info_ = LinePointsReducer_->ReduceLinePoints(left_line_from_line_follower_,right_line_from_line_follower_,max_distance);



            //LineValidationTableCreator_.SetImage(image_mono_bird_);
            if(line_points_reducer_return_info_.left_line_is_reduced_)
            {
                //LinePointsReducer_->DrawReducedLinePoints(image_rgb_bird_,LEFT_LINE);
                LinePointsReducer_->GetReducedLinePoints(left_line_from_line_points_reducer_,LEFT_LINE);
                LinePointsReducer_->GetLengthAndDirectionFromConsecutiveReducedLinePoints(left_line_lengths_and_directions_from_line_points_reducer_, LEFT_LINE);





               // LineValidationTableCreator_.FindValidPointsFromLineFollow(left_line_lengths_and_directions_from_line_points_reducer_, LEFT_TO_RIGHT);
               // LineValidationTableCreator_.FindValidPointsFromLineFollow(left_line_lengths_and_directions_from_line_points_reducer_, LEFT_TO_MID);
                //LineValidationTableCreator_.DrawLinePoints(image_rgb_bird_, LEFT_TO_RIGHT);
                //LineValidationTableCreator_.DrawLinePoints(image_rgb_bird_, LEFT_TO_MID);


            }

            if(line_points_reducer_return_info_.right_line_is_reduced_)
            {
               //LinePointsReducer_->DrawReducedLinePoints(image_rgb_bird_,RIGHT_LINE);
                LinePointsReducer_->GetReducedLinePoints(right_line_from_line_points_reducer_,RIGHT_LINE);
                LinePointsReducer_->GetLengthAndDirectionFromConsecutiveReducedLinePoints(right_line_lengths_and_directions_from_line_points_reducer_,RIGHT_LINE);



                //LineValidationTableCreator_.FindValidPointsFromLineFollow(right_line_lengths_and_directions_from_line_points_reducer_,RIGHT_TO_LEFT);
                //LineValidationTableCreator_.FindValidPointsFromLineFollow(right_line_lengths_and_directions_from_line_points_reducer_,RIGHT_TO_MID);
                //LineValidationTableCreator_.DrawLinePoints(image_rgb_bird_, RIGHT_TO_LEFT);
                //LineValidationTableCreator_.DrawLinePoints(image_rgb_bird_, RIGHT_TO_MID);




            }

            //LinePointsReducer_->CoutLengthAndDirectionFromConsecutiveReducedLinePoints();







        }




        mid_line_search_return_info_ = MidLineSearcher_->FindMidLineClusters(image_mono_bird_);



        if(mid_line_search_return_info_.has_found_mid_line_clusters)
        {
           // MidLineSearcher_->DrawClusters(image_rgb_bird_);
            //MidLineSearcher_->DrawSingleClusters(image_rgb_bird_);
            auto single_mid_line_clusters = MidLineSearcher_->GetSingleClusters();

            if(mid_line_search_return_info_.has_found_group)
            {
                //MidLineSearcher_->DrawGroupedMidLineClustersDirections(image_rgb_bird_);
                //MidLineSearcher_->CoutLengthAndDirectionOfConnectedClusters();

                 mid_line_groups_from_mid_line_searcher_ = MidLineSearcher_->GetGroupedMidLineClustersLengthAndDirection();

                //LineValidationTableCreator_.FindValidPointsFromMidLineSearch(mid_line_groups_from_mid_line_searcher_, MID_TO_LEFT);
                //LineValidationTableCreator_.FindValidPointsFromMidLineSearch(mid_line_groups_from_mid_line_searcher_, MID_TO_RIGHT);
                //LineValidationTableCreator_.DrawLinePoints(image_rgb_bird_, MID_TO_LEFT);
                //LineValidationTableCreator_.DrawLinePoints(image_rgb_bird_, MID_TO_RIGHT);
            }
         }


/*
        LineValidationTableCreator_.CreateValidationTables();


        LineValidationTableCreator_.SearchValidPoints();

        LineValidationTableCreator_.ExtractValidPoints();

        LineValidationTableCreator_.DrawDirectionInRangeTable(image_rgb_bird_);


        LineValidationTableCreator_.SetImage(image_mono_bird_);

        if(line_points_reducer_return_info_.left_line_is_reduced_)
        {
            LineValidationTableCreator_.FindValidPointsFromLineFollow(left_line_lengths_and_directions_from_line_points_reducer_, LEFT_TO_RIGHT);
            LineValidationTableCreator_.FindValidPointsFromLineFollow(left_line_lengths_and_directions_from_line_points_reducer_, LEFT_TO_MID);
        }

        if(line_points_reducer_return_info_.right_line_is_reduced_)
        {
            LineValidationTableCreator_.FindValidPointsFromLineFollow(right_line_lengths_and_directions_from_line_points_reducer_,RIGHT_TO_LEFT);
            LineValidationTableCreator_.FindValidPointsFromLineFollow(right_line_lengths_and_directions_from_line_points_reducer_,RIGHT_TO_MID);
        }

        if(mid_line_search_return_info_.has_found_group)
        {
            LineValidationTableCreator_.FindValidPointsFromMidLineSearch(mid_line_groups_from_mid_line_searcher_, MID_TO_LEFT);
            LineValidationTableCreator_.FindValidPointsFromMidLineSearch(mid_line_groups_from_mid_line_searcher_, MID_TO_RIGHT);
        }

        LineValidationTableCreator_.ValidateTrack(image_rgb_bird_);




        LineValidationTableCreator_.DrawSpline(rgb_spline);

        LineValidationTableCreator_.DrawSafetyRects(rgb_spline);


        LineValidationTableCreator_.ClearMemory();



       //LineValidationTableCreator_.DrawPointsInRect(image_rgb_bird_);


        //LineValidationTableCreator_.DrawMinMaxFromDirectionInRange(image_rgb_bird_);
        //LineValidationTableCreator_.DrawLastAdjacentPointMatch(image_rgb_bird_);





        //LineValidationTableCreator_.MergePoints();
        //LineValidationTableCreator_.JJ();

        //LineValidationTableCreator_.SearchMinMax();

        //LineValidationTableCreator_.DrawMergedPoints(image_rgb_bird_);


        //LineValidationTableCreator_.ComputePointScores();



        //LineValidationTableCreator_.DrawValidScorePoints(image_rgb_bird_);







        connected_component_search_return_info_ = ConnectedComponentsSearcher_->FindConnectedComponents(image_mono_bird_otsu_);

        if(connected_component_search_return_info_.has_found_mid_line_components)
        {
            //ConnectedComponentsSearcher_->DrawMidLineComponentsRect(image_rgb_bird_);

            if(connected_component_search_return_info_.has_found_mid_line_group)
            {
                //ConnectedComponentsSearcher_->DrawGroupedMidLineComponents(image_rgb_bird_);
                ConnectedComponentsSearcher_->DrawGroupedMidLineComponentsDirections(image_rgb_bird_);
            }
        }



        ClearTrackingData();


        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        //cout << "fps: " << 1/elapsed_secs << endl;

        warpPerspective(rgb_spline, image_rgb_warped_back_, birdseye_transformation_matrix_.inv(), Size(image_width_,image_height_), INTER_CUBIC | WARP_INVERSE_MAP);

        //imshow("input",cv_ptr->image);
        //imshow("bird",image_mono_bird_);
        //imshow("ostu bird",image_mono_bird_otsu_);
        //imshow("normal",image_rgb_);

        imshow("spline",rgb_spline);
        imshow("bird_rgb", image_rgb_bird_);
        //imshow("bird2",bird2);
        //imshow("warped_back",image_rgb_warped_back_);
        waitKey(1);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.",
        msg->encoding.c_str());
    }
};*/
