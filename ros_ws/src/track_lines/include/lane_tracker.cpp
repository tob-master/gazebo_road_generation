#include "lane_tracker.h"


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
        cv::cvtColor(image_mono_bird_, image_rgb_bird_, CV_GRAY2BGR);
        cv::cvtColor(image_mono_bird_, bird2, CV_GRAY2BGR);




        //cv::Mat image_rgb2_;

        //cv::cvtColor(image_mono_bird_, image_rgb2_, CV_GRAY2BGR);
        clock_t begin = clock();



        auto vanishing_point_search_return_info = VanishingPointSearcher_->FindVanishingPoint(image_mono_);

        //cout << info.vanishing_point << endl;
        //cout << info.car_mid_point_to_vanishing_point_angle << endl;

        if(vanishing_point_search_return_info.has_found_left_hough_line)
        {
           VanishingPointSearcher_->DrawHoughLines(image_rgb_, LEFT_LINE);
           //VanishingPointSearcher_->DrawWarpedPerspektiveHoughLines(image_rgb_bird_, LEFT_LINE);

           //VanishingPointSearcher_->GetWarpedPerspektiveHoughLin
        }

        if(vanishing_point_search_return_info.has_found_right_hough_line)
        {
            VanishingPointSearcher_->DrawHoughLines(image_rgb_, RIGHT_LINE);
            //VanishingPointSearcher_->DrawWarpedPerspektiveHoughLines(image_rgb_bird_, RIGHT_LINE);
        }

        if(vanishing_point_search_return_info.has_found_intersections)
        {

            VanishingPointSearcher_->DrawLineIntersections(image_rgb_);
        }

        if(vanishing_point_search_return_info.has_found_vanishing_point)
        {
            VanishingPointSearcher_->DrawVanishingPoint(image_rgb_);
            //VanishingPointSearcher_->DrawWarpedVanishingPointDirection(image_rgb_bird_);
        }





        //auto start_of_lines_search_return_info = StartOfLinesSearcher_->FindStartParameters(image_mono_bird_otsu_);

        //if(start_of_lines_search_return_info.has_found_start_parameters)
        if(vanishing_point_search_return_info.has_found_left_hough_line || vanishing_point_search_return_info.has_found_right_hough_line )
        {

            StartParameters start_parameters_for_line_follower = VanishingPointSearcher_->GetLineFollowerStartParameters();

            //StartOfLinesSearcher_->DrawStartParameters(image_rgb_bird_);

            //LineFollowerReturnInfo line_follower_return_info = LineFollower_->FollowLines(image_mono_bird_,StartOfLinesSearcher_->GetStartParameters());
            LineFollowerReturnInfo line_follower_return_info = LineFollower_->FollowLines(image_mono_bird_,start_parameters_for_line_follower);


            //cout << start_parameters_for_line_follower.right_x << " " << start_parameters_for_line_follower.right_y << " " << start_parameters_for_line_follower.right_angle << endl;

            //LineFollower_->CoutReturnInfo();

            vector<PointAndDirection> left_line, right_line;

            if(line_follower_return_info.left_line_iterations_counter >= 2)
            {
                LineFollower_->GetLine(left_line, LEFT_LINE);
                //LineFollower_->DrawLinePoints(image_rgb_bird_,LEFT_LINE);
            }

            if(line_follower_return_info.right_line_iterations_counter >= 2)
            {
                LineFollower_->GetLine(right_line, RIGHT_LINE);
                //LineFollower_->DrawLinePoints(image_rgb_bird_,RIGHT_LINE);
            }

            double max_distance = 5;

            auto line_points_reducer_return_info = LinePointsReducer_->ReduceLinePoints(left_line,right_line,max_distance);

            vector<ReducedPoints> left_line_points_reduced, right_line_points_reduced;
            vector<PointInDirection> left_line_points_reduced_length_direction,
                                          right_line_points_reduced_length_direction;


            ValidLinePointSearcher_.SetImage(image_mono_bird_);

            if(line_points_reducer_return_info.left_line_is_reduced)
            {
                //LinePointsReducer_->DrawReducedLinePoints(image_rgb_bird_,LEFT_LINE);
                LinePointsReducer_->GetReducedLinePoints(left_line_points_reduced,LEFT_LINE);
                LinePointsReducer_->GetLengthAndDirectionFromConsecutiveReducedLinePoints(left_line_points_reduced_length_direction, LEFT_LINE);





                ValidLinePointSearcher_.FindValidPointsFromLineFollow(left_line_points_reduced_length_direction, LEFT_TO_RIGHT);
                ValidLinePointSearcher_.FindValidPointsFromLineFollow(left_line_points_reduced_length_direction, LEFT_TO_MID);
                //ValidLinePointSearcher_.DrawLinePoints(image_rgb_bird_, LEFT_TO_RIGHT);
                //ValidLinePointSearcher_.DrawLinePoints(image_rgb_bird_, LEFT_TO_MID);


            }

            if(line_points_reducer_return_info.right_line_is_reduced)
            {
                //LinePointsReducer_->DrawReducedLinePoints(image_rgb_bird_,RIGHT_LINE);
                LinePointsReducer_->GetReducedLinePoints(right_line_points_reduced,RIGHT_LINE);
                LinePointsReducer_->GetLengthAndDirectionFromConsecutiveReducedLinePoints(right_line_points_reduced_length_direction,RIGHT_LINE);



                ValidLinePointSearcher_.FindValidPointsFromLineFollow(right_line_points_reduced_length_direction,RIGHT_TO_LEFT);
                ValidLinePointSearcher_.FindValidPointsFromLineFollow(right_line_points_reduced_length_direction,RIGHT_TO_MID);
                //ValidLinePointSearcher_.DrawLinePoints(image_rgb_bird_, RIGHT_TO_LEFT);
                //ValidLinePointSearcher_.DrawLinePoints(image_rgb_bird_, RIGHT_TO_MID);




            }

            //LinePointsReducer_->CoutLengthAndDirectionFromConsecutiveReducedLinePoints();







        }



        auto mid_line_search_return_info = MidLineSearcher_->FindMidLineClusters(image_mono_bird_);

        if(mid_line_search_return_info.has_found_mid_line_clusters)
        {
           // MidLineSearcher_->DrawClusters(image_rgb_bird_);
            //MidLineSearcher_->DrawSingleClusters(image_rgb_bird_);
            auto single_mid_line_clusters = MidLineSearcher_->GetSingleClusters();

            if(mid_line_search_return_info.has_found_group)
            {
                //MidLineSearcher_->DrawGroupedMidLineClustersDirections(image_rgb_bird_);
                //MidLineSearcher_->CoutLengthAndDirectionOfConnectedClusters();

                auto grouped_mid_lines = MidLineSearcher_->GetGroupedMidLineClustersLengthAndDirection();

                ValidLinePointSearcher_.FindValidPointsFromMidLineSearch(grouped_mid_lines, MID_TO_LEFT);
                ValidLinePointSearcher_.FindValidPointsFromMidLineSearch(grouped_mid_lines, MID_TO_RIGHT);
                //ValidLinePointSearcher_.DrawLinePoints(image_rgb_bird_, MID_TO_LEFT);
                //ValidLinePointSearcher_.DrawLinePoints(image_rgb_bird_, MID_TO_RIGHT);
            }
         }



        ValidLinePointSearcher_.CreateValidationTables();


       ValidLinePointSearcher_.SearchValidPoints();

       ValidLinePointSearcher_.ExtractValidPoints();


        ValidLinePointSearcher_.DrawDirectionInRangeTable(image_rgb_bird_);
        ValidLinePointSearcher_.DrawMinMaxFromDirectionInRange(image_rgb_bird_);


        ValidLinePointSearcher_.ClearValidationTables();


        //ValidLinePointSearcher_.MergePoints();
        //ValidLinePointSearcher_.JJ();

        //ValidLinePointSearcher_.SearchMinMax();

        //ValidLinePointSearcher_.DrawMergedPoints(image_rgb_bird_);


        //ValidLinePointSearcher_.ComputePointScores();



        //ValidLinePointSearcher_.DrawValidScorePoints(image_rgb_bird_);





/*

        ConnectedComponentsSearchReturnInfo connected_component_search_return_info =
        ConnectedComponentsSearcher_->FindConnectedComponents(image_mono_bird_otsu_);

        if(connected_component_search_return_info.has_found_mid_line_components)
        {
            //ConnectedComponentsSearcher_->DrawMidLineComponentsRect(image_rgb_bird_);

            if(connected_component_search_return_info.has_found_mid_line_group)
            {
                //ConnectedComponentsSearcher_->DrawGroupedMidLineComponents(image_rgb_bird_);
                ConnectedComponentsSearcher_->DrawGroupedMidLineComponentsDirections(image_rgb_bird_);
            }
        }

*/



        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        cout << "fps: " << 1/elapsed_secs << endl;

        //warpPerspective(image_rgb_bird_, image_rgb_warped_back_, birdseye_transformation_matrix_.inv(), Size(image_width_,image_height_), INTER_CUBIC | WARP_INVERSE_MAP);

        //imshow("input",cv_ptr->image);
        //imshow("bird",image_mono_bird_);
        //imshow("ostu bird",image_mono_bird_otsu_);
        imshow("normal",image_rgb_);
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
};


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
    n.getParam("/start_of_lines_search_init/car_position_in_frame", start_of_lines_search_init.car_position_in_frame);
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



void LaneTracker::LoadAllInitializationParameters()
{

    LoadStartOfLinesSearchInitializationParameters();
    LoadLineFollowerInitializationParameters();
    LoadBirdseyeInitializationParameters();
    LoadMidLineSearchInitializationParameters();
    LoadVanishingPointSearchInitializationParameters();
    LoadConnectedComponentsSearchInitializationParameters();
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

void mouse_callback(int  event, int  x, int  y, int  flag, void *param)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        std::cout << x << " " << y << std::endl;
    }
      /*if (event == EVENT_MOUSEMOVE)
      {
          std::cout << x << " " << y << std::endl;
      }*/
}


LaneTracker::LaneTracker(ros::NodeHandle* nh_):n(*nh_),it(*nh_)
{
  camera_subscriber_ = it.subscribe("/rrbot/camera1/image_raw", 1, &LaneTracker::imageCallback, this);


  namedWindow("bird_rgb", WINDOW_NORMAL);
  setMouseCallback("bird_rgb", mouse_callback);

  LoadAllInitializationParameters();
  InitializeBirdseyeTransformationMatrix();

  StartOfLinesSearcher_ = new StartOfLinesSearch(image_height_,image_width_,start_of_lines_search_init);
  LineFollower_         = new LineFollower(image_height_,image_width_,line_follower_init);
  LinePointsReducer_    = new LinePointsReducer;
  MidLineSearcher_       = new MidLineSearch(image_height_,image_width_,mid_line_search_init);
  VanishingPointSearcher_ = new VanishingPointSearch(birdseye_transformation_matrix_,vanishing_point_search_init);
  ConnectedComponentsSearcher_ = new ConnectedComponentsSearch(image_height_, image_width_, connected_components_search_init);

};


