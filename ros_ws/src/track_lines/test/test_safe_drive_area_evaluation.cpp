#include <gtest/gtest.h>
#include <climits>
#include "line_follower.h"
#include "line_validation_table_creation.h"
#include "vanishing_point_search.h"
#include "line_points_reducer.h"
#include "line_validation_table.h"
#include "line_validation_table_creation.h"
#include "datatypes.h"
#include "mid_line_search.h"
#include "safe_drive_area_evaluation.h"


LineFollowerInitializationParameters GetLineFollowerInitializationParameters(ros::NodeHandle &n)
{
    LineFollowerInitializationParameters line_follower_init;

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

    ros::spinOnce();

    return line_follower_init;
}

LineValidationTableCreationInitializationParameters GetLineValidationTableCreationInitializationParameters(ros::NodeHandle &n)
{

     LineValidationTableCreationInitializationParameters line_validation_table_creation_init;

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

     n.getParam("/line_validation_table_creation_init/max_distance_of_predicted_to_adjacent_point",line_validation_table_creation_init.max_distance_of_predicted_to_adjacent_point);
     n.getParam("/line_validation_table_creation_init/min_start_direction_of_line_points_in_drive_direction",line_validation_table_creation_init.min_start_direction_of_line_points_in_drive_direction);
     n.getParam("/line_validation_table_creation_init/max_start_direction_of_line_points_in_drive_direction",line_validation_table_creation_init.max_start_direction_of_line_points_in_drive_direction);
     n.getParam("/line_validation_table_creation_init/max_direction_difference_of_line_points_in_drive_direction",line_validation_table_creation_init.max_direction_difference_of_line_points_in_drive_direction);

     return line_validation_table_creation_init;

}


VanishingPointSearchInitializationParameters GetVanishingPointInitializationParameters(ros::NodeHandle &n)
{
    VanishingPointSearchInitializationParameters vanishing_point_search_init;

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

    ros::spinOnce();

    return vanishing_point_search_init;
}


Mat GetBirdseyeTransformationMatrix(Mat image, ros::NodeHandle &n)
{
    BirdseyeInitializationParameters birdseye_init;
    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/birdseye_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/birdseye_init/alpha", birdseye_init.alpha);
    n.getParam("/birdseye_init/beta", birdseye_init.beta);
    n.getParam("/birdseye_init/gamma", birdseye_init.gamma);
    n.getParam("/birdseye_init/fov", birdseye_init.fov);
    n.getParam("/birdseye_init/distance", birdseye_init.distance);



    ros::spinOnce();

    double alpha = ((double)birdseye_init.alpha - 90.)*PI/180;
    double beta = ((double)birdseye_init.beta - 90.)*PI/180;
    double gammma = ((double)birdseye_init.gamma - 90.)*PI/180;
    double f = (double) birdseye_init.fov;
    double dist = (double) birdseye_init.distance;


    double w = image.cols, h = image.rows;

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

    return A2 * (T * (R * A1));
}

LinePointsReduceInitializationParameters GetLinePointsReduceInitializationParameters(ros::NodeHandle &n)
{
    LinePointsReduceInitializationParameters line_points_reduce_init;

    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/line_points_reduce_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/line_points_reduce_init/max_distance_to_reduce_points", line_points_reduce_init.max_distance_to_reduce_points);

    return line_points_reduce_init;

}
MidLineSearchInitializationParameters GetMidLineSearchInitializationParameters(ros::NodeHandle &n)
{
    MidLineSearchInitializationParameters mid_line_search_init;

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

    ros::spinOnce();

    return mid_line_search_init;
}


SafeDriveAreaEvaluationInitializationParameters GetSafeDriveAreaEvaluationInitializationParameters(ros::NodeHandle &n)
{

    SafeDriveAreaEvaluationInitializationParameters safe_drive_area_evaluation_init;

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

    return safe_drive_area_evaluation_init;
}

Mat GetImageFromCamera(ros::NodeHandle &n)
{
    sensor_msgs::Image::ConstPtr ros_image;
    ros_image = ros::topic::waitForMessage<sensor_msgs::Image>("/rrbot/camera1/image_raw", n, ros::Duration(0.5));
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_image, "mono8");
    ros::spinOnce();

    circle(cv_ptr->image, Point(709,454),35, Scalar(0,0,0),CV_FILLED, 8,0);
    circle(cv_ptr->image, Point(574,454),35, Scalar(0,0,0),CV_FILLED, 8,0);

   return cv_ptr->image;
}

Mat GetImageColor(Mat image)
{
   Mat color;
   cvtColor(image, color, CV_GRAY2BGR);
   return color;
}

Mat GetBirdseyeViewImage(ros::NodeHandle &n, Mat image)
{
    Mat bird;
    Mat birdseye_transformation_matrix = GetBirdseyeTransformationMatrix(image, n);
    warpPerspective(image, bird, birdseye_transformation_matrix, image.size(), INTER_CUBIC | WARP_INVERSE_MAP);
    bird = bird(Rect(0,0,1280,417));
    return bird;
}

TEST(BooleanCmpTest, ShouldPass){


    //Mat image = imread("/home/tb/gazebo_road_generation/ros_ws/src/track_lines/test/test_images/start_of_lines_search_test_images/crossroad.png",
    //                   CV_LOAD_IMAGE_GRAYSCALE);

    ros::NodeHandle n;

    Mat image = GetImageFromCamera(n);
    Mat image_color= GetImageColor(image);
    Mat image_bird = GetBirdseyeViewImage(n,image);
    Mat image_bird_color = GetImageColor(image_bird);


    VanishingPointSearchInitializationParameters init = GetVanishingPointInitializationParameters(n);
    Mat birdseye_transformation_matrix = GetBirdseyeTransformationMatrix(image, n);
    VanishingPointSearch *VanishingPointSearcher   = new VanishingPointSearch(birdseye_transformation_matrix,init);
    VanishingPointSearcher->SetImage(image);
    VanishingPointSearcher->ClearMemory();
    VanishingPointSearchReturnInfo vaniching_point_search_return_info =  VanishingPointSearcher->FindVanishingPoint();
    StartParameters vanishing_point_start_parameters = VanishingPointSearcher->GetLineFollowerStartParameters();


    LineFollowerInitializationParameters line_follower_init = GetLineFollowerInitializationParameters(n);
    LineFollow *LineFollower = new LineFollow(image_bird.rows,image_bird.cols,line_follower_init);
    LineFollower->ClearMemory();
    LineFollower->SetImage(image_bird);
    //StartParameters start_parameters = StartParameters{544,380,90,true,669,380,90,true};

    LineFollower->SetStartParameters(vanishing_point_start_parameters);
    LineFollowerReturnInfo line_follower_return_info = LineFollower->FollowLines();
    vector<PointAndDirection> left_line_from_line_follower,right_line_from_line_follower;
    LineFollower->GetLine(left_line_from_line_follower, LEFT_LINE);
    LineFollower->GetLine(right_line_from_line_follower, RIGHT_LINE);



    LinePointsReduceInitializationParameters line_points_reducer_init = GetLinePointsReduceInitializationParameters(n);
    LinePointsReduce * LinePointsReducer = new LinePointsReduce(line_points_reducer_init);
    LinePointsReducerReturnInfo line_points_reducer_return_info = LinePointsReducer->ReduceLinePoints(left_line_from_line_follower,right_line_from_line_follower);
    vector<PointInDirection> left_line_lengths_and_directions_from_line_points_reducer,right_line_lengths_and_directions_from_line_points_reducer;
    LinePointsReducer->GetLengthAndDirectionFromConsecutiveReducedLinePoints(left_line_lengths_and_directions_from_line_points_reducer, LEFT_LINE);
    LinePointsReducer->GetLengthAndDirectionFromConsecutiveReducedLinePoints(right_line_lengths_and_directions_from_line_points_reducer,RIGHT_LINE);



    MidLineSearchInitializationParameters mid_line_search_init = GetMidLineSearchInitializationParameters(n);
    MidLineSearch *MidLineSearcher = new MidLineSearch(image_bird.rows, image_bird.cols, mid_line_search_init);
    MidLineSearcher->ClearMemory();
    MidLineSearcher->SetImage(image_bird);
    MidLineSearchReturnInfo mid_line_search_return_info = MidLineSearcher->FindMidLineClusters();
    vector<vector<PointInDirection>> mid_line_groups_from_mid_line_searcher = MidLineSearcher->GetGroupedMidLineClustersLengthAndDirection();


    LineValidationTableCreationInitializationParameters line_validation_table_creation_init =  GetLineValidationTableCreationInitializationParameters(n);
    LineValidationTableCreation *LineValidationTableCreator = new LineValidationTableCreation(image_bird.rows,image_bird.cols,line_validation_table_creation_init);
    LineValidationTableCreator->ClearMemory();
    LineValidationTableCreator->SetImage(image_bird);

    LineValidationTableCreator->FindValidPointsFromLineFollow(left_line_lengths_and_directions_from_line_points_reducer, LEFT_TO_RIGHT);
    LineValidationTableCreator->FindValidPointsFromLineFollow(left_line_lengths_and_directions_from_line_points_reducer, LEFT_TO_MID);
    LineValidationTableCreator->FindValidPointsFromLineFollow(right_line_lengths_and_directions_from_line_points_reducer,RIGHT_TO_LEFT);
    LineValidationTableCreator->FindValidPointsFromLineFollow(right_line_lengths_and_directions_from_line_points_reducer,RIGHT_TO_MID);
    LineValidationTableCreator->FindValidPointsFromMidLineSearch(mid_line_groups_from_mid_line_searcher, MID_TO_LEFT);
    LineValidationTableCreator->FindValidPointsFromMidLineSearch(mid_line_groups_from_mid_line_searcher, MID_TO_RIGHT);
    LineValidationTableCreationReturnInfo line_validatiohn_table_creation_return_info = LineValidationTableCreator->CreateLineValidationTables();
    vector<LineValidationTable> left_line_validation_table;
    vector<LineValidationTable> mid_line_validation_table;
    vector<LineValidationTable> right_line_validation_table;
    LineValidationTableCreator->GetLineValidationTables(left_line_validation_table,mid_line_validation_table,right_line_validation_table);
    vector<LineValidationTable> left_line_in_drive_direction_table;
    vector<LineValidationTable> mid_line_in_drive_direction_table;
    vector<LineValidationTable> right_line_in_drive_direction_table;
    LineValidationTableCreator->GetLinePointsInDriveDirection(left_line_in_drive_direction_table,mid_line_in_drive_direction_table,right_line_in_drive_direction_table);

    SafeDriveAreaEvaluationInitializationParameters safe_drive_area_evaluation_init = GetSafeDriveAreaEvaluationInitializationParameters(n);
    SafeDriveAreaEvaluation *SafeDriveAreaEvaluator = new SafeDriveAreaEvaluation(image_bird.rows, image_bird.cols, safe_drive_area_evaluation_init);
    SafeDriveAreaEvaluator->ClearMemory();
    SafeDriveAreaEvaluator->LoadLinePointsInDriveDirection(left_line_in_drive_direction_table,mid_line_in_drive_direction_table,right_line_in_drive_direction_table);
    vector<SafeDriveAreaEvaluationReturnInfo> safe_drive_area_evaluation_return_info =  SafeDriveAreaEvaluator->EvaluateTrackInDriveDirection();
    //SafeDriveAreaEvaluator->DrawEvaluatedSafetyAreasInDriveDirection(image_bird_color);
    //LinePointsReducer->DrawReducedLinePoints(image_bird_color,LEFT_LINE);
    //LinePointsReducer->DrawReducedLinePoints(image_bird_color,RIGHT_LINE);
    //MidLineSearcher->DrawGroupedMidLineClustersDirections(image_bird_color);
    //LineValidationTableCreator->DrawLinePointsInDriveDirection(image_bird_color);

    vector<int> priority_ids{
    PRIO_0_P1_AND_P2,
    PRIO_1_P1_AND_FP2,
    PRIO_2_P2_AND_FP1,
    PRIO_3_P1,
    PRIO_4_P2,
    PRIO_5_FP1_AND_FP2,
    PRIO_6_FP1,
    PRIO_7_FP2 };

    SafeDriveAreaEvaluator->DrawPriorityPoints(image_bird_color,priority_ids);

    imwrite("/home/tb/gazebo_road_generation/ros_ws/src/track_lines/test/test_images/safe_drive_area_evaluation/results/safe_drive_area_evaluation.png",image_bird_color);


    ASSERT_EQ(true, safe_drive_area_evaluation_return_info[0].max_line_is_continous);
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_line_follower");


    return RUN_ALL_TESTS();
}
