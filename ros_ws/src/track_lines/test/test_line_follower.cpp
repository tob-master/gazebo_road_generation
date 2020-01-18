#include <gtest/gtest.h>
#include <climits>
#include "line_follower.h"
#include "datatypes.h"
#include "vanishing_point_search.h"


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


//Arrange, Act and Assert
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

    VanishingPointSearchInitializationParameters vanishing_point_init = GetVanishingPointInitializationParameters(n);
    Mat birdseye_transformation_matrix = GetBirdseyeTransformationMatrix(image, n);
    VanishingPointSearch *VanishingPointSearcher   = new VanishingPointSearch(birdseye_transformation_matrix,vanishing_point_init);
    VanishingPointSearcher->SetImage(image);
    VanishingPointSearcher->ClearMemory();
    VanishingPointSearchReturnInfo vanishing_point_search_return_info =  VanishingPointSearcher->FindVanishingPoint();
    StartParameters vanishing_point_start_parameters = VanishingPointSearcher->GetLineFollowerStartParameters();

    //StartParameters start_parameters = StartParameters{544,380,90,true,669,380,90,true};
    LineFollowerInitializationParameters line_follower_init = GetLineFollowerInitializationParameters(n);
    LineFollow *LineFollower = new LineFollow(image_bird.rows,image_bird.cols,line_follower_init);
    LineFollower->ClearMemory();
    LineFollower->SetImage(image_bird);
    LineFollower->SetStartParameters(vanishing_point_start_parameters);
    LineFollowerReturnInfo r_info = LineFollower->FollowLines();
    LineFollower->DrawLinePoints(image_bird_color,LEFT_LINE);
    LineFollower->DrawLinePoints(image_bird_color,RIGHT_LINE);


    imwrite("/home/tb/gazebo_road_generation/ros_ws/src/track_lines/test/test_images/line_follower/results/line_follower.png",image_bird_color);

    ASSERT_EQ(false, r_info.right_line_max_iterations_exceeded);
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_line_follower");


    return RUN_ALL_TESTS();
}
