#include <gtest/gtest.h>
#include <climits>
#include "vanishing_point_search.h"
#include "datatypes.h"






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

TEST(BooleanCmpTest, ShouldPass){

     ros::NodeHandle n;

     Mat image = imread("/home/tb/gazebo_road_generation/ros_ws/src/track_lines/test/test_images/start_of_lines_search_test_images/crossroad.png",
                        CV_LOAD_IMAGE_GRAYSCALE);

    Mat birdseye_transformation_matrix = GetBirdseyeTransformationMatrix(image, n);
    VanishingPointSearchInitializationParameters init = GetVanishingPointInitializationParameters(n);


    clock_t begin = clock();
    VanishingPointSearch *VanishingPointSearcher   = new VanishingPointSearch(birdseye_transformation_matrix,init);


    VanishingPointSearcher->SetImage(image);
    VanishingPointSearcher->ClearMemory();
    VanishingPointSearchReturnInfo r_info =  VanishingPointSearcher->FindVanishingPoint();

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "fps: " << 1/elapsed_secs << endl;

    ASSERT_EQ(true, r_info.has_found_right_hough_line);
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_vanishing_point_search");


    return RUN_ALL_TESTS();
}

