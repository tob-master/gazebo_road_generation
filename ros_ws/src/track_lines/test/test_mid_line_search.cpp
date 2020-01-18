#include <gtest/gtest.h>
#include <climits>
#include "depth_first_search.h"
#include "mid_line_search.h"
#include "datatypes.h"



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

    MidLineSearchInitializationParameters init = GetMidLineSearchInitializationParameters(n);

    MidLineSearch *MidLineSearcher = new MidLineSearch(image_bird.rows, image_bird.cols, init);

    MidLineSearcher->ClearMemory();
    MidLineSearcher->SetImage(image_bird);

    MidLineSearchReturnInfo mid_line_search_return_info = MidLineSearcher->FindMidLineClusters();

    MidLineSearcher->DrawGroupedMidLineClustersDirections(image_bird_color);

    imwrite("/home/tb/gazebo_road_generation/ros_ws/src/track_lines/test/test_images/mid_line_search/results/mid_line_search.png",image_bird_color);

    ASSERT_EQ(true, mid_line_search_return_info.has_found_mid_line_clusters);
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_mid_line_search");


    return RUN_ALL_TESTS();
}
