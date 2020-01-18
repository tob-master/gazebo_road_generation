#include <gtest/gtest.h>
#include <climits>
#include "start_of_lines_search.h"
#include "datatypes.h"



StartOfLinesSearchInitializationParameters GetStartOfLinesSearchInitializationParameters(ros::NodeHandle &n)
{
    StartOfLinesSearchInitializationParameters start_of_lines_search_init;

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

    ros::spinOnce();

    return start_of_lines_search_init;
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


    StartOfLinesSearchInitializationParameters init = GetStartOfLinesSearchInitializationParameters(n);


    StartOfLinesSearch *StartOfLinesSearcher = new StartOfLinesSearch(image_bird.rows,image_bird.cols,init);
    StartOfLinesSearcher->ClearMemory();
    StartOfLinesSearcher->SetImage(image_bird);
    StartOfLinesSearchReturnInfo r_info = StartOfLinesSearcher->FindStartParameters();

    StartOfLinesSearcher->DrawStartParameters(image_bird_color);
    imwrite("/home/tb/gazebo_road_generation/ros_ws/src/track_lines/test/test_images/start_of_lines_search/results/start_of_lines_search.png",image_bird_color);

    //StartParameters start_parameters = StartOfLinesSearcher->GetStartParameters();

    ASSERT_EQ(true, r_info.has_found_start_parameters);
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_start_of_lines_search");


    return RUN_ALL_TESTS();
}

