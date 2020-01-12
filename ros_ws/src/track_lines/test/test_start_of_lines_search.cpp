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





TEST(BooleanCmpTest, ShouldPass){


    Mat image = imread("/home/tb/gazebo_road_generation/ros_ws/src/track_lines/test/test_images/start_of_lines_search_test_images/crossroad.png",
                       CV_LOAD_IMAGE_GRAYSCALE);


    ros::NodeHandle n;

    StartOfLinesSearchInitializationParameters init = GetStartOfLinesSearchInitializationParameters(n);


    StartOfLinesSearch *StartOfLinesSearcher = new StartOfLinesSearch(image.rows,image.cols,init);
    StartOfLinesSearcher->ClearMemory();
    StartOfLinesSearcher->SetImage(image);
    StartOfLinesSearchReturnInfo r_info = StartOfLinesSearcher->FindStartParameters();

    //StartParameters start_parameters = StartOfLinesSearcher->GetStartParameters();

    ASSERT_EQ(true, r_info.has_found_start_parameters);
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_start_of_lines_search");


    return RUN_ALL_TESTS();
}

