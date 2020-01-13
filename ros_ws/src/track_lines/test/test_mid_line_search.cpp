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





TEST(BooleanCmpTest, ShouldPass){


    Mat image = imread("/home/tb/gazebo_road_generation/ros_ws/src/track_lines/test/test_images/start_of_lines_search_test_images/crossroad.png",
                       CV_LOAD_IMAGE_GRAYSCALE);

    ros::NodeHandle n;

    MidLineSearchInitializationParameters init = GetMidLineSearchInitializationParameters(n);

    MidLineSearch *MidLineSearcher = new MidLineSearch(image.rows, image.cols, init);

    MidLineSearcher->ClearMemory();
    MidLineSearcher->SetImage(image);

    MidLineSearchReturnInfo r_info = MidLineSearcher->FindMidLineClusters();



    ASSERT_EQ(true, r_info.has_found_mid_line_clusters);
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_mid_line_search");


    return RUN_ALL_TESTS();
}
