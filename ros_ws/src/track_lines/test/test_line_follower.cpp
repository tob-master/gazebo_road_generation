#include <gtest/gtest.h>
#include <climits>
#include "line_follower.h"
#include "datatypes.h"



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





TEST(BooleanCmpTest, ShouldPass){


    Mat image = imread("/home/tb/gazebo_road_generation/ros_ws/src/track_lines/test/test_images/start_of_lines_search_test_images/crossroad.png",
                       CV_LOAD_IMAGE_GRAYSCALE);

    ros::NodeHandle n;

    LineFollowerInitializationParameters init = GetLineFollowerInitializationParameters(n);


    LineFollow *LineFollower = new LineFollow(image.rows,image.cols,init);
    LineFollower->ClearMemory();
    LineFollower->SetImage(image);


    StartParameters start_parameters = StartParameters{544,380,90,true,669,380,90,true};


    LineFollower->SetStartParameters(start_parameters);
    LineFollowerReturnInfo r_info = LineFollower->FollowLines();

    ASSERT_EQ(false, r_info.right_line_max_iterations_exceeded);
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_line_follower");


    return RUN_ALL_TESTS();
}
