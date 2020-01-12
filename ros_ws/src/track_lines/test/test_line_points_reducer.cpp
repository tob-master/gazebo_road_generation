#include <gtest/gtest.h>
#include <climits>
#include "line_points_reducer.h"
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

LinePointsReduceInitializationParameters GetLinePointsReduceInitializationParameters(ros::NodeHandle &n)
{
    LinePointsReduceInitializationParameters line_points_reduce_init;

    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/line_points_reduce_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/line_points_reduce_init/max_distance_to_reduce_points", line_points_reduce_init.max_distance_to_reduce_points);

    return line_points_reduce_init;

}




TEST(BooleanCmpTest, ShouldPass){


    Mat image = imread("/home/tb/gazebo_road_generation/ros_ws/src/track_lines/test/test_images/start_of_lines_search_test_images/crossroad.png",
                       CV_LOAD_IMAGE_GRAYSCALE);

    ros::NodeHandle n;

    LineFollowerInitializationParameters line_follower_init = GetLineFollowerInitializationParameters(n);
    LineFollow *LineFollower = new LineFollow(image.rows,image.cols,line_follower_init);
    LineFollower->ClearMemory();
    LineFollower->SetImage(image);
    StartParameters start_parameters = StartParameters{544,380,90,true,669,380,90,true};
    LineFollower->SetStartParameters(start_parameters);

    vector<PointAndDirection> left_line_from_line_follower, right_line_from_line_follower;
    LineFollower->GetLine(left_line_from_line_follower, LEFT_LINE);
    LineFollower->GetLine(right_line_from_line_follower, RIGHT_LINE);



/*
    vector<PointAndDirection> left_line_from_line_follower, right_line_from_line_follower;


    left_line_from_line_follower.push_back(PointAndDirection{10,15,90});
    left_line_from_line_follower.push_back(PointAndDirection{80,100,70});
    left_line_from_line_follower.push_back(PointAndDirection{233,140,40});
    left_line_from_line_follower.push_back(PointAndDirection{70,55,160});

    right_line_from_line_follower.push_back(PointAndDirection{10,15,50});
    right_line_from_line_follower.push_back(PointAndDirection{11,16,90});
    right_line_from_line_follower.push_back(PointAndDirection{13,17,20});
    right_line_from_line_follower.push_back(PointAndDirection{14,18,10});

    */


    LinePointsReduceInitializationParameters line_points_reducer_init = GetLinePointsReduceInitializationParameters(n);
    LinePointsReduce * LinePointsReducer = new LinePointsReduce(line_points_reducer_init);
    LinePointsReducerReturnInfo r_info = LinePointsReducer->ReduceLinePoints(left_line_from_line_follower,right_line_from_line_follower);

    ASSERT_EQ(false, r_info.right_line_is_reduced);
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_line_follower");


    return RUN_ALL_TESTS();
}
