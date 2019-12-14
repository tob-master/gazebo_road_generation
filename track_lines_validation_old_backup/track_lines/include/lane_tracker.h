#ifndef LANE_TRACKER_H
#define LANE_TRACKER_H

//#pragma once

#include "mid_line_search.h"
#include "start_of_lines_search.h"
#include "line_follower.h"
#include "line_points_reducer.h"
#include "vanishing_point_search.h"
#include "connected_components.h"
#include "valid_line_point_search.h"


#include "datatypes.h"
#include "line_validation_table.h"

class LaneTracker
{

private:

    ros::NodeHandle n;
    image_transport::ImageTransport it;

    image_transport::Subscriber camera_subscriber_;

    const int kInputImageWidth_  = 1280;
    const int kInputImageHeight_ =  720;
    const Size kInputImageSize_= Size(kInputImageWidth_,kInputImageHeight_);

    const int image_height_ = 417;
    const int image_width_ = 1280;

    const int kMinLineFollowerIterationsCount = 2;
            const double kMaxDistanceToReducePoints =5;


    bool left_hough_line_  = false;
    bool right_hough_line_ = false;

    bool left_line_follower_min_iterations_reached_ = false;
    bool right_line_follower_min_iterations_reached_ = false;

    bool left_line_is_reduced_  = false;
    bool right_line_is_reduced_ = false;

    bool mid_lines_found_ = false;

    bool mid_line_groups_found_ = false;

    StartOfLinesSearchInitializationParameters start_of_lines_search_init;
    LineFollowerInitializationParameters line_follower_init;
    BirdseyeInitializationParameters birdseye_init;
    MidLineSearchInitializationParameters mid_line_search_init;
    VanishingPointSearchInitializationParameters vanishing_point_search_init;
    ConnectedComponentsSearchInitializationParameters connected_components_search_init;

    Mat birdseye_transformation_matrix_;

    Mat image_mono_;
    Mat image_rgb_;
    Mat image_mono_bird_;
    Mat image_mono_bird_otsu_;
    Mat image_rgb_bird_;
    Mat image_rgb_warped_back_;



    StartOfLinesSearch *StartOfLinesSearcher_;


    LineFollower *LineFollower_;
    LinePointsReducer *LinePointsReducer_;
    MidLineSearch *MidLineSearcher_;
    VanishingPointSearch *VanishingPointSearcher_;
    ConnectedComponentsSearch *ConnectedComponentsSearcher_;

    ValidLinePointSearch ValidLinePointSearcher_;

    VanishingPointSearchReturnInfo vanishing_point_search_return_info_;
    StartParameters start_parameters_for_line_follower_;
    LineFollowerReturnInfo line_follower_return_info_;
    LinePointsReducerReturnInfo line_points_reducer_return_info_;
    MidLineSearchReturnInfo mid_line_search_return_info_;
    ConnectedComponentsSearchReturnInfo connected_component_search_return_info_;
    StartOfLinesSearchReturnInfo start_of_lines_search_return_info_;


    vector<PointAndDirection> left_line_from_line_follower_,
                              right_line_from_line_follower_;

    vector<ReducedPoints> left_line_from_line_points_reducer_,
                          right_line_from_line_points_reducer_;

    vector<PointInDirection> left_line_lengths_and_directions_from_line_points_reducer_,
                             right_line_lengths_and_directions_from_line_points_reducer_;


    vector<vector<PointInDirection>> mid_line_groups_from_mid_line_searcher_;

    Point pt = Point(-1,-1);
    bool newCoords = false;

    void ClearTrackingData();
void ReduceLinePoints();

    void LoadAllInitializationParameters();
    void LoadStartOfLinesSearchInitializationParameters();
    void LoadLineFollowerInitializationParameters();
    void LoadBirdseyeInitializationParameters();
    void LoadMidLineSearchInitializationParameters();
    void LoadVanishingPointSearchInitializationParameters();
    void LoadConnectedComponentsSearchInitializationParameters();

    void     DrawHoughLinesFront(Mat& rgb);
    void    DrawHoughLinesBird(Mat& rgb);
    void DrawLineFollowerBird(Mat& rgb);
void DrawReducedLinePointsBird(Mat &rgb);

void DrawAllMidLineClustersBird(Mat &rgb);


void DrawSingleMidLinesBird(Mat &rgb);
void DrawMidLineGroupsBird(Mat &rgb);

void DrawCCLMidLineRectComponentsBird(Mat &rgb);

void DrawCCLMidLineComponentsGroupsBird(Mat &rgb);

void DrawValidatedLines(Mat &rgb);

void DrawValidatedSplines(Mat& rgb);

void DrawValidatedSafetyAreas(Mat& rgb);

void DrawValidatedPointsOnDriveWay(Mat &rgb);

public:
  LaneTracker(ros::NodeHandle* nh_);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void InitializeBirdseyeTransformationMatrix();


};

#endif // LANE_TRACKER_H