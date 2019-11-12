#ifndef LANE_TRACKER_H
#define LANE_TRACKER_H

//#pragma once

#include "mid_line_search.h"
#include "start_of_lines_search.h"
#include "line_follower.h"
#include "line_points_reducer.h"
#include "vanishing_point_search.h"
#include "connected_components.h"


#include "datatypes.h"


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

    Point pt = Point(-1,-1);
    bool newCoords = false;


    void LoadAllInitializationParameters();
    void LoadStartOfLinesSearchInitializationParameters();
    void LoadLineFollowerInitializationParameters();
    void LoadBirdseyeInitializationParameters();
    void LoadMidLineSearchInitializationParameters();
    void LoadVanishingPointSearchInitializationParameters();
    void LoadConnectedComponentsSearchInitializationParameters();



public:
  LaneTracker(ros::NodeHandle* nh_);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void InitializeBirdseyeTransformationMatrix();


};

#endif // LANE_TRACKER_H
