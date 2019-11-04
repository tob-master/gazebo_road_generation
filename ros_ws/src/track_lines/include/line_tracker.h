#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

//#pragma once

#include "midline_search.h"
#include "line_classification.h"
#include "line_follower.h"
#include "line_points_reducer.h"
#include "houghline_transform.h"

//#include "houghline_birdseye_transformation_matrix_rm.h"
#include "own_datatypes.h"


class LineTracker
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

    Mat birdseye_transformation_matrix_;

    Mat image_mono_;
    Mat image_rgb_;



    StartOfLinesSearch *StartOfLinesSearcher_;
    LineFollower *LineFollower_;
    LinePointsReducer *LinePointsReducer_;
    MidLineSearch *MidLineSearcher;


    HoughLineTransform VanashingPoint;


    void LoadAllInitializationParameters();
    void LoadStartOfLinesSearchInitializationParameters();
    void LoadLineFollowerInitializationParameters();
    void LoadBirdseyeInitializationParameters();
    void LoadMidLineSearchInitializationParameters();

public:
  LineTracker(ros::NodeHandle* nh_);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void InitializeBirdseyeTransformationMatrix();


};

#endif // LINE_TRACKER_H
