#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

//#pragma once



#include "midline_search.h"
#include "line_classification.h"
#include "houghline_transform.h"
#include "own_datatypes.h"

class LineTracker
{

private:
    ros::NodeHandle n;
    image_transport::ImageTransport it;

    const int kLineWitdhMin = 3;
    const int kLineWidthMax = 9;

    const int kTrackWidthMax = 143;
    const int kTrackWidthMin = 117;

    Mat transfo;

    Mat grey;
    Mat rgb;



    Size taille;

    MidLineSearch MidLineSearcher;
    LineClassification LineClassifier;
    //HoughLineTransform HoughLine;


public:
  LineTracker(ros::NodeHandle* nh_);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void initBirdseye();
  void FollowLinePoints(Mat grey, vector<LineSearchStartParameters> line_search_start_parameters);
  image_transport::Subscriber image_sub;
};

#endif // LINE_TRACKER_H
