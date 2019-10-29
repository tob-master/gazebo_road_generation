#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

//#pragma once



#include "midline_search.h"
#include "line_classification.h"
//#include "houghline_transform.h"
#include "own_datatypes.h"




class LineTracker
{

private:

    enum {LEFT_LINE, RIGHT_LINE};

    ros::NodeHandle n;
    image_transport::ImageTransport it;

    const int kLineWitdhMin = 3;
    const int kLineWidthMax = 9;

    const int kTrackWidthMax = 143;
    const int kTrackWidthMin = 117;

     const int kMaxChangeInDegreePerIteration_ = 10;
         const int kMinDistanceToNotGotStuck_ = 3;

    Mat transfo;
        int got_stuck_counter_;
        bool got_stuck;

        int backwards_counter = 0;

    Mat grey;
    Mat rgb;

    int search_length_;
    int field_of_view_;

    int line_follow_iterations_counter_;
    vector<LineSearchFoundPointAndDirection> found_points_and_directions_left_line_;
    vector<LineSearchFoundPointAndDirection> found_points_and_directions_right_line_;


    vector<RDP_Point> left_line_points_for_rdp_;
    vector<RDP_Point> right_line_points_for_rdp_;

    vector<RDP_Point> left_line_rdp_reduced_;
    vector<RDP_Point> right_line_rdp_reduced_;

    Size taille;

    MidLineSearch MidLineSearcher;
    LineClassification LineClassifier;
    //HoughLineTransform HoughLine;


public:
  LineTracker(ros::NodeHandle* nh_);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void initBirdseye();
  void FollowLinePoints(Mat grey, vector<LineSearchStartParameters> line_search_start_parameters);
  int FollowLine(Mat grey, int start_x, int start_y, float search_direction, int line);
  Point ChangeToBrightestCoordinateWithinReach(Mat image, Point center_of_gravity);
  Point PolarCoordinate(int x, int y, float a, int l);
  int GetOtsuTheshold(Mat grey, int start_x, int start_y, float start_angle, float end_angle, float step);
  vector<LineSearchMoments> GetScannedMoments(Mat grey, int otsu_threshold, int start_x, int start_y, float start_angle, float end_angle, float step);
  Point GetCenterOfGravity(int start_x, int start_y, vector<LineSearchMoments> scanned_moments);
  float GetNewAngle(int start_x, int start_y, Point new_start_point);
  image_transport::Subscriber image_sub;
};

#endif // LINE_TRACKER_H
