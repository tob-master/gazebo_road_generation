#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <iostream>
#include <stdio.h>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "own_datatypes.h"
#include "own_utils.h"
#include "own_defines.h"

using namespace std;
using namespace cv;
using namespace line_follower;



class LineFollower
{

private:
            enum {LEFT_LINE, RIGHT_LINE};

            Mat image_;
            const int image_width_ = 1280;
            const int image_height_ = 417;

            const int kMaxIterations_ =  150;
            const int kSearchRadius_ = 10;
            const int kMaxWeightDirectionScaler_ = 6;

            const int kFieldOfView_ = 144;
            const float kStartAngleFieldOfView_ = (kFieldOfView_/2 ) * (PI/180);
            const float kEndAngleFieldOfView_   = (kFieldOfView_/2 ) * (PI/180) - 0.001;
            const float kStepFieldOfView_      = (kFieldOfView_/ 4) * (PI/180);

            const int kMaxConsecutiveBackSteps_ = 5;
            const int kMinTravelDistanceToNotGotStuck_ = 3;
            const int kMaxGotStuckCounts_ = 6;

            float start_of_search_;
            float end_of_search_;


            int start_left_x_;
            int start_left_y_;
            float start_angle_left_;

            int start_right_x_;
            int start_right_y_;
            float start_angle_right_;

            int iterations_counter_;
            int got_stuck_counter_;
            int walked_backwards_counter_;

            vector<PointAndDirection> left_line_points_and_directions_;
            vector<PointAndDirection> right_line_points_and_directions_;


            void SetImage(Mat image);
            void SetStartParameters(StartParameters start_parameters);
            void ResetCounters();
            void ClearMemory();
            int FollowLine(int x, int y, float search_direction, int line);


            Point GetPolarCoordinate(int x, int y, float angle, int radius);
            Point ChangeToBrightestCoordinateWithinReach(Point center_of_gravity);
            int GetPixelValue(Point point);
            vector<ScannedMoments> GetScannedMoments(int otsu_threshold, int x, int y);
            int GetOtsuThreshold(int x, int y);
            vector<int> ScanIntensitiesInSearchDirection(int x, int y);
            Point GetCenterOfGravity(int x, int y, vector<ScannedMoments> scanned_moments);


            void SearchMaxWeightMoment(SummedMoments &summed_moments, vector<ScannedMoments> scanned_moments);
            void SumUpMoments(SummedMoments &summed_moments, vector<ScannedMoments> scanned_moments);
            float GetNewAngle(int x, int y, Point new_start_point);
            bool MaxIterationsExceeded();
            bool SearchRadiusIsNotImage(int x, int y);
            void SetSearchDirectionParameters(float search_direction);
            void AddIteration(Point new_start_point, int new_angle, int line);



            bool IsWalkingBackwards(int y, Point new_start_point);
            bool HasGotStuck(int x, int y, Point new_start_point);
            bool CheckIterationsCounter();
            bool SearchRadiusIsNotInImage(int x, int y);


public:
    LineFollower();
    void FollowLines(Mat grey, StartParameters start_parameters);
    void DrawLinePoints(Mat &rgb);



};

#endif // LINE_FOLLOWER_H
