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

#include "datatypes.h"
#include "utils.h"
#include "defines.h"


using namespace std;
using namespace cv;
using namespace line_follower;

class LineFollower
{

    private:



        Mat image_;
        const int kImageWidth_;
        const int kImageHeight_;

        const int kMaxIterations_;
        const int kSearchRadius_;
        const int kMaxWeightDirectionScaler_;

        const int kFieldOfView_;
        const float kStartAngleFieldOfView_;
        const float kEndAngleFieldOfView_;
        const float kStepFieldOfView_;

        const int kMaxConsecutiveBackSteps_;
        const int kMinTravelDistanceToNotGotStuck_;
        const int kMaxGotStuckCounts_;

        float start_of_search_;
        float end_of_search_;

        int start_left_x_;
        int start_left_y_;
        float start_angle_left_;
        bool found_left_line_;

        int start_right_x_;
        int start_right_y_;
        float start_angle_right_;
        bool found_right_line_;

        int iterations_counter_;
        int got_stuck_counter_;
        int walked_backwards_counter_;

        bool left_line_max_iterations_exceeded_;
        bool left_line_search_radius_out_of_image_;
        bool left_line_has_got_stuck_;
        bool left_line_is_walking_backwards_;
        int left_line_iterations_counter_;
        int left_line_got_stuck_counter_;
        int left_line_walked_backwards_counter_;

        bool right_line_max_iterations_exceeded_;
        bool right_line_search_radius_out_of_image_;
        bool right_line_has_got_stuck_;
        bool right_line_is_walking_backwards_;
        int right_line_iterations_counter_;
        int right_line_got_stuck_counter_;
        int right_line_walked_backwards_counter_;


        vector<PointAndDirection> left_line_points_and_directions_;
        vector<PointAndDirection> right_line_points_and_directions_;


        void SetImage(Mat image);
        void SetStartParameters(StartParameters start_parameters);
        void ClearMemory();
        void ResetCounters();
        int FollowLine(int x, int y, float search_direction, int line);
        bool MaxIterationsExceeded(int line);
        bool SearchRadiusIsNotInImage(int x, int y, int line);
        void SetSearchDirectionParameters(float search_direction);
        int GetOtsuThreshold(int x, int y);
        vector<int> ScanIntensitiesInSearchDirection(int x, int y);
        int GetPixelValue(Point point);
        vector<ScannedMoments> GetScannedMoments(int otsu_threshold, int x, int y);
        Point GetPolarCoordinate(int x, int y, float angle, int radius);
        Point GetCenterOfGravity(int x, int y, vector<ScannedMoments> scanned_moments);
        void SearchMaxWeightMoment(SummedMoments &summed_moments, vector<ScannedMoments> scanned_moments);
        void SumUpMoments(SummedMoments &summed_moments, vector<ScannedMoments> scanned_moments);
        Point ChangeToBrightestCoordinateWithinReach(Point center_of_gravity);
        float GetNewAngle(int x, int y, Point new_start_point);
        bool HasGotStuck(int x, int y, Point new_start_point, int line);
        bool IsWalkingBackwards(int y, Point new_start_point, int line);
        void AddIteration(Point new_start_point, float new_angle, int line);

        void SaveCounterValuesToReturnInfo(int line);
        LineFollowerReturnInfo GetReturnInfo();


public:
        LineFollower(int image_height, int image_width, LineFollowerInitializationParameters init);
        LineFollowerReturnInfo FollowLines(Mat grey, StartParameters start_parameters);
        void DrawLinePoints(Mat &rgb, int line);
        void GetLine(vector<PointAndDirection> &_line, int line);
        void CoutReturnInfo();


};

#endif // LINE_FOLLOWER_H
