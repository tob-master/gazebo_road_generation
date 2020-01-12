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

class LineFollow
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



        void ResetCounters(int &iterations_counter, int &got_stuck_counter, int &walked_backwards_counter);
        int FollowLine(int x, int y, float search_direction, int line);

        bool MaxIterationsExceeded(const int iterations_counter,
                                   bool &left_line_max_iterations_exceeded,
                                   bool &right_line_max_iterations_exceeded,
                                   const int kMaxIterations,
                                   const int line_type);

        bool SearchRadiusIsNotInImage(int x,
                                      int y,
                                      bool &left_line_search_radius_out_of_image,
                                      bool &right_line_search_radius_out_of_image,
                                      const int kSearchRadius,
                                      const int kImageWidth,
                                      const int kImageHeight,
                                      int line_type);

        void SetSearchDirectionParameters(float search_direction,
                                          float &start_of_search,
                                          float &end_of_search,
                                          const float kStartAngleFieldOfView ,
                                          const float kEndAngleFieldOfView);
        int GetOtsuThreshold(int x,
                             int y,
                             Mat image,
                             const float start_of_search,
                             const float end_of_search,
                             const float kStepFieldOfView,
                             const int kSearchRadius);

        vector<int> ScanIntensitiesInSearchDirection(int x,
                                                     int y,
                                                     Mat image,
                                                     const float start_of_search,
                                                     const float end_of_search,
                                                     const float kStepFieldOfView,
                                                     const int kSearchRadius);
        int GetPixelValue(Mat image, Point point);

        vector<ScannedMoments> GetScannedMoments(int x,
                                                 int y,
                                                 Mat image,
                                                 int otsu_threshold,
                                                 const float  start_of_search,
                                                 const float end_of_search,
                                                 const float kStepFieldOfView,
                                                 const int kSearchRadius);

        Point GetPolarCoordinate(int x, int y, float angle, int radius);

        Point GetCenterOfGravity(int x, int y, vector<ScannedMoments> scanned_moments);
        void SearchMaxWeightMoment(SummedMoments &summed_moments, vector<ScannedMoments> scanned_moments);
        void SumUpMoments(SummedMoments &summed_moments, vector<ScannedMoments> scanned_moments);
        Point ChangeToBrightestCoordinateWithinReach(Mat image, Point center_of_gravity);
        float GetNewAngle(int x, int y, Point new_start_point);

        bool HasGotStuck(int x,
                         int y,
                         const Point new_start_point,
                         bool &left_line_has_got_stuck,
                         bool &right_line_has_got_stuck,
                         int &got_stuck_counter,
                         const int kMinTravelDistanceToNotGotStuck,
                         const int kMaxGotStuckCounts,
                         const int line_type);

        bool IsWalkingBackwards(int y,
                                Point new_start_point,
                                bool &left_line_is_walking_backwards,
                                bool &right_line_is_walking_backwards,
                                int  &walked_backwards_counter,
                                const int kMaxConsecutiveBackSteps,
                                const int line_type);

        void AddIteration(Point new_start_point,
                          vector<PointAndDirection> &left_line_points_and_directions,
                          vector<PointAndDirection> &right_line_points_and_directions,
                          int &iterations_counter,
                          const float new_angle,
                          const int line_type);

        void SaveCounterValuesToReturnInfo(const int iterations_counter,
                                           const int got_stuck_counter,
                                           const int walked_backwards_counter,
                                           int &left_line_iterations_counter,
                                           int &left_line_got_stuck_counter,
                                           int &left_line_walked_backwards_counter,
                                           int &right_line_iterations_counter,
                                           int &right_line_got_stuck_counter,
                                           int &right_line_walked_backwards_counter,
                                           int line_type);

        LineFollowerReturnInfo GetReturnInfo();


public:
        void SetImage(Mat image);
        void SetStartParameters(StartParameters start_parameters);
        void ClearMemory();
        LineFollow(int image_height, int image_width, LineFollowerInitializationParameters init);
        LineFollowerReturnInfo FollowLines();
        void DrawLinePoints(Mat &rgb, int line);
        void GetLine(vector<PointAndDirection> &_line, int line);
        void CoutReturnInfo();


};

#endif // LINE_FOLLOWER_H
