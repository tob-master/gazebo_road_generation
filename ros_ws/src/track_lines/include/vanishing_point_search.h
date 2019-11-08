#ifndef VANISHING_POINT_SEARCH_H
#define VANISHING_POINT_SEARCH_H

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

#include "dbscan.h"

#define MINIMUM_POINTS 3     // minimum number of cluster
#define EPSILON (5)  // distance for clustering, metre^2

#include "datatypes.h"
#include "utils.h"

using namespace std;
using namespace cv;
using namespace vanishing_point_search;


class VanishingPointSearch
{
    private:

        const cv::Matx33f birdseye_transformation_matrix_;

        const int kCannyLowThreshold_;
        const int kCannyHighThreshold_;
        const int kCannyKernelSize_;

        const int kHoghLinesRho_;
        const float kHoughLinesTheta_;
        const int kHoughLinesMinIntersections_;
        const int kHoughLinesMinLineLength_;
        const int kHoughLinesMaxLineGap_;

        const int kXROIStart_;
        const int kYROIStart_;
        const int kROIWidth_;
        const int kROIHeight_;

        const int kMinLeftLineAngle_;
        const int kMaxLeftLineAngle_;

        const int kMinRightLineAngle_;
        const int kMaxRightLineAngle_;

        const int kXMinLeftLine_;
        const int kXMaxLeftLine_;
        const int kXMinRightLine_;
        const int kXMaxRightLine_;

        const Point kCarMidPoint_;

        const float kMaxStandardDeviationForValidVanishingPoint_;

        Mat current_image_;
        Mat canny_image_;
        Mat hough_image_;
        Mat current_image_roi_;
        vector<Vec4i> hough_lines_;

        vector<HoughLinesInDriveDirection> hough_lines_in_drive_direction_;
        vector<HoughLinesInDriveDirection> left_hough_lines_in_drive_direction_;
        vector<HoughLinesInDriveDirection> right_hough_lines_in_drive_direction_;

        vector<HoughLinesPointsAndAngle> left_hough_lines_points_and_angle_;
        vector<HoughLinesPointsAndAngle> right_hough_lines_points_and_angle_;

        vector<Point> intersections_;
        Point vanishing_point_;
        Point warped_vanishing_point_;
        Point warped_car_mid_point_;

        int left_hough_lines_count_;
        int right_hough_lines_count_;
        int intersections_count_;

        bool has_found_left_hough_line_;
        bool has_found_right_hough_line_;
        bool has_found_intersections_;
        bool has_found_vanishing_point_;

        vector<HoughLinesWarpedPerspektive> left_hough_lines_warped_perspektive_;
        vector<HoughLinesWarpedPerspektive> right_hough_lines_warped_perspektive_;

        float car_mid_point_to_vanishing_point_angle_;

        void WarpCarMidPointToBirdsview();

        void SetImage(Mat image);
        void ClearMemory();
        void CropImageToRegionOfInterest();
        void ApplyCannyEdge();
        void ApplyHoughLines();
        void AddRegionOfInterestOffsetToHoughLinePoints();
        void ChangeLinePointsToDriveDirection();
        void GatherTrueRangeLeftAndRightLines();
        void RejectFalseLeftAndRightLineAngles();

        pair<double, double> ComputeLineIntersection(pair<double, double> A, pair<double, double> B,
                                                     pair<double, double> C, pair<double, double> D);

        void ComputeLeftAndRightHoughLineIntersections();
        void ApplyDBScan();
        void FilterVanishingPoint();

        void CheckFoundLeftAndRightHoughLines();
        void CheckFoundIntersections();

        void ComputeCarMidPointToVanishingPointAngle();

        void WarpPerspektiveOfHoughLines();

        VanishingPointSearchReturnInfo GetReturnInfo();




    public:
        VanishingPointSearch(Mat birdseye_transformation_matrix, VanishingPointSearchParameterInitialization init);

        VanishingPointSearchReturnInfo FindVanishingPoint(Mat image);

        void ShowCannyEdgeImage();
        void DrawHoughLines(Mat &image, int _line);
        void CoutHoughLines();
        void DrawWarpedPerspektiveHoughLines(Mat &rgb,int _line);
        void DrawLineIntersections(Mat &rgb);
        void DrawVanishingPoint(Mat &rgb);
        void DrawWarpedVanishingPointDirection(Mat &rgb);
};

#endif // VANISHING_POINT_SEARCH_H
