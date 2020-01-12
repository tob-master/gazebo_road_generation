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

//using namespace std;
//using namespace cv;
using namespace vanishing_point_search;


class VanishingPointSearch
{
    private:

        const cv::Mat frontalview_to_birdseye_transformation_matrix_;

        const int kCannyLowThreshold_;
        const int kCannyHighThreshold_;
        const int kCannyKernelSize_;

        const int kHoghLinesRho_;
        const float kHoughLinesTheta_;
        const int kHoughLinesMinintersections_;
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

        const cv::Point kCarMidPoint_;

        const float kMaxStandardDeviationForValidVanishingPoint_;

        cv::Mat current_image_;
        cv::Mat canny_image_;
        cv::Mat hough_image_;
        cv::Mat current_image_roi_;
        std::vector<Vec4i> hough_lines_;

        std::vector<HoughLinesInDriveDirection> hough_lines_in_drive_direction_;
        std::vector<HoughLinesInDriveDirection> left_hough_lines_in_drive_direction_;
        std::vector<HoughLinesInDriveDirection> right_hough_lines_in_drive_direction_;

        std::vector<HoughLinesPointsAndAngle> left_hough_lines_points_and_angle_;
        std::vector<HoughLinesPointsAndAngle> right_hough_lines_points_and_angle_;

        std::vector<Intersections> intersecting_lines_;
        std::vector<Intersections> vanishing_point_intersections_;
        StartParameters line_follower_start_parameters_;
        cv::Point vanishing_point_;
        cv::Point warped_vanishing_point_;
        cv::Point warped_car_mid_point_;

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

        void WarpCarMidPointToBirdsview(Mat frontalview_to_birdseye_transformation_matrix,
                                        Point &warped_car_mid_point,
                                        const Point kCarMidPoint);


        void CropImageToRegionOfInterest(Mat current_image,
                                         Mat &current_image_roi,
                                         const int kXROIStart,
                                         const int kYROIStart,
                                         const int kROIWidth,
                                         const int kROIHeight);

        void ApplyCannyEdge(Mat current_image_roi,
                            Mat &canny_image,
                            const int kCannyLowThreshold,
                            const int kCannyHighThreshold,
                            const int kCannyKernelSize);

        void ApplyHoughLines(Mat canny_image ,
                             vector<Vec4i> &hough_lines,
                             const int  kHoghLinesRho, const float kHoughLinesTheta,
                             const int kHoughLinesMinintersections,
                             const int kHoughLinesMinLineLength,
                             const int kHoughLinesMaxLineGap);

        void AddRegionOfInterestOffsetToHoughLinePoints(vector<Vec4i> &hough_lines,
                                                        const int kXROIStart,
                                                        const int kYROIStart);
        void ChangeLinePointsToDriveDirection(vector<Vec4i> hough_lines,
                                              vector<HoughLinesInDriveDirection> &hough_lines_in_drive_direction);

        void GatherTrueRangeLeftAndRightLines(vector<HoughLinesInDriveDirection> hough_lines_in_drive_direction,
                                              vector<HoughLinesInDriveDirection> &left_hough_lines_in_drive_direction,
                                              vector<HoughLinesInDriveDirection> &right_hough_lines_in_drive_direction,
                                              const int kXMinLeftLine,
                                              const int kXMaxLeftLine,
                                              const int kXMinRightLine,
                                              const int kXMaxRightLine);

        void RejectFalseLeftAndRightLineAngles(vector<HoughLinesInDriveDirection> left_hough_lines_in_drive_direction,
                                               vector<HoughLinesInDriveDirection> right_hough_lines_in_drive_direction,
                                               vector<HoughLinesPointsAndAngle> &left_hough_lines_points_and_angle,
                                               vector<HoughLinesPointsAndAngle> &right_hough_lines_points_and_angle,
                                               const int kMinLeftLineAngle,
                                               const int kMaxLeftLineAngle,
                                               const int kMinRightLineAngle,
                                               const int kMaxRightLineAngle);

        std::pair<double, double> ComputeLineIntersection(std::pair<double, double> A, std::pair<double, double> B,
                                                     std::pair<double, double> C, std::pair<double, double> D);


        //void ApplyDBScan();


        void CheckFoundLeftAndRightHoughLines(vector<HoughLinesPointsAndAngle> left_hough_lines_points_and_angle,
                                              vector<HoughLinesPointsAndAngle> right_hough_lines_points_and_angle,
                                              int &left_hough_lines_count,
                                              int &right_hough_lines_count,
                                              bool &has_found_left_hough_line,
                                              bool &has_found_right_hough_line);

        void ComputeLeftAndRightHoughLineIntersections(vector<HoughLinesPointsAndAngle> left_hough_lines_points_and_angle,
                                                       vector<HoughLinesPointsAndAngle> right_hough_lines_points_and_angle,
                                                       vector<Intersections> &intersecting_lines);

        void CheckFoundIntersections(vector<Intersections> intersecting_lines,
                                     int &intersections_count,
                                     bool &has_found_intersections);


        void FilterVanishingPoint(vector<Intersections> intersecting_lines,
                                  Point &vanishing_point,
                                  const float kMaxStandardDeviationForValidVanishingPoint);


        void ComputeCarMidPointToVanishingPointAngle(const Point kCarMidPoint,
                                                     const Point vanishing_point,
                                                     float &car_mid_point_to_vanishing_point_angle);

        void TransformHoughLinesToBirdseye();

        VanishingPointSearchReturnInfo GetReturnInfo();


        void SetLineFollowerStartParameters(vector<Intersections> vanishing_point_intersections,
                                            vector<HoughLinesPointsAndAngle> left_hough_lines_points_and_angle,
                                            vector<HoughLinesPointsAndAngle> right_hough_lines_points_and_angle,
                                            Mat frontalview_to_birdseye_transformation_matrix,
                                            vector<HoughLinesWarpedPerspektive> &left_hough_lines_warped_perspektive,
                                            vector<HoughLinesWarpedPerspektive> &right_hough_lines_warped_perspektive,
                                            StartParameters &line_follower_start_parameters,
                                            bool has_found_intersections,
                                            bool has_found_left_hough_line,
                                            bool has_found_right_hough_line);


    public:
        void SetImage(Mat image);
        void ClearMemory();

        VanishingPointSearch(Mat birdseye_transformation_matrix, VanishingPointSearchInitializationParameters init);

        VanishingPointSearchReturnInfo FindVanishingPoint();

        StartParameters GetLineFollowerStartParameters();

        void ShowCannyEdgeImage();
        void DrawHoughLines(Mat &image, int _line);
        void CoutHoughLines();
        void DrawWarpedPerspektiveHoughLines(Mat &rgb,int _line);
        void DrawLineIntersections(Mat &rgb);
        void DrawVanishingPoint(Mat &rgb);
        void DrawWarpedVanishingPointDirection(Mat &rgb);



};

#endif // VANISHING_POINT_SEARCH_H
