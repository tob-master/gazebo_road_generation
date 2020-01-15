#ifndef DATATYPES_H
#define DATATYPES_H

#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>


using namespace std;
using namespace cv;



struct SegmentStartIDAndWidth
{
    int start_id;
    int width;
};

enum {LEFT_LINE, MID_LINE,RIGHT_LINE };
enum {LEFT_TO_MID, RIGHT_TO_MID, LEFT_TO_RIGHT, RIGHT_TO_LEFT, MID_TO_LEFT, MID_TO_RIGHT};

// priorities from valid points
enum {
      PRIO_0_P1_AND_P2,
      PRIO_1_P1_AND_FP2,
      PRIO_2_P2_AND_FP1,
      PRIO_3_P1,
      PRIO_4_P2,
      PRIO_5_FP1_AND_FP2,
      PRIO_6_FP1,
      PRIO_7_FP2
     };


struct MinMaxLineElements
{
    Point x_min;
    Point x_max;
    Point y_min;
    Point y_max;
    bool initialized;
};


struct PointInDirection
{
   int x;
   int y;
   int length;
   float angle;
};

struct SafeDriveAreaEvaluationInitializationParameters
{
    int start_of_rect_safety_x;
    int start_of_rect_safety_y;
    int start_search_direction_of_rect_safety;
    int search_rect_width;
    int search_rect_height;
    int rect_border_distance_threshold_for_continous_line;
    int rect_step_length;

};



struct LineValidationTableCreationInitializationParameters
{
    int min_left_to_mid_line_distance;
    int max_left_to_mid_line_distance;
    int min_left_to_right_line_distance;
    int max_left_to_right_line_distance;
    int min_right_to_mid_line_distance;
    int max_right_to_mid_line_distance;
    int min_right_to_left_line_distance;
    int max_right_to_left_line_distance;
    int min_mid_to_left_line_distance;
    int max_mid_to_left_line_distance;
    int min_mid_to_right_line_distance;
    int max_mid_to_right_line_distance;

    int min_left_to_mid_pixel_intensity;
    int min_left_to_right_pixel_intensity;
    int min_right_to_mid_pixel_intensity;
    int min_right_to_left_pixel_intensity;
    int min_mid_to_right_pixel_intensity;
    int min_mid_to_left_pixel_intensity;

    int min_left_to_mid_line_width;
    int max_left_to_mid_line_width;
    int min_left_to_right_line_width;
    int max_left_to_right_line_width;
    int min_right_to_mid_line_width;
    int max_right_to_mid_line_width;
    int min_right_to_left_line_width;
    int max_right_to_left_line_width;
    int min_mid_to_left_line_width;
    int max_mid_to_left_line_width;
    int min_mid_to_right_line_width;
    int max_mid_to_right_line_width;

    int max_distance_of_predicted_to_adjacent_point;
    int min_start_direction_of_line_points_in_drive_direction;
    int max_start_direction_of_line_points_in_drive_direction;
    int max_direction_difference_of_line_points_in_drive_direction;
};


struct OnRoadSearchInitializationParameters
{
    int goal_line_intensity_threshold;
    int min_goal_segment_width;
    int max_goal_segment_width;
    int min_goal_segments_to_find;

    int max_crosswalk_forsight_distance;
    int max_crosswalk_forsight_step_size;
    int min_crosswalk_segment_width;
    int max_crosswalk_segment_width;
    int min_crosswalk_segments_to_find;

    int min_to_left_mid_line_direction_for_crossing;
    int max_to_left_mid_line_direction_for_crossing;
    int min_to_right_mid_line_direction_for_crossing;
    int max_to_right_mid_line_direction_for_crossing;
    int min_outline_direction_difference_for_crossing;
    int max_crossing_forsight_y;
    int min_to_left_left_line_direction_difference_for_crossing;
    int max_to_left_left_line_direction_difference_for_crossing;
    int min_to_right_right_line_direction_difference_for_crossing;
    int max_to_right_right_line_direction_difference_for_crossing;
    int max_left_line_height_for_crossing;
    int max_right_line_height_for_crossing;
    int max_left_line_size_for_crossing;
    int max_right_line_size_for_crossing;

   int max_lane_object_forsight_distance;
   int lane_object_forsight_step_size;

   int left_in_left_lane_lineiterator_end_offset;
   int left_in_right_lane_lineiterator_start_offset;

   int left_in_right_lane_lineiterator_end_offset;

   int right_in_left_lane_lineiterator_start_offset;
   int right_in_left_lane_lineiterator_end_offset;
   int right_in_right_lane_lineiterator_end_offset;


   int left_in_left_lane_radial_outerline_offset;
   int left_in_right_lane_radial_outerline_offset;
   int right_in_left_lane_radial_outerline_offset;
   int right_in_right_lane_radial_outerline_offset;

    int lane_object_radial_scan_step_size;
    int lane_object_radial_scan_radius;

    int min_marking_segments_threshold;

    int min_box_segments_threshold;
    int  min_white_pixels_for_box;

    int velocity_sign_template_height;
    int veloctiy_sign_template_width;
    int goal_line_field_of_view;

    int rect_top_left_point_for_classifier_roi_x;
    int rect_top_left_point_for_classifier_roi_y;

    int rect_bottom_right_point_for_classifier_roi_x;
    int rect_bottom_right_point_for_classifier_roi_y;

    int resize_height_for_classifier_roi;
    int resize_width_for_classifier_roi;

    int template_roi_size_x;
    int template_roi_size_y;
    int road_sign_intensity_threshold;
};

struct LinePointsReduceInitializationParameters
{
    int max_distance_to_reduce_points;
};


struct BirdseyeInitializationParameters
{
    int alpha;
    int beta;
    int gamma;
    int fov;
    int distance;
};

struct ConnectedComponentsSearchInitializationParameters
{
    int connection_count;
    int max_mid_line_component_size;
    int min_mid_line_component_size;
    int max_mid_line_component_volume;
    int min_mid_line_component_volume;
    int min_mid_line_component_distance;
    int max_mid_line_component_distance;
    int end_of_linkage_marker;
    float max_roi_center_to_centroid_distance;
    int car_position_x;
    int car_position_y;
};



struct StartOfLinesSearchInitializationParameters
{
    int top_row;
    int mid_row;
    int bottom_row;
    int min_line_width;
    int max_line_width;
    int min_track_width;
    int max_track_width;
    int window_size_for_line_search;
    int line_threshold;
    int mid_line_threshold;
    int window_size_for_mid_line_search;
    int max_distance_between_adjacent_row_pairs;
    int car_position_in_frame;
    int road_model_left_line;
    int road_model_right_line;
    int line_to_car_distance_threshold;

};

struct LineFollowerInitializationParameters
{
    int max_iterations;
    int search_radius;
    int max_weight_direction_scaler;
    int field_of_view;
    int max_consecutive_back_steps;
    int min_travel_distance_to_not_got_stuck;
    int max_got_stuck_counts;

};


struct MidLineSearchInitializationParameters
{
    int min_pixel_value_for_clustering;
    int max_radial_scan_out_of_cluster_value;
    float radial_scan_scaling_factor;
    int mid_line_length;
    int min_valuable_cluster_size;
    int min_cluster_distance;
    int max_cluster_distance;
    int car_position_x;
    int car_position_y;


};

struct VanishingPointSearchInitializationParameters
{
    int canny_low_threshold;
    int canny_high_threshold;
    int canny_kernel_size;

    int hough_lines_rho;
    float hough_lines_theta;
    int hough_lines_min_intersections;
    int hough_lines_min_line_length;
    int hough_lines_min_line_gap;

    int x_roi_start;
    int y_roi_start;
    int roi_width;
    int roi_height;

    int min_left_line_angle;
    int max_left_line_angle;

    int min_right_line_angle;
    int max_right_line_angle;

    int x_min_left_line;
    int x_max_left_line;
    int x_min_right_line;
    int x_max_right_line;

    int car_mid_position_x;
    int car_mid_position_y;

    float max_standard_deviation_for_valid_vanishing_point;

};

struct LineValidationTableCreationReturnInfo
{
    vector<Point> left_found_both_points_and_predictions;
    vector<Point> left_found_both_predictions;
    vector<Point> left_found_mid_prediction;
    vector<Point> left_found_right_prediction;
    vector<Point> left_found_both_points;
    vector<Point> left_found_mid_point;
    vector<Point> left_found_right_point;
    int left_line_size;

    vector<Point> mid_found_both_points_and_predictions;
    vector<Point> mid_found_both_predictions;
    vector<Point> mid_found_left_prediction;
    vector<Point> mid_found_right_prediction;
    vector<Point> mid_found_both_points;
    vector<Point> mid_found_left_point;
    vector<Point> mid_found_right_point;
    int mid_line_size;

    vector<Point> right_found_both_points_and_predictions;
    vector<Point> right_found_both_predictions;
    vector<Point> right_found_left_prediction;
    vector<Point> right_found_mid_prediction;
    vector<Point> right_found_both_points;
    vector<Point> right_found_left_point;
    vector<Point> right_found_mid_point;
    int right_line_size;

    void reset() { *this = {}; }
};


struct MidLineSearchReturnInfo
{
    bool has_found_mid_line_clusters;
    bool has_found_group;
    void reset() { *this = {}; }
};

struct ConnectedComponentsSearchReturnInfo
{
    bool has_found_mid_line_components;
    bool has_found_mid_line_group;
    void reset() { *this = {}; }
};


struct StartOfLinesSearchReturnInfo
{
    bool has_found_start_parameters;
        void reset() { *this = {}; }
};

struct LineFollowerReturnInfo
{
    bool left_line_max_iterations_exceeded;
    bool left_line_search_radius_out_of_image;
    bool left_line_has_got_stuck;
    bool left_line_is_walking_backwards;
    int left_line_iterations_counter;
    int left_line_got_stuck_counter;
    int left_line_walked_backwards_counter;

    bool right_line_max_iterations_exceeded;
    bool right_line_search_radius_out_of_image;
    bool right_line_has_got_stuck;
    bool right_line_is_walking_backwards;
    int right_line_iterations_counter;
    int right_line_got_stuck_counter;
    int right_line_walked_backwards_counter;
        void reset() { *this = {}; }
};

struct LinePointsReducerReturnInfo
{
    bool left_line_is_reduced;
    int left_line_reduced_size;
    bool right_line_is_reduced;
    int right_line_reduced_size;
        void reset() { *this = {}; }
};


struct VanishingPointSearchReturnInfo
{
    bool has_found_left_hough_line;
    int left_hough_lines_count;

    bool has_found_right_hough_line;
    int right_hough_lines_count;

    bool has_found_intersections;
    int  intersections_count;

    bool has_found_vanishing_point;

    cv::Point vanishing_point;

    float car_mid_point_to_vanishing_point_angle;
        void reset() { *this = {}; }

};

struct StartParameters
{
    int left_x;
    int left_y;
    float left_angle;
    bool found_left_line;

    int right_x;
    int right_y;
    float right_angle;
    bool found_right_line;
        void reset() { *this = {}; }

};



namespace connected_components_search
{
    struct ConnectedComponent
    {
        int centroid_x;
        int centroid_y;
        int roi_x;
        int roi_y;
        int roi_width;
        int roi_height;
        int mass;
    };

    struct LengthAndDirectionFromConnectedComponents
    {
       int x;
       int y;
       int length;
       float angle;
    };
}

namespace vanishing_point_search
{

    struct HoughLinesWarpedPerspektive
    {
        int x_bottom;
        int y_bottom;
        int x_top;
        int y_top;
    };

    struct HoughLinesInDriveDirection
    {
        int x_bottom;
        int y_bottom;
        int x_top;
        int y_top;
    };

    struct HoughLinesPointsAndAngle
    {
        int x_bottom;
        int y_bottom;
        int x_top;
        int y_top;
        int angle;
    };

    struct Intersections
    {
        int intersection_x;
        int intersection_y;
        int left_x_bottom;
        int left_y_bottom;
        int left_x_top;
        int left_y_top;
        int right_x_bottom;
        int right_y_bottom;
        int right_x_top;
        int right_y_top;
    };

};






namespace mid_line_search
{
    typedef std::reverse_iterator<std::_Rb_tree_iterator<pair<pair<int, int> const, vector<pair<int, int>,allocator<pair<int, int>>>>>> ReverseMidLineCoordinatesIterator;
    //typedef std::reverse_iterator<std::_Rb_tree_iterator<pair<pair<int, int> const, vector<pair<int, int>,allocator<pair<int, int>>>>>> ReverseMidLineCoordinatesIterator;

    struct ClusterBinKey
    {
        int x_cluster_bin_key;
        int y_cluster_bin_key;
    };

    struct TwoConnectedClustersCentersOfGravity
    {
        int x_top_center_of_gravity;
        int y_top_center_of_gravity;
        int x_bottom_center_of_gravity;
        int y_bottom_center_of_gravity;

    };

    struct SingleCluster
    {
        int x_center;
        int y_center;
        int angle;
    };

};


namespace line_points_reducer
{
    typedef std::pair<double, double> RamerDouglasPeuckerTypePoint;


    struct ReducedPoints
    {
        int x;
        int y;
    };


}




namespace line_follower
{
    struct PointAndDirection
    {
        int x;
        int y;
        float angle;
    };

    struct ScannedMoments
    {
        int x;
        int y;
        int intensity_sum;
    };


    struct SummedMoments
    {
        float moment_weight;
        float moment_x;
        float moment_y;
        int max_weight;
        int   max_weight_id;

    };
}





namespace valid_line_point_search
{
    struct RectSafetyTable
    {
        float percent_points_with_priority_0;
        float percent_points_with_priority_1;
        float percent_points_with_priority_2;
        float percent_points_with_priority_3;
        float percent_points_with_priority_4;
        float percent_points_with_priority_5;
        float percent_points_with_priority_6;
        float percent_points_with_priority_7;
        float percent_points_with_priority_8;

        float percent_points_in_rect;
        bool too_few_points_in_rect;
        bool rect_straight;
        bool rect_left_curve;
        bool rect_right_curve;
        bool y_min_in_rect_border_range;
        bool y_max_in_rect_border_range;

    };




    struct LastAdjacentPointMatch
    {

        Point last_left_point;
        Point last_mid_point;
        Point last_right_point;

        int last_left_id;
        int last_mid_id;
        int last_right_id;

        bool left_set;
        bool mid_set;
        bool right_set;

    };


    struct ValidPoints
    {
        int line_code;
        Point origin;

        bool left;
        bool mid;
        bool right;

        int next_directions_distance;
        int search_direction;

    };



    struct SearchLineDistanceThresholds
    {
        int min;
        int max;
    };


    struct SearchLineWidthThresholds
    {
        int min;
        int max;
    };



    struct RightValidationTable
    {
        Point origin;
        float search_direction;
        int next_direction_distance;

        Point left;
        Point mid;


        bool left_near_left_origin;
        bool mid_near_mid_origin;

        bool left_origin_equal_direction;
        bool mid_origin_equal_direction;

        bool origin_near_left_to_right;
        bool origin_near_mid_to_right;

        bool found_left;
        bool found_mid;
        int score;

    };


    struct MidValidationTable
    {
        Point origin;
        float   search_direction;
        int next_direction_distance;

        Point left;
        Point right;


        bool left_near_left_origin;
        bool right_near_right_origin;

        bool left_origin_equal_direction;
        bool right_origin_equal_direction;

        bool origin_near_left_to_mid;
        bool origin_near_right_to_mid;

        bool found_left;
        bool found_right;

        int score;
        int label;

    };


    struct LeftValidationTable
    {
        Point origin;
        float   search_direction;
        int next_direction_distance;

        Point mid;
        Point right;


        bool mid_near_mid_origin;
        bool right_near_right_origin;

        bool mid_origin_equal_direction;
        bool right_origin_equal_direction;

        bool origin_near_mid_to_left;
        bool origin_near_right_to_left;

        bool found_mid;
        bool found_right;


        int score;

    };


    struct ValidLinePointSearchInfo
    {
        Point origin;
        float search_direction;
        int next_direction_distance;
        Point adjacent_line_point;
    };


}


namespace start_of_lines_search
{



    struct TrueLineWidthRowId
    {
        int row_id;
    };

    struct TrueTrackWidthRowPairIds
    {
        int row_id1;
        int row_id2;
    };

    struct TrueAdjacentTrackWidthRowPairIds
    {
        int bottom_row_left_id;
        int bottom_row_right_id;

        int mid_row_left_id;
        int mid_row_right_id;
    };


    struct PatternMatchIds
    {
      int bottom_row_left_id;
      int bottom_row_right_id;
      int mid_row_left_id;
      int mid_row_right_id;
      int bottom_row_mid_id;
      int mid_row_mid_id;
      int top_row_mid_id;
    };
}



#endif // DATATYPES_H
