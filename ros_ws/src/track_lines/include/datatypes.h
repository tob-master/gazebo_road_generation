#ifndef DATATYPES_H
#define DATATYPES_H

#include <iostream>
using namespace std;


enum {LEFT_LINE, RIGHT_LINE};

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
    int max_connected_cluster_distance;

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


struct MidLineSearchReturnInfo
{
    bool has_found_mid_line_clusters;
    bool has_found_group;

};

struct StartOfLinesSearchReturnInfo
{
    bool has_found_start_parameters;
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
};

struct LinePointsReducerReturnInfo
{
    bool left_line_is_reduced;
    int left_line_reduced_size;
    bool right_line_is_reduced;
    int right_line_reduced_size;
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

};

struct StartParameters
{
    int left_x;
    int left_y;
    float left_angle;

    int right_x;
    int right_y;
    float right_angle;
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
};






namespace mid_line_search
{




    typedef std::reverse_iterator<std::_Rb_tree_iterator<pair<pair<int, int> const, vector<pair<int, int>,allocator<pair<int, int>>>>>> ReverseMidLineCoordinatesIterator;


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

    struct LengthAndDirectionFromConnectedClusters
    {
       int x;
       int y;
       int length;
       float angle;
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

    struct LengthAndDirectionFromConsecutiveReducedLinePoints
    {
       int x;
       int y;
       int length;
       float angle;
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

namespace start_of_lines_search
{




    struct StartAndWidth
    {
        int start_id;
        int width;
    };

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
