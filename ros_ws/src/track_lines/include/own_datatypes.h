#ifndef OWN_DATATYPES_H
#define OWN_DATATYPES_H


struct BirdseyeInitializationParameters
{
    int alpha;
    int beta;
    int gamma;
    int fov;
    int distance;
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



struct StartParameters
{
    int left_x;
    int left_y;
    float left_angle;

    int right_x;
    int right_y;
    float right_angle;
};

struct LineSearchMoments
{
    int x;
    int y;
    int sum;
};

struct LineSearchFoundPointAndDirection
{
    int x;
    int y;
    float angle;
};


struct TwoConnectedClusters
{
    int top_cog_x;
    int top_cog_y;
    int bottom_cog_x;
    int bottom_cog_y;

};

struct ConnectedClusterKeys
{
    int key_x_1;
    int key_y_1;

    int key_x_2;
    int key_y_2;
};



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
#endif // OWN_DATATYPES_H
