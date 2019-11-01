#ifndef OWN_DATATYPES_H
#define OWN_DATATYPES_H

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


namespace lineclassification
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
