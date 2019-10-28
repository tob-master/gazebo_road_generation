#ifndef OWN_DATATYPES_H
#define OWN_DATATYPES_H

struct LineSearchStartParameters
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



#endif // OWN_DATATYPES_H
