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



#endif // OWN_DATATYPES_H
