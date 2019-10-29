#ifndef OWN_UTILS_H
#define OWN_UTILS_H

#include <iostream>
#include <stdlib.h>
#include "own_defines.h"
using namespace std;





inline float CalculateAngle4Quadrants(int opposite, int adjacent)
{
    float angle = 0;

    if(adjacent != 0 && opposite != 0)
    {
        float div = float(abs(opposite))/ float(abs(adjacent));
        angle = atan(div);
        angle = angle * 180/PI;

        if (adjacent > 0 && opposite > 0)
        ;
        else if (adjacent < 0 && opposite > 0)
        {
            angle = 180 - angle;
        }
        else if (adjacent < 0 && opposite < 0)
        {
            angle = 180 + angle;
        }
        else if (adjacent > 0 && opposite < 0)
        {
            angle = 360 - angle;
        }
        else {
            cout << "something went wrong??" << endl;
        }
    }
    else if(adjacent > 0 && opposite == 0)
    {
            angle = 0;
    }
    else if(adjacent < 0 && opposite == 0)
    {
            angle = 180;
    }
    else if(adjacent == 0 && opposite > 0)
    {
            angle = 90;
    }
    else if(adjacent == 0 && opposite < 0)
    {
            angle = 270;
    }
    else
    {
        angle = 0;
    }

    return angle;
};


#endif // OWN_UTILS_H
