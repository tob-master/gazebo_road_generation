#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <stdlib.h>
#include "defines.h"



#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>


using namespace std;
using namespace cv;

inline void mouse_callback(int  event, int  x, int  y, int  flag, void *param)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        std::cout << x << " " << y << std::endl;
    }
      /*if (event == EVENT_MOUSEMOVE)
      {
          std::cout << x << " " << y << std::endl;
      }*/
}

inline void TransformPoint(int& x, int& y, Mat transformation_matrix)
{

    vector<Point2f> src = {Point2f(float(x), float(y))};
    vector<Point2f> dst;

    perspectiveTransform(src,dst,transformation_matrix);

    x = int(dst[0].x);
    y = int(dst[0].y);

}


inline float CalculateAngle4Quadrants(float opposite, float adjacent)
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


#endif // UTILS_H
