#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <stdio.h>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <image_transport/image_transport.h>
#include <ctime>



#define PI 3.14

using namespace cv;



Point pt(-1,-1);
bool newCoords = false;

void mouse_callback(int  event, int  x, int  y, int  flag, void *param)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        // Store point coordinates
        pt.x = x;
        pt.y = y;
        newCoords = true;
    }

      if (event == EVENT_MOUSEMOVE)
      {

          std::cout << x << " " << y << std::endl;


      }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pixel_coordinates_node");
  ros::NodeHandle nh;


  cv::Mat image = cv::imread("/home/tb/gazebo_road_generation/ros_ws/src/track_annotation/images/test2.png", CV_LOAD_IMAGE_COLOR);

    namedWindow("img", WINDOW_NORMAL);

    // Set callback
    setMouseCallback("img", mouse_callback);

    for (;;)
    {


        // Show last point clicked, if valid
        if (pt.x != -1 && pt.y != -1)
        {
            circle(image, pt, 1, Scalar(0, 0, 255));

            if (newCoords)
            {
                std::cout << "Clicked coordinates: " << pt << std::endl;
                newCoords = false;
            }
        }

        imshow("img", image);

        // Exit if 'q' is pressed
        if ((waitKey(1) & 0xFF) == 'q') break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}



