/*
 * image_listener.cpp
 *
 *  Created on: Apr 30, 2015
 *      Author: darrenl
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <stdio.h>
#include <iostream>

#define PI 3.14

using namespace cv;

static const std::string TOPIC_NAME = "/rrbot/camera1/image_raw";
int image_counter = 0;

// default values of the homography parameters
int alpha_=34;
int  beta_=90;
int gamma_=90;
int f_ = 211; 
int dist_ = 65;

double alpha = ((double)alpha_ - 90.)*PI/180;
double beta = ((double)beta_ - 90.)*PI/180;
double gammma = ((double)gamma_ - 90.)*PI/180;
double f = (double) f_;
double dist = (double) dist_;


double w = 1280., h = 720.;
Size taille(w,h);
// Projection 2D -> 3D matrix
Mat A1 = (Mat_<double>(4,3) <<
  1, 0, -w/2,
  0, 1, -h/2,
  0, 0,    0,
  0, 0,    1);

// Rotation matrices around the X,Y,Z axis
Mat RX = (Mat_<double>(4, 4) <<
  1,          0,           0, 0,
  0, cos(alpha), -sin(alpha), 0,
  0, sin(alpha),  cos(alpha), 0,
  0,          0,           0, 1);

Mat RY = (Mat_<double>(4, 4) <<
  cos(beta), 0, -sin(beta), 0,
          0, 1,          0, 0,
  sin(beta), 0,  cos(beta), 0,
          0, 0,          0, 1);

Mat RZ = (Mat_<double>(4, 4) <<
  cos(gammma), -sin(gammma), 0, 0,
  sin(gammma),  cos(gammma), 0, 0,
  0,          0,           1, 0,
  0,          0,           0, 1);

// Composed rotation matrix with (RX,RY,RZ)
Mat R = RX * RY * RZ;

// Translation matrix on the Z axis change dist will change the height
Mat T = (Mat_<double>(4, 4) <<           1, 0, 0, 0,           0, 1, 0, 0,           0, 0, 1, dist,           0, 0, 0, 1);       // Camera Intrisecs matrix 3D -> 2D
Mat A2 = (Mat_<double>(3,4) <<
  f, 0, w/2, 0,
  0, f, h/2, 0,
  0, 0,   1, 0);

// Final and overall transformation matrix
Mat transfo = A2 * (T * (R * A1));


// images
cv::Mat warped;
cv::Mat cropped;


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {


    try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");

            //Apply matrix transformation
            warpPerspective(cv_ptr->image, warped, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);

            cropped= warped(Rect(0,0,1280,417));

            cv::imshow("Result", cropped);
            cv::waitKey(1);

            char c[5];
            snprintf (c, 5, "%04d", image_counter);

            std::string formatted_count(c);
            std::string all_frames_path = "/home/tb/gazebo_road_generation/ros_ws/src/track_annotation/all_frames/frame_" + formatted_count + ".png";

            if(image_counter % 20 == 0){
                std::string annotated_frames_path = "/home/tb/gazebo_road_generation/ros_ws/src/track_annotation/annotated_frames/frame_" + formatted_count + ".png";
                cv::imwrite(annotated_frames_path, cropped);
            }

            cv::imwrite(all_frames_path, cropped);
            image_counter++;
            //cv::imshow("view", cv_bridge::toCvShare(msg, "mono8")->image);

        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'mono8'.",
            msg->encoding.c_str());
        }
	


}


int main(int argc, char **argv) {

    ros::init(argc, argv, "image_transport_subscriber");
    ros::NodeHandle nh;

    cv::namedWindow("Result", 1);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(TOPIC_NAME, 10000, imageCallback);

    ros::spin();
    cv::destroyWindow("Result");

    ros::shutdown();
    return 0;
}
