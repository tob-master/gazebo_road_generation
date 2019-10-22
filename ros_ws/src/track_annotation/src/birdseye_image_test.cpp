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

int main(int argc, char **argv) {

    ros::init(argc, argv, "birdseye_image_test");
    ros::NodeHandle nh;

    int alpha_=34;
    int  beta_=90;
    int gamma_=90;
    int f_ = 211;
    int dist_ = 65;

    cv::namedWindow("Result", 1);
    cv::createTrackbar("Alpha", "Result", &alpha_, 180);
    cv::createTrackbar("Beta", "Result", &beta_, 180);
    cv::createTrackbar("Gamma", "Result", &gamma_, 180);
    cv::createTrackbar("f", "Result", &f_, 2000);
    cv::createTrackbar("Distance", "Result", &dist_, 2000);

    cv::Mat destination;

    while(1){

        cv::Mat image = cv::imread("/home/tb/gazebo_road_generation/ros_ws/src/track_annotation/images/crossing.png", CV_LOAD_IMAGE_GRAYSCALE);

        double f, dist;
        double alpha, beta, gamma;
        alpha = ((double)alpha_ - 90.)*PI/180;
        beta = ((double)beta_ - 90.)*PI/180;
        gamma = ((double)gamma_ - 90.)*PI/180;
        f = (double) f_;
        dist = (double) dist_;

        Size taille = image.size();
        double w = (double)taille.width, h = (double)taille.height;

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
          cos(gamma), -sin(gamma), 0, 0,
          sin(gamma),  cos(gamma), 0, 0,
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

        // Apply matrix transformation
        cv::warpPerspective(image, destination, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);

        cv::imshow("Result", destination);
        cv::waitKey(30);
    }

    ros::spin();
    ros::shutdown();
    return 0;
}
