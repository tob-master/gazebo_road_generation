#include "houghline_transform.h"

HoughLineTransform::HoughLineTransform()
{

}

void HoughLineTransform::ApplyCannyEdge(Mat image, int low_threshold, int high_threshold, int kernel_size)
{
    Canny(image, canny_image_, low_threshold, high_threshold, kernel_size);
}

void HoughLineTransform::ShowCannyEdgeImage()
{
    imshow("Canny Imgae", canny_image_);
    waitKey(30);
}

void HoughLineTransform::ApplyHoughLines(int rho, float theta, int min_intersections, int min_line_length, int max_line_gap)
{

    cv::Rect myROI(0, 350, 1280, 67);

    hough_image_ = canny_image_(myROI);

    HoughLinesP(hough_image_, found_lines_, rho, theta, min_intersections, min_line_length, max_line_gap );
}

void HoughLineTransform::ShowHoughLines()
{

    cv::cvtColor(hough_image_, hough_image_, CV_GRAY2BGR);


    int min_angle =  87;

    for( size_t i = 0; i < found_lines_.size(); i++ )
    {
      Vec4i l = found_lines_[i];




      if(l[2]-l[0] == 0)
      {
          cout << "90" << endl;
          line( hough_image_, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
      }
      else
      {
          float angle = abs(atan((l[3]-l[1])/(l[2]-l[0])) * 180/PI);
          if(angle > min_angle)
          {
            cout << angle << endl;
            line( hough_image_, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
          }
      }




    }


    imshow("Hough Lines Imgae", hough_image_);
    waitKey(30);
}


