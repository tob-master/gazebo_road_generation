#include "vanishing_point_search.h"

VanishingPointSearch::VanishingPointSearch()
{

}


void VanishingPointSearch::SetImage(Mat image)
{
    current_image_ = image;
}

void VanishingPointSearch::CropToRegionOfInterest()
{
    current_image_roi_ =  current_image_(cv::Rect(kXROIStart_, kYROIStart_, kROIWidth_, kROIHeight_));
}

 void VanishingPointSearch::FindVanishingPoint(Mat image, Mat warp_matrix)
 {
     warp_matrix_ = warp_matrix;

     SetImage(image);
     ClearMemory();
     CropToRegionOfInterest();
     ApplyCannyEdge();
     ApplyHoughLines();

     ChangeLinePointsToDriveDirection();
     GatherTrueRangeLeftAndRightLines();
     RejectFalseLeftAndRightLineAngles();







 }

 void VanishingPointSearch::ClearMemory()
 {
     hough_lines_in_drive_direction_.clear();
     left_hough_lines_in_drive_direction_.clear();
     right_hough_lines_in_drive_direction_.clear();
     left_hough_lines_points_and_angle_.clear();
     right_hough_lines_points_and_angle_.clear();

     left_hough_lines_warped_perspektive_.clear();
     right_hough_lines_warped_perspektive_.clear();
 }


void VanishingPointSearch::ApplyCannyEdge()
{
    Canny(current_image_roi_, canny_image_, kLowThreshold_, kHighThreshold_, kKernelSize_);
}

void VanishingPointSearch::ShowCannyEdgeImage()
{
    imshow("Canny Imgage", canny_image_);
    waitKey(0);
}

void VanishingPointSearch::ApplyHoughLines()
{
    HoughLinesP(canny_image_, hough_lines_, kRho_, kTheta_, kMinIntersections, kMinLineLength, kMaxLineGap );
}



void VanishingPointSearch::ChangeLinePointsToDriveDirection()
{
    for( size_t i = 0; i < hough_lines_.size(); i++ )
    {
      int x1 = hough_lines_[i][0];
      int y1 = hough_lines_[i][1];
      int x2 = hough_lines_[i][2];
      int y2 = hough_lines_[i][3];

      if(y2 > y1)
      {
          int tmp;
          tmp = x1;
          x1  = x2;
          x2  = tmp;
          tmp = y1;
          y1  = y2;
          y2 = tmp;
      }

      hough_lines_in_drive_direction_.push_back(HoughLinesInDriveDirection{x1,y1,x2,y2});
    }
}


void VanishingPointSearch::GatherTrueRangeLeftAndRightLines()
{
    for(auto it: hough_lines_in_drive_direction_)
    {

        if(it.x_bottom >= kXMinLeftLine && it.x_bottom <= kXMaxLeftLine)
        {
            left_hough_lines_in_drive_direction_.push_back(it);
        }

        if(it.x_bottom >= kXMinRightLine && it.x_bottom <= kXMaxRightLine)
        {
            right_hough_lines_in_drive_direction_.push_back(it);
        }

    }
}


void VanishingPointSearch::RejectFalseLeftAndRightLineAngles()
{



    for(auto it: left_hough_lines_in_drive_direction_)
    {
        int opposite =  it.y_bottom - it.y_top;
        int adjacent =  it.x_top - it.x_bottom;
        int angle =CalculateAngle4Quadrants(opposite, adjacent);

        if(angle >= kMinLeftLineAngle &&  angle <= kMaxLeftLineAngle)
        {
            left_hough_lines_points_and_angle_.push_back({it.x_bottom,
                                                          it.y_bottom,
                                                          it.x_top,
                                                          it.y_top,
                                                          angle});
        }

    }

    for(auto it: right_hough_lines_in_drive_direction_)
    {
        int opposite =  it.y_bottom - it.y_top;
        int adjacent =  it.x_top - it.x_bottom;
        int angle =CalculateAngle4Quadrants(opposite, adjacent);

        if(angle >= kMinRightLineAngle &&  angle <= kMaxRightLineAngle)
        {
            right_hough_lines_points_and_angle_.push_back({it.x_bottom,
                                                          it.y_bottom,
                                                          it.x_top,
                                                          it.y_top,
                                                          angle});
        }

    }
}


void VanishingPointSearch::WarpPerspektiveOfHoughLines(int _line)
{

    if(_line == LEFT_LINE)
    {
        left_hough_lines_warped_perspektive_.clear();


        for (auto it: left_hough_lines_points_and_angle_)
        {
            int x1 = it.x_bottom + kXROIStart_;
            int y1 = it.y_bottom + kYROIStart_;

            int x2 = it.x_top + kXROIStart_;
            int y2 = it.y_top + kYROIStart_;

            cv::Point2f bottom = Point2f(float(x1), float(y1));
            cv::Point2f top    = Point2f(float(x2), float(y2));

            vector<Point2f> src, dst;
            src.push_back(bottom);
            src.push_back(top);

            cv::perspectiveTransform(src,dst,warp_matrix_.inv());

            x1 = int(dst[0].x);
            y1 = int(dst[0].y);

            x2 = int(dst[1].x);
            y2 = int(dst[1].y);

            left_hough_lines_warped_perspektive_.push_back({x1,y1,x2,y2});

        }
    }

    if(_line == RIGHT_LINE)
    {
       right_hough_lines_warped_perspektive_.clear();

        for (auto it: right_hough_lines_points_and_angle_)
        {
            int x1 = it.x_bottom + kXROIStart_;
            int y1 = it.y_bottom + kYROIStart_;

            int x2 = it.x_top + kXROIStart_;
            int y2 = it.y_top + kYROIStart_;

            cv::Point2f bottom = Point2f(float(x1), float(y1));
            cv::Point2f top    = Point2f(float(x2), float(y2));

            vector<Point2f> src, dst;
            src.push_back(bottom);
            src.push_back(top);

            cv::perspectiveTransform(src,dst,warp_matrix_.inv());

            x1 = int(dst[0].x);
            y1 = int(dst[0].y);

            x2 = int(dst[1].x);
            y2 = int(dst[1].y);

            right_hough_lines_warped_perspektive_.push_back({x1,y1,x2,y2});

        }
    }
}


void VanishingPointSearch::DrawWarpedPerspektiveHoughLines(Mat &rgb, int _line)
{



    WarpPerspektiveOfHoughLines(_line);

    if(_line == LEFT_LINE)
    {
        for (auto it: left_hough_lines_warped_perspektive_)
        {
            int x1 = it.x_bottom;
            int y1 = it.y_bottom;

            int x2 = it.x_top;
            int y2 = it.y_top;

            line( rgb,Point(x1,y1), Point(x2,y2), Scalar(0,0,255), 3, CV_AA);
        }
    }

    if(_line == RIGHT_LINE)
    {
        for (auto it: right_hough_lines_warped_perspektive_)
        {
            int x1 = it.x_bottom;
            int y1 = it.y_bottom;

            int x2 = it.x_top;
            int y2 = it.y_top;

            line( rgb, Point(x1, y1), Point(x2, y2), Scalar(0,255,0), 3, CV_AA);
        }
    }
}


void VanishingPointSearch::DrawHoughLines(Mat &image, int _line)
{

    if(_line == LEFT_LINE)
    {
        for (auto it: left_hough_lines_points_and_angle_)
        {
            int x1 = it.x_bottom + kXROIStart_;
            int y1 = it.y_bottom + kYROIStart_;

            int x2 = it.x_top + kXROIStart_;
            int y2 = it.y_top + kYROIStart_;

            line( image,Point(x1,y1), Point(x2,y2), Scalar(0,0,255), 3, CV_AA);
        }
    }

    if(_line == RIGHT_LINE)
    {
        for (auto it: right_hough_lines_points_and_angle_)
        {
            int x1 = it.x_bottom + kXROIStart_;
            int y1 = it.y_bottom + kYROIStart_;

            int x2 = it.x_top + kXROIStart_;
            int y2 = it.y_top + kYROIStart_;

            line( image, Point(x1, y1), Point(x2, y2), Scalar(0,255,0), 3, CV_AA);
        }
    }

}


void VanishingPointSearch::CoutHoughLines()
{
    cout << "___VanishingPointSearch HoughLines___" << endl;

        for (auto it: left_hough_lines_points_and_angle_)
        {
            int x1 = it.x_bottom + kXROIStart_;
            int y1 = it.y_bottom + kYROIStart_;

            int x2 = it.x_top + kXROIStart_;
            int y2 = it.y_top + kYROIStart_;

            int angle = it.angle;

            cout << "LEFT: \t(" << x1 << ","<<y1 << ") \t(" << x2 << "," << y2 << ")  \tangle: " << angle<< "°" << endl;

        }

        for (auto it: right_hough_lines_points_and_angle_)
        {
            int x1 = it.x_bottom + kXROIStart_;
            int y1 = it.y_bottom + kYROIStart_;

            int x2 = it.x_top + kXROIStart_;
            int y2 = it.y_top + kYROIStart_;

            int angle = it.angle;

            cout << "RIGHT: \t(" << x1 << ","<< y1 << ") \t(" << x2 << "," << y2 << ")  \tangle: " << angle <<"°" << endl;
        }
        cout << "#######################################" << endl;
}


/*
#define pdd pair<double, double>

pdd lineLineIntersection(pdd A, pdd B, pdd C, pdd D)
{
    // Line AB represented as a1x + b1y = c1
    double a1 = B.second - A.second;
    double b1 = A.first - B.first;
    double c1 = a1*(A.first) + b1*(A.second);

    // Line CD represented as a2x + b2y = c2
    double a2 = D.second - C.second;
    double b2 = C.first - D.first;
    double c2 = a2*(C.first)+ b2*(C.second);

    double determinant = a1*b2 - a2*b1;

    if (determinant == 0)
    {
        // The lines are parallel. This is simplified
        // by returning a pair of FLT_MAX
        return make_pair(FLT_MAX, FLT_MAX);
    }
    else
    {
        double x = (b2*c1 - b1*c2)/determinant;
        double y = (a1*c2 - a2*c1)/determinant;
        return make_pair(x, y);
    }
}



void VanishingPointSearch::ComputeIntersections()
{






    if(found_lines_.size()>= 2)
    {

        for(int i=0; i<found_lines_.size()-1;i++)
        {
             pdd A = make_pair(found_lines_[i][0], found_lines_[i][1]);
             pdd B = make_pair(found_lines_[i][2], found_lines_[i][3]);
             pdd C = make_pair(found_lines_[i+1][0], found_lines_[i+1][1]);
             pdd D = make_pair(found_lines_[i+1][2], found_lines_[i+1][3]);

             pdd intersection = lineLineIntersection(A, B, C, D);


             if (intersection.first == FLT_MAX &&
                 intersection.second==FLT_MAX)
             {
                 cout << i << " The given lines AB and CD are parallel.\n";
             }

             else
             {
                 // NOTE: Further check can be applied in case
                 // of line segments. Here, we have considered AB
                 // and CD as lines
                 cout << "The intersection of the given lines AB "
                         "and CD is: ";
                 cout << i << "(" << intersection.first << ", " << intersection.second<< ")" << endl;
             }
        }
    }
}
*/
