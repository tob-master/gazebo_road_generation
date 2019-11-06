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

     ComputeLeftAndRightHoughLineIntersections();

     FilterVanishingPoint();
     //ApplyDBScan();
 }


 void VanishingPointSearch::FilterVanishingPoint()
 {
     if(intersections_.size() > 0)
     {
         float x_sum = 0;
         float y_sum = 0;

         for(auto it: intersections_)
         {
             x_sum += it.x;
             y_sum += abs(it.y);

             cout << it.x << " " << it.y << endl;
         }

         float x_mean = x_sum / intersections_.size();
         float y_mean = y_sum / intersections_.size();

         int valid_point_counter = 0;
         int std_add = 0;
         float standard_deviation = 0;
         do{
         x_sum = 0;
         y_sum = 0;
         valid_point_counter = 0;

         for (auto it: intersections_)
         {
              standard_deviation= sqrt(pow(it.x-x_mean,2) + pow(abs(it.y)-y_mean,2));

              cout << standard_deviation << endl;
             if(standard_deviation < kMaxStandardDeviationForValidVanishingPoint_+std_add)
             {
                 x_sum += it.x;
                 y_sum += abs(it.y);
                 ++valid_point_counter;
             }

         }
         ++std_add;
         }while(valid_point_counter == 0);

         int x = (x_sum / valid_point_counter) + kXROIStart_;
         int y = kYROIStart_ - (y_sum / valid_point_counter);

         vanishing_point_ = Point(x,y);
         cout <<intersections_.size() <<" " << standard_deviation << " " <<  vanishing_point_ << " " << valid_point_counter << " " << x_mean << " " << y_mean << " " << x_sum << " " << y_sum << endl;
         has_found_vanishing_point_ = true;
    }
     else {
         has_found_vanishing_point_ = false;
     }


 }

 void VanishingPointSearch::DrawVanishingPoint(Mat &rgb)
 {
        if(has_found_vanishing_point_)
        {
            circle(rgb, vanishing_point_, 7, Scalar(255, 0, 255));
            circle(rgb, CarMidPoint_, 7, Scalar(255, 0, 255));
            line( rgb,CarMidPoint_, vanishing_point_, Scalar(255,0,255), 3, CV_AA);

            int opposite =  CarMidPoint_.y - vanishing_point_.y;
            int adjacent =  vanishing_point_.x - CarMidPoint_.x;

            int angle = CalculateAngle4Quadrants(opposite, adjacent);

            string text = to_string(angle) + " deg";

            putText(rgb, text, Point(CarMidPoint_.x + 30,CarMidPoint_.y),
                FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);

        }
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

     intersections_.clear();
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


void VanishingPointSearch::ComputeLeftAndRightHoughLineIntersections()
{



    for (int i = 0; i < left_hough_lines_points_and_angle_.size(); ++i)
    {
         double left_x_bottom = left_hough_lines_points_and_angle_[i].x_bottom;
         double left_y_bottom = left_hough_lines_points_and_angle_[i].y_bottom;
         double left_x_top = left_hough_lines_points_and_angle_[i].x_top;
         double left_y_top = left_hough_lines_points_and_angle_[i].y_top;

         pair<double, double> A = make_pair(left_x_bottom,left_y_bottom);
         pair<double, double> B = make_pair(left_x_top,left_y_top);

        for (int j = 0; j < right_hough_lines_points_and_angle_.size(); ++j)
        {
            double right_x_bottom = right_hough_lines_points_and_angle_[j].x_bottom;
            double right_y_bottom = right_hough_lines_points_and_angle_[j].y_bottom;
            double right_x_top = right_hough_lines_points_and_angle_[j].x_top;
            double right_y_top = right_hough_lines_points_and_angle_[j].y_top;

            pair<double, double> C = make_pair(right_x_bottom,right_y_bottom);
            pair<double, double> D = make_pair(right_x_top,right_y_top);

            pair<double, double> intersection = ComputeLineIntersection(A, B, C, D);

            if (intersection.first == FLT_MAX &&
                intersection.second==FLT_MAX)
            {
                //cout << i << " The given lines AB and CD are parallel.\n";
            }

            else
            {
                //cout << "The intersection of the given lines AB "
                //        "and CD is: ";
                //cout << i << "(" << intersection.first << ", " << intersection.second<< ")" << endl;


                intersections_.push_back(Point((int)intersection.first, (int)intersection.second));

            }

        }
    }
}


void VanishingPointSearch::ApplyDBScan()
{
    vector<DBScanPoint> intersections_dbscan_;

    for (auto it : intersections_) {


    intersections_dbscan_.push_back(DBScanPoint{float(it.x),
                                                float(it.y),
                                                0.0,
                                                UNCLASSIFIED});
    }
    DBSCAN ds(MINIMUM_POINTS, EPSILON, intersections_dbscan_);

    ds.run();



    vector<DBScanPoint> db_points = ds.getDBScanPoint();
    int num_points = ds.getTotalPointSize();


    int i = 0;
    printf("Number of points: %u\n"
        " x     y     z     cluster_id\n"
        "-----------------------------\n"
        , num_points);
    while (i < num_points)
    {
          printf("%5.2lf %5.2lf %5.2lf: %d\n",
                 db_points[i].x,
                 db_points[i].y, db_points[i].z,
                 db_points[i].clusterID);
          ++i;
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




pair<double, double> VanishingPointSearch::ComputeLineIntersection(pair<double, double> A, pair<double, double> B, pair<double, double> C, pair<double, double> D)
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





void VanishingPointSearch::DrawLineIntersections(Mat &rgb)
{
    for(auto it: intersections_)
    {
        int x = it.x + kXROIStart_;
        int y = it.y + kYROIStart_;

        circle(rgb, Point(x,y), 7, Scalar(255, 0, 255));
    }
}
