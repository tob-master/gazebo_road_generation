#include "vanishing_point_search.h"

VanishingPointSearch::VanishingPointSearch(Mat birdseye_transformation_matrix, VanishingPointSearchInitializationParameters init):
frontalview_to_birdseye_transformation_matrix_(birdseye_transformation_matrix.inv()),
kCannyLowThreshold_(init.canny_low_threshold),
kCannyHighThreshold_(init.canny_high_threshold),
kCannyKernelSize_(init.canny_kernel_size),
kHoghLinesRho_(init.hough_lines_rho),
kHoughLinesTheta_(init.hough_lines_theta),
kHoughLinesMinintersections_(init.hough_lines_min_intersections),
kHoughLinesMinLineLength_(init.hough_lines_min_line_length),
kHoughLinesMaxLineGap_(init.hough_lines_min_line_gap),
kXROIStart_(init.x_roi_start),
kYROIStart_(init.y_roi_start),
kROIWidth_(init.roi_width),
kROIHeight_(init.roi_height),
kMinLeftLineAngle_(init.min_left_line_angle),
kMaxLeftLineAngle_(init.max_left_line_angle),
kMinRightLineAngle_(init.min_right_line_angle),
kMaxRightLineAngle_(init.max_right_line_angle),
kXMinLeftLine_(init.x_min_left_line),
kXMaxLeftLine_(init.x_max_left_line),
kXMinRightLine_(init.x_min_right_line),
kXMaxRightLine_(init.x_max_right_line),
kCarMidPoint_(Point(init.car_mid_position_x,init.car_mid_position_y)),
kMaxStandardDeviationForValidVanishingPoint_(init.max_standard_deviation_for_valid_vanishing_point)
{

    WarpCarMidPointToBirdsview();
}

void VanishingPointSearch::WarpCarMidPointToBirdsview()
{

    int x = kCarMidPoint_.x;
    int y = kCarMidPoint_.y;

    TransformPoint(x, y, frontalview_to_birdseye_transformation_matrix_);

    warped_car_mid_point_ = Point(x,y);
}

void VanishingPointSearch::SetImage(Mat image)
{
    current_image_ = image;
}

void VanishingPointSearch::CropImageToRegionOfInterest()
{
    current_image_roi_ =  current_image_(cv::Rect(kXROIStart_, kYROIStart_, kROIWidth_, kROIHeight_));
}



void VanishingPointSearch::CheckFoundLeftAndRightHoughLines()
{
    left_hough_lines_count_ = left_hough_lines_points_and_angle_.size();
    right_hough_lines_count_ = right_hough_lines_points_and_angle_.size();

    if(left_hough_lines_count_ > 0) has_found_left_hough_line_ = true;
    else has_found_left_hough_line_ = false;

    if(right_hough_lines_count_ > 0) has_found_right_hough_line_ = true;
    else has_found_right_hough_line_ = false;

}

void VanishingPointSearch::CheckFoundIntersections()
{
    intersections_count_ = intersecting_lines_.size();

    if(intersections_count_ > 0) has_found_intersections_ = true;
    else has_found_intersections_ = false;
}

void VanishingPointSearch::AddRegionOfInterestOffsetToHoughLinePoints()
{
    for( size_t i = 0; i < hough_lines_.size(); i++ )
    {
      hough_lines_[i][0] = hough_lines_[i][0] + kXROIStart_;
      hough_lines_[i][1] = hough_lines_[i][1] + kYROIStart_;
      hough_lines_[i][2] = hough_lines_[i][2] + kXROIStart_;
      hough_lines_[i][3] = hough_lines_[i][3] + kYROIStart_;
    }
}


void  VanishingPointSearch::ComputeCarMidPointToVanishingPointAngle()
{
    int opposite =  kCarMidPoint_.y - vanishing_point_.y;
    int adjacent =  vanishing_point_.x - kCarMidPoint_.x;

    car_mid_point_to_vanishing_point_angle_ = CalculateAngle4Quadrants(opposite, adjacent);
}


VanishingPointSearchReturnInfo VanishingPointSearch::GetReturnInfo()
{


    return VanishingPointSearchReturnInfo{has_found_left_hough_line_,
                                          left_hough_lines_count_,
                                          has_found_right_hough_line_,
                                          right_hough_lines_count_,
                                          has_found_intersections_,
                                          intersections_count_,
                                          has_found_vanishing_point_,
                                          vanishing_point_,
                                          car_mid_point_to_vanishing_point_angle_};
}

 VanishingPointSearchReturnInfo VanishingPointSearch::FindVanishingPoint(Mat image)
 {

     SetImage(image);
     ClearMemory();
     CropImageToRegionOfInterest();
     ApplyCannyEdge();
     ApplyHoughLines();

     AddRegionOfInterestOffsetToHoughLinePoints();

     ChangeLinePointsToDriveDirection();
     GatherTrueRangeLeftAndRightLines();
     RejectFalseLeftAndRightLineAngles();

     CheckFoundLeftAndRightHoughLines();



     if(has_found_left_hough_line_ && has_found_right_hough_line_)
     {
         ComputeLeftAndRightHoughLineIntersections();

         CheckFoundIntersections();

         if(has_found_intersections_)
         {
             FilterVanishingPoint();
             ComputeCarMidPointToVanishingPointAngle();
             has_found_vanishing_point_ = true;
            //ApplyDBScan();
         }
     }

     //TransformHoughLinesToBirdseye();
     SetLineFollowerStartParameters();

    return GetReturnInfo();
 }


 void VanishingPointSearch::SetLineFollowerStartParameters()
 {
    if(has_found_intersections_)
    {
        int left_x_bottom_mean=0;
        int left_y_bottom_mean=0;
        int left_x_top_mean=0;
        int left_y_top_mean=0;
        int right_x_bottom_mean=0;
        int right_y_bottom_mean=0;
        int right_x_top_mean=0;
        int right_y_top_mean=0;

        for(auto it : vanishing_point_intersections_)
        {
            left_x_bottom_mean += it.left_x_bottom;
            left_y_bottom_mean += it.left_y_bottom;
            left_x_top_mean += it.left_x_top;
            left_y_top_mean += it.left_y_top;
            right_x_bottom_mean += it.right_x_bottom;
            right_y_bottom_mean += it.right_y_bottom;
            right_x_top_mean += it.right_x_top;
            right_y_top_mean += it.right_y_top;
        }

        left_x_bottom_mean /= vanishing_point_intersections_.size();
        left_y_bottom_mean /= vanishing_point_intersections_.size();
        left_x_top_mean /= vanishing_point_intersections_.size();
        left_y_top_mean /= vanishing_point_intersections_.size();
        right_x_bottom_mean /= vanishing_point_intersections_.size();
        right_y_bottom_mean /= vanishing_point_intersections_.size();
        right_x_top_mean /= vanishing_point_intersections_.size();
        right_y_top_mean /= vanishing_point_intersections_.size();


        TransformPoint(left_x_bottom_mean, left_y_bottom_mean, frontalview_to_birdseye_transformation_matrix_);
        TransformPoint(left_x_top_mean, left_y_top_mean, frontalview_to_birdseye_transformation_matrix_);
        TransformPoint(right_x_bottom_mean, right_y_bottom_mean, frontalview_to_birdseye_transformation_matrix_);
        TransformPoint(right_x_top_mean, right_y_top_mean, frontalview_to_birdseye_transformation_matrix_);


        left_hough_lines_warped_perspektive_.push_back({left_x_bottom_mean,left_y_bottom_mean,
                                                        left_x_top_mean,left_y_top_mean});

        right_hough_lines_warped_perspektive_.push_back({right_x_bottom_mean,right_y_bottom_mean,
                                                         right_x_top_mean,right_y_top_mean});

        int left_adjacent  = left_x_top_mean  - left_x_bottom_mean;
        int right_adjacent = right_x_top_mean - right_x_bottom_mean;

        int left_opposite =  left_y_bottom_mean - left_y_top_mean;
        int right_opposite = right_y_bottom_mean - right_y_top_mean;

        float left_angle = CalculateAngle4Quadrants(left_opposite, left_adjacent);
        float right_angle = CalculateAngle4Quadrants(right_opposite, right_adjacent);



        line_follower_start_parameters_ = StartParameters{left_x_top_mean,
                                                                  left_y_top_mean,
                                                                  left_angle,
                                                                  true,
                                                                  right_x_top_mean,
                                                                  right_y_top_mean,
                                                                  right_angle,
                                                                  true};

    }
    else
    {
        if(has_found_left_hough_line_)
        {
          //cout << " ";// left_hough_lines_warped_perspektive_

          int x_bottom_mean=0;
          int y_bottom_mean=0;
          int x_top_mean=0;
          int y_top_mean=0;


          for(auto it : left_hough_lines_points_and_angle_)
          {
              x_bottom_mean += it.x_bottom;
              y_bottom_mean += it.y_bottom;
              x_top_mean += it.x_top;
              y_top_mean += it.y_top;
          }

          x_bottom_mean /= left_hough_lines_points_and_angle_.size();
          y_bottom_mean /= left_hough_lines_points_and_angle_.size();
          x_top_mean /= left_hough_lines_points_and_angle_.size();
          y_top_mean /= left_hough_lines_points_and_angle_.size();


          TransformPoint(x_bottom_mean, y_bottom_mean, frontalview_to_birdseye_transformation_matrix_);
          TransformPoint(x_top_mean, y_top_mean, frontalview_to_birdseye_transformation_matrix_);


          left_hough_lines_warped_perspektive_.push_back({x_bottom_mean,y_bottom_mean,
                                                          x_top_mean,y_top_mean});

          int adjacent  = x_top_mean  - x_bottom_mean;
          int opposite =  y_bottom_mean - y_top_mean;

          float angle = CalculateAngle4Quadrants(opposite, adjacent);

          line_follower_start_parameters_ = StartParameters{x_top_mean,
                                                                    y_top_mean,
                                                                    angle,
                                                                    true,
                                                                    0,
                                                                    0,
                                                                    0,
                                                                    false};


/*
          for (auto it: left_hough_lines_points_and_angle_)
          {

              int x1 = it.x_bottom;
              int y1 = it.y_bottom;

              int x2 = it.x_top;
              int y2 = it.y_top;



              TransformPoint(x1, y1, frontalview_to_birdseye_transformation_matrix_);
              TransformPoint(x2, y2, frontalview_to_birdseye_transformation_matrix_);


              left_hough_lines_warped_perspektive_.push_back({x1,y1,x2,y2});

          }
*/


        }

        if(has_found_right_hough_line_)
        {
            int x_bottom_mean=0;
            int y_bottom_mean=0;
            int x_top_mean=0;
            int y_top_mean=0;


            for(auto it : right_hough_lines_points_and_angle_)
            {
                x_bottom_mean += it.x_bottom;
                y_bottom_mean += it.y_bottom;
                x_top_mean += it.x_top;
                y_top_mean += it.y_top;
            }

            x_bottom_mean /= right_hough_lines_points_and_angle_.size();
            y_bottom_mean /= right_hough_lines_points_and_angle_.size();
            x_top_mean /= right_hough_lines_points_and_angle_.size();
            y_top_mean /= right_hough_lines_points_and_angle_.size();



            TransformPoint(x_bottom_mean, y_bottom_mean, frontalview_to_birdseye_transformation_matrix_);
            TransformPoint(x_top_mean, y_top_mean, frontalview_to_birdseye_transformation_matrix_);

            right_hough_lines_warped_perspektive_.push_back({x_bottom_mean,y_bottom_mean,
                                                            x_top_mean,y_top_mean});




            int adjacent  = x_top_mean  - x_bottom_mean;
            int opposite =  y_bottom_mean - y_top_mean;

            float angle = CalculateAngle4Quadrants(opposite, adjacent);

            line_follower_start_parameters_ = StartParameters{0,
                                                                      0,
                                                                      0,
                                                                      false,
                                                                      x_top_mean,
                                                                      y_top_mean,
                                                                      angle,
                                                                      true};

        }
    }
 }

StartParameters VanishingPointSearch::GetLineFollowerStartParameters()
{
    return line_follower_start_parameters_;
}

void VanishingPointSearch::DrawWarpedVanishingPointDirection(Mat &rgb)
{

    int length = 800;

    float angle = car_mid_point_to_vanishing_point_angle_ * PI/180;

    int x_offset = cos(angle) * length;
    int y_offset = sin(angle) * length;

    Point direction_point = Point(warped_car_mid_point_.x + x_offset, warped_car_mid_point_.y - y_offset);

    line( rgb,warped_car_mid_point_, direction_point, Scalar(255,0,255), 3, CV_AA);

}



 void VanishingPointSearch::FilterVanishingPoint()
 {

         float x_sum = 0;
         float y_sum = 0;

         for(auto it: intersecting_lines_)
         {
             x_sum += it.intersection_x;
             y_sum += it.intersection_y;
         }

         float x_mean = x_sum / intersecting_lines_.size();
         float y_mean = y_sum / intersecting_lines_.size();

         int valid_point_counter = 0;
         int std_add = 0;
         float standard_deviation = 0;

        /*
         float mean_std =  0;
         for (auto it: intersecting_lines_)
         {
              mean_std += sqrt(pow(it.x-x_mean,2) + pow(abs(it.y)-y_mean,2));
         }
         mean_std /= intersecting_lines_.size();
        */

         do{
             x_sum = 0;
             y_sum = 0;
             valid_point_counter = 0;

             for (auto it: intersecting_lines_)
             {
                  standard_deviation= sqrt(pow(it.intersection_x-x_mean,2) + pow(abs(it.intersection_y)-y_mean,2));

                 if(standard_deviation < kMaxStandardDeviationForValidVanishingPoint_+std_add)
                 {
                     x_sum += it.intersection_x;
                     y_sum += abs(it.intersection_y);
                     vanishing_point_intersections_.push_back(it);
                     ++valid_point_counter;
                 }
             }
             ++std_add;
         }while(valid_point_counter == 0);

         int x = (x_sum / valid_point_counter);
         int y = (y_sum / valid_point_counter);

         vanishing_point_ = Point(x,y);
 }

 void VanishingPointSearch::DrawVanishingPoint(Mat &rgb)
 {
        if(has_found_vanishing_point_)
        {
            circle(rgb, vanishing_point_, 7, Scalar(255, 0, 255));
            circle(rgb, kCarMidPoint_, 7, Scalar(255, 0, 255));
            line( rgb,kCarMidPoint_, vanishing_point_, Scalar(255,0,255), 3, CV_AA);

            int opposite =  kCarMidPoint_.y - vanishing_point_.y;
            int adjacent =  vanishing_point_.x - kCarMidPoint_.x;

            int angle = CalculateAngle4Quadrants(opposite, adjacent);

            string text = to_string(angle) + " deg";

            putText(rgb, text, Point(kCarMidPoint_.x + 30,kCarMidPoint_.y),
                FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);

        }
 }

 void VanishingPointSearch::ClearMemory()
 {

    car_mid_point_to_vanishing_point_angle_ = FLT_MAX;
    vanishing_point_ = Point(-1,-1);

     has_found_left_hough_line_ = false;
     has_found_right_hough_line_ = false;
     has_found_intersections_ = false;
     has_found_vanishing_point_ = false;

     right_hough_lines_count_ = 0;
     left_hough_lines_count_ = 0;
     intersections_count_ = 0;

     hough_lines_in_drive_direction_.clear();
     left_hough_lines_in_drive_direction_.clear();
     right_hough_lines_in_drive_direction_.clear();
     left_hough_lines_points_and_angle_.clear();
     right_hough_lines_points_and_angle_.clear();

     left_hough_lines_warped_perspektive_.clear();
     right_hough_lines_warped_perspektive_.clear();

     intersecting_lines_.clear();
     vanishing_point_intersections_.clear();

 }


void VanishingPointSearch::ApplyCannyEdge()
{
    Canny(current_image_roi_, canny_image_, kCannyLowThreshold_, kCannyHighThreshold_, kCannyKernelSize_);
}

void VanishingPointSearch::ShowCannyEdgeImage()
{
    imshow("Canny Imgage", canny_image_);
    waitKey(0);
}

void VanishingPointSearch::ApplyHoughLines()
{
    HoughLinesP(canny_image_, hough_lines_, kHoghLinesRho_, kHoughLinesTheta_, kHoughLinesMinintersections_, kHoughLinesMinLineLength_, kHoughLinesMaxLineGap_ );
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

        if(it.x_bottom >= kXMinLeftLine_ && it.x_bottom <= kXMaxLeftLine_)
        {
            left_hough_lines_in_drive_direction_.push_back(it);
        }

        if(it.x_bottom >= kXMinRightLine_ && it.x_bottom <= kXMaxRightLine_)
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

        if(angle >= kMinLeftLineAngle_ &&  angle <= kMaxLeftLineAngle_)
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

        if(angle >= kMinRightLineAngle_ &&  angle <= kMaxRightLineAngle_)
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
                //TODO: What to do if parallel?
                //cout << i << " The given lines AB and CD are parallel.\n";
            }

            else
            {
                intersecting_lines_.push_back(Intersections{(int)intersection.first,
                                              (int)intersection.second,
                                              (int)left_x_bottom,
                                              (int)left_y_bottom,
                                              (int)left_x_top,
                                              (int)left_y_top,
                                              (int)right_x_bottom,
                                              (int)right_y_bottom,
                                              (int)right_x_top,
                                              (int)right_y_top});
            }

        }
    }
}

void VanishingPointSearch::TransformHoughLinesToBirdseye()
{
    if(has_found_left_hough_line_)
    {



        for (auto it: left_hough_lines_points_and_angle_)
        {

            int x1 = it.x_bottom;
            int y1 = it.y_bottom;

            int x2 = it.x_top;
            int y2 = it.y_top;



            TransformPoint(x1, y1, frontalview_to_birdseye_transformation_matrix_);
            TransformPoint(x2, y2, frontalview_to_birdseye_transformation_matrix_);


            left_hough_lines_warped_perspektive_.push_back({x1,y1,x2,y2});

        }
    }

    if(has_found_right_hough_line_)
    {


        for (auto it: right_hough_lines_points_and_angle_)
        {

            int x1 = it.x_bottom;
            int y1 = it.y_bottom;

            int x2 = it.x_top;
            int y2 = it.y_top;

            TransformPoint(x1, y1, frontalview_to_birdseye_transformation_matrix_);
            TransformPoint(x2, y2, frontalview_to_birdseye_transformation_matrix_);

            right_hough_lines_warped_perspektive_.push_back({x1,y1,x2,y2});

        }
    }
}



void VanishingPointSearch::DrawWarpedPerspektiveHoughLines(Mat &rgb, int _line)
{


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

            int x1 = it.x_bottom;
            int y1 = it.y_bottom;

            int x2 = it.x_top;
            int y2 = it.y_top;

            line( image,Point(x1,y1), Point(x2,y2), Scalar(0,0,255), 3, CV_AA);
        }
    }

    if(_line == RIGHT_LINE)
    {
        for (auto it: right_hough_lines_points_and_angle_)
        {

            int x1 = it.x_bottom;
            int y1 = it.y_bottom;

            int x2 = it.x_top;
            int y2 = it.y_top;

            line( image, Point(x1, y1), Point(x2, y2), Scalar(0,255,0), 3, CV_AA);
        }
    }

}


void VanishingPointSearch::CoutHoughLines()
{
    cout << "___VanishingPointSearch HoughLines___" << endl;

        for (auto it: left_hough_lines_points_and_angle_)
        {

            int x1 = it.x_bottom;
            int y1 = it.y_bottom;

            int x2 = it.x_top;
            int y2 = it.y_top;

            int angle = it.angle;

            cout << "LEFT: \t(" << x1 << ","<<y1 << ") \t(" << x2 << "," << y2 << ")  \tangle: " << angle<< "°" << endl;

        }

        for (auto it: right_hough_lines_points_and_angle_)
        {

            int x1 = it.x_bottom;
            int y1 = it.y_bottom;

            int x2 = it.x_top;
            int y2 = it.y_top;

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
    for(auto it: intersecting_lines_)
    {

        int x = it.intersection_x;
        int y = it.intersection_y;

        circle(rgb, Point(x,y), 7, Scalar(255, 0, 255));
    }
}
/*
void VanishingPointSearch::ApplyDBScan()
{
    vector<DBScanPoint> intersections_dbscan_;

    for (auto it : intersecting_lines_) {


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
*/
