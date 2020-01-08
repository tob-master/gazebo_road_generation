#include "on_road_search.h"

OnRoadSearch::OnRoadSearch()
{
    image_template_10 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_10_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
    image_template_20 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_20_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
    image_template_30 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_30_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
    image_template_40 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_40_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
    image_template_50 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_50_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
    image_template_60 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_60_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
    image_template_70 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_70_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
    image_template_80 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_80_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
    image_template_90 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_90_ground.png", CV_LOAD_IMAGE_GRAYSCALE);

    Size resize_size(kVelocitySignTemplateWidth_,kVelocitySignTemplateHeight_);

    resize(image_template_10,image_template_10,resize_size);
    resize(image_template_20,image_template_20,resize_size);
    resize(image_template_30,image_template_30,resize_size);
    resize(image_template_40,image_template_40,resize_size);
    resize(image_template_50,image_template_50,resize_size);
    resize(image_template_60,image_template_60,resize_size);
    resize(image_template_70,image_template_70,resize_size);
    resize(image_template_80,image_template_80,resize_size);
    resize(image_template_90,image_template_90,resize_size);

}

float OnRoadSearch::GetOrthogonalAngle(float angle, int SEARCH_LINE_CODE)
{
    int  angle_ = 0;

    if(SEARCH_LINE_CODE == LEFT_TO_MID || SEARCH_LINE_CODE == LEFT_TO_RIGHT || SEARCH_LINE_CODE == MID_TO_RIGHT) angle_ = (angle - 90);

    if(SEARCH_LINE_CODE == RIGHT_TO_MID || SEARCH_LINE_CODE == RIGHT_TO_LEFT || SEARCH_LINE_CODE == MID_TO_LEFT) angle_ = (angle + 90);

    if(angle_ > 359) angle_ = angle_ % 360;

    if(angle_ < 0) angle_ =  360 - abs(angle_);

    return float(angle_);
}


double OnRoadSearch::MatchTemplateSQDIFF(Mat image, Mat template_image)
{

    Mat template_match;

    double min_val; double max_val; Point min_loc; Point max_loc;

    matchTemplate( image, template_image, template_match, CV_TM_SQDIFF );

    minMaxLoc( template_match, &min_val, &max_val, &min_loc, &max_loc, Mat() );

    return min_val;

    //rectangle( image_rgb_bird_, maxLoc10, Point( maxLoc10.x + 20, maxLoc10.y + 20 ), Scalar(255,255,0), 2, 8, 0 );

}

double OnRoadSearch::MatchTemplateCCOEFF(Mat image, Mat template_image)
{

    Mat template_match;

    double min_val; double max_val; Point min_loc; Point max_loc;

    matchTemplate( image, template_image, template_match, CV_TM_CCOEFF );

    minMaxLoc( template_match, &min_val, &max_val, &min_loc, &max_loc, Mat() );

    return min_val;

}

Point OnRoadSearch::GetPolarCoordinate(int x, int y, float angle, int radius)
{
    x = x + radius * cos(angle) + 0.5;
    y = y - radius * sin(angle) + 0.5;

    return Point(x,y);
}

vector<pair<string,int>> OnRoadSearch::GatherBlackAndWhiteRoadSegments(Mat scanned_line_mat)
{
    vector<pair<string,int>> black_white_road_segments;

     vector<int> line_segments;
     line_segments.push_back(-1);

     for (int i = 0; i < scanned_line_mat.rows; ++i)
      {
         for (int j = 0; j < scanned_line_mat.cols; ++j)
         {
             int px = (int)scanned_line_mat.at<uchar>(i,j);

             if(line_segments[line_segments.size()-1] == px)
             {
                 line_segments.push_back(px);
             }
             else
             {
                 if(px == 255 && line_segments[line_segments.size()-1] != -1)
                 {
                     black_white_road_segments.push_back(make_pair("black",line_segments.size()));
                 }
                 else if(px == 0 && line_segments[line_segments.size()-1] != -1)
                 {
                     black_white_road_segments.push_back(make_pair("white",line_segments.size()));
                 }
                 line_segments.clear();
                 line_segments.push_back(px);
             }
         }
    }

     if(line_segments[line_segments.size()-1] == 255)
     {
         black_white_road_segments.push_back(make_pair("white",line_segments.size()));
     }
     else{
         black_white_road_segments.push_back(make_pair("black",line_segments.size()));
     }

     return black_white_road_segments;
}

void OnRoadSearch::SetStartParameters(StartParameters start_parameters)
{
    start_left_x_ = start_parameters.left_x;
    start_left_y_ = start_parameters.left_y;
    start_right_x_ = start_parameters.right_x;
    start_right_y_ = start_parameters.right_y;
    start_angle_left_= start_parameters.left_angle;
    start_angle_right_ = start_parameters.right_angle;
}


bool OnRoadSearch::SearchGoalLine()
{

    //int start_left_x_ = vanishing_point_start_parameters.left_x;
    //int start_left_y_ = vanishing_point_start_parameters.left_y;
    //float start_angle_left_ = vanishing_point_start_parameters.left_angle;
    float orthogonal_angle = GetOrthogonalAngle(start_angle_left_,LEFT_TO_RIGHT);

    for(float current_angle=(orthogonal_angle-kGoalLineFieldOfView_); current_angle<orthogonal_angle+kGoalLineFieldOfView_; current_angle+=1)
    {
        float angle =  (current_angle*PI)/180;
        Point line_iterator_end = GetPolarCoordinate(start_left_x_, start_left_y_, angle, 120);

        LineIterator line_iterator(image_, Point(start_left_x_,start_left_y_),line_iterator_end , 8);

        //line( image_rgb_bird_, Point(start_left_x_,start_left_y_),line_iterator_end, Scalar(255,0,0), 1, CV_AA);
        vector<uchar> scanned_line;

        for(int i=0; i<line_iterator.count; i++,line_iterator++)  scanned_line.push_back(**line_iterator);

        Mat scanned_line_mat(scanned_line);
        threshold(scanned_line_mat, scanned_line_mat, GoalLineIntesitiyThreshold_, 255, CV_THRESH_BINARY);

        vector<pair<string,int>> black_white_road_segments = GatherBlackAndWhiteRoadSegments(scanned_line_mat);

         int goal_line_counter = 0;

         for(auto it : black_white_road_segments)
         {
             if(it.second >= kMinGoalSegmentWidth_ && it.second <= kMaxGoalSegmentWidth_) goal_line_counter++;
         }

         if(goal_line_counter >= kMinGoalSegmentsToFind_)
         {
             //circle(image_rgb_bird_,Point(start_left_x_,start_left_y_), 5, Scalar(0,255,0),CV_FILLED);
            found_goal_line_ = true;
            return true;
         }
      }
    found_goal_line_ = false;
    return false;
}


void OnRoadSearch::SearchOnRoad(vector<LineValidationTable> left_table, vector<LineValidationTable> right_table)
{

    vector<PointInDirection> markings;
    vector<PointInDirection> boxes;

    SearchGoalLine();
    SearchCrossWalk();
    SearchCrossRoad();
    SearchRoadObject(left_table, markings, boxes, LEFT_TO_RIGHT, LEFT_IN_RIGHT_LANE);
    SearchRoadObject(left_table, markings, boxes, LEFT_TO_RIGHT, LEFT_IN_LEFT_LANE);
    SearchRoadObject(right_table, markings, boxes, RIGHT_TO_LEFT, RIGHT_IN_LEFT_LANE);
    SearchRoadObject(right_table, markings, boxes, RIGHT_TO_LEFT, RIGHT_IN_RIGHT_LANE);



    if(markings.size()>1 && boxes.size()==0 && !found_cross_walk_ && !found_crossing_ && !found_goal_line_)TemplateMatchMarking(markings);
    else if(boxes.size()>1 && !found_cross_walk_ && !found_crossing_ && !found_goal_line_) cout << "box!" << endl;
    else if(found_crossing_) cout << "CROSSING!" << endl;
    else if(found_cross_walk_) cout << "CROSSWALK!" << endl;
    else if(found_goal_line_) cout << "GOALLINE!" << endl;

   //cout << (boxes.size()>1) << " " << (markings.size()>1) << " " <<  found_goal_line_ << " " << found_cross_walk_ << " " << found_crossing_ << endl;

    //if(boxes.size()>1) cout << "box!" << endl;
    //else if(markings.size()>1) cout << "mark!" << endl;

}

void OnRoadSearch::TemplateMatchMarking(vector<PointInDirection> markings)
{
    int current_x = markings[0].x;
    int current_y = markings[0].y;
    float current_direction = markings[0].angle;

    int start_roi_x = ((current_x-kTemplateRoiSizeX_) < 0)            ? 0               : (current_x-kTemplateRoiSizeX_);
    int start_roi_y = ((current_y-kTemplateRoiSizeY_) < 0)            ? 0               : (current_y-kTemplateRoiSizeY_);
    int end_roi_x   = ((current_x+kTemplateRoiSizeX_) >= image_.cols) ? (image_.cols-1) : (current_x+kTemplateRoiSizeX_);
    int end_roi_y   = ((current_y+kTemplateRoiSizeY_) >= image_.rows) ? (image_.rows-1) : (current_y+kTemplateRoiSizeY_);

     Mat roi_image = image_(Rect(Point(start_roi_x,start_roi_y),Point(end_roi_x,end_roi_y)));

     cv::Mat rotation_mat = cv::getRotationMatrix2D(Point((end_roi_x-start_roi_x)/2,(end_roi_y-start_roi_y)/2), 90-current_direction, 1.0);

     cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), roi_image.size(), 90-current_direction).boundingRect2f();

     Mat warped_roi_image;

     cv::warpAffine(roi_image, warped_roi_image, rotation_mat, bbox.size());

     //imshow("imgg",warped_roi_image);

     std::vector<double> template_scores;

     template_scores.push_back(MatchTemplateCCOEFF(warped_roi_image,image_template_10));
     template_scores.push_back(MatchTemplateCCOEFF(warped_roi_image,image_template_20));
     template_scores.push_back(MatchTemplateCCOEFF(warped_roi_image,image_template_30));
     template_scores.push_back(MatchTemplateCCOEFF(warped_roi_image,image_template_40));
     template_scores.push_back(MatchTemplateCCOEFF(warped_roi_image,image_template_50));
     template_scores.push_back(MatchTemplateCCOEFF(warped_roi_image,image_template_60));
     template_scores.push_back(MatchTemplateCCOEFF(warped_roi_image,image_template_70));
     template_scores.push_back(MatchTemplateCCOEFF(warped_roi_image,image_template_80));
     template_scores.push_back(MatchTemplateCCOEFF(warped_roi_image,image_template_90));

     //imshow("tmp10",image_template_90);

     auto it =  std::max_element( std::begin(template_scores), std::end(template_scores) );

     int id = std::distance(template_scores.begin(), it) + 1;

     std::cout << id << "0 km/h" << endl;// score:" << template_scores[id]  << endl;

}

void OnRoadSearch::SearchRoadObject(vector<LineValidationTable> current_table, vector<PointInDirection>& markings, vector<PointInDirection>& boxes, int SEARCH_DIRECTION, int SEARCH_DISTANCE_CODE)
{
    int line_distance = 0;



    switch(SEARCH_DISTANCE_CODE)
    {
        case LEFT_IN_LEFT_LANE:   line_distance = kLeftInLeftLaneDistance;     break;
        case LEFT_IN_RIGHT_LANE:  line_distance = kLeftInRightLaneDistance;    break;
        case RIGHT_IN_LEFT_LANE:  line_distance = kRightInLeftLaneDistance;    break;
        case RIGHT_IN_RIGHT_LANE: line_distance = kRightInRightLaneDistance;   break;
    }

    int current_pos = 0;

    while(current_pos < current_table.size() && current_pos < kMaxLaneObjectForsightDistance_)
    {
        vector<int> radial_vec_1;
        vector<int> radial_vec_2;

        float direction = current_table[current_pos].GetDirection();
        Point origin    = current_table[current_pos].GetOriginPoint();
        float orthogonal_direction = GetOrthogonalAngle(direction, SEARCH_DIRECTION);



        int current_x  = origin.x + line_distance * cos(orthogonal_direction*PI/180);
        int current_y  = origin.y - line_distance * sin(orthogonal_direction*PI/180);

        for(float current_angle=0; current_angle<359; current_angle+=kLaneObjectRadialScanStepSize)
        {
            float angle =  (current_angle*PI)/180;

            Point radial_point_1 = GetPolarCoordinate(current_x, current_y, angle, kLaneObjectRadialScanRadius1);
            Point radial_point_2 = GetPolarCoordinate(current_x, current_y, angle, kLaneObjectRadialScanRadius2);

            radial_vec_1.push_back((int)image_.at<uchar>(radial_point_1));
            radial_vec_2.push_back((int)image_.at<uchar>(radial_point_2));

            //image_rgb_bird_.at<Vec3b>(circle_point_2) = Vec3b(255,255,0);
        }
        //line( image_rgb_bird_,origin, Point(current_left_x,current_left_y), Scalar(255,255,0), 3, CV_AA);



        int radial_px_count_1 = std::count_if(radial_vec_1.begin(), radial_vec_1.end(), [](int i) {return i > 0;});
        int radial_px_count_2 = std::count_if(radial_vec_2.begin(), radial_vec_2.end(), [](int i) {return i > 0;});



        if((radial_px_count_1 < kMaxPxCountForLaneMarkingRadialScanRadius1 && radial_px_count_2 < kMaxPxCountForLaneMarkingRadialScanRadius2) &&
           (radial_px_count_1 > kMinPxCountForLaneMarkingRadialScanRadius1 && radial_px_count_2 > kMinPxCountForLaneMarkingRadialScanRadius2) )
        {
            markings.push_back(PointInDirection{current_x,current_y,0,direction});
        }

        else if (radial_px_count_1 > kMinPxCountForBoxRadialScanRadius1 && radial_px_count_2 > kMinPxCountForBoxRadialScanRadius2)
        {
            boxes.push_back(PointInDirection{current_x,current_y,0,direction});
        }
        current_pos+= kLaneObjectForsightStepSize;

    }

}

void OnRoadSearch::ExtractMinMaxLineElements( vector<LineValidationTable>  line,  MinMaxLineElements& line_minmax_elements )
{

    if(line.size() > 0)
    {
        auto x_it = minmax_element(begin(line),end(line),
                                            [&](LineValidationTable& line_point1, LineValidationTable& line_point2)
                                            {
                                                return line_point1.GetOriginPoint().x < line_point2.GetOriginPoint().x;
                                            });

        auto y_it = minmax_element(begin(line),end(line),
                                           [&](LineValidationTable& line_point1, LineValidationTable& line_point2)
                                           {
                                               return line_point1.GetOriginPoint().y < line_point2.GetOriginPoint().y;
                                           });

        int x_min_id = std::distance(line.begin(), x_it.first);
        int x_max_id = std::distance(line.begin(), x_it.second);
        int y_min_id = std::distance(line.begin(), y_it.first);
        int y_max_id = std::distance(line.begin(), y_it.second);

        line_minmax_elements = MinMaxLineElements{line[x_min_id].GetOriginPoint(),
                                                  line[x_max_id].GetOriginPoint(),
                                                  line[y_min_id].GetOriginPoint(),
                                                  line[y_max_id].GetOriginPoint(),
                                                  true};
    }
}


void OnRoadSearch::LoadValidationTables(vector<LineValidationTable> left_line_validation_table,
                                        vector<LineValidationTable> mid_line_validation_table,
                                        vector<LineValidationTable> right_line_validation_table)
{
    left_line_validation_table_ = left_line_validation_table;
    mid_line_validation_table_ = mid_line_validation_table;
    right_line_validation_table_ = right_line_validation_table;
}

void OnRoadSearch::SearchForStrongDirectionDifferencesInValidationtables()
{
    found_mid_crossing_ = false;
    if(mid_line_validation_table_.size()>0){
    if(mid_line_validation_table_[mid_line_validation_table_.size()-1].GetLabel() > 0)
    {
     for(auto it: mid_line_validation_table_)
       {
           if( it.GetDirection()>kMinToLeftMidLineDirectionForCrossing_ ||
               it.GetDirection() < kMaxToLeftMidLineDirectionForCrossing_ ||
               (it.GetDirection() < kMaxToRightMidLineDirectionForCrossing_ &&
                it.GetDirection() > kMinToRightMidLineDirectionForCrossing_))
           {
                   found_mid_crossing_ = true;

           }
       }
    }}

       found_left_crossing_ = false;
       if(left_line_validation_table_.size()>0)
       {
           int tmp_direction = left_line_validation_table_[0].GetDirection();

           for(int i=0; i<left_line_validation_table_.size(); i++)
           {
               if( left_line_validation_table_[i].GetOriginPoint().y < kMaxCrossingForsightY_) break;
               int current_direction =  left_line_validation_table_[i].GetDirection();
               int direction_difference = abs(current_direction - tmp_direction);

               if(direction_difference > kMinOutLineDirectionDifferenceForCrossing_)// && (360 - 80) < direction_difference)
               {
                   if(current_direction > kMinToLeftLeftLineDirectionForCrossing_ && current_direction < kMaxToLeftLeftLineDirectionForCrossing_)
                   {    found_left_crossing_ = true;
                        break;
                   }
               }
               tmp_direction = current_direction;
           }

       }

       found_right_crossing_ = false;
       if(right_line_validation_table_.size()>0)
       {
           int tmp_direction = right_line_validation_table_[0].GetDirection();
           for(int i=0; i<right_line_validation_table_.size(); i++)
           {
               if( right_line_validation_table_[i].GetOriginPoint().y < kMaxCrossingForsightY_) break;
               int current_direction =  right_line_validation_table_[i].GetDirection();
               int direction_difference = abs(current_direction - tmp_direction);

               if(direction_difference > kMinOutLineDirectionDifferenceForCrossing_)// || (360 - 80) < direction_difference)
               {
                   if(current_direction > kMinToRightRightLineDirectionForCrossing_ || current_direction < kMaxToRightRightLineDirectionForCrossing_)
                   {    found_right_crossing_ = true;
                        break;
                   }
               }
               tmp_direction = current_direction;
           }

       }
}

void OnRoadSearch::LoadInDriveDirectionTables(vector<LineValidationTable> left_line_points_in_drive_direction,
                                              vector<LineValidationTable> right_line_points_in_drive_direction)
{
    left_line_points_in_drive_direction_  = left_line_points_in_drive_direction;
    right_line_points_in_drive_direction_ = right_line_points_in_drive_direction;
}

void OnRoadSearch::SearchCrossRoad()
{

    SearchForStrongDirectionDifferencesInValidationtables();

    MinMaxLineElements left_line_minmax_elements_;
    MinMaxLineElements right_line_minmax_elements_;

    ExtractMinMaxLineElements(left_line_points_in_drive_direction_ , left_line_minmax_elements_ );
    ExtractMinMaxLineElements(right_line_points_in_drive_direction_, right_line_minmax_elements_ );

    left_line_in_crossing_height_ = left_line_minmax_elements_.y_min.y > MaxLeftLineYHeightForCrossing;
    right_line_in_crossing_height_ = right_line_minmax_elements_.y_min.y  > MaxRightLineYHeightForCrossing;
    left_line_in_crossing_size_  = left_line_points_in_drive_direction_.size() < MaxLeftLineSizetForCrossing;
    right_line_in_crossing_size_ = right_line_points_in_drive_direction_.size()< MaxRightLineSizeForCrossing ;

    found_crossing_ = false;


    if(found_left_crossing_ && found_right_crossing_ && found_mid_crossing_ && left_line_in_crossing_size_ && right_line_in_crossing_height_ && left_line_in_crossing_size_ && right_line_in_crossing_size_)
    {  found_crossing_= true;}

    else if(found_left_crossing_ && found_mid_crossing_ && left_line_in_crossing_size_ && right_line_in_crossing_height_ && left_line_in_crossing_size_ && right_line_in_crossing_size_)
    { found_crossing_= true;}

    else if(found_right_crossing_ && found_mid_crossing_ && left_line_in_crossing_size_ && right_line_in_crossing_height_ && left_line_in_crossing_size_ && right_line_in_crossing_size_)
    { found_crossing_= true;}

    else if (found_left_crossing_ && found_right_crossing_ && left_line_in_crossing_size_ && right_line_in_crossing_height_ && left_line_in_crossing_size_ && right_line_in_crossing_size_)
    { found_crossing_= true;}

    else if (found_right_crossing_ && found_mid_crossing_ && right_line_in_crossing_height_ && left_line_in_crossing_size_ && right_line_in_crossing_size_)
    {found_crossing_= true;}

    else if(found_mid_crossing_ && right_line_in_crossing_height_ && right_line_in_crossing_height_ && left_line_in_crossing_size_ && right_line_in_crossing_size_)
    { found_crossing_= true;}

    else if(found_mid_crossing_ && left_line_in_crossing_height_ && left_line_in_crossing_size_ && right_line_in_crossing_size_)
    {found_crossing_= true;}

    else if(found_mid_crossing_ && right_line_in_crossing_height_ && left_line_in_crossing_size_ && right_line_in_crossing_size_)
    { found_crossing_= true;}





}


/*
        {

            int line_distance = 95;
            int current_left_x  = origin.x + line_distance * cos(orthogonal_direction*PI/180);
            int current_left_y  = origin.y - line_distance * sin(orthogonal_direction*PI/180);
            for(float current_angle=0; current_angle<359; current_angle+=2)
            {
                float angle =  (current_angle*PI)/180;
                //cout << orthogonal_angle << " " << angle << " " <<  vanishing_point_start_parameters.left_angle <<  endl;
                Point circle_point = GetPolarCoordinate(current_left_x, current_left_y, angle, 20);
                Point circle_point_2 = GetPolarCoordinate(current_left_x, current_left_y, angle, 5);

             //LineIterator line_iterator(image_mono_bird_, origin, Point(curren_left_x,curren_left_y), 8);

                left_right_radial_vec_1.push_back((int)image_mono_bird_.at<uchar>(circle_point));
                left_right_radial_vec_2.push_back((int)image_mono_bird_.at<uchar>(circle_point_2));

                //image_rgb_bird_.at<Vec3b>(circle_point) = Vec3b(255,255,0);
                //image_rgb_bird_.at<Vec3b>(circle_point_2) = Vec3b(255,255,0);
            }
             //line( image_rgb_bird_,origin, Point(curren_left_x,curren_left_y), Scalar(255,0,255), 3, CV_AA);
        }
        left_search_i+=40;

        //for(auto it: left_left_radial_vec_1) cout << it << " " ;
        //cout << "#######################" << endl;
        //for(auto it: left_left_radial_vec_2) cout << it << " " ;


        int num_items1 = std::count_if(left_right_radial_vec_1.begin(), left_right_radial_vec_1.end(), [](int i) {return i > 0;});
        int num_items2 = std::count_if(left_right_radial_vec_2.begin(), left_right_radial_vec_2.end(), [](int i) {return i > 0;});
        //std::count(left_left_radial_vec_1.begin(), left_left_radial_vec_1.end(), 0);
        //int num_items2 = std::count(left_left_radial_vec_2.begin(), left_left_radial_vec_2.end(), 0);


        //cout << "left: " << num_items1 << " " << num_items2 <<  endl;

        if((num_items1 < 120 && num_items2 < 120) && (num_items1>30 && num_items2 > 20) )
        {
            int line_distance = 95;

            int current_left_x  = origin.x + line_distance * cos(orthogonal_direction*PI/180);
            int current_left_y  = origin.y - line_distance * sin(orthogonal_direction*PI/180);


            int start_crop_x = 0;
            int start_crop_y = 0;
            int end_crop_x = 0;
            int end_crop_y = 0;

            int offset_x = 100;
            int offset_y = 100;

            if(current_left_x-offset_x < 0)
            {
                start_crop_x = 0;
            }
            else{
                start_crop_x = current_left_x-offset_x;
            }
            if(current_left_y - offset_y < 0)
            {
                start_crop_y = 0;
            }
            else{
                start_crop_y   = current_left_y - offset_y;
            }
            if(current_left_x+offset_x > image_mono_bird_.cols)
            {
                end_crop_x =  image_mono_bird_.cols;
            }
            else{
                end_crop_x = current_left_x+200;
            }
            if(current_left_y+offset_y > image_mono_bird_.rows)
            {
                end_crop_y = image_mono_bird_.rows;
            }
            else{
                end_crop_y = current_left_y+offset_y;
            }

            Mat cropped = image_mono_bird_(Rect(Point(start_crop_x,start_crop_y),Point(end_crop_x,end_crop_y)));

            //circle(image_rgb_bird_,Point(ff,gg), 5, Scalar(0,0,255),CV_FILLED);

             cv::Mat rot = cv::getRotationMatrix2D(Point((end_crop_x-start_crop_x)/2,(end_crop_y-start_crop_y)/2), 90-direction, 1.0);

             cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), cropped.size(), 90-direction).boundingRect2f();
              // adjust transformation matrix
              //rot.at<double>(0,2) += bbox.width/2.0 - image_rgb_bird_.cols/2.0;
              //rot.at<double>(1,2) += bbox.height/2.0 - image_rgb_bird_.rows/2.0;

              //int ff =  rot.at<double>(0,2) + bbox.width/2.0 - image_rgb_bird_.cols/2.0;
              //int gg =  rot.at<double>(1,2) + bbox.height/2.0 - image_rgb_bird_.rows/2.0;

              //cout << ff <<" " << gg << endl;

             //circle(image_rgb_bird_,Point(ff,gg), 5, Scalar(0,0,255),CV_FILLED);

             Mat image_template_cropped;
//Mat image_template_cropped = image_mono_bird_(Rect(Point(577,256),Point(721,344)));
             //cout << rot << endl;
             cv::warpAffine(cropped, image_template_cropped, rot, bbox.size());

             imshow("warped",image_template_cropped);


             Mat image_template_10 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_10_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
             Mat image_template_20 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_20_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
             Mat image_template_30 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_30_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
             Mat image_template_40 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_40_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
             Mat image_template_50 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_50_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
             Mat image_template_60 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_60_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
             Mat image_template_70 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_70_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
             Mat image_template_80 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_80_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
             Mat image_template_90 = imread("/home/tb/gazebo_road_generation/ros_ws/scripts/scenarios/speed_limit_ground/speed_limit_90_ground.png", CV_LOAD_IMAGE_GRAYSCALE);
             //cout << image_template.size() << endl;



             //imshow("eee",image_template_cropped);

             //waitKey(0);

             Size resize_size(30,65);//the dst image size,e.g.100x100
             resize(image_template_10,image_template_10,resize_size);
             resize(image_template_20,image_template_20,resize_size);
             resize(image_template_30,image_template_30,resize_size);
             resize(image_template_40,image_template_40,resize_size);
             resize(image_template_50,image_template_50,resize_size);
             resize(image_template_60,image_template_60,resize_size);
             resize(image_template_70,image_template_70,resize_size);
             resize(image_template_80,image_template_80,resize_size);
             resize(image_template_90,image_template_90,resize_size);

             std::vector<int> template_scores;

             template_scores.push_back(MatchTemplateSQDIFF(image_template_cropped,image_template_10));
             template_scores.push_back(MatchTemplateSQDIFF(image_template_cropped,image_template_20));
             template_scores.push_back(MatchTemplateSQDIFF(image_template_cropped,image_template_30));
             template_scores.push_back(MatchTemplateSQDIFF(image_template_cropped,image_template_40));
             template_scores.push_back(MatchTemplateSQDIFF(image_template_cropped,image_template_50));
             template_scores.push_back(MatchTemplateSQDIFF(image_template_cropped,image_template_60));
             template_scores.push_back(MatchTemplateSQDIFF(image_template_cropped,image_template_70));
             template_scores.push_back(MatchTemplateSQDIFF(image_template_cropped,image_template_80));
             template_scores.push_back(MatchTemplateSQDIFF(image_template_cropped,image_template_90));

             auto it =  std::min_element( std::begin(template_scores), std::end(template_scores) );

             int id = std::distance(template_scores.begin(), it) + 1;

             std::cout << id << "0 km/h score:" << template_scores[id]  << endl;

/*

             template_scores.push_back(MatchTemplateCCOEFF(image_template_cropped,image_template_10));
             template_scores.push_back(MatchTemplateCCOEFF(image_template_cropped,image_template_20));
             template_scores.push_back(MatchTemplateCCOEFF(image_template_cropped,image_template_30));
             template_scores.push_back(MatchTemplateCCOEFF(image_template_cropped,image_template_40));
             template_scores.push_back(MatchTemplateCCOEFF(image_template_cropped,image_template_50));
             template_scores.push_back(MatchTemplateCCOEFF(image_template_cropped,image_template_60));
             template_scores.push_back(MatchTemplateCCOEFF(image_template_cropped,image_template_70));
             template_scores.push_back(MatchTemplateCCOEFF(image_template_cropped,image_template_80));
             template_scores.push_back(MatchTemplateCCOEFF(image_template_cropped,image_template_90));


             template_scores.push_back(MatchTemplateCCORR(image_template_cropped,image_template_10));
             template_scores.push_back(MatchTemplateCCORR(image_template_cropped,image_template_20));
             template_scores.push_back(MatchTemplateCCORR(image_template_cropped,image_template_30));
             template_scores.push_back(MatchTemplateCCORR(image_template_cropped,image_template_40));
             template_scores.push_back(MatchTemplateCCORR(image_template_cropped,image_template_50));
             template_scores.push_back(MatchTemplateCCORR(image_template_cropped,image_template_60));
             template_scores.push_back(MatchTemplateCCORR(image_template_cropped,image_template_70));
             template_scores.push_back(MatchTemplateCCORR(image_template_cropped,image_template_80));
             template_scores.push_back(MatchTemplateCCORR(image_template_cropped,image_template_90));


             auto it =  std::max_element( std::begin(template_scores), std::end(template_scores) );

             int id = std::distance(template_scores.begin(), it) + 1;

             std::cout << id << "0 km/h score:" << template_scores[id]  << endl;



        }


    }
}
*/

bool OnRoadSearch::SearchCrossWalk()
{

   /* int start_left_x_ = vanishing_point_start_parameters.left_x;
    int start_left_y_ = vanishing_point_start_parameters.left_y;
    float start_angle_left_ = vanishing_point_start_parameters.left_angle;

    int start_right_x_ = vanishing_point_start_parameters.right_x;
    int start_right_y_ = vanishing_point_start_parameters.right_y;
    int start_angle_right_ = vanishing_point_start_parameters.right_angle;
*/
    for(int current_distance=0; current_distance<kMaxCrossWalkForsightDistance_; current_distance+=kMaxCrossWalkForsightStepSize_)
    {
        int curren_left_x  = start_left_x_ + current_distance * cos(start_angle_left_*PI/180);
        int curren_left_y  = start_left_y_ - current_distance * sin(start_angle_left_*PI/180);

        int curren_right_x  = start_right_x_ + current_distance * cos(start_angle_right_*PI/180);
        int curren_right_y  = start_right_y_ - current_distance * sin(start_angle_right_*PI/180);

         LineIterator line_iterator(image_, Point(curren_left_x,curren_left_y), Point(curren_right_x,curren_right_y), 8);

         vector<uchar> scanned_line;

         for(int i=0; i<line_iterator.count; i++,line_iterator++) scanned_line.push_back(**line_iterator);

        Mat scanned_line_mat(scanned_line);

        threshold(scanned_line_mat, scanned_line_mat, 100, 255, CV_THRESH_BINARY);

       vector<pair<string,int>> black_white_road_segments = GatherBlackAndWhiteRoadSegments(scanned_line_mat);

        int crosswalk_counter = 0;

        for(auto it : black_white_road_segments)
        {
            if(it.second >= kMinCrossWalkSegmentWidth_ && it.second <= kMaxCrossWalkSegmentWidth_) crosswalk_counter++;
        }

        if(crosswalk_counter >= kMinCrossWalkSegmentsToFind_)
        {
            //circle(image_rgb_bird_,Point(start_left_x_,start_left_y_), 5, Scalar(0,0,255),CV_FILLED);
            //circle(image_rgb_bird_,Point(start_right_x_,start_right_y_), 5, Scalar(0,0,255),CV_FILLED);
            //cout << "Crosswalk ! " << crosswalk_counter << endl;
            found_cross_walk_ = true;
            return true;
        }

     }
    found_cross_walk_ = false;
    return false;

}


