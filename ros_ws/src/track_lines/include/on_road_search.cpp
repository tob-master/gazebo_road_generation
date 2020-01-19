#include "on_road_search.h"

OnRoadSearch::OnRoadSearch(
ros::NodeHandle* nh_,
OnRoadSearchInitializationParameters init)
:it_(*nh_),
kGoalLineIntensityThreshold_(init.goal_line_intensity_threshold),
kMinGoalSegmentWidth_(init.min_goal_segment_width),
kMaxGoalSegmentWidth_(init.max_goal_segment_width),
kMinGoalSegmentsToFind_(init.min_goal_segments_to_find),
kMaxCrossWalkForsightDistance_(init.max_crosswalk_forsight_distance),
kMaxCrossWalkForsightStepSize_(init.max_crosswalk_forsight_step_size),
kMinCrossWalkSegmentWidth_(init.min_crosswalk_segment_width),
kMaxCrossWalkSegmentWidth_(init.max_crosswalk_segment_width),
kMinCrossWalkSegmentsToFind_(init.min_crosswalk_segments_to_find),
kMinToLeftMidLineDirectionForCrossing_(init.min_to_left_mid_line_direction_for_crossing),
kMaxToLeftMidLineDirectionForCrossing_(init.max_to_left_mid_line_direction_for_crossing),
kMinToRightMidLineDirectionForCrossing_(init.min_to_right_mid_line_direction_for_crossing),
kMaxToRightMidLineDirectionForCrossing_(init.max_to_right_mid_line_direction_for_crossing),
kMinOutLineDirectionDifferenceForCrossing_(init.min_outline_direction_difference_for_crossing),
kMaxCrossingForsightY_(init.max_crossing_forsight_y),
kMinToLeftLeftLineDirectionForCrossing_(init.min_to_left_left_line_direction_difference_for_crossing),
kMaxToLeftLeftLineDirectionForCrossing_(init.max_to_left_left_line_direction_difference_for_crossing),
kMinToRightRightLineDirectionForCrossing_(init.min_to_right_right_line_direction_difference_for_crossing),
kMaxToRightRightLineDirectionForCrossing_(init.max_to_right_right_line_direction_difference_for_crossing),
MaxLeftLineYHeightForCrossing_(init.max_left_line_height_for_crossing),
MaxRightLineYHeightForCrossing_(init.max_right_line_height_for_crossing),
MaxLeftLineSizeForCrossing_(init.max_left_line_size_for_crossing),
MaxRightLineSizeForCrossing_(init.max_right_line_size_for_crossing),
kMaxLaneObjectForsightDistance_(init.max_lane_object_forsight_distance),
kLaneObjectForsightStepSize_(init.lane_object_forsight_step_size),
kLeftInLeftLaneLineIteratorEndOffset_(init.left_in_left_lane_lineiterator_end_offset),
kLeftInRightLaneLineIteratorStartOffset_(init.left_in_right_lane_lineiterator_start_offset),
kLeftInRightLaneLineIteratorEndOffset_(init.left_in_right_lane_lineiterator_end_offset),
kRightInLeftLaneLineIteratorStartOffset_(init.right_in_left_lane_lineiterator_start_offset),
kRightInLeftLaneLineIteratorEndOffset_(init.right_in_left_lane_lineiterator_end_offset),
kRightInRightLaneLineIteratorEndOffset_(init.right_in_right_lane_lineiterator_end_offset),
kLeftInLeftLaneRadialOuterLineOffset_(init.left_in_left_lane_radial_outerline_offset),
kLeftInRightLaneRadialOuterLineOffset_(init.left_in_right_lane_radial_outerline_offset),
kRightInLeftLaneRadialOuterLineOffset_(init.right_in_left_lane_radial_outerline_offset),
kRightInRightLaneRadialOuterLineOffset_(init.right_in_right_lane_radial_outerline_offset),
kLaneObjectRadialScanStepSize_(init.lane_object_radial_scan_step_size),
kLaneObjectRadialScanRadius_(init.lane_object_radial_scan_radius),
kMinMarkingSegmentsThreshold_(init.min_marking_segments_threshold),
kMinBoxSegmentsThreshold_(init.min_box_segments_threshold),
kMinWhitePixelsForBox_(init.min_white_pixels_for_box),
kVelocitySignTemplateHeight_(init.velocity_sign_template_height),
kVelocitySignTemplateWidth_(init.veloctiy_sign_template_width),
kGoalLineFieldOfView_(init.goal_line_field_of_view),
kRectTopLeftPointForClassifierRoi_(init.rect_top_left_point_for_classifier_roi_x,init.rect_top_left_point_for_classifier_roi_y),
kRectBottomRightPointForClassifierRoi_(init.rect_bottom_right_point_for_classifier_roi_x,init.rect_bottom_right_point_for_classifier_roi_y),
kResizeHeightForClassifierRoi_(init.resize_height_for_classifier_roi),
kResizeWidthForClassifierRoi_(init.resize_width_for_classifier_roi),
kTemplateRoiSizeX_(init.template_roi_size_x),
kTemplateRoiSizeY_(init.template_roi_size_y),
kRoadSignIntensityThreshold_(init.road_sign_intensity_threshold)
{

     // use for classifying speed markings in lane
    road_sign_image_publisher_  = it_.advertise("road_sign/image", 1);
    /*
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
    */
}

void OnRoadSearch::LoadSafeDriveAreaEvaluation(
vector<SafeDriveAreaEvaluationReturnInfo> safe_drive_area_evaluation)
{
    safe_drive_area_evaluation_ = safe_drive_area_evaluation;
}

void OnRoadSearch::LoadValidationTables(
vector<LineValidationTable> left_line_validation_table,
vector<LineValidationTable> mid_line_validation_table,
vector<LineValidationTable> right_line_validation_table)
{
    left_line_validation_table_ = left_line_validation_table;
    mid_line_validation_table_ = mid_line_validation_table;
    right_line_validation_table_ = right_line_validation_table;
}

void OnRoadSearch::LoadInDriveDirectionTables(
vector<LineValidationTable> left_line_points_in_drive_direction,
vector<LineValidationTable> right_line_points_in_drive_direction)
{
    left_line_points_in_drive_direction_  = left_line_points_in_drive_direction;
    right_line_points_in_drive_direction_ = right_line_points_in_drive_direction;
}

void OnRoadSearch::SetStartParameters(
StartParameters start_parameters)
{
    start_left_x_ = start_parameters.left_x;
    start_left_y_ = start_parameters.left_y;
    start_right_x_ = start_parameters.right_x;
    start_right_y_ = start_parameters.right_y;
    start_angle_left_= start_parameters.left_angle;
    start_angle_right_ = start_parameters.right_angle;
}

void OnRoadSearch::SearchOnRoad()
{
    SearchGoalLine(
    image_,
    start_left_x_,
    start_left_y_,
    start_angle_left_,
    kGoalLineFieldOfView_,
    kGoalLineIntensityThreshold_,
    kMinGoalSegmentWidth_,
    kMaxGoalSegmentWidth_,
    kMinGoalSegmentsToFind_,
    found_goal_line_,
    goal_line_mid_point_);

    SearchCrossWalk(
    image_,
    found_cross_walk_,
    cross_walk_mid_point_,
    start_left_x_,
    start_left_y_,
    start_angle_left_,
    start_right_x_,
    start_right_y_,
    start_angle_right_,
    kMinCrossWalkSegmentWidth_,
    kMaxCrossWalkSegmentWidth_,
    kMinCrossWalkSegmentsToFind_,
    kMaxCrossWalkForsightDistance_,
    kMaxCrossWalkForsightStepSize_);

    SearchCrossRoad(
    left_line_validation_table_,
    mid_line_validation_table_,
    right_line_validation_table_,
    left_line_points_in_drive_direction_,
    right_line_points_in_drive_direction_,
    crossing_mid_point_,
    found_left_crossing_pattern_,
    found_mid_crossing_pattern_,
    found_right_crossing_pattern_,
    left_line_in_crossing_height_,
    right_line_in_crossing_height_,
    left_line_in_crossing_size_,
    right_line_in_crossing_size_,
    found_crossing_,
    kMinToLeftMidLineDirectionForCrossing_,
    kMaxToLeftMidLineDirectionForCrossing_,
    kMaxToRightMidLineDirectionForCrossing_,
    kMinToRightMidLineDirectionForCrossing_,
    kMinOutLineDirectionDifferenceForCrossing_,
    kMaxCrossingForsightY_,
    kMinToLeftLeftLineDirectionForCrossing_,
    kMaxToLeftLeftLineDirectionForCrossing_,
    kMinToRightRightLineDirectionForCrossing_,
    kMaxToRightRightLineDirectionForCrossing_,
    MaxLeftLineYHeightForCrossing_,
    MaxRightLineYHeightForCrossing_,
    MaxLeftLineSizeForCrossing_,
    MaxRightLineSizeForCrossing_);

    if(!found_cross_walk_ && !found_crossing_ && !found_goal_line_)
    {
        SearchRoadObject(
        image_,
        left_line_points_in_drive_direction_,
        LEFT_TO_RIGHT,
        LEFT_IN_RIGHT_LANE,
        road_object_mid_point_,
        found_box_,
        found_marking_,
        kLeftInLeftLaneRadialOuterLineOffset_,
        kLeftInRightLaneRadialOuterLineOffset_,
        kRightInLeftLaneRadialOuterLineOffset_,
        kRightInRightLaneRadialOuterLineOffset_,
        kMaxLaneObjectForsightDistance_,
        kLeftInLeftLaneLineIteratorEndOffset_,
        kLeftInRightLaneLineIteratorStartOffset_,
        kLeftInRightLaneLineIteratorEndOffset_,
        kRightInLeftLaneLineIteratorStartOffset_,
        kRightInLeftLaneLineIteratorEndOffset_,
        kRightInRightLaneLineIteratorEndOffset_,
        kLaneObjectRadialScanStepSize_,
        kLaneObjectRadialScanRadius_,
        kMinWhitePixelsForBox_,
        kRoadSignIntensityThreshold_,
        kMinBoxSegmentsThreshold_,
        kMinMarkingSegmentsThreshold_,
        kLaneObjectForsightStepSize_);

        //LEFT_TO_RIGHT, LEFT_IN_LEFT_LANE
        //RIGHT_TO_LEFT, RIGHT_IN_LEFT_LANE

        SearchRoadObject(
        image_,
        right_line_points_in_drive_direction_,
        RIGHT_TO_LEFT,
        RIGHT_IN_RIGHT_LANE,
        road_object_mid_point_,
        found_box_,
        found_marking_,
        kLeftInLeftLaneRadialOuterLineOffset_,
        kLeftInRightLaneRadialOuterLineOffset_,
        kRightInLeftLaneRadialOuterLineOffset_,
        kRightInRightLaneRadialOuterLineOffset_,
        kMaxLaneObjectForsightDistance_,
        kLeftInLeftLaneLineIteratorEndOffset_,
        kLeftInRightLaneLineIteratorStartOffset_,
        kLeftInRightLaneLineIteratorEndOffset_,
        kRightInLeftLaneLineIteratorStartOffset_,
        kRightInLeftLaneLineIteratorEndOffset_,
        kRightInRightLaneLineIteratorEndOffset_,
        kLaneObjectRadialScanStepSize_,
        kLaneObjectRadialScanRadius_,
        kMinWhitePixelsForBox_,
        kRoadSignIntensityThreshold_,
        kMinBoxSegmentsThreshold_,
        kMinMarkingSegmentsThreshold_,
        kLaneObjectForsightStepSize_);

    }
    else{
        found_box_ = false;
        found_marking_ = false;
    }

    if(found_marking_)
    {
        SendMarkingImageToClassifier(
        image_,
        kRectTopLeftPointForClassifierRoi_,
        kRectBottomRightPointForClassifierRoi_,
        kResizeHeightForClassifierRoi_,
        kResizeWidthForClassifierRoi_);

        for(auto &iter: safe_drive_area_evaluation_)
        {
            vector<vector<Point>> search_rect =  iter.search_rect;

                double res = pointPolygonTest(search_rect[0], road_object_mid_point_, false);
                if(res>=0)
                {
                     iter.marking = true;
                }
         }
    }
    if(found_box_)
    {

        for(auto &iter: safe_drive_area_evaluation_)
        {
            vector<vector<Point>> search_rect =  iter.search_rect;

                double res = pointPolygonTest(search_rect[0], road_object_mid_point_, false);
                if(res>=0)
                {
                     iter.box = true;

                }
        }
    }
    if(found_crossing_)
    {
        for(auto &iter: safe_drive_area_evaluation_)
        {
            vector<vector<Point>> search_rect =  iter.search_rect;

                double res = pointPolygonTest(search_rect[0], crossing_mid_point_, false);
                if(res>=0)
                {
                     iter.crossing = true;

                }
         }

    }
    if(found_cross_walk_)
    {
        for(auto &iter: safe_drive_area_evaluation_)
        {
            vector<vector<Point>> search_rect =  iter.search_rect;

                double res = pointPolygonTest(search_rect[0], cross_walk_mid_point_, false);
                if(res>=0)
                {
                     iter.crosswalk = true;
                }
         }
    }
    if(found_goal_line_)
    {
        for(auto &iter: safe_drive_area_evaluation_)
        {
            vector<vector<Point>> search_rect =  iter.search_rect;

                double res = pointPolygonTest(search_rect[0], goal_line_mid_point_, false);
                if(res>=0)
                {
                     iter.goalline = true;

                }
         }

     }
}

bool OnRoadSearch::SearchGoalLine(
Mat image,
const int start_left_x,
const int start_left_y,
const float start_angle_left,
const int kGoalLineFieldOfView,
const int kGoalLineIntensityThreshold,
const int kMinGoalSegmentWidth,
const int kMaxGoalSegmentWidth,
const int kMinGoalSegmentsToFind,
bool &found_goal_line,
Point &goal_line_mid_point)
{

    float orthogonal_angle = GetOrthogonalAngle(start_angle_left,LEFT_TO_RIGHT);

    for(float current_angle=(orthogonal_angle-kGoalLineFieldOfView); current_angle<orthogonal_angle+kGoalLineFieldOfView; current_angle+=1)
    {
        float angle =  (current_angle*PI)/180;
        Point line_iterator_end = GetPolarCoordinate(start_left_x, start_left_y, angle, 120);

        LineIterator line_iterator(image, Point(start_left_x,start_left_y),line_iterator_end , 8);

        //line( image, Point(start_left_x_,start_left_y_),line_iterator_end, Scalar(255,255,255), 1, CV_AA);
        vector<uchar> scanned_line;

        for(int i=0; i<line_iterator.count; i++,line_iterator++)  scanned_line.push_back(**line_iterator);


        Mat scanned_line_mat(scanned_line);
        threshold(scanned_line_mat, scanned_line_mat, kGoalLineIntensityThreshold, 255, CV_THRESH_BINARY);

        vector<pair<string,int>> black_white_road_segments = GatherBlackAndWhiteRoadSegments(scanned_line_mat);

         int goal_line_counter = 0;

         for(auto it : black_white_road_segments)
         {
             if(it.second >= kMinGoalSegmentWidth && it.second <= kMaxGoalSegmentWidth) goal_line_counter++;
         }

         if(goal_line_counter >= kMinGoalSegmentsToFind)
         {
             //circle(image_rgb_bird_,Point(start_left_x_,start_left_y_), 5, Scalar(0,255,0),CV_FILLED);

             int line_mid_x = (int)((line_iterator_end.x + start_left_x) / 2);
             int line_mid_y = (int)((line_iterator_end.y + start_left_y) / 2);

             goal_line_mid_point = Point(line_mid_x,line_mid_y);
             found_goal_line = true;
             return true;
         }


      }

    //imwrite("dddeee.png",image);
    //imshow("image",image);
    //waitKey(0);
    found_goal_line = false;
    goal_line_mid_point = Point(-1,-1);
    return false;
}



float OnRoadSearch::GetOrthogonalAngle(
float angle,
int SEARCH_LINE_CODE)
{
    int  angle_ = 0;

    if(SEARCH_LINE_CODE == LEFT_TO_MID || SEARCH_LINE_CODE == LEFT_TO_RIGHT || SEARCH_LINE_CODE == MID_TO_RIGHT) angle_ = (angle - 90);

    if(SEARCH_LINE_CODE == RIGHT_TO_MID || SEARCH_LINE_CODE == RIGHT_TO_LEFT || SEARCH_LINE_CODE == MID_TO_LEFT) angle_ = (angle + 90);

    if(angle_ > 359) angle_ = angle_ % 360;

    if(angle_ < 0) angle_ =  360 - abs(angle_);

    return float(angle_);
}

Point OnRoadSearch::GetPolarCoordinate(
int x,
int y,
float angle,
int radius)
{
    x = x + radius * cos(angle) + 0.5;
    y = y - radius * sin(angle) + 0.5;

    return Point(x,y);
}

vector<pair<string,int>> OnRoadSearch::GatherBlackAndWhiteRoadSegments(
Mat scanned_line_mat)
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


bool OnRoadSearch::SearchCrossWalk(
Mat image,
bool &found_cross_walk,
Point &cross_walk_mid_point,
const int start_left_x,
const int start_left_y,
const float start_angle_left,
const int start_right_x,
const int start_right_y,
const float start_angle_right,
const int kMinCrossWalkSegmentWidth,
const int kMaxCrossWalkSegmentWidth,
const int kMinCrossWalkSegmentsToFind,
const int kMaxCrossWalkForsightDistance,
const int kMaxCrossWalkForsightStepSize)
{

    for(int current_distance=0; current_distance<kMaxCrossWalkForsightDistance; current_distance+=kMaxCrossWalkForsightStepSize)
    {
        int curren_left_x  = start_left_x + current_distance * cos(start_angle_left*PI/180);
        int curren_left_y  = start_left_y - current_distance * sin(start_angle_left*PI/180);

        int curren_right_x  = start_right_x + current_distance * cos(start_angle_right*PI/180);
        int curren_right_y  = start_right_y - current_distance * sin(start_angle_right*PI/180);

         LineIterator line_iterator(image, Point(curren_left_x,curren_left_y), Point(curren_right_x,curren_right_y), 8);

         vector<uchar> scanned_line;

         for(int i=0; i<line_iterator.count; i++,line_iterator++) scanned_line.push_back(**line_iterator);

        Mat scanned_line_mat(scanned_line);

        threshold(scanned_line_mat, scanned_line_mat, 100, 255, CV_THRESH_BINARY);

       vector<pair<string,int>> black_white_road_segments = GatherBlackAndWhiteRoadSegments(scanned_line_mat);

        int crosswalk_counter = 0;

        for(auto it : black_white_road_segments)
        {
            if(it.second >= kMinCrossWalkSegmentWidth && it.second <= kMaxCrossWalkSegmentWidth) crosswalk_counter++;
        }

        if(crosswalk_counter >= kMinCrossWalkSegmentsToFind)
        {
            //circle(image_rgb_bird_,Point(start_left_x_,start_left_y_), 5, Scalar(0,0,255),CV_FILLED);
            //circle(image_rgb_bird_,Point(start_right_x_,start_right_y_), 5, Scalar(0,0,255),CV_FILLED);
            int line_mid_x = (int)((curren_right_x + curren_left_x) / 2);
            int line_mid_y = (int)((curren_right_y + curren_left_y) / 2);

            cross_walk_mid_point = Point(line_mid_x,line_mid_y);

            found_cross_walk = true;
            return true;
        }

     }
    cross_walk_mid_point = Point(-1,-1);
    found_cross_walk = false;
    return false;
}

void OnRoadSearch::SearchCrossRoad(
vector<LineValidationTable> left_line_validation_table,
vector<LineValidationTable>mid_line_validation_table,
vector<LineValidationTable>right_line_validation_table,
vector<LineValidationTable> left_line_points_in_drive_direction,
vector<LineValidationTable> right_line_points_in_drive_direction,
Point &crossing_mid_point,
bool &found_left_crossing_pattern,
bool &found_mid_crossing_pattern,
bool &found_right_crossing_pattern,
bool &left_line_in_crossing_height,
bool &right_line_in_crossing_height,
bool &left_line_in_crossing_size,
bool &right_line_in_crossing_size,
bool &found_crossing,
const int kMinToLeftMidLineDirectionForCrossing,
const int kMaxToLeftMidLineDirectionForCrossing,
const int kMaxToRightMidLineDirectionForCrossing,
const int kMinToRightMidLineDirectionForCrossing,
const int kMinOutLineDirectionDifferenceForCrossing,
const int kMaxCrossingForsightY,
const int kMinToLeftLeftLineDirectionForCrossing,
const int kMaxToLeftLeftLineDirectionForCrossing,
const int kMinToRightRightLineDirectionForCrossing,
const int kMaxToRightRightLineDirectionForCrossing,
const int MaxLeftLineYHeightForCrossing,
const int MaxRightLineYHeightForCrossing,
const int MaxLeftLineSizeForCrossing,
const int MaxRightLineSizeForCrossing)
{

    Point crossing_left_line_point = Point(-1,-1);
    Point crossing_right_line_point = Point(-1,-1);

    SearchForStrongDirectionDifferencesInValidationtables(
    left_line_validation_table,
    mid_line_validation_table,
    right_line_validation_table,
    crossing_left_line_point,
    crossing_right_line_point,
    found_mid_crossing_pattern,
    found_left_crossing_pattern,
    found_right_crossing_pattern,
    kMinToLeftMidLineDirectionForCrossing,
    kMaxToLeftMidLineDirectionForCrossing,
    kMaxToRightMidLineDirectionForCrossing,
    kMinToRightMidLineDirectionForCrossing,
    kMinOutLineDirectionDifferenceForCrossing,
    kMaxCrossingForsightY,
    kMinToLeftLeftLineDirectionForCrossing,
    kMaxToLeftLeftLineDirectionForCrossing,
    kMinToRightRightLineDirectionForCrossing,
    kMaxToRightRightLineDirectionForCrossing);

    MinMaxLineElements left_line_minmax_elements_;
    MinMaxLineElements right_line_minmax_elements_;

    ExtractMinMaxLineElements(left_line_points_in_drive_direction , left_line_minmax_elements_ );
    ExtractMinMaxLineElements(right_line_points_in_drive_direction, right_line_minmax_elements_ );

    left_line_in_crossing_height = left_line_minmax_elements_.y_min.y > MaxLeftLineYHeightForCrossing;
    right_line_in_crossing_height = right_line_minmax_elements_.y_min.y  > MaxRightLineYHeightForCrossing;
    left_line_in_crossing_size  = left_line_points_in_drive_direction.size() < MaxLeftLineSizeForCrossing;
    right_line_in_crossing_size = right_line_points_in_drive_direction.size()< MaxRightLineSizeForCrossing ;

    found_crossing = false;


    if(found_left_crossing_pattern && found_right_crossing_pattern && found_mid_crossing_pattern && left_line_in_crossing_size && right_line_in_crossing_height && left_line_in_crossing_size && right_line_in_crossing_size)
    {
        int line_mid_x = (int)((crossing_left_line_point.x +  crossing_right_line_point.x) / 2);
        int line_mid_y = (int)((crossing_left_line_point.y +  crossing_right_line_point.y) / 2);

        crossing_mid_point = Point(line_mid_x,line_mid_y);
        found_crossing= true;
    }
    else if(found_left_crossing_pattern && found_mid_crossing_pattern && left_line_in_crossing_size && right_line_in_crossing_height && left_line_in_crossing_size && right_line_in_crossing_size)
    {
        int line_mid_x = (int)(crossing_left_line_point.x);
        int line_mid_y = (int)(crossing_left_line_point.y);

        crossing_mid_point = Point(line_mid_x,line_mid_y);
        found_crossing= true;
    }
    else if(found_right_crossing_pattern && found_mid_crossing_pattern && left_line_in_crossing_size && right_line_in_crossing_height && left_line_in_crossing_size && right_line_in_crossing_size)
    {
        int line_mid_x = (int)(crossing_right_line_point.x);
        int line_mid_y = (int)(crossing_right_line_point.y);

        crossing_mid_point = Point(line_mid_x,line_mid_y);
        found_crossing= true;
    }
    else if (found_left_crossing_pattern && found_right_crossing_pattern && left_line_in_crossing_size && right_line_in_crossing_height && left_line_in_crossing_size && right_line_in_crossing_size)
    {
        int line_mid_x = (int)((crossing_left_line_point.x +  crossing_right_line_point.x) / 2);
        int line_mid_y = (int)((crossing_left_line_point.y +  crossing_right_line_point.y) / 2);

        crossing_mid_point = Point(line_mid_x,line_mid_y);

        found_crossing= true;
    }
    else if (found_right_crossing_pattern && found_mid_crossing_pattern && right_line_in_crossing_height && left_line_in_crossing_size && right_line_in_crossing_size)
    {
        int line_mid_x = (int)(crossing_right_line_point.x);
        int line_mid_y = (int)(crossing_right_line_point.y);

        crossing_mid_point = Point(line_mid_x,line_mid_y);
        found_crossing= true;
    }
}

void OnRoadSearch::SearchForStrongDirectionDifferencesInValidationtables(
vector<LineValidationTable> left_line_validation_table,
vector<LineValidationTable>mid_line_validation_table,
vector<LineValidationTable>right_line_validation_table,
Point & crossing_left_line_point,
Point & crossing_right_line_point,
bool &found_mid_crossing_pattern,
bool &found_left_crossing_pattern,
bool &found_right_crossing_pattern,
const int kMinToLeftMidLineDirectionForCrossing,
const int kMaxToLeftMidLineDirectionForCrossing,
const int kMaxToRightMidLineDirectionForCrossing,
const int kMinToRightMidLineDirectionForCrossing,
const int kMinOutLineDirectionDifferenceForCrossing,
const int kMaxCrossingForsightY,
const int kMinToLeftLeftLineDirectionForCrossing,
const int kMaxToLeftLeftLineDirectionForCrossing,
const int kMinToRightRightLineDirectionForCrossing,
const int kMaxToRightRightLineDirectionForCrossing
)
{
    found_mid_crossing_pattern = false;
    if(mid_line_validation_table.size()>0){
    if(mid_line_validation_table[mid_line_validation_table.size()-1].GetLabel() > 0)
    {
     for(auto it: mid_line_validation_table)
       {
           if( it.GetDirection()>kMinToLeftMidLineDirectionForCrossing ||
               it.GetDirection() < kMaxToLeftMidLineDirectionForCrossing ||
               (it.GetDirection() < kMaxToRightMidLineDirectionForCrossing &&
                it.GetDirection() > kMinToRightMidLineDirectionForCrossing))
           {
                   found_mid_crossing_pattern = true;

           }
       }
    }}

       found_left_crossing_pattern = false;
       if(left_line_validation_table.size()>0)
       {
           int tmp_direction = left_line_validation_table[0].GetDirection();

           for(int i=0; i<left_line_validation_table.size(); i++)
           {
               if( left_line_validation_table[i].GetOriginPoint().y < kMaxCrossingForsightY) break;
               int current_direction =  left_line_validation_table[i].GetDirection();
               int direction_difference = abs(current_direction - tmp_direction);

               if(direction_difference > kMinOutLineDirectionDifferenceForCrossing)// && (360 - 80) < direction_difference)
               {
                   if(current_direction > kMinToLeftLeftLineDirectionForCrossing && current_direction < kMaxToLeftLeftLineDirectionForCrossing)
                        crossing_left_line_point = left_line_validation_table[i].GetOriginPoint();
                   {    found_left_crossing_pattern = true;
                        break;
                   }
               }
               tmp_direction = current_direction;
           }

       }

       found_right_crossing_pattern = false;
       if(right_line_validation_table.size()>0)
       {
           int tmp_direction = right_line_validation_table[0].GetDirection();
           for(int i=0; i<right_line_validation_table.size(); i++)
           {
               if( right_line_validation_table[i].GetOriginPoint().y < kMaxCrossingForsightY) break;
               int current_direction =  right_line_validation_table[i].GetDirection();
               int direction_difference = abs(current_direction - tmp_direction);

               if(direction_difference > kMinOutLineDirectionDifferenceForCrossing)// || (360 - 80) < direction_difference)
               {
                   if(current_direction > kMinToRightRightLineDirectionForCrossing || current_direction < kMaxToRightRightLineDirectionForCrossing)
                   {   crossing_right_line_point = right_line_validation_table[i].GetOriginPoint();
                       found_right_crossing_pattern = true;
                        break;
                   }
               }
               tmp_direction = current_direction;
           }

       }
}

void OnRoadSearch::ExtractMinMaxLineElements(
vector<LineValidationTable>  line,
MinMaxLineElements& line_minmax_elements )
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


void OnRoadSearch::SearchRoadObject(
Mat image,
vector<LineValidationTable> current_table,
int search_direction,
int search_distance_code,
Point &road_object_mid_point,
bool &found_box,
bool &found_marking,
const int kLeftInLeftLaneRadialOuterLineOffset,
const int kLeftInRightLaneRadialOuterLineOffset,
const int kRightInLeftLaneRadialOuterLineOffset,
const int kRightInRightLaneRadialOuterLineOffset,
const int kMaxLaneObjectForsightDistance,
const int kLeftInLeftLaneLineIteratorEndOffset,
const int kLeftInRightLaneLineIteratorStartOffset,
const int kLeftInRightLaneLineIteratorEndOffset,
const int kRightInLeftLaneLineIteratorStartOffset,
const int kRightInLeftLaneLineIteratorEndOffset,
const int kRightInRightLaneLineIteratorEndOffset,
const int kLaneObjectRadialScanStepSize,
const int kLaneObjectRadialScanRadius,
const int kMinWhitePixelsForBox,
const int kRoadSignIntensityThreshold,
const int kMinBoxSegmentsThreshold,
const int kMinMarkingSegmentsThreshold,
const int kLaneObjectForsightStepSize)
{
    int radial_outer_line_offset = 0;

    Point line_iterator_start;
    Point line_iterator_end;

    switch(search_distance_code)
    {
        case LEFT_IN_LEFT_LANE:   radial_outer_line_offset = kLeftInLeftLaneRadialOuterLineOffset;     break;
        case LEFT_IN_RIGHT_LANE:  radial_outer_line_offset = kLeftInRightLaneRadialOuterLineOffset;    break;
        case RIGHT_IN_LEFT_LANE:  radial_outer_line_offset = kRightInLeftLaneRadialOuterLineOffset;    break;
        case RIGHT_IN_RIGHT_LANE: radial_outer_line_offset = kRightInRightLaneRadialOuterLineOffset;   break;
    }

    int current_pos = 0;

    while(current_pos < current_table.size() && current_pos < kMaxLaneObjectForsightDistance)
    {
        vector<uchar> radial_vec;
        float direction = current_table[current_pos].GetDirection();
        Point origin    = current_table[current_pos].GetOriginPoint();
        float orthogonal_direction = GetOrthogonalAngle(direction, search_direction);


        switch(search_distance_code)
        {
            case LEFT_IN_LEFT_LANE:     line_iterator_start = origin;
                                        line_iterator_end   = Point(origin.x + kLeftInLeftLaneLineIteratorEndOffset * cos(orthogonal_direction*PI/180),
                                                                    origin.y - kLeftInLeftLaneLineIteratorEndOffset * sin(orthogonal_direction*PI/180));
                                        break;

            case LEFT_IN_RIGHT_LANE:    line_iterator_start = Point(origin.x + kLeftInRightLaneLineIteratorStartOffset * cos(orthogonal_direction*PI/180),
                                                                    origin.y - kLeftInRightLaneLineIteratorStartOffset * sin(orthogonal_direction*PI/180));
                                        line_iterator_end   = Point(origin.x + kLeftInRightLaneLineIteratorEndOffset * cos(orthogonal_direction*PI/180),
                                                                    origin.y - kLeftInRightLaneLineIteratorEndOffset * sin(orthogonal_direction*PI/180));
                                        break;

            case RIGHT_IN_LEFT_LANE:    line_iterator_start = Point(origin.x + kRightInLeftLaneLineIteratorStartOffset * cos(orthogonal_direction*PI/180),
                                                                    origin.y - kRightInLeftLaneLineIteratorStartOffset * sin(orthogonal_direction*PI/180));
                                        line_iterator_end   = Point(origin.x + kRightInLeftLaneLineIteratorEndOffset * cos(orthogonal_direction*PI/180),
                                                                    origin.y - kRightInLeftLaneLineIteratorEndOffset * sin(orthogonal_direction*PI/180));
                                        break;

            case RIGHT_IN_RIGHT_LANE:   line_iterator_start = origin;
                                        line_iterator_end   = Point(origin.x + kRightInRightLaneLineIteratorEndOffset * cos(orthogonal_direction*PI/180),
                                                                    origin.y - kRightInRightLaneLineIteratorEndOffset * sin(orthogonal_direction*PI/180));
                                        break;
        }


        int current_x  = origin.x + radial_outer_line_offset * cos(orthogonal_direction*PI/180);
        int current_y  = origin.y - radial_outer_line_offset * sin(orthogonal_direction*PI/180);

        for(float current_angle=0; current_angle<359; current_angle+=kLaneObjectRadialScanStepSize)
        {
            float angle =  (current_angle*PI)/180;

            Point radial_point = GetPolarCoordinate(current_x, current_y, angle, kLaneObjectRadialScanRadius);

            radial_vec.push_back(image.at<uchar>(radial_point));
            //image.at<uchar>(radial_point) = 255;
            //image_rgb_bird_.at<Vec3b>(circle_point_2) = Vec3b(255,255,0);
        }

        //imshow("rad_scan",image);

        LineIterator line_iterator(image, line_iterator_start, line_iterator_end, 8);
        //line( image,line_iterator_start,  line_iterator_end, Scalar(255,255,255), 1, CV_AA);

        vector<uchar> scanned_line;

        for(int i=0; i<line_iterator.count; i++,line_iterator++) scanned_line.push_back(**line_iterator);

        Mat radial_mat(radial_vec);
        Mat scanned_line_mat(scanned_line);

        threshold(radial_mat, radial_mat, kRoadSignIntensityThreshold, 255, CV_THRESH_BINARY);
        threshold(scanned_line_mat, scanned_line_mat, kRoadSignIntensityThreshold, 255, CV_THRESH_BINARY);

        vector<pair<string,int>> black_white_road_segments_radial_vec = GatherBlackAndWhiteRoadSegments(radial_mat);
        vector<pair<string,int>> black_white_road_segments_scanned_line = GatherBlackAndWhiteRoadSegments(scanned_line_mat);

        int box_segments_counter = 0;

        for(auto it : black_white_road_segments_radial_vec)
        {
            if(it.second >= kMinWhitePixelsForBox && it.first == "white") box_segments_counter++;
        }

        for(auto it : black_white_road_segments_scanned_line)
        {
            if(it.second >= kMinWhitePixelsForBox && it.first == "white") box_segments_counter++;
        }

        if(box_segments_counter>=kMinBoxSegmentsThreshold)
        {
            road_object_mid_point = Point(current_x,current_y);
            found_box= true;
            return;
        }
        else{
            found_box = false;
        }

        if(!box_segments_counter>=kMinBoxSegmentsThreshold &&
           black_white_road_segments_radial_vec.size()>=kMinMarkingSegmentsThreshold &&
           black_white_road_segments_scanned_line.size()>=kMinMarkingSegmentsThreshold)
        {
            road_object_mid_point = Point(current_x,current_y);
            found_marking = true;
            return;
        }
        else {
            found_marking = false;
        }

        current_pos+= kLaneObjectForsightStepSize;
    }
    road_object_mid_point = Point(-1,-1);

    //imwrite("ddkk.png",image);
    //imshow("k",image);
    //waitKey(0);
}


void OnRoadSearch::DrawEvaluatedSafetyAreas(
Mat& rgb)
{

    for(auto it: safe_drive_area_evaluation_)
    {
        vector<vector<Point>> search_rect   = it.search_rect;
        Point rect_mid_point = it.rect_mid_point;




        if(it.goalline)
        {   string str_score = to_string(it.max_line_score) + " GOALLINE!";
            drawContours(rgb, search_rect, -1, Scalar(0,0,255), 2, LINE_8);
            putText(rgb, str_score, rect_mid_point,FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
        }
        else if(it.crosswalk)
        {
            string str_score = to_string(it.max_line_score) + " CROSSWALK!";
            drawContours(rgb, search_rect, -1, Scalar(0,0,255), 2, LINE_8);
            putText(rgb, str_score, rect_mid_point,FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
        }
        else if(it.box)
        {
            string str_score = to_string(it.max_line_score) + " BOX!";
            drawContours(rgb, search_rect, -1, Scalar(0,0,255), 2, LINE_8);
            putText(rgb, str_score, rect_mid_point,FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
        }
        else if(it.marking)
        {
            string str_score = to_string(it.max_line_score) + " MARKING!";
            drawContours(rgb, search_rect, -1, Scalar(0,0,255), 2, LINE_8);
            putText(rgb, str_score, rect_mid_point,FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
        }
        else if(it.crossing)
        {
            string str_score = to_string(it.max_line_score) + " CROSSING!";
            drawContours(rgb, search_rect, -1, Scalar(0,0,255), 2, LINE_8);
            putText(rgb, str_score, rect_mid_point,FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
        }
        else {
            string str_score = to_string(it.max_line_score);
            drawContours(rgb, search_rect, -1, Scalar(0,255,0), 2, LINE_8);
            putText(rgb, str_score, rect_mid_point,FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,0), 1, CV_AA);
        }


    }
}

void OnRoadSearch::SendMarkingImageToClassifier(
Mat image,
Point kRectTopLeftPointForClassifierRoi,
Point kRectBottomRightPointForClassifierRoi,
const int kResizeHeightForClassifierRoi,
const int kResizeWidthForClassifierRoi)
{

    //cout << kRectTopLeftPointForClassifierRoi << kRectBottomRightPointForClassifierRoi << endl;
    Mat roi_image = image(Rect(kRectTopLeftPointForClassifierRoi,kRectBottomRightPointForClassifierRoi));

    resize(roi_image,roi_image,Size(kResizeHeightForClassifierRoi,kResizeWidthForClassifierRoi));

    imshow("roi_image",roi_image);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", roi_image).toImageMsg();
    road_sign_image_publisher_.publish(msg);

}










double OnRoadSearch::MatchTemplateSQDIFF(
Mat image,
Mat template_image)
{

    Mat template_match;

    double min_val; double max_val; Point min_loc; Point max_loc;

    matchTemplate( image, template_image, template_match, CV_TM_SQDIFF );

    minMaxLoc( template_match, &min_val, &max_val, &min_loc, &max_loc, Mat() );

    return min_val;

    //rectangle( image_rgb_bird_, maxLoc10, Point( maxLoc10.x + 20, maxLoc10.y + 20 ), Scalar(255,255,0), 2, 8, 0 );

}

double OnRoadSearch::MatchTemplateCCOEFF(
Mat image,
Mat template_image)
{

    Mat template_match;

    double min_val; double max_val; Point min_loc; Point max_loc;

    matchTemplate( image, template_image, template_match, CV_TM_CCOEFF );

    minMaxLoc( template_match, &min_val, &max_val, &min_loc, &max_loc, Mat() );

    return min_val;

}


void OnRoadSearch::TemplateMatchMarking(
vector<PointInDirection> markings)
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

     imshow("imgg",warped_roi_image);

     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", warped_roi_image).toImageMsg();


     road_sign_image_publisher_.publish(msg);

    /*
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours( image_, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    int idx = 0;
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        Scalar color( rand()&255, rand()&255, rand()&255 );
        drawContours( image_, contours, idx, color, CV_FILLED, 8, hierarchy );
    }


    imshow( "Components", image_ );
    waitKey(0);
    */

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



