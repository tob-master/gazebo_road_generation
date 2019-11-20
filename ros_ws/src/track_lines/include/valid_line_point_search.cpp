#include "valid_line_point_search.h"

ValidLinePointSearch::ValidLinePointSearch()
{

}




float ValidLinePointSearch::GetOrthogonalAngle(float angle, int SEARCH_LINE_CODE)
{
    int  angle_ = 0;

    if(SEARCH_LINE_CODE == LEFT_TO_MID || SEARCH_LINE_CODE == LEFT_TO_RIGHT || SEARCH_LINE_CODE == MID_TO_RIGHT) angle_ = (angle - 90);

    if(SEARCH_LINE_CODE == RIGHT_TO_MID || SEARCH_LINE_CODE == RIGHT_TO_LEFT || SEARCH_LINE_CODE == MID_TO_LEFT) angle_ = (angle + 90);

    if(angle_ > 359) angle_ = angle_ % 360;

    if(angle_ < 0) angle_ =  360 - abs(angle_);

    return float(angle_);
}


int ValidLinePointSearch::GetPixelValue(int x, int y)
{
    return (int)current_image_.at<uchar>(Point(x,y));
}

int ValidLinePointSearch::GetPixelValue(Point point)
{
    return (int)current_image_.at<uchar>(point);
}


SearchLineDistanceThresholds ValidLinePointSearch::GetSearchLineDistanceThresholds(int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return SearchLineDistanceThresholds{kMinLeftToMidLineDistance_,kMaxLeftToMidLineDistance_};

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) return SearchLineDistanceThresholds{kMinLeftToRightLineDistance_,kMaxLeftToRightLineDistance_};

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return SearchLineDistanceThresholds{kMinRightToMidLineDistance_,kMaxRightToMidLineDistance_};

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return SearchLineDistanceThresholds{kMinRightToLeftLineDistance_,kMaxRightToLeftLineDistance_};

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return SearchLineDistanceThresholds{kMinMidToLeftLineDistance_,kMaxMidToLeftLineDistance_};

    if(SEARCH_LINE_CODE == MID_TO_RIGHT)return SearchLineDistanceThresholds{kMinMidToRightLineDistance_,kMaxMidToRightLineDistance_};
}



int ValidLinePointSearch::GetMinPixelIntensityThreshold(int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return kMinLeftToMidPixelIntensity_;

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT)return kMinLeftToRightPixelIntensity_;

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return kMinRightToMidPixelIntensity_;

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return kMinRightToLeftPixelIntensity_;

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return kMinMidToRightPixelIntensity_;

    if(SEARCH_LINE_CODE == MID_TO_LEFT)return kMinMidToLeftPixelIntensity_;
}

void ValidLinePointSearch::SearchOrthogonalValues(int point_in_search_direction_x,
                                                  int point_in_search_direction_y,
                                                  float orthogonal_angle,
                                                  vector<int>& orthogonal_line_activations,
                                                  vector<Point>& orthogonal_line_points,
                                                  int SEARCH_LINE_CODE)
{

     SearchLineDistanceThresholds search_line_distance_thresholds = GetSearchLineDistanceThresholds(SEARCH_LINE_CODE);
     int min_pixel_intensity_threshold = GetMinPixelIntensityThreshold(SEARCH_LINE_CODE);

     for(int current_distance=search_line_distance_thresholds.min; current_distance<search_line_distance_thresholds.max; current_distance++)
     {
         int orthogonal_point_in_search_direction_x  = point_in_search_direction_x + current_distance * cos(orthogonal_angle*PI/180);
         int orthogonal_point_in_search_direction_y  = point_in_search_direction_y - current_distance * sin(orthogonal_angle*PI/180);

         Point current_point = Point(orthogonal_point_in_search_direction_x,
                                     orthogonal_point_in_search_direction_y);

         if(orthogonal_point_in_search_direction_x < kImageWidth_ && orthogonal_point_in_search_direction_x >= 0 &&
            orthogonal_point_in_search_direction_y < kImageHeight_ && orthogonal_point_in_search_direction_y >= 0)
         {
             int pixel_value = GetPixelValue(current_point);

             if(pixel_value > min_pixel_intensity_threshold)
             {
                 orthogonal_line_activations.push_back(1);
                 orthogonal_line_points.push_back(current_point);
             }
             else
             {
                 orthogonal_line_activations.push_back(0);
                 orthogonal_line_points.push_back(current_point);
             }
         }
         else
         {
             orthogonal_line_activations.push_back(0);
             orthogonal_line_points.push_back(current_point);
         }
     }
}

SearchLineWidthThresholds ValidLinePointSearch::GetSearchLineWidthThresholds(int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return SearchLineWidthThresholds{kMinLeftToMidLineWidth_,kMaxLeftToMidLineWidth_};

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT)return SearchLineWidthThresholds{kMinLeftToRightLineWidth_,kMaxLeftToRightLineWidth_};

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return SearchLineWidthThresholds{kMinRightToMidLineWidth_,kMaxRightToMidLineWidth_};

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return SearchLineWidthThresholds{kMinRightToLeftLineWidth_,kMaxRightToLeftLineWidth_};

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return SearchLineWidthThresholds{kMinMidToRightLineWidth_,kMaxMidToRightLineWidth_};

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return SearchLineWidthThresholds{kMinMidToLeftLineWidth_,kMaxMidToLeftLineWidth_};
}



bool ValidLinePointSearch::CheckLineMatch(vector<int> orthogonal_line_activations, SegmentStartIDAndWidth& line_match, int SEARCH_LINE_CODE)
{
    vector<SegmentStartIDAndWidth> segments;
    int segment_width = 0;

    // Find start and length of segments in scanned direction
    for(int i=0; i<orthogonal_line_activations.size(); i++)
    {
        segment_width=0;

        if(orthogonal_line_activations[i] > 0)
        {
            int start_id = i;

            while(orthogonal_line_activations[i] > 0 && i <orthogonal_line_activations.size())
            {
                segment_width++;
                i++;
            }
            segments.push_back(SegmentStartIDAndWidth{start_id,segment_width});
        }
    }
    // Check if there are no other segments and if width of midline is correct
    SearchLineWidthThresholds search_line_width_thresholds = GetSearchLineWidthThresholds(SEARCH_LINE_CODE);

    if(segments.size() == 1)
    {
        int width = segments[0].width;

        if(width >= search_line_width_thresholds.min && width <= search_line_width_thresholds.max)
        {
           line_match = segments[0];
           return true;
        }
    }
    return false;
}


void ValidLinePointSearch::SetOuterLineDirections(vector<PointInDirection> line_directions, int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID || SEARCH_LINE_CODE == LEFT_TO_RIGHT) left_line_directions_ = line_directions;

    else if(SEARCH_LINE_CODE == RIGHT_TO_MID || SEARCH_LINE_CODE == RIGHT_TO_LEFT) right_line_directions_ = line_directions;

    else cout << "False Search Line Code (SetOuterLineDirections)!" << endl;
}



void ValidLinePointSearch::FindValidPointsFromMidLineSearch(vector<vector<PointInDirection>> mid_line_directions_clusters, int SEARCH_LINE_CODE)
{
   //ClearMemory(SEARCH_LINE_CODE);

   mid_line_directions_clusters_ = mid_line_directions_clusters;

   for(auto &line_directions: mid_line_directions_clusters_)
   {
       FindValidPoints(line_directions, SEARCH_LINE_CODE);

       if(SEARCH_LINE_CODE == MID_TO_RIGHT)
       {
           mid_to_right_search_info_clusters_.push_back(mid_to_right_search_info_);
           mid_to_right_search_info_.clear();
       }

       if(SEARCH_LINE_CODE == MID_TO_LEFT)
       {
           mid_to_left_search_info_clusters_.push_back(mid_to_left_search_info_);
           mid_to_left_search_info_.clear();
       }
   }

}


double Distance2d(const Point& p, const RightValidationTable& rhs)
{
    return sqrt(pow(p.x-rhs.origin.x,2) + pow(p.y-rhs.origin.y,2));
}


double Distance2d(const Point& p, const LeftValidationTable& rhs)
{
    return sqrt(pow(p.x-rhs.origin.x,2) + pow(p.y-rhs.origin.y,2));
}

double Distance2d(const Point& p, const MidValidationTable& rhs)
{
    return sqrt(pow(p.x-rhs.origin.x,2) + pow(p.y-rhs.origin.y,2));
}

double Distance2d(const Point& p, LineValidationTable* & rhs)
{
    return sqrt(pow(p.x-rhs->GetOriginPoint().x,2) + pow(p.y-rhs->GetOriginPoint().y,2));
}

double Distance2d(const Point p1, const Point p2)
{
    return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));
}




bool ValidLinePointSearch::AdjacentValidationTableIsEmpty(int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return mid_line_validation_table_.empty();

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) return right_line_validation_table_.empty();

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return left_line_validation_table_.empty();

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return right_line_validation_table_.empty();

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return left_line_validation_table_.empty();

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return mid_line_validation_table_.empty();
}

void ValidLinePointSearch::FindMinDistanceFromPredictionToAdjacentPoint(int SEARCH_LINE_CODE,
                                                                        Point adjacent_point_prediction,
                                                                        Point &min_distance_adjacent_point,
                                                                        int &min_distance_adjacent_point_id,
                                                                        int &min_distance)
{
     vector<LineValidationTable*> table;

    if(SEARCH_LINE_CODE == LEFT_TO_MID) table = mid_line_validation_table_;

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) table = right_line_validation_table_;

    if(SEARCH_LINE_CODE == MID_TO_LEFT) table = left_line_validation_table_;

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) table = right_line_validation_table_;

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) table = left_line_validation_table_;

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) table = mid_line_validation_table_;


    auto min_distance_it = std::min_element(begin(table),end(table),
                                             [&](LineValidationTable* & lhs, LineValidationTable* & rhs)
                                             { return Distance2d(adjacent_point_prediction, lhs) < Distance2d(adjacent_point_prediction, rhs); });

    int min_distance_id = std::distance(table.begin(), min_distance_it);
    Point origin = table[min_distance_id]->GetOriginPoint();

    min_distance = Distance2d(origin,adjacent_point_prediction);
    min_distance_adjacent_point = origin;
    min_distance_adjacent_point_id = min_distance_id;

}


int ValidLinePointSearch::GetAdjacentPointDirection(int SEARCH_LINE_CODE, int min_distance_adjacent_point_id)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id]->GetDirection();

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id]->GetDirection();

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id]->GetDirection();

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id]->GetDirection();

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id]->GetDirection();

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id]->GetDirection();
}





Point ValidLinePointSearch::GetAdjacentPointOriginPrediction(int SEARCH_LINE_CODE, int min_distance_adjacent_point_id)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id]->GetLeftLinePointPrediction();

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id]->GetLeftLinePointPrediction();

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id]->GetMidLinePointPrediction();

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id]->GetMidLinePointPrediction();

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id]->GetRightLinePointPrediction();

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id]->GetRightLinePointPrediction();
}


void ValidLinePointSearch::ExamineValidationTable(int SEARCH_LINE_CODE, vector<LineValidationTable*>&table)
{
    for(int i=0; i<table.size(); i++)
    {
        Point adjacent_point_prediction = table[i]->GetAdjacentPointPrediction(SEARCH_LINE_CODE);

        if(adjacent_point_prediction == EMPTY_POINT_){ continue;}
        else{table[i]->SetAdjacentPointPredictionFound(SEARCH_LINE_CODE,true);}

        if(AdjacentValidationTableIsEmpty(SEARCH_LINE_CODE)){continue;}

        Point min_distance_adjacent_point;
        int min_distance_adjacent_point_id;
        int min_distance;

        FindMinDistanceFromPredictionToAdjacentPoint(SEARCH_LINE_CODE,adjacent_point_prediction,min_distance_adjacent_point,min_distance_adjacent_point_id,min_distance);

        if(min_distance < kMaxPointDistance_)
        {
            table[i]->SetAdjacentPointPrediction(SEARCH_LINE_CODE,true);
            table[i]->SetAdjacentPointId(SEARCH_LINE_CODE,min_distance_adjacent_point_id);
        }
        else {continue;}

        int adjacent_point_direction = GetAdjacentPointDirection(SEARCH_LINE_CODE,min_distance_adjacent_point_id);
        int origin_point_direction   = table[i]->GetDirection();

        int direction_difference = abs(adjacent_point_direction - origin_point_direction);

        if(direction_difference < kMaxDirectionDifference_ || (360 - kMaxDirectionDifference_) < direction_difference)
        {
            table[i]->SetAdjacentPointDirectionInRange(SEARCH_LINE_CODE,true);
        }

        Point origin_prediction = GetAdjacentPointOriginPrediction(SEARCH_LINE_CODE,min_distance_adjacent_point_id);

        if(origin_prediction == EMPTY_POINT_){ continue; }

        Point origin  = table[i]->GetOriginPoint();
        double point_distance = Distance2d(origin,origin_prediction);

        if(point_distance < kMaxPointDistance_)
        {
            table[i]->SetAdjacentPointOriginPrediction(SEARCH_LINE_CODE,true);
        }


    }
}


void ValidLinePointSearch::SearchValidPoints()
{
/*
    cout << "rs: " << right_points_validation_table_.size() << endl;
    cout << "ls: " << left_points_validation_table_.size() << endl;
   cout << "ms: " << mid_points_validation_table_.size() << endl;
*/


    // right to left point validation


    ExamineValidationTable(LEFT_TO_MID, left_line_validation_table_);

/*
    for(int i=0; i<right_points_validation_table_.size(); i++)
    {
            Point left  = right_points_validation_table_[i].left;

            if(left == Point(-1,-1)){ continue; }
            else { right_points_validation_table_[i].found_left = true; }


            if(left_points_validation_table_.empty()){ continue; }

            auto left_min_distance_it = std::min_element(begin(left_points_validation_table_),end(left_points_validation_table_),
                                                     [&](const LeftValidationTable& lhs, const LeftValidationTable& rhs)
                                                     { return Distance2d(left, lhs) < Distance2d(left, rhs); });

            int left_min_distance_id = std::distance(left_points_validation_table_.begin(), left_min_distance_it);
            Point left_origin = left_points_validation_table_[left_min_distance_id].origin;


            double point_distance = Distance2d(left,left_origin);

            if(point_distance < kMaxPointDistance){ right_points_validation_table_[i].left_near_left_origin = true; }
            else { continue; }


            int left_search_direction  = left_points_validation_table_[left_min_distance_id].search_direction;
            int right_search_direction = right_points_validation_table_[i].search_direction;



            int direction_difference = abs(left_search_direction - right_search_direction);

            if(direction_difference < kMaxDirectionDifference || (360 - kMaxDirectionDifference) < direction_difference)
            {
                right_points_validation_table_[i].left_origin_equal_direction = true;
            }


            Point left_to_right = left_points_validation_table_[left_min_distance_id].right;

            if(left_to_right == Point(-1,-1)){ continue; }

            Point origin  = right_points_validation_table_[i].origin;

            point_distance = Distance2d(origin,left_to_right);
            if(point_distance < kMaxPointDistance) right_points_validation_table_[i].origin_near_left_to_right = true;

    }


    // right to mid point validation
    for(int i=0; i<right_points_validation_table_.size(); i++)
    {


        Point mid  = right_points_validation_table_[i].mid;

        if(mid == Point(-1,-1)){ continue; }
        else { right_points_validation_table_[i].found_mid = true; }


        if(mid_points_validation_table_.empty()){ continue; }

        auto mid_min_distance_it = std::min_element(begin(mid_points_validation_table_),end(mid_points_validation_table_),
                                                 [&](const MidValidationTable& lhs, const MidValidationTable& rhs)
                                                 { return Distance2d(mid, lhs) < Distance2d(mid, rhs); });

        int mid_min_distance_id = std::distance(mid_points_validation_table_.begin(), mid_min_distance_it);
        Point mid_origin = mid_points_validation_table_[mid_min_distance_id].origin;


        double point_distance = Distance2d(mid,mid_origin);

        if(point_distance < kMaxPointDistance){ right_points_validation_table_[i].mid_near_mid_origin = true; }
        else { continue; }


        int mid_search_direction  = mid_points_validation_table_[mid_min_distance_id].search_direction;
        int right_search_direction = right_points_validation_table_[i].search_direction;



        int direction_difference = abs(mid_search_direction - right_search_direction);

        if(direction_difference < kMaxDirectionDifference || (360 - kMaxDirectionDifference) < direction_difference)
        {
            right_points_validation_table_[i].mid_origin_equal_direction = true;
        }


        Point mid_to_right = mid_points_validation_table_[mid_min_distance_id].right;

        if(mid_to_right == Point(-1,-1)){ continue; }

        Point origin  = right_points_validation_table_[i].origin;

        point_distance = Distance2d(origin,mid_to_right);
        if(point_distance < kMaxPointDistance) right_points_validation_table_[i].origin_near_mid_to_right = true;

    }


    // mid to left point validation
    for(int i=0; i<mid_points_validation_table_.size(); i++)
    {


        Point left  = mid_points_validation_table_[i].left;

        if(left == Point(-1,-1)){ continue; }
        else { mid_points_validation_table_[i].found_left = true; }


        if(left_points_validation_table_.empty()){ continue; }

        auto left_min_distance_it = std::min_element(begin(left_points_validation_table_),end(left_points_validation_table_),
                                                 [&](const LeftValidationTable& lhs, const LeftValidationTable& rhs)
                                                 { return Distance2d(left, lhs) < Distance2d(left, rhs); });

        int left_min_distance_id = std::distance(left_points_validation_table_.begin(), left_min_distance_it);
        Point left_origin =  left_points_validation_table_[left_min_distance_id].origin;


        double point_distance = Distance2d(left,left_origin);

        if(point_distance < kMaxPointDistance){ mid_points_validation_table_[i].left_near_left_origin = true; }
        else { continue; }


        int left_search_direction  = left_points_validation_table_[left_min_distance_id].search_direction;
        int mid_search_direction = mid_points_validation_table_[i].search_direction;



        int direction_difference = abs(left_search_direction - mid_search_direction);

        if(direction_difference < kMaxDirectionDifference || (360 - kMaxDirectionDifference) < direction_difference)
        {
            mid_points_validation_table_[i].left_origin_equal_direction = true;
        }


        Point left_to_mid = left_points_validation_table_[left_min_distance_id].mid;

        if(left_to_mid == Point(-1,-1)){ continue; }

        Point origin  = mid_points_validation_table_[i].origin;

        point_distance = Distance2d(origin,left_to_mid);
        if(point_distance < kMaxPointDistance) mid_points_validation_table_[i].origin_near_left_to_mid = true;

    }

    // mid to right point validation
    for(int i=0; i<mid_points_validation_table_.size(); i++)
    {


        Point right  = mid_points_validation_table_[i].right;

        if(right == Point(-1,-1)){ continue; }
        else { mid_points_validation_table_[i].found_right = true; }


        if(right_points_validation_table_.empty()){ continue; }

        auto right_min_distance_it = std::min_element(begin(right_points_validation_table_),end(right_points_validation_table_),
                                                 [&](const RightValidationTable& lhs, const RightValidationTable& rhs)
                                                 { return Distance2d(right, lhs) < Distance2d(right, rhs); });

        int right_min_distance_id = std::distance(right_points_validation_table_.begin(), right_min_distance_it);
        Point right_origin =  right_points_validation_table_[right_min_distance_id].origin;


        double point_distance = Distance2d(right,right_origin);

        if(point_distance < kMaxPointDistance){ mid_points_validation_table_[i].right_near_right_origin = true; }
        else { continue; }


        int right_search_direction  = right_points_validation_table_[right_min_distance_id].search_direction;
        int mid_search_direction = mid_points_validation_table_[i].search_direction;



        int direction_difference = abs(right_search_direction - mid_search_direction);

        if(direction_difference < kMaxDirectionDifference || (360 - kMaxDirectionDifference) < direction_difference)
        {
            mid_points_validation_table_[i].right_origin_equal_direction = true;
        }


        Point right_to_mid = right_points_validation_table_[right_min_distance_id].mid;

        if(right_to_mid == Point(-1,-1)){ continue; }

        Point origin  = mid_points_validation_table_[i].origin;

        point_distance = Distance2d(origin,right_to_mid);
        if(point_distance < kMaxPointDistance) mid_points_validation_table_[i].origin_near_right_to_mid = true;

    }


    // left to right point validation
    for(int i=0; i<left_points_validation_table_.size(); i++)
    {


        Point right  = left_points_validation_table_[i].right;

        if(right == Point(-1,-1)){ continue; }
        else { left_points_validation_table_[i].found_right = true; }


        if(right_points_validation_table_.empty()){ continue; }

        auto right_min_distance_it = std::min_element(begin(right_points_validation_table_),end(right_points_validation_table_),
                                                 [&](const RightValidationTable& lhs, const RightValidationTable& rhs)
                                                 { return Distance2d(right, lhs) < Distance2d(right, rhs); });

        int right_min_distance_id = std::distance(right_points_validation_table_.begin(), right_min_distance_it);
        Point right_origin =  right_points_validation_table_[right_min_distance_id].origin;


        double point_distance = Distance2d(right,right_origin);

        if(point_distance < kMaxPointDistance){ left_points_validation_table_[i].right_near_right_origin = true; }
        else { continue; }


        int right_search_direction  = right_points_validation_table_[right_min_distance_id].search_direction;
        int left_search_direction = left_points_validation_table_[i].search_direction;



        int direction_difference = abs(right_search_direction - left_search_direction);

        if(direction_difference < kMaxDirectionDifference || (360 - kMaxDirectionDifference) < direction_difference)
        {
            left_points_validation_table_[i].right_origin_equal_direction = true;
        }


        Point right_to_left = right_points_validation_table_[right_min_distance_id].left;

        if(right_to_left == Point(-1,-1)){ continue; }

        Point origin  = left_points_validation_table_[i].origin;

        point_distance = Distance2d(origin,right_to_left);
        if(point_distance < kMaxPointDistance) left_points_validation_table_[i].origin_near_right_to_left = true;

    }



    // left to mid point validation
    for(int i=0; i<left_points_validation_table_.size(); i++)
    {


        Point mid  = left_points_validation_table_[i].mid;

        if(mid == Point(-1,-1)){ continue; }
        else { left_points_validation_table_[i].found_mid = true; }


        if(mid_points_validation_table_.empty()){ continue; }

        auto mid_min_distance_it = std::min_element(begin(mid_points_validation_table_),end(mid_points_validation_table_),
                                                 [&](const MidValidationTable& lhs, const MidValidationTable& rhs)
                                                 { return Distance2d(mid, lhs) < Distance2d(mid, rhs); });

        int mid_min_distance_id = std::distance(mid_points_validation_table_.begin(), mid_min_distance_it);
        Point mid_origin = mid_points_validation_table_[mid_min_distance_id].origin;


        double point_distance = Distance2d(mid,mid_origin);

        if(point_distance < kMaxPointDistance){ left_points_validation_table_[i].mid_near_mid_origin = true; }
        else { continue; }


        int mid_search_direction  = mid_points_validation_table_[mid_min_distance_id].search_direction;
        int left_search_direction = left_points_validation_table_[i].search_direction;



        int direction_difference = abs(mid_search_direction - left_search_direction);

        if(direction_difference < kMaxDirectionDifference || (360 - kMaxDirectionDifference) < direction_difference)
        {
            left_points_validation_table_[i].mid_origin_equal_direction = true;
        }


        Point mid_to_left = mid_points_validation_table_[mid_min_distance_id].left;

        if(mid_to_left == Point(-1,-1)){ continue; }

        Point origin  = left_points_validation_table_[i].origin;

        point_distance = Distance2d(origin,mid_to_left);
        if(point_distance < kMaxPointDistance) left_points_validation_table_[i].origin_near_mid_to_left = true;

    }

*/

}

MinMaxLineElements ValidLinePointSearch::GetLinesMinMaxElements(vector<Point> line)
{


    auto x_it = minmax_element(begin(line),end(line),
                                        [&](const Point& p1, const Point& p2)
                                        { return p1.x < p2.x; });

    auto y_it = minmax_element(begin(line),end(line),
                                        [&](const Point& p1, const Point& p2)
                                        { return p1.y < p2.y; });


    int x_min_id = std::distance(line.begin(), x_it.first);
    int x_max_id = std::distance(line.begin(), x_it.second);
    int y_min_id = std::distance(line.begin(), y_it.first);
    int y_max_id = std::distance(line.begin(), y_it.second);



    return MinMaxLineElements{line[x_min_id],
                                    line[x_max_id],
                                    line[y_min_id],
                                    line[y_max_id]};

}






void ValidLinePointSearch::SearchMinMax()
{
/*
    vector<pair<int,int>> line_point_connections;
    cout << "r size: " << r.size() << endl;

    if(r.size() > 2)
    {
         FindLinePointConnections(r,line_point_connections);


         int num_vertices = r.size();

         DepthFirstSearch ConnectedMidLineGroupFinder(num_vertices);

         for(auto it:line_point_connections){ ConnectedMidLineGroupFinder.addEdge(it.first, it.second); }

         vector<vector<int>> connected_mid_line_groups = ConnectedMidLineGroupFinder.connectedComponents();


         cout << "num_groups: " << connected_mid_line_groups.size() << endl;
    }
*/

    cout << "r: " << r.size() << endl;
    cout << "l: " << l.size() << endl;
    cout << "m: " << m.size() << endl;



    MinMaxLineElements rminmax = GetLinesMinMaxElements(r);
    MinMaxLineElements mminmax = GetLinesMinMaxElements(m);
    MinMaxLineElements lminmax = GetLinesMinMaxElements(l);



    //cout << "r_x_min: " << rminmax.x_min << endl;
    //cout << "r_x_max: " << rminmax.x_max << endl;
    //cout << "r_y_min: " << rminmax.y_min << endl;
    //cout << "r_y_max: " << rminmax.y_max << endl;
}



void ValidLinePointSearch::MergePoints()
{



    for(int i=0; i<right_points_validation_table_.size(); i++)
    {

        Point origin_point = right_points_validation_table_[i].origin;
        Point left_point   = right_points_validation_table_[i].left;
        Point mid_point    = right_points_validation_table_[i].mid;

        bool left      = right_points_validation_table_[i].found_left;
        bool mid       = right_points_validation_table_[i].found_mid;
        bool left_near = right_points_validation_table_[i].left_near_left_origin;
        bool mid_near  = right_points_validation_table_[i].mid_near_mid_origin;

        int  next_direction_distance = right_points_validation_table_[i].next_direction_distance;
        int  search_direction = right_points_validation_table_[i].search_direction;


       if(left && mid && left_near && mid_near)
       {
           r_info.push_back(ValidPoints{RIGHT_LINE, origin_point, true, true, false, next_direction_distance, search_direction});

            //l.push_back(left_point);
           // m.push_back(mid_point);
          r.push_back(origin_point);
       }
       /*else if(left && mid && !left_near && mid_near)
       {
           l.push_back(left_point);
           //m.push_back(mid_point);
           r.push_back(origin_point);
       }*/
       else if(left && left_near)
       {
           r_info.push_back(ValidPoints{RIGHT_LINE, origin_point, true, false, false, next_direction_distance, search_direction});

           //l.push_back(left_point);
           r.push_back(origin_point);

       }
       else if(mid && mid_near)
       {
           r_info.push_back(ValidPoints{RIGHT_LINE, origin_point, false, true, false, next_direction_distance, search_direction});

           //m.push_back(mid_point);
           r.push_back(origin_point);
       }

    }

    for(int i=0; i<left_points_validation_table_.size(); i++)
    {

        Point origin_point = left_points_validation_table_[i].origin;
        Point right_point   = left_points_validation_table_[i].right;
        Point mid_point    = left_points_validation_table_[i].mid;

        bool right      = left_points_validation_table_[i].found_right;
        bool mid       = left_points_validation_table_[i].found_mid;
        bool right_near = left_points_validation_table_[i].right_near_right_origin;
        bool mid_near  = left_points_validation_table_[i].mid_near_mid_origin;

        int  next_direction_distance = left_points_validation_table_[i].next_direction_distance;
        int  search_direction = left_points_validation_table_[i].search_direction;


       if(mid && right && right_near && mid_near)
       {
            l_info.push_back(ValidPoints{LEFT_LINE, origin_point, false, true, true, next_direction_distance, search_direction});
            l.push_back(origin_point);
            //m.push_back(mid_point);
            //r.push_back(right_point);
       }
       else if(right && right_near)
       {
           l_info.push_back(ValidPoints{LEFT_LINE, origin_point, false, false, true, next_direction_distance, search_direction});
           l.push_back(origin_point);
           //r.push_back(right_point);
       }
       else if(mid && mid_near)
       {
           l_info.push_back(ValidPoints{LEFT_LINE, origin_point, false, true, false, next_direction_distance, search_direction});
           l.push_back(origin_point);
           //m.push_back(mid_point);
       }

    }

    for(int i=0; i<mid_points_validation_table_.size(); i++)
    {

        Point origin_point = mid_points_validation_table_[i].origin;
        Point right_point   = mid_points_validation_table_[i].right;
        Point left_point    = mid_points_validation_table_[i].left;

        bool right      = mid_points_validation_table_[i].found_right;
        bool left       = mid_points_validation_table_[i].found_left;
        bool right_near = mid_points_validation_table_[i].right_near_right_origin;
        bool left_near  = mid_points_validation_table_[i].left_near_left_origin;

        int  next_direction_distance = mid_points_validation_table_[i].next_direction_distance;
        int  search_direction = mid_points_validation_table_[i].search_direction;


       if(left && right && right_near && left_near)
       {
           m_info.push_back(ValidPoints{MID_LINE, origin_point, true, false, true, next_direction_distance, search_direction});
            //l.push_back(left_point);
            m.push_back(origin_point);
            //r.push_back(right_point);
       }
       else if(right && right_near)
       {
           m_info.push_back(ValidPoints{MID_LINE, origin_point, false, false, true, next_direction_distance, search_direction});
           m.push_back(origin_point);
           //r.push_back(right_point);

       }
       else if(left && left_near)
       {
            m_info.push_back(ValidPoints{MID_LINE, origin_point, true, false, false, next_direction_distance, search_direction});
           //l.push_back(left_point);
           m.push_back(origin_point);

       }

    }



}


void ValidLinePointSearch::JJ()
{

    int tmp_direction = m_info[0].search_direction;
    for(int i=0; i<m_info.size(); i++)
    {

        int direction =  m_info[i].search_direction;

        if(abs(tmp_direction - direction) > 40)
        {
            cout << tmp_direction << " " << direction << endl;
            cout <<"mid: " << m_info[i].origin<< "\t" << m_info[i].next_directions_distance<< "\t" << m_info[i].search_direction << "\t" << m_info[i].left << "\t" << m_info[i].right << endl;

        }

        tmp_direction = direction;

    }


    tmp_direction = l_info[0].search_direction;

    for(int i=0; i<l_info.size(); i++)
    {

        int direction =  l_info[i].search_direction;

        if(abs(tmp_direction - direction) > 40)
        {
                        cout << tmp_direction << " " << direction << endl;
            cout <<"left: " <<l_info[i].origin<< "\t" << l_info[i].next_directions_distance<< "\t" << l_info[i].search_direction << "\t" << l_info[i].left << "\t" << l_info[i].right << endl;

        }

        tmp_direction = direction;

    }



    tmp_direction = r_info[0].search_direction;

    for(int i=0; i<r_info.size(); i++)
    {

        int direction =  r_info[i].search_direction;

        int direction_difference = abs(tmp_direction - direction);



        if(direction_difference > 40 || (360 - 40) > direction_difference)
        {
                        cout << tmp_direction << " " << direction << endl;
            cout <<"right: "<< r_info[i].origin<< "\t" << r_info[i].next_directions_distance<< "\t" << r_info[i].search_direction << "\t" << r_info[i].left << "\t" << r_info[i].right << endl;

        }

        tmp_direction = direction;

    }





}



void ValidLinePointSearch::ComputePointScores()
{

    for(int i=0; i<right_points_validation_table_.size(); i++)
    {
       int score = 0;
       if(right_points_validation_table_[i].found_left) score++;
       if(right_points_validation_table_[i].found_mid) score++;
       if(right_points_validation_table_[i].left_near_left_origin) score++;
       if(right_points_validation_table_[i].mid_near_mid_origin) score++;
       if(right_points_validation_table_[i].left_origin_equal_direction) score++;
       if(right_points_validation_table_[i].mid_origin_equal_direction) score++;
       if(right_points_validation_table_[i].origin_near_left_to_right) score++;
       if(right_points_validation_table_[i].origin_near_mid_to_right) score++;

       right_points_validation_table_[i].score = score;
    }



    for(int i=0; i<mid_points_validation_table_.size(); i++)
    {
       int score = 0;
       if(mid_points_validation_table_[i].found_left) score++;
       if(mid_points_validation_table_[i].found_right) score++;
       if(mid_points_validation_table_[i].left_near_left_origin) score++;
       if(mid_points_validation_table_[i].right_near_right_origin) score++;
       if(mid_points_validation_table_[i].left_origin_equal_direction) score++;
       if(mid_points_validation_table_[i].right_origin_equal_direction) score++;
       if(mid_points_validation_table_[i].origin_near_left_to_mid) score++;
       if(mid_points_validation_table_[i].origin_near_right_to_mid) score++;

       mid_points_validation_table_[i].score = score;
    }

    for(int i=0; i<left_points_validation_table_.size(); i++)
    {
       int score = 0;
       if(left_points_validation_table_[i].found_mid) score++;
       if(left_points_validation_table_[i].found_right) score++;
       if(left_points_validation_table_[i].mid_near_mid_origin) score++;
       if(left_points_validation_table_[i].right_near_right_origin) score++;
       if(left_points_validation_table_[i].mid_origin_equal_direction) score++;
       if(left_points_validation_table_[i].right_origin_equal_direction) score++;
       if(left_points_validation_table_[i].origin_near_mid_to_left) score++;
       if(left_points_validation_table_[i].origin_near_right_to_left) score++;

       left_points_validation_table_[i].score = score;
    }

}


void ValidLinePointSearch::DrawTables(Mat &rgb)
{
    for(int i=0; i<left_line_validation_table_.size(); i++)
    {
        if(left_line_validation_table_[i]->GetMidPrediction())// && left_line_validation_table_[i]->GetRightPrediction())
        {
            circle(rgb, left_line_validation_table_[i]->GetOriginPoint(), 1, Scalar(0, 0, 255),CV_FILLED );
        }
    }
}

void ValidLinePointSearch::DrawValidScorePoints(Mat &rgb)
{

    const int kMinScore = 0;

    for(int i=0; i<right_points_validation_table_.size(); i++)
    {
        if(right_points_validation_table_[i].score >= kMinScore && right_points_validation_table_[i].found_mid == true && right_points_validation_table_[i].found_left == true)
        {
            circle(rgb, right_points_validation_table_[i].origin, 1, Scalar(0, 0, 255),CV_FILLED );
            //circle(rgb, right_points_validation_table_[i].left, 4, Scalar(0, 0, 255),CV_FILLED );
            //circle(rgb, right_points_validation_table_[i].mid, 4, Scalar(0, 0, 255),CV_FILLED );
        }
    }

    for(int i=0; i<mid_points_validation_table_.size(); i++)
    {
        if(mid_points_validation_table_[i].score >= kMinScore && mid_points_validation_table_[i].found_right == true && mid_points_validation_table_[i].found_left == true)
        {
            circle(rgb, mid_points_validation_table_[i].origin, 1, Scalar(0, 255, 0),CV_FILLED );
            //circle(rgb, mid_points_validation_table_[i].left, 4, Scalar(0, 255, 0),CV_FILLED );
            //circle(rgb, mid_points_validation_table_[i].right, 4, Scalar(0, 255, 0),CV_FILLED );
        }
    }


    for(int i=0; i<left_points_validation_table_.size(); i++)
    {
        if(left_points_validation_table_[i].score >= kMinScore && left_points_validation_table_[i].found_right == true && left_points_validation_table_[i].found_mid == true)
        {
            circle(rgb, left_points_validation_table_[i].origin, 1, Scalar(255, 0, 0),CV_FILLED );
            //circle(rgb, left_points_validation_table_[i].mid, 4, Scalar(255, 0, 0),CV_FILLED );
            //circle(rgb, left_points_validation_table_[i].right, 4, Scalar(255, 0, 0),CV_FILLED );
        }
    }

}

void ValidLinePointSearch::DrawMergedPoints(Mat &rgb)
{
    for(int i=0; i<l.size(); i++)
    {

        circle(rgb, l[i], 1, Scalar(255, 0, 0),CV_FILLED );

    }

    for(int i=0; i<m.size(); i++)
    {

        circle(rgb, m[i], 1, Scalar(0, 255, 0),CV_FILLED );

    }

    for(int i=0; i<r.size(); i++)
    {

        circle(rgb, r[i], 1, Scalar(0, 0, 255),CV_FILLED );

    }
}





void ValidLinePointSearch::CreateValidationTables()
{


    for(int i=0; i<left_to_mid_search_info_.size(); i++)
    {
        left_line_validation_table_.push_back(new LineValidationTable(LEFT_LINE,
                                                                  left_to_mid_search_info_[i].origin,
                                                                  left_to_mid_search_info_[i].search_direction,
                                                                  left_to_mid_search_info_[i].next_direction_distance,
                                                                  left_to_mid_search_info_[i].adjacent_line_point,
                                                                  left_to_right_search_info_[i].adjacent_line_point,
                                                                  0));
    }

    for(int i=0; i<mid_to_left_search_info_clusters_.size(); i++)
    {
        for(int j=0; j<mid_to_left_search_info_clusters_[i].size(); j++)
        {
            mid_line_validation_table_.push_back(new LineValidationTable(MID_LINE,
                                                                      mid_to_left_search_info_clusters_[i][j].origin,
                                                                      mid_to_left_search_info_clusters_[i][j].search_direction,
                                                                      mid_to_left_search_info_clusters_[i][j].next_direction_distance,
                                                                      mid_to_left_search_info_clusters_[i][j].adjacent_line_point,
                                                                      mid_to_right_search_info_clusters_[i][j].adjacent_line_point,
                                                                      i));
        }
    }




    for(int i=0; i<right_to_left_search_info_.size(); i++)
    {
        right_line_validation_table_.push_back(new LineValidationTable(RIGHT_LINE,
                                                                  right_to_left_search_info_[i].origin,
                                                                  right_to_left_search_info_[i].search_direction,
                                                                  right_to_left_search_info_[i].next_direction_distance,
                                                                  right_to_left_search_info_[i].adjacent_line_point,
                                                                  right_to_mid_search_info_[i].adjacent_line_point,
                                                                  0));
    }

}


void ValidLinePointSearch::ClearValidationTables()
{
    right_points_validation_table_.clear();
    mid_points_validation_table_.clear();
    left_points_validation_table_.clear();

    left_line_directions_.clear();
    right_line_directions_.clear();
    mid_line_directions_clusters_.clear();

    left_to_mid_search_info_.clear();
    left_to_right_search_info_.clear();

    right_to_mid_search_info_.clear();
    right_to_left_search_info_.clear();

    mid_to_right_search_info_.clear();
    mid_to_left_search_info_.clear();

    mid_to_right_search_info_clusters_.clear();
    mid_to_left_search_info_clusters_.clear();


    left_line_validation_table_.clear();
      mid_line_validation_table_.clear();
          right_line_validation_table_.clear();

    l.clear();
    m.clear();
    r.clear();


    l_info.clear();
   m_info.clear();
    r_info.clear();

}
void ValidLinePointSearch::FindValidPointsFromLineFollow(vector<PointInDirection> line_directions, int SEARCH_LINE_CODE)
{
    //ClearMemory(SEARCH_LINE_CODE);

    SetOuterLineDirections(line_directions,SEARCH_LINE_CODE);

    FindValidPoints(line_directions, SEARCH_LINE_CODE);



}


void ValidLinePointSearch::FindValidPoints(vector<PointInDirection> line_directions, int SEARCH_LINE_CODE)
{
    for(int i=0; i<line_directions.size(); i++)
    {
        int   current_point_x       = line_directions[i].x;
        int   current_point_y       = line_directions[i].y;
        float angle_to_next_point   = line_directions[i].angle;
        int   length_to_next_point  = line_directions[i].length;

        float orthogonal_angle = GetOrthogonalAngle(angle_to_next_point, SEARCH_LINE_CODE);

        for(int current_length  = 0; current_length < length_to_next_point; current_length++)
        {
             int point_in_search_direction_x = current_point_x + current_length * cos(angle_to_next_point*PI/180);
             int point_in_search_direction_y = current_point_y - current_length * sin(angle_to_next_point*PI/180);
             int current_to_next_point_distance = length_to_next_point - current_length;

            vector<int> orthogonal_line_activations;
            vector<Point> orthogonal_line_points;

            SearchOrthogonalValues(point_in_search_direction_x,
                                   point_in_search_direction_y,
                                   orthogonal_angle,
                                   orthogonal_line_activations,
                                   orthogonal_line_points,
                                   SEARCH_LINE_CODE);

            SegmentStartIDAndWidth line_match;

            bool is_matched = CheckLineMatch(orthogonal_line_activations,line_match,SEARCH_LINE_CODE);

            SafeLinePoint(line_match, orthogonal_line_points,is_matched,point_in_search_direction_x,
                          point_in_search_direction_y,current_to_next_point_distance,angle_to_next_point,SEARCH_LINE_CODE);

        }
    }
}


void ValidLinePointSearch::SafeLinePoint(SegmentStartIDAndWidth line_match, vector<Point> orthogonal_line_points, bool is_matched,
                                         int point_in_search_direction_x,int point_in_search_direction_y, int current_to_next_point_distance,
                                         float angle_to_next_point,int SEARCH_LINE_CODE)
{
    Point origin(point_in_search_direction_x,point_in_search_direction_y);
    float search_direction = angle_to_next_point;
    int distance_to_next_direction = current_to_next_point_distance;

    Point matched_point;

    if(is_matched)
    {
        int width = line_match.width;
        int start_id = line_match.start_id;

        int end_id = start_id + width;


        int x_mean = 0;
        int y_mean = 0;


        for(int i=start_id; i<end_id; i++)
        {
            x_mean += orthogonal_line_points[i].x;
            y_mean += orthogonal_line_points[i].y;
        }

        x_mean /= width;
        y_mean /= width;

        matched_point = Point(x_mean,y_mean);

    }
    else {
        matched_point = Point(-1,-1);
    }

    if(SEARCH_LINE_CODE == LEFT_TO_MID) left_to_mid_search_info_.push_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) left_to_right_search_info_.push_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) right_to_mid_search_info_.push_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) right_to_left_search_info_.push_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) mid_to_right_search_info_.push_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == MID_TO_LEFT) mid_to_left_search_info_.push_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});
}

void ValidLinePointSearch::DrawLinePoints(Mat &rgb, int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID)
    {
        for(auto it: left_to_mid_search_info_)
            circle(rgb, it.adjacent_line_point, 4, Scalar(0, 0, 255),CV_FILLED );
    }

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT)
    {
        for(auto it: left_to_right_search_info_)
            circle(rgb, it.adjacent_line_point, 4, Scalar(0, 0, 255),CV_FILLED );
    }


    if(SEARCH_LINE_CODE == RIGHT_TO_MID)
    {
        for(auto it: right_to_mid_search_info_)
            circle(rgb, it.adjacent_line_point, 2, Scalar(0, 255, 0),CV_FILLED );
    }

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT)
    {
        for(auto it: right_to_left_search_info_)
            circle(rgb, it.adjacent_line_point, 4, Scalar(0, 255, 0),CV_FILLED );
    }

    if(SEARCH_LINE_CODE == MID_TO_LEFT)
    {
        /*std::vector<Vec3b> colors(mid_to_left_search_info_clusters_.size());

        for (int label = 0; label < mid_to_left_search_info_clusters_.size(); ++label)
        {
            colors[label] = Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
        }
        */
        //int color_id = 0;
        for(auto i: mid_to_left_search_info_clusters_)
        {
            for(auto ii: i)
            {
                circle(rgb, ii.adjacent_line_point, 2,  Scalar(255, 0, 0)/*colors[color_id]*/,CV_FILLED );
            }
            //color_id++;
        }
    }

    if(SEARCH_LINE_CODE == MID_TO_RIGHT)
    {

       /* std::vector<Vec3b> colors(mid_to_right_search_info_clusters_.size());

        for (int label = 0; label < mid_to_right_search_info_clusters_.size(); ++label)
        {
            colors[label] = Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
        }
        */
        //int color_id = 0;
        for(auto i: mid_to_right_search_info_clusters_)
        {
            for(auto ii: i)
            {
                circle(rgb, ii.adjacent_line_point, 2, Scalar(255, 0, 0)/*colors[color_id]*/,CV_FILLED );
            }
            //color_id++;
        }
    }
}

void ValidLinePointSearch::SetImage(Mat image)
{
    current_image_ = image;
}


void ValidLinePointSearch::ClearMemory(int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) left_to_mid_search_info_.clear();

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) left_to_right_search_info_.clear();

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) right_to_mid_search_info_.clear();

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) right_to_left_search_info_.clear();

    if(SEARCH_LINE_CODE == MID_TO_RIGHT)
    {
        mid_to_right_search_info_clusters_.clear();
        mid_to_right_search_info_.clear();
    }

    if(SEARCH_LINE_CODE == MID_TO_LEFT)
    {
        mid_to_left_search_info_clusters_.clear();
        mid_to_left_search_info_.clear();
    }

}
