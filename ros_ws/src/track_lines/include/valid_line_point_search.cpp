#include "valid_line_point_search.h"


ValidLinePointSearch::ValidLinePointSearch()
{
    left_line_minmax_elements_.initialized = false;
    mid_line_minmax_elements_.initialized = false;
    right_line_minmax_elements_.initialized = false;

    left_line_last_adjacent_point_match_.last_left_point = Point(-1,-1);
    left_line_last_adjacent_point_match_.last_mid_point = Point(-1,-1);
    left_line_last_adjacent_point_match_.last_right_point = Point(-1,-1);
    left_line_last_adjacent_point_match_.left_set =  false;
    left_line_last_adjacent_point_match_.mid_set =  false;
    left_line_last_adjacent_point_match_.right_set =  false;


    mid_line_last_adjacent_point_match_.last_left_point = Point(-1,-1);
    mid_line_last_adjacent_point_match_.last_mid_point = Point(-1,-1);
    mid_line_last_adjacent_point_match_.last_right_point  = Point(-1,-1);
    mid_line_last_adjacent_point_match_.left_set =  false;
    mid_line_last_adjacent_point_match_.mid_set =  false;
    mid_line_last_adjacent_point_match_.right_set =  false;


    right_line_last_adjacent_point_match_.last_left_point = Point(-1,-1);
    right_line_last_adjacent_point_match_.last_mid_point = Point(-1,-1);
    right_line_last_adjacent_point_match_.last_right_point  = Point(-1,-1);
    right_line_last_adjacent_point_match_.left_set =  false;
    right_line_last_adjacent_point_match_.mid_set =  false;
    right_line_last_adjacent_point_match_.right_set =  false;

    namedWindow("im", WINDOW_NORMAL);
    setMouseCallback("im", mouse_callback);

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

     for(int current_distance=search_line_distance_thresholds.min; current_distance < search_line_distance_thresholds.max; current_distance++)
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
                 orthogonal_line_activations.emplace_back(1);
                 orthogonal_line_points.emplace_back(current_point);
             }
             else
             {
                 orthogonal_line_activations.emplace_back(0);
                 orthogonal_line_points.emplace_back(current_point);
             }
         }
         else
         {
             orthogonal_line_activations.emplace_back(0);
             orthogonal_line_points.emplace_back(current_point);
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
            segments.emplace_back(SegmentStartIDAndWidth{start_id,segment_width});
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
           mid_to_right_search_info_clusters_.emplace_back(mid_to_right_search_info_);
           mid_to_right_search_info_.clear();
       }

       if(SEARCH_LINE_CODE == MID_TO_LEFT)
       {
           mid_to_left_search_info_clusters_.emplace_back(mid_to_left_search_info_);
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

double ValidLinePointSearch::Distance2d(const Point& p, LineValidationTable  hs)
{
    return sqrt(pow(p.x-hs.GetOriginPoint().x,2) + pow(p.y-hs.GetOriginPoint().y,2));
}

double ValidLinePointSearch::Distance2d(const Point p1, const Point p2)
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

     vector<LineValidationTable> table;

    if(SEARCH_LINE_CODE == LEFT_TO_MID) table = mid_line_validation_table_;

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) table = right_line_validation_table_;

    if(SEARCH_LINE_CODE == MID_TO_LEFT) table = left_line_validation_table_;

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) table = right_line_validation_table_;

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) table = left_line_validation_table_;

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) table = mid_line_validation_table_;

//TODO: make lambda copyable   mutable move



    auto min_distance_it = std::min_element(begin(table),end(table),
                                             [&](const LineValidationTable&  lhs, const LineValidationTable&  rhs)
                                             {
                                                //auto lhs_ = move(lhs);
                                                //auto rhs_ = move(rhs);
                                                return Distance2d(adjacent_point_prediction, lhs) < Distance2d(adjacent_point_prediction, rhs); }
                                             );

    int min_distance_id = std::distance(table.begin(), min_distance_it);
    Point origin = table[min_distance_id].GetOriginPoint();

    min_distance = Distance2d(origin,adjacent_point_prediction);
    min_distance_adjacent_point = origin;
    min_distance_adjacent_point_id = min_distance_id;

}








Point ValidLinePointSearch::GetAdjacentPointOriginPrediction(int SEARCH_LINE_CODE, int min_distance_adjacent_point_id)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id].GetLeftLinePointPrediction();

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id].GetLeftLinePointPrediction();

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id].GetMidLinePointPrediction();

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id].GetMidLinePointPrediction();

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id].GetRightLinePointPrediction();

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id].GetRightLinePointPrediction();
}



int ValidLinePointSearch::GetAdjacentPointDirection(int SEARCH_LINE_CODE, int min_distance_adjacent_point_id)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id].GetDirection();
}


void ValidLinePointSearch::ExamineValidationTable(int SEARCH_LINE_CODE, vector<LineValidationTable>&table)
{


  for(int i=0; i<table.size(); i++)
    {
        Point adjacent_point_prediction = table[i].GetAdjacentPointPrediction(SEARCH_LINE_CODE);

        if(adjacent_point_prediction == EMPTY_POINT_){ continue;}
        else{table[i].SetAdjacentPointPredictionFound(SEARCH_LINE_CODE,true);}

        if(AdjacentValidationTableIsEmpty(SEARCH_LINE_CODE)){continue;}

        Point min_distance_adjacent_point;
        int min_distance_adjacent_point_id;
        int min_distance;

        FindMinDistanceFromPredictionToAdjacentPoint(SEARCH_LINE_CODE,adjacent_point_prediction,min_distance_adjacent_point,min_distance_adjacent_point_id,min_distance);

        if(min_distance < kMaxPointDistance_)
        {
            table[i].SetAdjacentPointPrediction(SEARCH_LINE_CODE,true);
            table[i].SetAdjacentPointId(SEARCH_LINE_CODE,min_distance_adjacent_point_id);
        }
        else {continue;}

        // TODO: For more performance skip this
        // Check if direction of points is in range
        int adjacent_point_direction = GetAdjacentPointDirection(SEARCH_LINE_CODE,min_distance_adjacent_point_id);
        int origin_point_direction   = table[i].GetDirection();

        int direction_difference = abs(adjacent_point_direction - origin_point_direction);

        if(direction_difference < kMaxDirectionDifference_ || (360 - kMaxDirectionDifference_) < direction_difference)
        {
            table[i].SetAdjacentPointDirectionInRange(SEARCH_LINE_CODE,true);
        }

        Point origin_prediction = GetAdjacentPointOriginPrediction(SEARCH_LINE_CODE,min_distance_adjacent_point_id);

        if(origin_prediction == EMPTY_POINT_){ continue; }

        Point origin  = table[i].GetOriginPoint();
        double point_distance = Distance2d(origin,origin_prediction);

        if(point_distance < kMaxPointDistance_)
        {
            table[i].SetAdjacentPointOriginPrediction(SEARCH_LINE_CODE,true);
        }


    }
}


void ValidLinePointSearch::ExtractDirectionsInRange(vector<LineValidationTable>& line_validation_table, vector<LineValidationTable>& line_direction_in_range )
{

    // only take line points which have low change in direction
    // if they have then theres an issue like a cross section

    if(line_validation_table.size()>0)
    {
        int cut_off_id = 0;
        bool cut_off = false;

        int tmp_direction = line_validation_table[0].GetDirection();

        if(tmp_direction > kMinStartDirectionOnSameLine_ && tmp_direction < kMaxStartDirectionOnSameLine_)
        {
            for(int i=0; i<line_validation_table.size(); i++)
            {
                int current_direction =  line_validation_table[i].GetDirection();
                int direction_difference = abs(current_direction - tmp_direction);

                if(direction_difference > kMaxDirectionDifferenceOnSameLine_ || (360 - kMaxDirectionDifferenceOnSameLine_) < direction_difference)
                {
                    cut_off_id =  i;
                    cut_off = true;
                    break;
                }
                tmp_direction = current_direction;
            }

            if(cut_off)
            {
                for(int i=0; i<cut_off_id; i++) line_direction_in_range.emplace_back(line_validation_table[i]);
            }
            else{
                    copy(line_validation_table.begin(), line_validation_table.end(), back_inserter(line_direction_in_range));
            }
        }
    }
}

void ValidLinePointSearch::ExtractMinMaxLineElements( vector<LineValidationTable>  line,  MinMaxLineElements& line_minmax_elements )
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


void ValidLinePointSearch::ExtractLastMatchingPoints(vector<LineValidationTable> line_validation_table_,
                                                     vector<LineValidationTable> line_direction_in_range_,
                                                     int LINE_CODE)
{
    int last_id_adjacent1 = 0;
    int last_id_adjacent2 = 0;
    bool changed_adjacent1 = false;
    bool changed_adjacent2 = false;


    for(int i=0; i<line_direction_in_range_.size(); i++)
    {
        if(line_direction_in_range_[i].GetAdjacent1Prediction(LINE_CODE))
        {
            last_id_adjacent1 = line_direction_in_range_[i].GetAdjacent1Id(LINE_CODE);
            changed_adjacent1 = true;
        }

        if(line_direction_in_range_[i].GetAdjacent2Prediction(LINE_CODE))
        {
            last_id_adjacent2 = line_direction_in_range_[i].GetAdjacent2Id(LINE_CODE);
            changed_adjacent2 = true;
        }
    }

    if(LINE_CODE == LEFT_LINE)
    {
        LastAdjacentPointMatch last_adjacent_point_match;

        if(changed_adjacent1)
        {
            int last_mid_id    = last_id_adjacent1;
            Point last_mid_point = mid_line_validation_table_[last_mid_id].GetOriginPoint();

            last_adjacent_point_match.mid_set        = true;
            last_adjacent_point_match.last_mid_point = last_mid_point;
            last_adjacent_point_match.last_mid_id    = last_mid_id;
        }

        if(changed_adjacent2)
        {
            int last_right_id    = last_id_adjacent2;
            Point last_right_point = right_line_validation_table_[last_right_id].GetOriginPoint();

            last_adjacent_point_match.right_set        = true;
            last_adjacent_point_match.last_right_point = last_right_point;
            last_adjacent_point_match.last_right_id    = last_right_id;
        }

        left_line_last_adjacent_point_match_ = last_adjacent_point_match;

    }


    if(LINE_CODE == MID_LINE)
    {
        LastAdjacentPointMatch last_adjacent_point_match;

        if(changed_adjacent1)
        {
            int last_left_id    = last_id_adjacent1;
            Point last_left_point = left_line_validation_table_[last_left_id].GetOriginPoint();

            last_adjacent_point_match.left_set        = true;
            last_adjacent_point_match.last_left_point = last_left_point;
            last_adjacent_point_match.last_left_id    = last_left_id;
        }

        if(changed_adjacent2)
        {
            int last_right_id    = last_id_adjacent2;
            Point last_right_point = right_line_validation_table_[last_right_id].GetOriginPoint();

            last_adjacent_point_match.right_set        = true;
            last_adjacent_point_match.last_right_point = last_right_point;
            last_adjacent_point_match.last_right_id    = last_right_id;
        }

        mid_line_last_adjacent_point_match_ = last_adjacent_point_match;

    }

    if(LINE_CODE == RIGHT_LINE)
    {
        LastAdjacentPointMatch last_adjacent_point_match;

        if(changed_adjacent1)
        {
            int last_left_id    = last_id_adjacent1;
            Point last_left_point = left_line_validation_table_[last_left_id].GetOriginPoint();

            last_adjacent_point_match.left_set        = true;
            last_adjacent_point_match.last_left_point = last_left_point;
            last_adjacent_point_match.last_left_id    = last_left_id;
        }

        if(changed_adjacent2)
        {
            int last_mid_id    = last_id_adjacent2;
            Point last_mid_point = mid_line_validation_table_[last_mid_id].GetOriginPoint();

            last_adjacent_point_match.mid_set        = true;
            last_adjacent_point_match.last_mid_point = last_mid_point;
            last_adjacent_point_match.last_mid_id    = last_mid_id;
        }

        right_line_last_adjacent_point_match_ = last_adjacent_point_match;

    }

}




void ValidLinePointSearch::ExtractValidPoints()
{

ExtractDirectionsInRange(left_line_validation_table_, left_line_direction_in_range_ );
ExtractDirectionsInRange(mid_line_validation_table_, mid_line_direction_in_range_ );
ExtractDirectionsInRange(right_line_validation_table_,right_line_direction_in_range_ );

ExtractMinMaxLineElements(left_line_direction_in_range_ , left_line_minmax_elements_ );
ExtractMinMaxLineElements(mid_line_direction_in_range_  , mid_line_minmax_elements_ );
ExtractMinMaxLineElements(right_line_direction_in_range_, right_line_minmax_elements_ );

ExtractLastMatchingPoints(left_line_direction_in_range_,left_line_direction_in_range_,LEFT_LINE);
ExtractLastMatchingPoints(mid_line_direction_in_range_,mid_line_direction_in_range_,MID_LINE);
ExtractLastMatchingPoints(right_line_direction_in_range_,right_line_direction_in_range_,RIGHT_LINE);





}


void ValidLinePointSearch::GetLinesPointsInRect( vector<LineValidationTable> line_direction_in_range_, vector<vector<Point>> contours, vector<int>& line_points_in_rect_id,
                                                    vector<LineValidationTable>& line_points_in_rect_)
{
    for(int i=0; i<line_direction_in_range_.size(); i++)
    {
        Point origin = line_direction_in_range_[i].GetOriginPoint();
        double res = pointPolygonTest(contours[0], origin, false);
        if(res>=0)
        {
            line_points_in_rect_id.push_back(i);


            line_points_in_rect_.push_back(line_direction_in_range_[i]);

        }
    }
}

vector<vector<Point>> ValidLinePointSearch::GetSearchRect(Point rect_mid, float search_direction)
{

    float rect_length_radius = kSearchRectLength_ / 2;
    float rect_height_radius = kSearchRectHeight_ / 2;

    int length_to_corner = sqrt(pow(rect_length_radius,2)+pow(rect_height_radius,2));
    int corner_angle = 90 - atan(rect_height_radius/rect_length_radius) * 180/PI;

    int rect_top_right_angle =  search_direction - corner_angle;
    int rect_top_left_angle =  search_direction + corner_angle;
    int rect_bottom_left_angle = search_direction + 180 - corner_angle;
    int rect_bottom_right_angle = search_direction + 180 + corner_angle;

    if(rect_bottom_right_angle > 359) rect_top_right_angle %= 360;
    if(rect_top_right_angle < 0)   rect_top_right_angle = 360 - abs(rect_top_right_angle);

    if(rect_top_left_angle > 359) rect_top_left_angle %= 360;
    if(rect_top_left_angle < 0)   rect_top_left_angle = 360 - abs(rect_top_left_angle);

    if(rect_bottom_left_angle > 359) rect_bottom_left_angle %= 360;
    if(rect_bottom_left_angle < 0)   rect_bottom_left_angle = 360 - abs(rect_bottom_left_angle);

    if(rect_bottom_right_angle > 359) rect_bottom_right_angle %= 360;
    if(rect_bottom_right_angle < 0)   rect_bottom_right_angle = 360 - abs(rect_bottom_right_angle);

    float rect_top_right_angle_f    = rect_top_right_angle * (PI/180);
    float rect_top_left_angle_f     = rect_top_left_angle * (PI/180);
    float rect_bottom_left_angle_f  = rect_bottom_left_angle * (PI/180);
    float rect_bottom_right_angle_f = rect_bottom_right_angle * (PI/180);

    int rect_top_left_x_offset = length_to_corner * cos(rect_top_right_angle_f);
    int rect_top_left_y_offset = -length_to_corner * sin(rect_top_right_angle_f);
    Point rect_top_left(rect_mid.x + rect_top_left_x_offset, rect_mid.y + rect_top_left_y_offset);

    int rect_top_right_x_offset = length_to_corner * cos(rect_top_left_angle_f);
    int rect_top_right_y_offset = -length_to_corner * sin(rect_top_left_angle_f);
    Point rect_top_right(rect_mid.x + rect_top_right_x_offset, rect_mid.y + rect_top_right_y_offset);

    int rect_bottom_left_x_offset = length_to_corner * cos(rect_bottom_left_angle_f);
    int rect_bottom_left_y_offset = -length_to_corner * sin(rect_bottom_left_angle_f);
    Point rect_bottom_left(rect_mid.x + rect_bottom_left_x_offset, rect_mid.y + rect_bottom_left_y_offset);

    int rect_bottom_right_x_offset = length_to_corner * cos(rect_bottom_right_angle_f);
    int rect_bottom_right_y_offset = -length_to_corner * sin(rect_bottom_right_angle_f);
    Point rect_bottom_right(rect_mid.x + rect_bottom_right_x_offset, rect_mid.y + rect_bottom_right_y_offset);



    vector<vector<Point>> contours(1);

    contours[0].push_back(rect_top_left);
    contours[0].push_back(rect_top_right);
    contours[0].push_back(rect_bottom_left);
    contours[0].push_back(rect_bottom_right);



    return contours;
}




void ValidLinePointSearch::CheckRectSafety(vector<vector<Point>> search_rect,vector<LineValidationTable>line_points_in_rect_,vector<vector<LineValidationTable>> priority_table_,
                                           RectSafetyTable& rect_safety)
{


    if(line_points_in_rect_.size()>0)
    {


        float all_percent_found = (line_points_in_rect_.size() / kSearchRectHeight_) * 100;

        float line_points_count = 0;

        if(line_points_in_rect_.size() > kSearchRectHeight_) line_points_count = line_points_in_rect_.size();
        else line_points_count = kSearchRectHeight_;


        float prio_0_percent_found = (priority_table_[0].size() / line_points_count) * 100;
        float prio_1_percent_found = (priority_table_[1].size() / line_points_count) * 100;
        float prio_2_percent_found = (priority_table_[2].size() / line_points_count) * 100;
        float prio_3_percent_found = (priority_table_[3].size() / line_points_count) * 100;
        float prio_4_percent_found = (priority_table_[4].size() / line_points_count) * 100;
        float prio_5_percent_found = (priority_table_[5].size() / line_points_count) * 100;
        float prio_6_percent_found = (priority_table_[6].size() / line_points_count) * 100;
        float prio_7_percent_found = (priority_table_[7].size() / line_points_count) * 100;
        float prio_8_percent_found = (priority_table_[8].size() / line_points_count) * 100;
        float prio_9_percent_found = (priority_table_[9].size() / line_points_count) * 100;
        float prio_10_percent_found = (priority_table_[10].size() / line_points_count) * 100;
        float prio_11_percent_found = (priority_table_[11].size() / line_points_count) * 100;
        float prio_12_percent_found = (priority_table_[12].size() / line_points_count) * 100;
        float prio_13_percent_found = (priority_table_[13].size() / line_points_count) * 100;




        MinMaxLineElements rect_min_max;


        ExtractMinMaxLineElements(line_points_in_rect_, rect_min_max );


        Point x_min = rect_min_max.x_min;
        Point y_min = rect_min_max.y_min;
        Point x_max = rect_min_max.x_max;
        Point y_max = rect_min_max.y_max;



        //cout << all_percent_found<< "%" <<    endl;





        bool too_few_points_in_rect_ = false;
        bool rect_straight_ = false;
        bool rect_left_curve_ = false;
        bool rect_right_curve_ = false;

        int yDistance = sqrt(pow(y_max.x-y_min.x,2) + pow(y_max.y -y_min.y,2));



        if(x_min.x == x_max.x)
        {
            if(yDistance > kMinYDistanceInRect_)
            {
                //cout << "rect_straight_" << endl;
                rect_straight_ = true;
            }
            else {
                too_few_points_in_rect_ = true;
            }
        }
        else if( (x_max.y - x_min.y) > 0)
        {
            if(yDistance >kMinYDistanceInRect_)
            {
                if((x_max.y - x_min.y) > kMinStraightDifferenceForStraightLineInRect_)
                {
                    rect_left_curve_ = true;
                    //cout << "left curve" << endl;
                }
                else {
                    rect_straight_ = true;
                    //cout << "rect_straight_" << endl;
                }
            }
            else {
               too_few_points_in_rect_ = true;
            }
        }
        else if((x_min.y - x_max.y) > 0)
        {
            if(yDistance >kMinYDistanceInRect_)
            {
                if((x_min.y - x_max.y) > kMinStraightDifferenceForStraightLineInRect_)
                {
                    rect_right_curve_ = true;
                    //cout << "right curve" << endl;
                }
                else{
                    rect_straight_ = true;
                     //cout << "rect_straight_" << endl;
                }
            }
            else {
                too_few_points_in_rect_ = true;
            }
        }
        else {
            cout << "wrong4" << endl;
        }


        double res = 0;

        bool y_min_in_rect_border_range_ = false;
        bool y_max_in_rect_border_range_ = false;


       /* double res = pointPolygonTest(search_rect[0], x_min, true);
        if(res>=0)
        {
            cout << res << "px  xmin: "<< x_min<< endl;
        }
*/
        res = pointPolygonTest(search_rect[0], y_min, true);
        if(res>=0 && res < kRectBorderDistanceThreshold_)
        {
            y_min_in_rect_border_range_ = true;
            //cout << res << "px  ymin: "<< y_min<< endl;
        }
/*
        res = pointPolygonTest(search_rect[0], x_max, true);
        if(res>=0)
        {
            cout << res << "px  xmax: "<< x_max<< endl;
        }
*/
        res = pointPolygonTest(search_rect[0], y_max, true);
        if(res>=0 && res < kRectBorderDistanceThreshold_)
        {
            y_max_in_rect_border_range_ = true;
            //cout << res << "px  ymax: "<< y_max<< endl;
        }


        /*
        cout << prio_0_percent_found << "%"<< endl;
        cout << prio_1_percent_found << "%"<< endl;
        cout << prio_2_percent_found << "%"<< endl;
        cout << prio_3_percent_found << "%"<< endl;
        cout << prio_4_percent_found << "%"<< endl;
        cout << prio_5_percent_found << "%"<< endl;
        cout << prio_6_percent_found << "%"<< endl;
        cout << prio_7_percent_found << "%"<< endl;
        cout << prio_8_percent_found << "%"<< endl;
        cout << prio_9_percent_found << "%"<< endl;
        cout << prio_10_percent_found << "%"<< endl;
        cout << prio_11_percent_found << "%"<< endl;
        cout << prio_12_percent_found << "%"<< endl;
        cout << prio_13_percent_found << "%"<< endl;*/


        rect_safety.percent_points_with_priority_0 = prio_0_percent_found;
        rect_safety.percent_points_with_priority_1 = prio_1_percent_found;
        rect_safety.percent_points_with_priority_2 = prio_2_percent_found;
        rect_safety.percent_points_with_priority_3 = prio_3_percent_found;
        rect_safety.percent_points_with_priority_4 = prio_4_percent_found;
        rect_safety.percent_points_with_priority_5 = prio_5_percent_found;
        rect_safety.percent_points_with_priority_6 = prio_6_percent_found;
        rect_safety.percent_points_with_priority_7 = prio_7_percent_found;
        rect_safety.percent_points_with_priority_8 = prio_8_percent_found;
        rect_safety.percent_points_with_priority_9 = prio_9_percent_found;
        rect_safety.percent_points_with_priority_10 = prio_10_percent_found;
        rect_safety.percent_points_with_priority_11 = prio_11_percent_found;
        rect_safety.percent_points_with_priority_12 = prio_12_percent_found;
        rect_safety.percent_points_with_priority_13 = prio_13_percent_found;
        rect_safety.percent_points_in_rect = all_percent_found;

        rect_safety.too_few_points_in_rect = too_few_points_in_rect_;
        rect_safety.rect_straight = rect_straight_;
        rect_safety.rect_left_curve = rect_left_curve_;
        rect_safety.rect_right_curve = rect_right_curve_;

        rect_safety.y_min_in_rect_border_range = y_min_in_rect_border_range_;
        rect_safety.y_max_in_rect_border_range = y_max_in_rect_border_range_;



    }
    else{
        EmtpySafetyTable(rect_safety);
    }

}

void ValidLinePointSearch::EmtpySafetyTable(RectSafetyTable& rect_safety)
{
    rect_safety.percent_points_with_priority_0 = 0;
    rect_safety.percent_points_with_priority_1 = 0;
    rect_safety.percent_points_with_priority_2 = 0;
    rect_safety.percent_points_with_priority_3 = 0;
    rect_safety.percent_points_with_priority_4 = 0;
    rect_safety.percent_points_with_priority_5 = 0;
    rect_safety.percent_points_with_priority_6 = 0;
    rect_safety.percent_points_with_priority_7 = 0;
    rect_safety.percent_points_with_priority_8 = 0;
    rect_safety.percent_points_with_priority_9 = 0;
    rect_safety.percent_points_with_priority_10 = 0;
    rect_safety.percent_points_with_priority_11 = 0;
    rect_safety.percent_points_with_priority_12 = 0;
    rect_safety.percent_points_with_priority_13 = 0;
    rect_safety.percent_points_in_rect = 0;

    rect_safety.too_few_points_in_rect = false;
    rect_safety.rect_straight = false;
    rect_safety.rect_left_curve = false;
    rect_safety.rect_right_curve = false;

    rect_safety.y_min_in_rect_border_range = false;
    rect_safety.y_max_in_rect_border_range = false;
}

void ValidLinePointSearch::ClearAllFollowTrackTables()
{
    left_line_points_in_rect_ids_.clear();
    mid_line_points_in_rect_ids_.clear();
    right_line_points_in_rect_ids_.clear();

    left_line_points_in_rect_.clear();
    mid_line_points_in_rect_.clear();
    right_line_points_in_rect_.clear();

    for(auto &it: left_priority_ids_) it.clear();
    for(auto &it: mid_priority_ids_) it.clear();
    for(auto &it: right_priority_ids_) it.clear();

    for(auto &it: left_priority_table_) it.clear();
    for(auto &it: mid_priority_table_) it.clear();
    for(auto &it: right_priority_table_) it.clear();

    EmtpySafetyTable(left_line_rect_safety_);
    EmtpySafetyTable(mid_line_rect_safety_);
    EmtpySafetyTable(right_line_rect_safety_);
}

void ValidLinePointSearch::GetNewRectMidPoint(float new_search_direction,Point rect_mid_point, Point& new_rect_mid_point)
{
    int new_search_direction_i = new_search_direction;

    if(new_search_direction_i > 359) new_search_direction_i %= 360;
    if(new_search_direction_i < 0)   new_search_direction_i = 360 - abs(new_search_direction_i);

    new_search_direction  = new_search_direction_i * (PI/180);
    int x_offset = kRectStepLength_ * cos(new_search_direction);
    int y_offset = -kRectStepLength_ * sin(new_search_direction);


    //cout << x_offset <<" " << y_offset<< " " << mean_direction_i << " " << mean_point << endl;

    //Point new_rect_mid_point(mean_point.x + x_offset, mean_point.y + y_offset);

    Point point(rect_mid_point.x + x_offset, rect_mid_point.y + y_offset);

    new_rect_mid_point = point;

}

void ValidLinePointSearch::FollowTrack(float search_direction, Point rect_mid_point, Mat &rgb)
{


            ClearAllFollowTrackTables();

            vector<vector<Point>> search_rect = GetSearchRect(rect_mid_point,search_direction);

            GetLinesPointsInRect(left_line_direction_in_range_,
                                 search_rect,
                                 left_line_points_in_rect_ids_,
                                 left_line_points_in_rect_);

            GetLinesPointsInRect(mid_line_direction_in_range_,
                                 search_rect,
                                 mid_line_points_in_rect_ids_,
                                 mid_line_points_in_rect_);

            GetLinesPointsInRect(right_line_direction_in_range_,
                                 search_rect,
                                 right_line_points_in_rect_ids_,
                                 right_line_points_in_rect_);


            FillPriorityTables(left_line_direction_in_range_,
                               left_line_points_in_rect_ids_,
                               mid_line_direction_in_range_,
                               mid_line_points_in_rect_ids_,
                               right_line_direction_in_range_,
                               right_line_points_in_rect_ids_);



            CheckRectSafety(search_rect,
                            left_line_points_in_rect_,
                            left_priority_table_,
                            left_line_rect_safety_);

            CheckRectSafety(search_rect,
                            mid_line_points_in_rect_,
                            mid_priority_table_,
                            mid_line_rect_safety_);

            CheckRectSafety(search_rect,
                            right_line_points_in_rect_,
                            right_priority_table_,
                            right_line_rect_safety_);


            //CoutRectSafetyTables();




            GatherRectSafetyInfo(track_safety_rects_,search_direction,rect_mid_point);

            float new_search_direction = -1;
            FindNewSearchDirection(track_safety_rects_,new_search_direction);


            //cout <<"new_search_direction: " << new_search_direction <<endl;

            if(new_search_direction == -1)new_search_direction = search_direction;


            Point new_rect_mid_point;

            GetNewRectMidPoint(new_search_direction, rect_mid_point, new_rect_mid_point);


            if(RectMidPointOutOfImage(new_rect_mid_point)) return;

            //cout << new_search_direction << " " << new_rect_mid_point << endl;

/*
 *
            drawContours(rgb, search_rect, -1, Scalar(0,255,0), 2, LINE_8);

            imshow("im", rgb);

            waitKey(1);
*/


            FollowTrack(new_search_direction, new_rect_mid_point, rgb);




/*
            rect_info_.push_back(RectInfo{
                                            left_line_points_in_rect_,
                                            mid_line_points_in_rect_,
                                            right_line_points_in_rect_,

                                            left_priority_table_,
                                            mid_priority_table_,
                                            right_priority_table_,

                                            left_line_rect_safety_,
                                            mid_line_rect_safety_,
                                            right_line_rect_safety_,

                                            rect_mid_point,
                                            search_direction
                                        });
*/




            //cout <<"new mid: " <<  new_rect_mid_point << endl;
            //cout <<"new angle: " <<  new_search_direction << endl;

            //counterr++;

/*
            cout << "found left side:" << endl;
            cout <<  mid_priority_ids_[PRIO_9_P1].size()
                 << " " <<  right_priority_ids_[PRIO_9_P1].size() << endl;

            cout << "found mid :" << endl;
            cout << left_priority_ids_[PRIO_9_P1].size()
                 << " " <<  right_priority_ids_[PRIO_10_P2].size() << endl;


            cout << "found right side:" << endl;
            cout << left_priority_ids_[PRIO_10_P2].size()
                 << " " <<  mid_priority_ids_[PRIO_10_P2].size() << endl;


            cout << "########\n\n";
*/





}

bool ValidLinePointSearch::RectMidPointOutOfImage(Point rect_mid_point)
{
    if(rect_mid_point.x >= kImageWidth_ || rect_mid_point.x < 0){ return true;}

    else if(rect_mid_point.y >= kImageHeight_ || rect_mid_point.y < 0){ return true;}

    else{ return false;}

}

void ValidLinePointSearch::FindNewSearchDirection(vector<TrackSafetyRect> track_safety_rects_, float& search_direction)
{

    int last_id = track_safety_rects_.size() - 1;
    int MAX_LINE = track_safety_rects_[last_id].MAX_LINE;

    cout << "LSCORE: " <<  track_safety_rects_[last_id].LSCORE << endl;
    cout << "MSCORE: " << track_safety_rects_[last_id].MSCORE << endl;
    cout << "RSCORE: " << track_safety_rects_[last_id].RSCORE << endl;
    cout << "TRACKSCORE: " << track_safety_rects_[last_id].TRACKSCORE << endl;
    cout << "MAX_LINE: " <<  track_safety_rects_[last_id].MAX_LINE << endl;
    cout << "MAX_LINE_SCORE: " << track_safety_rects_[last_id].MAX_LINE_SCORE << endl;

    cout << "LEFT_CONTINUOUS: " << track_safety_rects_[last_id].LEFT_CONTINUOUS << endl;
    cout << "MID_CONTINUOUS: " << track_safety_rects_[last_id].MID_CONTINUOUS << endl;
    cout << "RIGHT_CONTINUOUS: " << track_safety_rects_[last_id].RIGHT_CONTINUOUS << endl;

    cout << "search_direction: " << track_safety_rects_[last_id].search_direction << endl;
    cout << "rect_mid_point: " << track_safety_rects_[last_id].rect_mid_point << endl;


    vector<pair<int,float>> left_safest_directions1, left_safest_directions2,
                            mid_safest_directions1,  mid_safest_directions2,
                            right_safest_directions1, right_safest_directions2;



    GetSafestDirections(track_safety_rects_[last_id].left_safest_table1,
                        track_safety_rects_[last_id].left_safest_table2,
                        left_safest_directions1,left_safest_directions2);


    GetSafestDirections(track_safety_rects_[last_id].mid_safest_table1,
                        track_safety_rects_[last_id].mid_safest_table2,
                        mid_safest_directions1,mid_safest_directions2);


    GetSafestDirections(track_safety_rects_[last_id].right_safest_table1,
                        track_safety_rects_[last_id].right_safest_table2,
                        right_safest_directions1,right_safest_directions2);

    cout << "LEFT DIRS" << endl;
    for(auto it:left_safest_directions1) cout << it.first << " " << it.second << endl;
    for(auto it:left_safest_directions2) cout << it.first << " " << it.second << endl;

    cout << "MID DIRS" << endl;
    for(auto it:mid_safest_directions1) cout << it.first << " " << it.second << endl;
    for(auto it:mid_safest_directions2) cout << it.first << " " << it.second << endl;

    cout << "RIGHT DIRS" << endl;
    for(auto it:right_safest_directions1) cout << it.first << " " << it.second << endl;
    for(auto it:right_safest_directions2) cout << it.first << " " << it.second << endl;

    vector<pair<int,float>> safest_directions1, safest_directions2;

    if(MAX_LINE==LEFT_LINE)
    {
        safest_directions1=left_safest_directions1;
        safest_directions2=left_safest_directions2;
    }
    else if(MAX_LINE==MID_LINE)
    {
        safest_directions1=mid_safest_directions1 ;
        safest_directions2=mid_safest_directions2;
    }
    else if(MAX_LINE==RIGHT_LINE)
    {
        safest_directions1=right_safest_directions1 ;
        safest_directions2=right_safest_directions2;
    }
    else{ cout << "unbelievable value!" << endl; exit(0);}



    if(safest_directions1.size()>0 && safest_directions2.size()>0)
    {
        cout << "SAFE1 " << safest_directions1[safest_directions1.size()-1].first << " " << safest_directions2[safest_directions2.size()-1].first << endl;
        search_direction = (safest_directions1[safest_directions1.size()-1].first + safest_directions2[safest_directions2.size()-1].first) / 2;
    }
    else if(safest_directions1.size()>0)
    {
        cout << "SAFE2 " << safest_directions1[safest_directions1.size()-1].first  << endl;
        search_direction = safest_directions1[safest_directions1.size()-1].first;
    }
    else if(safest_directions2.size()>0)
    {
        cout << "SAFE3 " << safest_directions2[safest_directions2.size()-1].first  << endl;
        search_direction = safest_directions2[safest_directions2.size()-1].first;
    }
    else
    {
        cout << "SAFE4 -1"   << endl;
        search_direction = -1;
    }

}






int ValidLinePointSearch::CountDigits(unsigned long long int n)
{
    if(n<=0) return 1;
    else     return floor(log10(n) + 1);

}

void ValidLinePointSearch::GatherRectSafetyInfo(vector<TrackSafetyRect>& track_safety_rects_,float search_direction, Point rect_mid_point )
{
    bool left_too_few_points_in_rect = left_line_rect_safety_.too_few_points_in_rect;
    bool mid_too_few_points_in_rect = mid_line_rect_safety_.too_few_points_in_rect;
    bool right_too_few_points_in_rect = right_line_rect_safety_.too_few_points_in_rect;

    bool left_y_min_in_rect_border_range = left_line_rect_safety_.y_min_in_rect_border_range;
    bool mid_y_min_in_rect_border_range = mid_line_rect_safety_.y_min_in_rect_border_range;
    bool right_y_min_in_rect_border_range = right_line_rect_safety_.y_min_in_rect_border_range;

    bool left_y_max_in_rect_border_range = left_line_rect_safety_.y_max_in_rect_border_range;
    bool mid_y_max_in_rect_border_range = mid_line_rect_safety_.y_max_in_rect_border_range;
    bool right_y_max_in_rect_border_range = right_line_rect_safety_.y_max_in_rect_border_range;

    bool left_line_is_safe = false;
    bool mid_line_is_safe = false;
    bool right_line_is_safe = false;

    if(
            !left_too_few_points_in_rect &&
            left_y_min_in_rect_border_range &&
            left_y_max_in_rect_border_range) left_line_is_safe = true;

    if(
            !mid_too_few_points_in_rect &&
            mid_y_min_in_rect_border_range &&
            mid_y_max_in_rect_border_range) mid_line_is_safe = true;

    if(
            !right_too_few_points_in_rect &&
            right_y_min_in_rect_border_range &&
            right_y_max_in_rect_border_range) right_line_is_safe = true;


    int L2 = ceil(left_line_rect_safety_.percent_points_with_priority_2);
    int M2 = ceil(mid_line_rect_safety_.percent_points_with_priority_2);
    int R2 = ceil(right_line_rect_safety_.percent_points_with_priority_2);

    int L7 = ceil(left_line_rect_safety_.percent_points_with_priority_7);
    int M7 = ceil(mid_line_rect_safety_.percent_points_with_priority_7);
    int R7 = ceil(right_line_rect_safety_.percent_points_with_priority_7);

    int L8 = ceil(left_line_rect_safety_.percent_points_with_priority_8);
    int M8 = ceil(mid_line_rect_safety_.percent_points_with_priority_8);
    int R8 = ceil(right_line_rect_safety_.percent_points_with_priority_8);

    int L9 =ceil( left_line_rect_safety_.percent_points_with_priority_9);
    int M9 = ceil(mid_line_rect_safety_.percent_points_with_priority_9);
    int R9 = ceil(right_line_rect_safety_.percent_points_with_priority_9);

    int L10 = ceil(left_line_rect_safety_.percent_points_with_priority_10);
    int M10 = ceil(mid_line_rect_safety_.percent_points_with_priority_10);
    int R10 = ceil(right_line_rect_safety_.percent_points_with_priority_10);

    int L11 = ceil(left_line_rect_safety_.percent_points_with_priority_11);
    int M11 = ceil(mid_line_rect_safety_.percent_points_with_priority_11);
    int R11 = ceil(left_line_rect_safety_.percent_points_with_priority_11);

    int L12 = ceil(left_line_rect_safety_.percent_points_with_priority_12);
    int M12 = ceil(mid_line_rect_safety_.percent_points_with_priority_12);
    int R12 = ceil(right_line_rect_safety_.percent_points_with_priority_12);

    int L13 = ceil(left_line_rect_safety_.percent_points_with_priority_13);
    int M13 = ceil(mid_line_rect_safety_.percent_points_with_priority_13);
    int R13 = ceil(right_line_rect_safety_.percent_points_with_priority_13);

    unsigned long long int LSCORE = 0;
    unsigned long long int MSCORE = 0;
    unsigned long long int RSCORE = 0;
    unsigned long long int TRACKSCORE = 0;

    //long long int METRIC_CONTINOUS_VAL = 10000000000;
    long int METRIC_2_VAL =  100000000;
    long int METRIC_7_VAL =  1000000;
    int METRIC_8_VAL =  1000000;
    int METRIC_9_VAL =  10000;
    int METRIC_10_VAL = 10000;
    int METRIC_11_VAL = 100;
    int METRIC_12_VAL = 1;
    int METRIC_13_VAL = 1;

   /* int METRIC_CONTINOUS_VAL_DIGITS = CountDigits(METRIC_CONTINOUS_VAL);
    int METRIC_2_VAL_DIGITS = CountDigits(METRIC_2_VAL);
    int METRIC_7_8_VAL_DIGITS = CountDigits(METRIC_7_VAL);
    int METRIC_9_10_VAL_DIGITS = CountDigits(METRIC_9_VAL);
    int METRIC_11_VAL_DIGITS = CountDigits(METRIC_11_VAL);
*/
    LSCORE += (L2<100)  ?  L2* METRIC_2_VAL : 99* METRIC_2_VAL;
    LSCORE += (L7<100)  ?  L7* METRIC_7_VAL : 99* METRIC_7_VAL;
    LSCORE += (L8<100)  ?  L8* METRIC_8_VAL : 99* METRIC_8_VAL;
    LSCORE += (L9<100)  ?  L9* METRIC_9_VAL : 99* METRIC_9_VAL;
    LSCORE += (L10<100) ? L10*METRIC_10_VAL : 99*METRIC_10_VAL;
    LSCORE += (L11<100) ? L11*METRIC_11_VAL : 99*METRIC_11_VAL;
    LSCORE += (L12<100) ? L12*METRIC_12_VAL : 99*METRIC_12_VAL;
    LSCORE += (L13<100) ? L13*METRIC_13_VAL : 99*METRIC_13_VAL;


    MSCORE += (M2<100)  ?  M2* METRIC_2_VAL : 99* METRIC_2_VAL;
    MSCORE += (M7<100)  ?  M7* METRIC_7_VAL : 99* METRIC_7_VAL;
    MSCORE += (M8<100)  ?  M8* METRIC_8_VAL : 99* METRIC_8_VAL;
    MSCORE += (M9<100)  ?  M9* METRIC_9_VAL : 99* METRIC_9_VAL;
    MSCORE += (M10<100) ? M10*METRIC_10_VAL : 99*METRIC_10_VAL;
    MSCORE += (M11<100) ? M11*METRIC_11_VAL : 99*METRIC_11_VAL;
    MSCORE += (M12<100) ? M12*METRIC_12_VAL : 99*METRIC_12_VAL;
    MSCORE += (M13<100) ? M13*METRIC_13_VAL : 99*METRIC_13_VAL;


    RSCORE += (R2<100)  ?  R2* METRIC_2_VAL : 99* METRIC_2_VAL;
    RSCORE += (R7<100)  ?  R7* METRIC_7_VAL : 99* METRIC_7_VAL;
    RSCORE += (R8<100)  ?  R8* METRIC_8_VAL : 99* METRIC_8_VAL;
    RSCORE += (R9<100)  ?  R9* METRIC_9_VAL : 99* METRIC_9_VAL;
    RSCORE += (R10<100) ? R10*METRIC_10_VAL : 99*METRIC_10_VAL;
    RSCORE += (R11<100) ? R11*METRIC_11_VAL : 99*METRIC_11_VAL;
    RSCORE += (R12<100) ? R12*METRIC_12_VAL : 99*METRIC_12_VAL;
    RSCORE += (R13<100) ? R13*METRIC_13_VAL : 99*METRIC_13_VAL;


    unsigned long long int TMP_SCORE = 0;

    TMP_SCORE = (L2+M2+R2)/3;
    TRACKSCORE += (TMP_SCORE<100) ? TMP_SCORE*METRIC_2_VAL : 99*METRIC_2_VAL;

    TMP_SCORE = (L7+M7+R7)/3;
    TRACKSCORE += (TMP_SCORE<100) ? TMP_SCORE*METRIC_7_VAL : 99*METRIC_7_VAL;

    TMP_SCORE = (L8+M8+R8)/3;
    TRACKSCORE += (TMP_SCORE<100) ? TMP_SCORE*METRIC_8_VAL : 99*METRIC_8_VAL;

    TMP_SCORE = (L9+M9+R9)/3;
    TRACKSCORE += (TMP_SCORE<100) ? TMP_SCORE*METRIC_9_VAL : 99*METRIC_9_VAL;

    TMP_SCORE = (L10+M10+R10)/3;
    TRACKSCORE += (TMP_SCORE<100) ? TMP_SCORE*METRIC_10_VAL : 99*METRIC_10_VAL;

    TMP_SCORE = (L11+M11+R11)/3;
    TRACKSCORE += (TMP_SCORE<100) ? TMP_SCORE*METRIC_11_VAL : 99*METRIC_11_VAL;

    TMP_SCORE = (L12+M12+R12)/3;
    TRACKSCORE += (TMP_SCORE<100) ? TMP_SCORE*METRIC_12_VAL : 99*METRIC_12_VAL;

    TMP_SCORE = (L13+M13+R13)/3;
    TRACKSCORE += (TMP_SCORE<100) ? TMP_SCORE*METRIC_13_VAL : 99*METRIC_13_VAL;


    //if(left_line_is_safe){ LSCORE += METRIC_CONTINOUS_VAL; TRACKSCORE += METRIC_CONTINOUS_VAL;}
    //if(mid_line_is_safe) { MSCORE += METRIC_CONTINOUS_VAL; TRACKSCORE += METRIC_CONTINOUS_VAL;}
    //if(right_line_is_safe) { RSCORE += METRIC_CONTINOUS_VAL; TRACKSCORE += METRIC_CONTINOUS_VAL;}


    cout <<"LSCORE: "   << LSCORE     << " " << CountDigits(LSCORE) <<endl;
    cout <<"MSCORE: "   << MSCORE     << " " << CountDigits(MSCORE) <<endl;
    cout <<"RSCORE: "   << RSCORE     << " " << CountDigits(RSCORE) <<endl;
    cout <<"TRACKS: "   << TRACKSCORE << " " << CountDigits(TRACKSCORE) <<endl;


    vector<unsigned long long int> SCORES{LSCORE,MSCORE,RSCORE};

    auto max_score_it = std::max_element(SCORES.begin(),SCORES.end());
    int max_score_id = std::distance(SCORES.begin(), max_score_it);

    int MAX_LINE = -1;

    int MAX_LINE_SCORE = CountDigits(SCORES[max_score_id]);
    bool MAX_LINE_CONTINUOUS = false;

    switch(max_score_id)
    {
        case LEFT_LINE:{
                            cout << "LEFT IS SAFEST" << endl;
                            MAX_LINE_CONTINUOUS = left_line_is_safe;
                            MAX_LINE = LEFT_LINE;
                            break;}
        case MID_LINE:{
                            cout << "MID IS SAFEST" << endl;
                            MAX_LINE_CONTINUOUS = mid_line_is_safe;
                            MAX_LINE = MID_LINE;
                            break;}
        case RIGHT_LINE:{
                            cout << "RIGHT IS SAFEST" << endl;
                            MAX_LINE_CONTINUOUS = right_line_is_safe;
                            MAX_LINE = RIGHT_LINE;
                            break;}
        default:
                            cout << "NO MAX ???" << endl;
                            exit(0);
    }




    cout << "MAX_LINE_SCORE " << MAX_LINE_SCORE << endl;



    vector<LineValidationTable> left_safest_table1, left_safest_table2,
                                mid_safest_table1, mid_safest_table2,
                                right_safest_table1, right_safest_table2;



    GetSafestTables(left_safest_table1,left_safest_table2,left_priority_table_,LSCORE);
    GetSafestTables(mid_safest_table1,mid_safest_table2,mid_priority_table_,MSCORE);
    GetSafestTables(right_safest_table1,right_safest_table2,right_priority_table_,RSCORE);



    track_safety_rects_.push_back(TrackSafetyRect{   LSCORE,
                                                          MSCORE,
                                                          RSCORE,
                                                          TRACKSCORE,
                                                          MAX_LINE,
                                                          MAX_LINE_SCORE,
                                                          MAX_LINE_CONTINUOUS,
                                                          left_line_is_safe,
                                                          mid_line_is_safe,
                                                          right_line_is_safe,
                                                          left_safest_table1,
                                                          left_safest_table2,
                                                          mid_safest_table1,
                                                          mid_safest_table2,
                                                          right_safest_table1,
                                                          right_safest_table2,
                                                          search_direction,
                                                          rect_mid_point});













    //if(LSCORE<100 && MSCORE<100 && RSCORE<100){  mean_direction_ =-1; return;}

    //float left_safe_direction = -1;
    //float mid_safe_direction = -1;
    //float right_safe_direction = -1;
/*
    cout << "FULLLINE " << left_line_is_safe << " " << mid_line_is_safe << " " << right_line_is_safe << endl;

    int left_line_priority = -1;
    int mid_line_priority = -1;
    int right_line_priority = -1;
    int track_priority = -1;



    mean_direction_ =  mean_direction;

    left_line_priority_ = left_line_priority;
    mid_line_priority_ = mid_line_priority;
    right_line_priority_ = right_line_priority;
    track_priority_ = track_priority;
*/
    /*
        cout << "LEFT Directions And Count" << endl;
        cout <<"Size: ";
        for(auto it : left_safest_directions1)  cout << it.second << " ";
        cout <<"\nValues: ";
        for(auto it : left_safest_directions1)  cout << it.first << " ";
        cout <<"\nSize: ";
        for(auto it : left_safest_directions2)  cout << it.second << " ";
        cout <<"\nValues: ";
        for(auto it : left_safest_directions2) cout << it.first << " ";
        cout <<"\n###"<< endl;


        cout << "MID Directions And Count" << endl;
        cout <<"Size: ";
        for(auto it : mid_safest_directions1)  cout << it.second << " ";
        cout <<"\nValues: ";
        for(auto it : mid_safest_directions1)  cout << it.first << " ";
        cout <<"\nSize: ";
        for(auto it : mid_safest_directions2)  cout << it.second << " ";
        cout <<"\nValues: ";
        for(auto it : mid_safest_directions2) cout << it.first << " ";
        cout <<"\n###"<< endl;


        cout << "RIGHT Directions And Count" << endl;
        cout <<"Size: ";
        for(auto it : right_safest_directions1)  cout << it.second << " ";
        cout <<"\nValues: ";
        for(auto it : right_safest_directions1)  cout << it.first << " ";
        cout <<"\nSize: ";
        for(auto it : right_safest_directions2)  cout << it.second << " ";
        cout <<"\nValues: ";
        for(auto it : right_safest_directions2) cout << it.first << " ";
        cout <<"\n###"<< endl;
        //unsigned long long int MAX_VAL = METRIC_2_VAL * 99;
    */
        //cout <<"Percent: "<< (((long double)LSCORE) / ((long double)MAX_VAL)) * 100 <<endl;

    /*

    GetSafeDirection(left_priority_table_,left_safe_direction,left_line_priority);
    GetSafeDirection(mid_priority_table_,mid_safe_direction,mid_line_priority);
    GetSafeDirection(right_priority_table_,right_safe_direction,right_line_priority);

    int rect_prio = -1;
    int rect_prob = -1;

    if(LSCORE<100 && MSCORE<100 && RSCORE<100 && !left_line_is_safe && !mid_line_is_safe && !right_line_is_safe){  mean_direction_ =-1; return;}

    if(!left_line_is_safe && !mid_line_is_safe && !right_line_is_safe)
    {
        SearchPriority(left_line_rect_safety_,
                            mid_line_rect_safety_,
                            right_line_rect_safety_,
                            rect_prio,
                            rect_prob,
                            mean_direction);
    }
    else if(left_line_is_safe && mid_line_is_safe && right_line_is_safe)
    {
        mean_direction =(left_safe_direction + mid_safe_direction + right_safe_direction) / 3;
        track_priority = 0;
    }
    else if(left_line_is_safe && mid_line_is_safe)
    {
        mean_direction =(left_safe_direction + mid_safe_direction) / 2;
        track_priority = 1;
    }
    else if(left_line_is_safe && right_line_is_safe)
    {
        mean_direction =(left_safe_direction + right_safe_direction) / 2;
        track_priority = 2;
    }
    else if(mid_line_is_safe && right_line_is_safe)
    {
        mean_direction =(mid_safe_direction + right_safe_direction) / 2;
        track_priority = 3;
    }
    else if(left_line_is_safe)
    {
         mean_direction =left_safe_direction;
         track_priority = 4;
    }
    else if(mid_line_is_safe)
    {
        mean_direction = mid_safe_direction;
        track_priority = 5;
    }
    else if(right_line_is_safe)
    {
         mean_direction =right_safe_direction;
         track_priority = 6;
    }
    else {
        cout << "UNU"<< endl;
        exit(1);
    }

    mean_direction_ =  mean_direction;

    left_line_priority_ = left_line_priority;
    mid_line_priority_ = mid_line_priority;
    right_line_priority_ = right_line_priority;
    track_priority_ = track_priority;
*/
    //cout << "trace1" << endl;


    /*
        vector<int> left_prios;
        vector<int> mid_prios;
        vector<int> right_prios;


        left_prios.push_back(CheckPriorityProbabilities(L2,M2,R2));
        left_prios.push_back(CheckPriorityProbabilities(L2,-1,-1));
        left_prios.push_back(CheckPriorityProbabilities(L7,M7,-1));
        left_prios.push_back(CheckPriorityProbabilities(L8,R7,-1));
        left_prios.push_back(CheckPriorityProbabilities(L7,-1,-1));
        left_prios.push_back(CheckPriorityProbabilities(L8,-1,-1));
        left_prios.push_back(CheckPriorityProbabilities(L9,M9,-1));
        left_prios.push_back(CheckPriorityProbabilities(L10,R9,-1));
        left_prios.push_back(CheckPriorityProbabilities(L9,-1,-1));
        left_prios.push_back(CheckPriorityProbabilities(L10,-1,-1));
        left_prios.push_back(CheckPriorityProbabilities(L11,-1,-1));
        left_prios.push_back(CheckPriorityProbabilities(L12,-1,-1));
        left_prios.push_back(CheckPriorityProbabilities(L13,-1,-1));

        mid_prios.push_back(CheckPriorityProbabilities(M2,L2,R2));
        mid_prios.push_back(CheckPriorityProbabilities(M2,-1,-1));
        mid_prios.push_back(CheckPriorityProbabilities(M7,L7,-1));
        mid_prios.push_back(CheckPriorityProbabilities(M8,R8,-1));
        mid_prios.push_back(CheckPriorityProbabilities(M7,-1,-1));
        mid_prios.push_back(CheckPriorityProbabilities(M8,-1,-1));
        mid_prios.push_back(CheckPriorityProbabilities(M9,L9,-1));
        mid_prios.push_back(CheckPriorityProbabilities(M10,R10,-1));
        mid_prios.push_back(CheckPriorityProbabilities(M9,-1,-1));
        mid_prios.push_back(CheckPriorityProbabilities(M10,-1,-1));
        mid_prios.push_back(CheckPriorityProbabilities(M11,-1,-1));
        mid_prios.push_back(CheckPriorityProbabilities(M12,-1,-1));
        mid_prios.push_back(CheckPriorityProbabilities(M13,-1,-1));

        right_prios.push_back(CheckPriorityProbabilities(R2,L2,M2));
        right_prios.push_back(CheckPriorityProbabilities(R2,-1,-1));
        right_prios.push_back(CheckPriorityProbabilities(R7,L8,-1));
        right_prios.push_back(CheckPriorityProbabilities(R8,M8,-1));
        right_prios.push_back(CheckPriorityProbabilities(R7,-1,-1));
        right_prios.push_back(CheckPriorityProbabilities(R8,-1,-1));
        right_prios.push_back(CheckPriorityProbabilities(R10,M10,-1));
        right_prios.push_back(CheckPriorityProbabilities(R9,L10,-1));
        right_prios.push_back(CheckPriorityProbabilities(R9,-1,-1));
        right_prios.push_back(CheckPriorityProbabilities(R10,-1,-1));
        right_prios.push_back(CheckPriorityProbabilities(R11,-1,-1));
        right_prios.push_back(CheckPriorityProbabilities(R12,-1,-1));
        right_prios.push_back(CheckPriorityProbabilities(R13,-1,-1));

        int left_probability = -1;
        int left_priority = -1;

        int mid_probability = -1;
        int mid_priority = -1;

        int right_probability = -1;
        int right_priority = -1;

        GetPriorityProbabilities(left_prios,left_probability,left_priority);
        GetPriorityProbabilities(mid_prios,mid_probability,mid_priority);
        GetPriorityProbabilities(right_prios,right_probability,right_priority);


       // cout <<"LProb: "<< left_probability << "\tLPrio: " << left_priority << endl;
       // cout <<"MProb: "<< mid_probability << "\tLMrio: " << mid_priority << endl;
       // cout <<"RProb: "<< right_probability << "\tLRrio: " << right_priority << endl;


        vector<int> prio_search{left_priority,mid_priority,right_priority};

        int id;
        int prio;

        do
        {
            auto it = std::min_element(prio_search.begin(),prio_search.end());
            id = std::distance(prio_search.begin(), it);
            prio = prio_search[id];
            if(prio_search[id] == -1) prio_search[id] = 100;

        }while(prio==-1);

        cout << "ChoosenPrio: " << prio << endl;
    */


}

void ValidLinePointSearch::GetSafestDirections(vector<LineValidationTable> safest_table1,vector<LineValidationTable> safest_table2,
                    vector<pair<int,float>>& safest_line_directions1, vector<pair<int,float>>& safest_line_directions2)
{
    vector<float> safest_directions1, safest_directions2;
    vector<int> safest_dircetions1_count;
    vector<int> safest_dircetions2_count;


    for(auto it : safest_table1)  safest_directions1.push_back(it.GetDirection());
    for(auto it : safest_table2) safest_directions2.push_back(it.GetDirection());

    auto it_1 = std::unique(safest_directions1.begin(),safest_directions1.end());
    auto it_2 = std::unique(safest_directions2.begin(),safest_directions2.end());

    int unique1_distance = std::distance(safest_directions1.begin(), it_1);
    int unique2_distance = std::distance(safest_directions2.begin(), it_2);

    for(auto it = safest_directions1.begin(); it != it_1; ++it)
    {
        int unique_direction_count = std::count(safest_directions1.begin(), safest_directions1.end(), *it);
        safest_dircetions1_count.push_back(unique_direction_count);
    }

    for(auto it = safest_directions2.begin(); it != it_2; ++it)
    {
        int unique_direction_count = std::count(safest_directions2.begin(), safest_directions2.end(), *it);
        safest_dircetions2_count.push_back(unique_direction_count);

    }

    safest_directions1.resize(unique1_distance);
    safest_directions2.resize(unique2_distance);


    for(int i=0; i<safest_directions1.size();i++) safest_line_directions1.push_back(make_pair(safest_directions1[i],safest_dircetions1_count[i]));
    for(int i=0; i<safest_directions2.size();i++) safest_line_directions2.push_back(make_pair(safest_directions2[i],safest_dircetions2_count[i]));
}

void ValidLinePointSearch::GetSafestTables(vector<LineValidationTable>& safest_table1 , vector<LineValidationTable>& safest_table2,
                                           vector<vector<LineValidationTable>> priority_table, unsigned long long int SCORE)
{
    switch(CountDigits(SCORE))
    {

        case 1:
        case 2: safest_table1 = priority_table[PRIO_12_FP1];
                safest_table2 = priority_table[PRIO_13_FP2];
                break;
        case 3:
        case 4: safest_table1 = priority_table[PRIO_11_FP1_AND_FP2];
                safest_table2 = priority_table[PRIO_11_FP1_AND_FP2];
                break;
        case 5:
        case 6: safest_table1 = priority_table[PRIO_9_P1];
                safest_table2 = priority_table[PRIO_10_P2];
                break;
        case 7:
        case 8: safest_table1 = priority_table[PRIO_7_P1_AND_FP2];
                safest_table2 = priority_table[PRIO_8_P2_AND_FP1];
                break;
        case 9:
        case 10: safest_table1 = priority_table[PRIO_2_P1_AND_P2];
                 safest_table2 = priority_table[PRIO_2_P1_AND_P2];
                 break;
        default: cout << "unpossible ???" << endl; exit(0); break;
    }
}




void ValidLinePointSearch::DrawSpline(Mat &rgb)
{


    if(right_lane_drive_points_.size()>3)
    {
        std::vector<double> X, Y;
        int x_tmp = -1;


        for(int i=0;i< right_lane_drive_points_.size(); i+=1)
        {

            int x = 417 - double(right_lane_drive_points_[i].y);
            if(x <= x_tmp) continue;
            int y = double(right_lane_drive_points_[i].x);

            X.push_back(x);
            Y.push_back(y);

            //cout << x << " " << y << endl;
            x_tmp = x;
        }

        int start = 417 - right_lane_drive_points_[0].y;
        int end = x_tmp;
    //cout << "ss " << start << " " << end << endl;
        //X[0]=0; X[1]=100; X[2]=200; X[3]=300; X[4]=400;
        //Y[0]=0; Y[1]=100; Y[2]=200; Y[3]=300; Y[4]=400;
        if(X.size() > 2 && Y.size()>2)
        {
            tk::spline s;
            s.set_points(X,Y);    // currently it is required that X is already sorted

            //double x=1.5;

            //printf("spline at %f is %f\n", x, s(x));


                Vec3b color;
                color.val[0] = 0;
                color.val[1] = 255;
                color.val[2] = 0;


            for(int i=start; i<=end; i++)
            {
                    int x = int(s(i));
                    int y = rgb.rows -i;

                    rgb.at<Vec3b>(Point(x,y)) = color;
                    rgb.at<Vec3b>(Point(x+1,y)) = color;
                    rgb.at<Vec3b>(Point(x-1,y)) = color;
                    rgb.at<Vec3b>(Point(x,y+1)) = color;
                    rgb.at<Vec3b>(Point(x,y-1)) = color;
                    rgb.at<Vec3b>(Point(x-1,y-1)) = color;
                    rgb.at<Vec3b>(Point(x-1,y+1)) = color;
                    rgb.at<Vec3b>(Point(x+1,y-1)) = color;
                    rgb.at<Vec3b>(Point(x+1,y+1)) = color;
            }
        }
    }


    if(left_lane_drive_points_.size()>3)
    {
        std::vector<double> X, Y;
        int x_tmp = -1;


        for(int i=0;i< left_lane_drive_points_.size(); i+=1)
        {

            int x = 417 - double(left_lane_drive_points_[i].y);
            if(x <= x_tmp) continue;
            int y = double(left_lane_drive_points_[i].x);

            X.push_back(x);
            Y.push_back(y);

            //cout << x << " " << y << endl;
            x_tmp = x;
        }

        int start = 417 - left_lane_drive_points_[0].y;
        int end = 200;
    //cout << "ss " << start << " " << end << endl;
        //X[0]=0; X[1]=100; X[2]=200; X[3]=300; X[4]=400;
        //Y[0]=0; Y[1]=100; Y[2]=200; Y[3]=300; Y[4]=400;
        if(X.size() > 2 && Y.size()>2)
        {
            tk::spline s;
            s.set_points(X,Y);    // currently it is required that X is already sorted

            //double x=1.5;

            //printf("spline at %f is %f\n", x, s(x));


                Vec3b color;
                color.val[0] = 0;
                color.val[1] = 0;
                color.val[2] = 255;


            for(int i=start; i<=end; i++)
            {
                int x = int(s(i));
                int y = rgb.rows -i;

                rgb.at<Vec3b>(Point(x,y)) = color;
                rgb.at<Vec3b>(Point(x+1,y)) = color;
                rgb.at<Vec3b>(Point(x-1,y)) = color;
                rgb.at<Vec3b>(Point(x,y+1)) = color;
                rgb.at<Vec3b>(Point(x,y-1)) = color;
                rgb.at<Vec3b>(Point(x-1,y-1)) = color;
                rgb.at<Vec3b>(Point(x-1,y+1)) = color;
                rgb.at<Vec3b>(Point(x+1,y-1)) = color;
                rgb.at<Vec3b>(Point(x+1,y+1)) = color;
            }
        }
    }

}

void ValidLinePointSearch::CoutRectSafetyTables()
{


    cout << "P0:  " << left_line_rect_safety_.percent_points_with_priority_0 << "\t"<<  mid_line_rect_safety_.percent_points_with_priority_0 << "\t"<<  right_line_rect_safety_.percent_points_with_priority_0 << endl;
    cout << "P1:  " << left_line_rect_safety_.percent_points_with_priority_1 << "\t"<<  mid_line_rect_safety_.percent_points_with_priority_1 << "\t"<<  right_line_rect_safety_.percent_points_with_priority_1 <<endl;
    cout << "P2:  " << left_line_rect_safety_.percent_points_with_priority_2 << "\t"<<  mid_line_rect_safety_.percent_points_with_priority_2 << "\t"<< right_line_rect_safety_.percent_points_with_priority_2 <<endl;
    cout << "P3:  " << left_line_rect_safety_.percent_points_with_priority_3 << "\t"<<  mid_line_rect_safety_.percent_points_with_priority_3 << "\t"<< right_line_rect_safety_.percent_points_with_priority_3 <<endl;
    cout << "P4:  " << left_line_rect_safety_.percent_points_with_priority_4 << "\t"<<  mid_line_rect_safety_.percent_points_with_priority_4 << "\t"<< right_line_rect_safety_.percent_points_with_priority_4 <<endl;
    cout << "P5:  " << left_line_rect_safety_.percent_points_with_priority_5 << "\t"<<  mid_line_rect_safety_.percent_points_with_priority_5 << "\t"<< right_line_rect_safety_.percent_points_with_priority_5 <<endl;
    cout << "P6:  " << left_line_rect_safety_.percent_points_with_priority_6 << "\t"<<  mid_line_rect_safety_.percent_points_with_priority_6 << "\t"<< right_line_rect_safety_.percent_points_with_priority_6 <<endl;
    cout << "P7:  " << left_line_rect_safety_.percent_points_with_priority_7 << "\t"<<  mid_line_rect_safety_.percent_points_with_priority_7 << "\t"<< right_line_rect_safety_.percent_points_with_priority_7 <<endl;
    cout << "P8:  " << left_line_rect_safety_.percent_points_with_priority_8 << "\t"<<  mid_line_rect_safety_.percent_points_with_priority_8 << "\t"<< right_line_rect_safety_.percent_points_with_priority_8 <<endl;
    cout << "P9:  " << left_line_rect_safety_.percent_points_with_priority_9 << "\t"<<  mid_line_rect_safety_.percent_points_with_priority_9 << "\t"<< right_line_rect_safety_.percent_points_with_priority_9 <<endl;
    cout << "P10: " << left_line_rect_safety_.percent_points_with_priority_10 << "\t"<< mid_line_rect_safety_.percent_points_with_priority_10 << "\t"<< right_line_rect_safety_.percent_points_with_priority_10 <<endl;
    cout << "P11: " << left_line_rect_safety_.percent_points_with_priority_11 << "\t"<< mid_line_rect_safety_.percent_points_with_priority_11 << "\t"<< right_line_rect_safety_.percent_points_with_priority_11 <<endl;
    cout << "P12: " << left_line_rect_safety_.percent_points_with_priority_12 << "\t"<< mid_line_rect_safety_.percent_points_with_priority_12 << "\t"<< right_line_rect_safety_.percent_points_with_priority_12 <<endl;
    cout << "P13: " << left_line_rect_safety_.percent_points_with_priority_13 << "\t"<< mid_line_rect_safety_.percent_points_with_priority_13 << "\t"<< right_line_rect_safety_.percent_points_with_priority_13 <<endl;
    cout << "ALL: " << left_line_rect_safety_.percent_points_in_rect << "\t"<< mid_line_rect_safety_.percent_points_in_rect << "\t"<< right_line_rect_safety_.percent_points_in_rect <<endl;
    cout << "TFW: " << left_line_rect_safety_.too_few_points_in_rect << "\t"<< mid_line_rect_safety_.too_few_points_in_rect << "\t"<< right_line_rect_safety_.too_few_points_in_rect <<endl;
    cout << "STR: " << left_line_rect_safety_.rect_straight << "\t"<< mid_line_rect_safety_.rect_straight << "\t"<< right_line_rect_safety_.rect_straight <<endl;
    cout << "LFT: " << left_line_rect_safety_.rect_left_curve << "\t"<< mid_line_rect_safety_.rect_left_curve << "\t"<< right_line_rect_safety_.rect_left_curve <<endl;
    cout << "RIG: " << left_line_rect_safety_.rect_right_curve << "\t"<< mid_line_rect_safety_.rect_right_curve << "\t"<< right_line_rect_safety_.rect_right_curve <<endl;
    cout << "YMN: " << left_line_rect_safety_.y_min_in_rect_border_range << "\t"<< mid_line_rect_safety_.y_min_in_rect_border_range << "\t"<< right_line_rect_safety_.y_min_in_rect_border_range <<endl;
    cout << "YMX: " << left_line_rect_safety_.y_max_in_rect_border_range << "\t"<< mid_line_rect_safety_.y_max_in_rect_border_range << "\t"<< right_line_rect_safety_.y_max_in_rect_border_range <<endl;



}

void ValidLinePointSearch::DrawSafetyRects(Mat& rgb_spline)
{
/*
                                                            LSCORE,
                                                             MSCORE,
                                                             RSCORE,
                                                             TRACKSCORE,
                                                             MAX_LINE,
                                                             MAX_LINE_SCORE,
                                                             MAX_LINE_CONTINUOUS,
                                                             left_line_is_safe,
                                                             mid_line_is_safe,
                                                             right_line_is_safe,
                                                             left_safest_table1,
                                                             left_safest_table2,
                                                             mid_safest_table1,
                                                             mid_safest_table2,
                                                             right_safest_table1,
                                                             right_safest_table2,
                                                             search_direction,
                                                             rect_mid_point*/

    for(auto it: track_safety_rects_)
    {
        Point rect_mid_point   = it.rect_mid_point;
        float search_direction = it.search_direction;

        vector<vector<Point>> search_rect = GetSearchRect(rect_mid_point,search_direction);

        string str_score = to_string(it.MAX_LINE_SCORE);


        drawContours(rgb_spline, search_rect, -1, Scalar(255,255,0), 2, LINE_8);
        putText(rgb_spline, str_score, rect_mid_point,FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,255), 1, CV_AA);

    }
}


void ValidLinePointSearch::FindSafePointsForSpline()
{


    for(auto it: track_safety_rects_)
    {
        int  MAX_LINE_SCORE = it.MAX_LINE_SCORE;
        int  MAX_LINE = it.MAX_LINE;

        if(MAX_LINE_SCORE >= 0)
        {
            if(MAX_LINE == LEFT_LINE)
            {
                for(auto itt : it.left_safest_table1)
                {
                    Point safe_point = itt.GetOriginPoint();
                    float direction  = itt.GetDirection();

                    float right_lane_orthogonal_angle = GetOrthogonalAngle(direction, LEFT_TO_RIGHT);
                    float left_lane_orthogonal_angle = GetOrthogonalAngle(direction, LEFT_TO_RIGHT);


                    int right_lane_x  = safe_point.x + kLeftToRightDriveLaneOffset_ * cos(right_lane_orthogonal_angle*PI/180);
                    int right_lane_y  = safe_point.y - kLeftToRightDriveLaneOffset_ * sin(right_lane_orthogonal_angle*PI/180);

                    int left_lane_x  = safe_point.x + kLeftToLeftDriveLaneOffset_ * cos(left_lane_orthogonal_angle*PI/180);
                    int left_lane_y  = safe_point.y - kLeftToLeftDriveLaneOffset_ * sin(left_lane_orthogonal_angle*PI/180);


                    right_lane_drive_points_.push_back(Point(right_lane_x,right_lane_y));
                    left_lane_drive_points_.push_back(Point(left_lane_x,left_lane_y));
                }

                for(auto itt : it.left_safest_table2)
                {
                    Point safe_point = itt.GetOriginPoint();
                    float direction  = itt.GetDirection();

                    float right_lane_orthogonal_angle = GetOrthogonalAngle(direction, LEFT_TO_RIGHT);
                    float left_lane_orthogonal_angle = GetOrthogonalAngle(direction, LEFT_TO_RIGHT);


                    int right_lane_x  = safe_point.x + kLeftToRightDriveLaneOffset_ * cos(right_lane_orthogonal_angle*PI/180);
                    int right_lane_y  = safe_point.y - kLeftToRightDriveLaneOffset_ * sin(right_lane_orthogonal_angle*PI/180);

                    int left_lane_x  = safe_point.x + kLeftToLeftDriveLaneOffset_ * cos(left_lane_orthogonal_angle*PI/180);
                    int left_lane_y  = safe_point.y - kLeftToLeftDriveLaneOffset_ * sin(left_lane_orthogonal_angle*PI/180);

                    right_lane_drive_points_.push_back(Point(right_lane_x,right_lane_y));
                    left_lane_drive_points_.push_back(Point(left_lane_x,left_lane_y));
                }

            }
            else if(MAX_LINE == MID_LINE)
            {

                for(auto itt : it.mid_safest_table1)
                {
                    Point safe_point = itt.GetOriginPoint();
                    float direction  = itt.GetDirection();

                    float right_lane_orthogonal_angle = GetOrthogonalAngle(direction, MID_TO_RIGHT);
                    float left_lane_orthogonal_angle = GetOrthogonalAngle(direction, MID_TO_LEFT);

                    int right_lane_x  = safe_point.x + kMidToLeftDriveLaneOffset_ * cos(right_lane_orthogonal_angle*PI/180);
                    int right_lane_y  = safe_point.y - kMidToLeftDriveLaneOffset_ * sin(right_lane_orthogonal_angle*PI/180);

                    int left_lane_x  = safe_point.x + kMidToLeftDriveLaneOffset_ * cos(left_lane_orthogonal_angle*PI/180);
                    int left_lane_y  = safe_point.y - kMidToLeftDriveLaneOffset_ * sin(left_lane_orthogonal_angle*PI/180);

                    right_lane_drive_points_.push_back(Point(right_lane_x,right_lane_y));
                    left_lane_drive_points_.push_back(Point(left_lane_x,left_lane_y));
                }

                for(auto itt : it.mid_safest_table2)
                {
                    Point safe_point = itt.GetOriginPoint();
                    float direction  = itt.GetDirection();

                    float right_lane_orthogonal_angle = GetOrthogonalAngle(direction, MID_TO_RIGHT);
                    float left_lane_orthogonal_angle = GetOrthogonalAngle(direction, MID_TO_LEFT);

                    int right_lane_x  = safe_point.x + kMidToLeftDriveLaneOffset_ * cos(right_lane_orthogonal_angle*PI/180);
                    int right_lane_y  = safe_point.y - kMidToLeftDriveLaneOffset_ * sin(right_lane_orthogonal_angle*PI/180);

                    int left_lane_x  = safe_point.x + kMidToLeftDriveLaneOffset_ * cos(left_lane_orthogonal_angle*PI/180);
                    int left_lane_y  = safe_point.y - kMidToLeftDriveLaneOffset_ * sin(left_lane_orthogonal_angle*PI/180);

                    right_lane_drive_points_.push_back(Point(right_lane_x,right_lane_y));
                    left_lane_drive_points_.push_back(Point(left_lane_x,left_lane_y));
                }
            }
            else if(MAX_LINE == RIGHT_LINE)
            {

                for(auto itt : it.right_safest_table1)
                {
                    Point safe_point = itt.GetOriginPoint();
                    float direction  = itt.GetDirection();

                    float right_lane_orthogonal_angle = GetOrthogonalAngle(direction, RIGHT_TO_LEFT);
                    float left_lane_orthogonal_angle = GetOrthogonalAngle(direction, RIGHT_TO_LEFT);

                    int right_lane_x  = safe_point.x + kRightToRightDriveLaneOffset_ * cos(right_lane_orthogonal_angle*PI/180);
                    int right_lane_y  = safe_point.y - kRightToRightDriveLaneOffset_ * sin(right_lane_orthogonal_angle*PI/180);

                    int left_lane_x  = safe_point.x + kRightToLeftDriveLaneOffset_ * cos(left_lane_orthogonal_angle*PI/180);
                    int left_lane_y  = safe_point.y - kRightToLeftDriveLaneOffset_ * sin(left_lane_orthogonal_angle*PI/180);

                    right_lane_drive_points_.push_back(Point(right_lane_x,right_lane_y));
                    left_lane_drive_points_.push_back(Point(left_lane_x,left_lane_y));

                }

                for(auto itt : it.right_safest_table2)
                {
                    Point safe_point = itt.GetOriginPoint();
                    float direction  = itt.GetDirection();

                    float right_lane_orthogonal_angle = GetOrthogonalAngle(direction, RIGHT_TO_LEFT);
                    float left_lane_orthogonal_angle = GetOrthogonalAngle(direction, RIGHT_TO_LEFT);

                    int right_lane_x  = safe_point.x + kRightToRightDriveLaneOffset_ * cos(right_lane_orthogonal_angle*PI/180);
                    int right_lane_y  = safe_point.y - kRightToRightDriveLaneOffset_ * sin(right_lane_orthogonal_angle*PI/180);

                    int left_lane_x  = safe_point.x + kRightToLeftDriveLaneOffset_ * cos(left_lane_orthogonal_angle*PI/180);
                    int left_lane_y  = safe_point.y - kRightToLeftDriveLaneOffset_ * sin(left_lane_orthogonal_angle*PI/180);

                    right_lane_drive_points_.push_back(Point(right_lane_x,right_lane_y));
                    left_lane_drive_points_.push_back(Point(left_lane_x,left_lane_y));
                }
            }
        }


    }



}

void ValidLinePointSearch::ValidateTrack(Mat &rgb)
{
    CreateValidationTables();


    SearchValidPoints();

    ExtractValidPoints();


    FollowTrack(kStartSearchDirectionOfRectSafety_,kStartMidPointOfRectSafety_, rgb);



    FindSafePointsForSpline();

    cout << "################################################################" << endl;
   // imshow("im", rgb);

   // waitKey(0);

    //cout << "ook" << endl;

    //DrawSearchRect(rgb);

}



void ValidLinePointSearch::FillPriorityTables(vector<LineValidationTable>& left_line_direction_in_range_,
                                              vector<int> left_line_points_in_rect_ids_,
                                              vector<LineValidationTable>& mid_line_direction_in_range_,
                                              vector<int> mid_line_points_in_rect_ids_,
                                              vector<LineValidationTable>& right_line_direction_in_range_,
                                              vector<int> right_line_points_in_rect_ids_)
{


    for(int i=0; i<left_line_points_in_rect_ids_.size(); i++)
    {
        LineValidationTable left_table = left_line_direction_in_range_[left_line_points_in_rect_ids_[i]];


        bool found_mid_point = left_table.GetFoundMidPoint();
        bool mid_prediction  = left_table.GetMidPrediction();
        bool found_right_point = left_table.GetFoundRightPoint();
        bool right_prediction = left_table.GetRightPrediction();
        bool left_to_mid_directions_in_range = left_table.GetLeftToMidDirectionsInRange();
        bool left_to_right_directions_in_range = left_table.GetLeftToRightDirectionsInRange();

        /*
        if(found_mid_point) left_to_mid_found_count++;
        if(mid_prediction) left_to_mid_true_prediction_count++;
        if(found_right_point) left_to_right_found_count++;
        if(right_prediction) left_to_right_true_prediction_count++;
        if(left_to_mid_directions_in_range) left_to_mid_directions_in_range_count++;
        if(left_to_right_directions_in_range) left_to_right_directions_in_range_count++;
        */

        FillPriorityTable(left_table,i,found_mid_point,found_right_point,mid_prediction,right_prediction,
                          left_to_mid_directions_in_range,left_to_right_directions_in_range,left_priority_ids_,left_priority_table_);

    }

    for(int i=0; i<mid_line_points_in_rect_ids_.size(); i++)
    {

        LineValidationTable mid_table = mid_line_direction_in_range_[mid_line_points_in_rect_ids_[i]];

        bool found_left_point = mid_table.GetFoundLeftPoint();
        bool left_prediction  = mid_table.GetLeftPrediction();
        bool found_right_point = mid_table.GetFoundRightPoint();
        bool right_prediction = mid_table.GetRightPrediction();
        bool mid_to_left_directions_in_range = mid_table.GetMidToLeftDirectionsInRange();
        bool mid_to_right_directions_in_range = mid_table.GetMidToRightDirectionsInRange();
        /*
        if(found_left_point) mid_to_left_found_count++;
        if(left_prediction) mid_to_left_true_prediction_count++;
        if(found_right_point) mid_to_right_found_count++;
        if(right_prediction) mid_to_right_true_prediction_count++;
        if(mid_to_left_directions_in_range) mid_to_left_directions_in_range_count++;
        if(mid_to_right_directions_in_range) mid_to_right_directions_in_range_count++;
        */

        FillPriorityTable(mid_table,i,found_left_point,found_right_point,left_prediction,right_prediction,mid_to_left_directions_in_range,mid_to_right_directions_in_range,mid_priority_ids_,mid_priority_table_);
    }

    for(int i=0; i<right_line_points_in_rect_ids_.size(); i++)
    {
       LineValidationTable right_table =  right_line_direction_in_range_[right_line_points_in_rect_ids_[i]];

       bool found_left_point = right_table.GetFoundLeftPoint();
       bool left_prediction  = right_table.GetLeftPrediction();
       bool found_mid_point = right_table.GetFoundMidPoint();
       bool mid_prediction = right_table.GetMidPrediction();
       bool right_to_left_directions_in_range = right_table.GetRightToLeftDirectionsInRange();
       bool right_to_mid_directions_in_range = right_table.GetRightToMidDirectionsInRange();

       /*
       if(found_left_point) right_to_left_found_count++;
       if(left_prediction) right_to_left_true_prediction_count++;
       if(found_mid_point) right_to_mid_found_count++;
       if(mid_prediction) right_to_mid_true_prediction_count++;
       if(right_to_left_directions_in_range) right_to_left_directions_in_range_count++;
       if(right_to_mid_directions_in_range) right_to_mid_directions_in_range++;
        */
       FillPriorityTable(right_table,i,found_left_point,found_mid_point,left_prediction,mid_prediction,right_to_left_directions_in_range,right_to_mid_directions_in_range,right_priority_ids_,right_priority_table_);

    }




}

void ValidLinePointSearch::FillPriorityTable(LineValidationTable table, int i, bool found_point1,bool found_point2,bool prediction1,bool prediction2,
                                             bool directions_in_range1, bool directions_in_range2, vector<vector<int>>& priority_ids,vector<vector<LineValidationTable>>& priority_table)
{

    if(prediction1 && prediction2)
    {
        priority_ids[2].push_back(i);
        priority_table[2].push_back(table);
        return;
    }


    if(prediction1 && found_point2 || prediction2 && found_point1)
    {
        if(prediction1 && found_point2 )
        {
            priority_ids[7].push_back(i);
            priority_table[7].push_back(table);
        }

        if (prediction2 && found_point1)
        {
            priority_ids[8].push_back(i);
            priority_table[8].push_back(table);
        }

        return;
    }


    if(prediction1 || prediction2)
    {
        if(prediction1)
        {
            priority_ids[9].push_back(i);
            priority_table[9].push_back(table);
        }

        if(prediction2)
        {
            priority_ids[10].push_back(i);
            priority_table[10].push_back(table);
        }

        return;
    }


    if(found_point1 && found_point2)
    {
        priority_ids[11].push_back(i);
        priority_table[11].push_back(table);
        return;
    }


    if(found_point1 || found_point2)
    {
        if(found_point1)
        {
            priority_ids[12].push_back(i);
            priority_table[12].push_back(table);
        }

        if(found_point2)
        {
            priority_ids[13].push_back(i);
            priority_table[13].push_back(table);
        }

        return;
    }


    //cout << "point with no features found" << endl;


    return;

/*
    if(prediction1 && prediction2 && directions_in_range1 && directions_in_range2)
    {
        //priority_ids[0].push_back(i);
        //priority_table[0].push_back(table);
        //return;
    }
    else if(prediction1 && prediction2 && (directions_in_range1 ||  directions_in_range2))
    {
        //priority_ids[1].push_back(i);
        //priority_table[1].push_back(table);
        //return;
    }*/


    /*
    else if(prediction1 & found_point2 & directions_in_range1 && directions_in_range2)
    {
        //priority_ids[3].push_back(i);
        //priority_table[3].push_back(table);
        //return;
    }
    else if(prediction2 && found_point1 && directions_in_range1 && directions_in_range2)
    {
        //priority_ids[4].push_back(i);
        //priority_table[4].push_back(table);
        //return;
    }
    else if(prediction1 && found_point2 && (directions_in_range1 || directions_in_range2))
    {
        //priority_ids[5].push_back(i);
        //priority_table[5].push_back(table);
        //return;
    }
    else if(prediction2 && found_point1 && (directions_in_range1 || directions_in_range2))
    {
        //priority_ids[6].push_back(i);
        //priority_table[6].push_back(table);
        //return;
    }*/

}


vector<float> ValidLinePointSearch::GetUniqueDirectionsInRect(vector<LineValidationTable> table, vector<int> rect_ids, vector<vector<int>> priority_ids, int priority)
{
    vector<float> unique_directions;

    float tmp_direction = -1;

    for(int i=0; i<priority_ids[priority].size(); i++)
    {
        float direction = table[rect_ids[priority_ids[priority][i]]].GetDirection();

        if(tmp_direction != direction)
        {
            unique_directions.push_back(direction);
        }

        tmp_direction = direction;
    }

    return unique_directions;
}


void ValidLinePointSearch::DrawValidatedPointsOnDriveWay(Mat &rgb)
{
    Vec3b color;
    color.val[0] = 0;
    color.val[1] = 255;
    color.val[2] = 0;

    for(auto it:right_lane_drive_points_)
    {
        int x = it.x;
        int y = it.y;

        rgb.at<Vec3b>(Point(x,y)) = color;
        rgb.at<Vec3b>(Point(x+1,y)) = color;
        rgb.at<Vec3b>(Point(x-1,y)) = color;
        rgb.at<Vec3b>(Point(x,y+1)) = color;
        rgb.at<Vec3b>(Point(x,y-1)) = color;
        rgb.at<Vec3b>(Point(x-1,y-1)) = color;
        rgb.at<Vec3b>(Point(x-1,y+1)) = color;
        rgb.at<Vec3b>(Point(x+1,y-1)) = color;
        rgb.at<Vec3b>(Point(x+1,y+1)) = color;
    }

    color.val[0] = 0;
    color.val[1] = 0;
    color.val[2] = 255;

    for(auto it:left_lane_drive_points_)
    {
        int x = it.x;
        int y = it.y;

        rgb.at<Vec3b>(Point(x,y)) = color;
        rgb.at<Vec3b>(Point(x+1,y)) = color;
        rgb.at<Vec3b>(Point(x-1,y)) = color;
        rgb.at<Vec3b>(Point(x,y+1)) = color;
        rgb.at<Vec3b>(Point(x,y-1)) = color;
        rgb.at<Vec3b>(Point(x-1,y-1)) = color;
        rgb.at<Vec3b>(Point(x-1,y+1)) = color;
        rgb.at<Vec3b>(Point(x+1,y-1)) = color;
        rgb.at<Vec3b>(Point(x+1,y+1)) = color;
    }


}




void ValidLinePointSearch::SearchValidPoints()
{


    // right to left point validation

    ExamineValidationTable(LEFT_TO_MID, left_line_validation_table_);
    ExamineValidationTable(LEFT_TO_RIGHT, left_line_validation_table_);

    ExamineValidationTable(MID_TO_LEFT, mid_line_validation_table_);
    ExamineValidationTable(MID_TO_RIGHT, mid_line_validation_table_);

    ExamineValidationTable(RIGHT_TO_LEFT, right_line_validation_table_);
    ExamineValidationTable(RIGHT_TO_MID, right_line_validation_table_);
}











void ValidLinePointSearch::DrawDirectionInRangeTable(Mat &rgb)
{


    for(int i=0; i<left_line_direction_in_range_.size(); i++)
    {

            circle(rgb, left_line_direction_in_range_[i].GetOriginPoint(), 1, Scalar(0, 0, 255),CV_FILLED );

    }

    for(int i=0; i<mid_line_direction_in_range_.size(); i++)
    {

            circle(rgb, mid_line_direction_in_range_[i].GetOriginPoint(), 1, Scalar(0, 255, 0),CV_FILLED );

    }

    for(int i=0; i<right_line_direction_in_range_.size(); i++)
    {

            circle(rgb, right_line_direction_in_range_[i].GetOriginPoint(), 1, Scalar(255, 0, 0),CV_FILLED );

    }

}






void ValidLinePointSearch::CreateValidationTables()
{


    for(int i=0; i<left_to_mid_search_info_.size(); i++)
    {
        left_line_validation_table_.emplace_back(LineValidationTable(LEFT_LINE,
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
            mid_line_validation_table_.emplace_back(LineValidationTable(MID_LINE,
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
        right_line_validation_table_.emplace_back(LineValidationTable(RIGHT_LINE,
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

    left_lane_drive_points_.clear();
    right_lane_drive_points_.clear();

    track_safety_rects_.clear();







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

    left_line_direction_in_range_.clear();
    mid_line_direction_in_range_.clear();
    right_line_direction_in_range_.clear();

    left_line_minmax_elements_.initialized = false;
    mid_line_minmax_elements_.initialized = false;
    right_line_minmax_elements_.initialized = false;


    left_line_last_adjacent_point_match_.left_set =  false;
    left_line_last_adjacent_point_match_.mid_set =  false;
    left_line_last_adjacent_point_match_.right_set =  false;

    mid_line_last_adjacent_point_match_.left_set =  false;
    mid_line_last_adjacent_point_match_.mid_set =  false;
    mid_line_last_adjacent_point_match_.right_set =  false;

    right_line_last_adjacent_point_match_.left_set =  false;
    right_line_last_adjacent_point_match_.mid_set =  false;
    right_line_last_adjacent_point_match_.right_set =  false;


    left_line_points_in_rect_ids_.clear();
    mid_line_points_in_rect_ids_.clear();
    right_line_points_in_rect_ids_.clear();


    for(auto &it: left_priority_ids_) it.clear();
    for(auto &it: mid_priority_ids_) it.clear();
    for(auto &it: right_priority_ids_) it.clear();



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

    if(SEARCH_LINE_CODE == LEFT_TO_MID) left_to_mid_search_info_.emplace_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) left_to_right_search_info_.emplace_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) right_to_mid_search_info_.emplace_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) right_to_left_search_info_.emplace_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) mid_to_right_search_info_.emplace_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == MID_TO_LEFT) mid_to_left_search_info_.emplace_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});
}



void ValidLinePointSearch::SetImage(Mat image)
{
    current_image_ = image;
}

