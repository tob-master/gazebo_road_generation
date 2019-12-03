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

double Distance2d(const Point& p, LineValidationTable  hs)
{
    return sqrt(pow(p.x-hs.GetOriginPoint().x,2) + pow(p.y-hs.GetOriginPoint().y,2));
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


int ValidLinePointSearch::GetAdjacentPointDirection(int SEARCH_LINE_CODE, int min_distance_adjacent_point_id)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id].GetDirection();
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


void ValidLinePointSearch::CombineLines()
{
/*
Point origin_;
int line_code_;
int label_;

float   search_direction_;
int     next_direction_distance_;

Point left_point_;
Point mid_point_;
Point right_point_;

int left_id_;
int mid_id_;
int right_id_;

bool left_prediction_;
bool mid_prediction_;
bool right_prediction_;

bool left_to_mid_directions_in_range_;
bool left_to_right_directions_in_range_;
bool mid_to_left_directions_in_range_;
bool mid_to_right_directions_in_range_;
bool right_to_left_directions_in_range_;
bool right_to_mid_directions_in_range_;

bool left_to_origin_prediction_;
bool mid_to_origin_prediction_;
bool right_to_origin_prediction_;

bool found_left_point_;
bool found_mid_point_;
bool found_right_point_;
*/

int last_mid = 0;
int last_right = 0;
bool changed_mid = false;
bool changed_right = false;


for(int i=0; i<left_line_direction_in_range_.size(); i++)
{
    if(left_line_direction_in_range_[i].GetMidPrediction())
    {
        last_mid = left_line_direction_in_range_[i].GetMidPointId();
        changed_mid = true;
    }

    if(left_line_direction_in_range_[i].GetRightPrediction())
    {
        last_right = left_line_direction_in_range_[i].GetRightPointId();
        changed_right = true;
    }
}

if(changed_mid)
{
    cout << "lm: " << last_mid << " P: " << mid_line_validation_table_[last_mid].GetOriginPoint() << endl;
}

if(changed_mid)
{
    cout << " lr: " << last_right << " P: " << right_line_validation_table_[last_right].GetOriginPoint() << endl;
}



/*
for(int i=0; i<left_line_direction_in_range_.size(); i++)
{
    left_line_direction_in_range_[i].GetMidPrediction();
    !left_line_direction_in_range_[i].GetRightPrediction();
    left_line_direction_in_range_[i].found_right_point_();

        apped right

    left_line_direction_in_range_[i].GetRightPrediction();
    !left_line_direction_in_range_[i].GetMidPrediction();
    left_line_direction_in_range_[i].found_mid_point_();

        append mid

    left_line_direction_in_range_[i].GetRightPrediction();
    left_line_direction_in_range_[i].GetMidPrediction();

        left point is safe


     y_min
}
*/

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

    float rect_length_radius = rect_length_ / 2;
    float rect_height_radius = rect_height_ / 2;

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

void ValidLinePointSearch::DrawSearchRect(Mat &rgb)
{

    for(auto search_rect: examined_regions_)
    {

        drawContours(rgb, search_rect, -1, Scalar(0,255,0), 2, LINE_8);

        circle(rgb, search_rect[0][0]/*rect_top_left*/, 7, Scalar(0, 0, 255),CV_FILLED );
        circle(rgb, search_rect[0][1]/*rect_top_right*/, 7, Scalar(0, 255, 255),CV_FILLED );
        circle(rgb, search_rect[0][2]/*rect_bottom_left*/, 7, Scalar(255, 0, 255),CV_FILLED );
        circle(rgb, search_rect[0][3]/*rect_bottom_right*/, 7, Scalar(255, 0, 0),CV_FILLED );

    }
}

int counterr = 0;

void ValidLinePointSearch::CheckRectSafety(vector<vector<Point>> search_rect,vector<LineValidationTable>line_points_in_rect_,vector<vector<LineValidationTable>> priority_table_,
                                           RectSafetyTable& rect_safety)
{


    if(line_points_in_rect_.size()>0)
    {


        float all_percent_found = (line_points_in_rect_.size() / rect_height_) * 100;

        float prio_0_percent_found = (priority_table_[0].size() / rect_height_) * 100;
        float prio_1_percent_found = (priority_table_[1].size() / rect_height_) * 100;
        float prio_2_percent_found = (priority_table_[2].size() / rect_height_) * 100;
        float prio_3_percent_found = (priority_table_[3].size() / rect_height_) * 100;
        float prio_4_percent_found = (priority_table_[4].size() / rect_height_) * 100;
        float prio_5_percent_found = (priority_table_[5].size() / rect_height_) * 100;
        float prio_6_percent_found = (priority_table_[6].size() / rect_height_) * 100;
        float prio_7_percent_found = (priority_table_[7].size() / rect_height_) * 100;
        float prio_8_percent_found = (priority_table_[8].size() / rect_height_) * 100;
        float prio_9_percent_found = (priority_table_[9].size() / rect_height_) * 100;
        float prio_10_percent_found = (priority_table_[10].size() / rect_height_) * 100;
        float prio_11_percent_found = (priority_table_[11].size() / rect_height_) * 100;
        float prio_12_percent_found = (priority_table_[12].size() / rect_height_) * 100;
        float prio_13_percent_found = (priority_table_[13].size() / rect_height_) * 100;




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

void ValidLinePointSearch::FollowTrack(float search_direction, Point rect_mid_point, Mat &rgb)
{
            cout << search_direction <<" " << rect_mid_point<< " " << counterr << endl;



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


            //for(auto &it: left_line_rect_safety_) it = 0;
            //for(auto &it: mid_line_rect_safety_) it = 0;
            //for(auto &it: right_line_rect_safety_) it = 0;

            vector<vector<Point>> search_rect = GetSearchRect(rect_mid_point,search_direction);




            examined_regions_.push_back(search_rect);


            GetLinesPointsInRect(left_line_direction_in_range_,search_rect,left_line_points_in_rect_ids_,left_line_points_in_rect_);
            GetLinesPointsInRect(mid_line_direction_in_range_,search_rect,mid_line_points_in_rect_ids_,mid_line_points_in_rect_);
            GetLinesPointsInRect(right_line_direction_in_range_,search_rect,right_line_points_in_rect_ids_,right_line_points_in_rect_);

            //cout << left_line_points_in_rect_ids_.size() << " " << mid_line_points_in_rect_ids_.size() << " " <<right_line_points_in_rect_ids_.size() << endl;

            FillPriorityTables();

            //CheckRectSafety(search_rect,left_line_points_in_rect_,left_priority_table_,left_line_rect_safety_);



            CheckRectSafety(search_rect,left_line_points_in_rect_,left_priority_table_, left_line_rect_safety_);
            CheckRectSafety(search_rect,mid_line_points_in_rect_,mid_priority_table_, mid_line_rect_safety_);
            CheckRectSafety(search_rect,right_line_points_in_rect_,right_priority_table_, right_line_rect_safety_);


            CoutRectSafetyTables();


            float mean_direction;
            int priority;



            FindNewSearchDirection(mean_direction);






            if(mean_direction == -1) ExaminePriorityTables(mean_direction);

            cout << "mean dir: " << mean_direction << endl;

            if(mean_direction == -1) return;




            int mean_direction_i = mean_direction;

            if(mean_direction_i > 359) mean_direction_i %= 360;
            if(mean_direction_i < 0)   mean_direction_i = 360 - abs(mean_direction_i);




            float mean_direction_f    = mean_direction_i * (PI/180);
            int x_offset = kRectStepLength_ * cos(mean_direction_f);
            int y_offset = -kRectStepLength_ * sin(mean_direction_f);


            //cout << x_offset <<" " << y_offset<< " " << mean_direction_i << " " << mean_point << endl;

            //Point new_rect_mid_point(mean_point.x + x_offset, mean_point.y + y_offset);

            Point new_rect_mid_point(rect_mid_point.x + x_offset, rect_mid_point.y + y_offset);



            float new_search_direction = float(mean_direction_i);







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



            rect_mid_points_.push_back(new_rect_mid_point);

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
*//*
            drawContours(rgb, search_rect, -1, Scalar(0,255,0), 2, LINE_8);

            imshow("im", rgb);

            waitKey(0);

*/
            FollowTrack(new_search_direction, new_rect_mid_point, rgb);





}

void ValidLinePointSearch::FindNewSearchDirection(float& mean_direction_)
{

  /*  float percent_points_with_priority_0;
    float percent_points_with_priority_1;
    float percent_points_with_priority_2;
    float percent_points_with_priority_3;
    float percent_points_with_priority_4;
    float percent_points_with_priority_5;
    float percent_points_with_priority_6;
    float percent_points_with_priority_7;
    float percent_points_with_priority_8;
    float percent_points_with_priority_9;
    float percent_points_with_priority_10;
    float percent_points_with_priority_11;
    float percent_points_with_priority_12;
    float percent_points_with_priority_13;
*/
    int left_points_found_percentage = left_line_rect_safety_.percent_points_in_rect;
    int mid_points_found_percentage = mid_line_rect_safety_.percent_points_in_rect;
    int right_points_found_percentage = right_line_rect_safety_.percent_points_in_rect;

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

    int kMinFoundPercentage_ = 40;


    if(left_points_found_percentage > kMinFoundPercentage_ && !left_too_few_points_in_rect && left_y_min_in_rect_border_range && left_y_max_in_rect_border_range) left_line_is_safe = true;
    if(mid_points_found_percentage > kMinFoundPercentage_ && !mid_too_few_points_in_rect && mid_y_min_in_rect_border_range && mid_y_max_in_rect_border_range) mid_line_is_safe = true;
    if(right_points_found_percentage > kMinFoundPercentage_ && !right_too_few_points_in_rect && right_y_min_in_rect_border_range && right_y_max_in_rect_border_range) right_line_is_safe = true;

    float left_safe_direction = -1;
    float mid_safe_direction = -1;
    float right_safe_direction = -1;

    int left_priority = -1;
    int mid_priority = -1;
    int right_priority = -1;

    if(left_line_is_safe)  GetSafeDirection(left_priority_table_,left_safe_direction,left_priority);
    if(mid_line_is_safe)   GetSafeDirection(mid_priority_table_,mid_safe_direction,mid_priority);
    if(right_line_is_safe) GetSafeDirection(right_priority_table_,right_safe_direction,right_priority);

    float mean_direction = -1;

    if(!left_line_is_safe && !mid_line_is_safe && !right_line_is_safe)
    {
        mean_direction_ = mean_direction;
        return;
    }



    if(left_line_is_safe && mid_line_is_safe && right_line_is_safe)
    {
        mean_direction =(left_safe_direction + mid_safe_direction + right_safe_direction) / 3;
    }
    else if(left_line_is_safe && mid_line_is_safe)
    {
        mean_direction =(left_safe_direction + mid_safe_direction) / 2;
    }
    else if(left_line_is_safe && right_line_is_safe)
    {
        mean_direction =(left_safe_direction + right_safe_direction) / 2;
    }
    else if(mid_line_is_safe && right_line_is_safe)
    {
        mean_direction =(mid_safe_direction + right_safe_direction) / 2;
    }
    else if(left_line_is_safe)
    {
         mean_direction =left_safe_direction;
    }
    else if(mid_line_is_safe)
    {
        mean_direction = mid_safe_direction;
    }
    else if(right_line_is_safe)
    {
         mean_direction =right_safe_direction;
    }
    else {
        mean_direction = -1;
    }

    mean_direction_ =  mean_direction;

}


void ValidLinePointSearch::GetSafeDirection( vector<vector<LineValidationTable>> priority_table_, float& safe_direction_, int& priority_)
{
    int safe_direction = -1;
    int priority = -1;

    if(priority_table_[0].size() > 0)
    {
        safe_direction = priority_table_[0][priority_table_[0].size()-1].GetDirection();
        priority = 0;
    }
    else if(priority_table_[1].size() > 0)
    {
        safe_direction = priority_table_[1][priority_table_[1].size()-1].GetDirection();
                priority = 1;
    }
    else if(priority_table_[2].size() > 0)
    {
        safe_direction = priority_table_[2][priority_table_[2].size()-1].GetDirection();
               priority = 2;
    }
    else if(priority_table_[3].size() > 0)
    {
       safe_direction = priority_table_[3][priority_table_[3].size()-1].GetDirection();
               priority = 3;
    }
    else if(priority_table_[4].size() > 0)
    {
        safe_direction = priority_table_[4][priority_table_[4].size()-1].GetDirection();
               priority = 4;
    }
    else if(priority_table_[5].size() > 0)
    {
        safe_direction = priority_table_[5][priority_table_[5].size()-1].GetDirection();
               priority = 5;
    }
    else if(priority_table_[6].size() > 0)
    {
        safe_direction = priority_table_[6][priority_table_[6].size()-1].GetDirection();
                priority = 6;
    }
    else if(priority_table_[7].size() > 0)
    {
        safe_direction = priority_table_[7][priority_table_[7].size()-1].GetDirection();
                priority = 7;
    }
    else if(priority_table_[8].size() > 0)
    {
        safe_direction = priority_table_[8][priority_table_[8].size()-1].GetDirection();
                priority = 8;
    }
    else if(priority_table_[9].size() > 0)
    {
        safe_direction = priority_table_[9][priority_table_[9].size()-1].GetDirection();
                priority = 9;
    }
    else if(priority_table_[10].size() > 0)
    {
        safe_direction = priority_table_[10][priority_table_[10].size()-1].GetDirection();
                priority = 10;
    }
    else if(priority_table_[11].size() > 0)
    {
        safe_direction = priority_table_[11][priority_table_[11].size()-1].GetDirection();
               priority = 11;
    }
    else if(priority_table_[12].size() > 0)
    {
        safe_direction = priority_table_[12][priority_table_[12].size()-1].GetDirection();
                priority = 12;
    }
    else if(priority_table_[13].size() > 0)
    {
        safe_direction = priority_table_[13][priority_table_[13].size()-1].GetDirection();
                priority = 13;
    }
    else
    {
        safe_direction = -1;
        priority = -1;
    }

    safe_direction_ =  safe_direction;
    priority_ = priority;

}



void ValidLinePointSearch::DrawSpline(Mat &rgb)
{

    std::vector<double> X, Y;
/*
    X.push_back(92);
        Y.push_back(400);
    X.push_back(127);
        Y.push_back(400);
    X.push_back(162);
        Y.push_back(400);
    X.push_back(197);
        Y.push_back(400);
    X.push_back(232);
        Y.push_back(400);
    X.push_back(267);
        Y.push_back(400);
*/

    int x_tmp = -1;


    for(auto it: rect_mid_points_)
    {

        int x = 417 - double(it.y);
        if(x <= x_tmp) break;
        int y = double(it.x);

        X.push_back(x);
        Y.push_back(y);

        cout << x << " " << y << endl;
        x_tmp = x;
    }

    /*for(int i=0; i<mid_line_direction_in_range_.size(); i++)
    {
        Point p = mid_line_direction_in_range_[i].GetOriginPoint();
        cout << p << endl;
        X.push_back(double(417-p.y));
        Y.push_back(double(p.x));
    }
*/
    int start = 417 - rect_mid_points_[0].y;
    int end = x_tmp;
cout << "ss " << start << " " << end << endl;
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
            color.val[2] = 255;


        for(int i=start; i<=end; i++)
        {
                rgb.at<Vec3b>(Point(int(s(i)),rgb.rows -i)) = color;
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

void ValidLinePointSearch::ValidateTrack(Mat &rgb)
{


    Point rect_mid = Point(612,360);

    float search_direction = 90;

    FollowTrack(search_direction,rect_mid, rgb);

    cout << "ook" << endl;

    //DrawSearchRect(rgb);

}



void ValidLinePointSearch::FillPriorityTables()
{
    /*
    Point origin_;
    int line_code_;
    int label_;

    float   search_direction_;
    int     next_direction_distance_;

    Point left_point_;
    Point mid_point_;
    Point right_point_;

    int left_id_;
    int mid_id_;
    int right_id_;

    bool left_prediction_;
    bool mid_prediction_;
    bool right_prediction_;

    bool left_to_mid_directions_in_range_;
    bool left_to_right_directions_in_range_;
    bool mid_to_left_directions_in_range_;
    bool mid_to_right_directions_in_range_;
    bool right_to_left_directions_in_range_;
    bool right_to_mid_directions_in_range_;

    bool left_to_origin_prediction_;
    bool mid_to_origin_prediction_;
    bool right_to_origin_prediction_;

    bool found_left_point_;
    bool found_mid_point_;
    bool found_right_point_;
    */


    // TODO Check if not always the same point
    /*
    int mid_to_left_found_count = 0;
    int mid_to_left_true_prediction_count = 0;
    int mid_to_left_directions_in_range_count = 0;

    int right_to_left_found_count = 0;
    int right_to_left_true_prediction_count = 0;
    int right_to_left_directions_in_range_count = 0;

    int left_to_mid_found_count = 0;
    int left_to_mid_true_prediction_count = 0;
    int left_to_mid_directions_in_range_count = 0;

    int right_to_mid_found_count = 0;
    int right_to_mid_true_prediction_count = 0;
    int right_to_mid_directions_in_range = 0;

    int left_to_right_found_count = 0;
    int left_to_right_true_prediction_count = 0;
    int left_to_right_directions_in_range_count = 0;

    int mid_to_right_found_count = 0;
    int mid_to_right_true_prediction_count = 0;
    int mid_to_right_directions_in_range_count = 0;
    */



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





    //cout << "";

    //Find right line

    //cout << right_table.GetLeftPoint() << " " << left_line_direction_in_range_[right_table.GetLeftPointId()].GetOriginPoint()<< endl;

    //cout << mid_to_left_found_count << " " << right_to_left_found_count << " " << mid_to_left_true_prediction_count << " " << right_to_left_true_prediction_count << endl;

}

void ValidLinePointSearch::FillPriorityTable(LineValidationTable table, int i, bool found_point1,bool found_point2,bool prediction1,bool prediction2,
                                             bool directions_in_range1, bool directions_in_range2, vector<vector<int>>& priority_ids,vector<vector<LineValidationTable>>& priority_table)
{




    if(prediction1 && prediction2 && directions_in_range1 && directions_in_range2)
    {
        priority_ids[0].push_back(i);
        priority_table[0].push_back(table);
        //return;
    }

    if(prediction1 && prediction2 && (directions_in_range1 ||  directions_in_range2))
    {
        priority_ids[1].push_back(i);
        priority_table[1].push_back(table);
        //return;
    }

    if(prediction1 && prediction2)
    {
        priority_ids[2].push_back(i);
        priority_table[2].push_back(table);
        //return;
    }   

    if(prediction1 & found_point2 & directions_in_range1 && directions_in_range2)
    {
        priority_ids[3].push_back(i);
        priority_table[3].push_back(table);
        //return;
    }

    if(prediction2 && found_point1 && directions_in_range1 && directions_in_range2)
    {
        priority_ids[4].push_back(i);
        priority_table[4].push_back(table);
        //return;
    }

    if(prediction1 && found_point2 && (directions_in_range1 || directions_in_range2))
    {
        priority_ids[5].push_back(i);
        priority_table[5].push_back(table);
        //return;
    }

    if(prediction2 && found_point1 && (directions_in_range1 || directions_in_range2))
    {
        priority_ids[6].push_back(i);
        priority_table[6].push_back(table);
        //return;
    }

    if(prediction1 && found_point2)
    {
        priority_ids[7].push_back(i);
        priority_table[7].push_back(table);
        //return;
    }

    if(prediction2 && found_point1)
    {
        priority_ids[8].push_back(i);
        priority_table[8].push_back(table);
        //return;
    }

    if(prediction1)
    {
        priority_ids[9].push_back(i);
        priority_table[9].push_back(table);
        //return;
    }


    if(prediction2)
    {
        priority_ids[10].push_back(i);
        priority_table[10].push_back(table);
        //return;
    }


    if(found_point1 && found_point2)
    {
        priority_ids[11].push_back(i);
        priority_table[11].push_back(table);
        //return;
    }

    if(found_point1)
    {
        priority_ids[12].push_back(i);
        priority_table[12].push_back(table);
        //return;
    }

    if(found_point2)
    {
        priority_ids[13].push_back(i);
        priority_table[13].push_back(table);
        //return;
    }

    return;
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


void  ValidLinePointSearch::ExaminePriorityTables(float& mean_direction)
{



    //int p_both_score = p_left_mid_right_count + p_mid_left_right_count + p_right_left_mid_count;
    //int p_left_score =  p_left_mid_count + p_left_right_count;
    //int p_mid_score =   p_mid_left_count + p_mid_right_count;
    //int p_right_score = p_right_left_count + p_right_mid_count;


    //int kMinAllLinesFoundScore_ = 10;
    //int kMinLineFoundScore_ = 10;


    //bool all_lines_found = p_both_score > kMinAllLinesFoundScore_;

    //bool left_line_found  = p_left_score > kMindLineFoundScore_;
    //bool mid_line_found   = p_mid_score > kMindLineFoundScore_;
    //bool right_line_found = p_right_score > kMindLineFoundScore_;

    //cout << "Predi b: " << p_both_score << " l: " << p_left_score << " m: " << p_mid_score << " r: " << p_right_score << endl;




    //int f_both_score = f_left_mid_right_count + f_mid_left_right_count + f_right_left_mid_count;
    //int f_left_score =f_left_mid_count + f_left_right_count;
    //int f_mid_score =  f_mid_left_count + f_mid_right_count;
    //int f_right_score = f_right_left_count + f_right_mid_count;

    //cout << "Found b: " << f_both_score << " l: " << f_left_score << " m: " << f_mid_score << " r: " << f_right_score << endl;
/*

    int left_in_rect_size = left_line_points_in_rect_ids_.size();
    int mid_in_rect_size = mid_line_points_in_rect_ids_.size();
    int right_in_rect_size = right_line_points_in_rect_ids_.size();

    cout << "Size in rect l: " << left_in_rect_size << " m: " << mid_in_rect_size << " r: " << right_in_rect_size << endl;

    int left_line_size = left_line_direction_in_range_.size();
    int mid_line_size = mid_line_direction_in_range_.size();
    int right_line_size = right_line_direction_in_range_.size();

    cout << "Size in range l: " << left_line_size << " m: " << mid_line_size << " r: " << right_line_size << endl;


   Point last_mid_left =     left_line_last_adjacent_point_match_.last_mid_point;
   Point last_right_left =       left_line_last_adjacent_point_match_.last_right_point;

    Point last_left_mid =       mid_line_last_adjacent_point_match_.last_left_point;
     Point last_right_mid =      mid_line_last_adjacent_point_match_.last_right_point;

     Point last_left_right =      right_line_last_adjacent_point_match_.last_left_point;
      Point last_mid_right =     right_line_last_adjacent_point_match_.last_mid_point;


      cout << "Last match ml: " << last_mid_left << " rl: " << last_right_left << endl;
      cout << "Last match lm: " << last_left_mid << " rm: " << last_right_mid << endl;
      cout << "Last match lr: " << last_left_right << " mr: " << last_mid_right << endl;


      Point lx_min = left_line_minmax_elements_.x_min;
      Point lx_max = left_line_minmax_elements_.x_max;
      Point ly_min = left_line_minmax_elements_.y_min;
      Point ly_max = left_line_minmax_elements_.y_max;

      Point mx_min = mid_line_minmax_elements_.x_min;
      Point mx_max = mid_line_minmax_elements_.x_max;
      Point my_min = mid_line_minmax_elements_.y_min;
      Point my_max = mid_line_minmax_elements_.y_max;

      Point rx_min = right_line_minmax_elements_.x_min;
      Point rx_max = right_line_minmax_elements_.x_max;
      Point ry_min = right_line_minmax_elements_.y_min;
      Point ry_max = right_line_minmax_elements_.y_max;

      cout << "MinMax l: " << lx_min << " " << lx_max << " " <<  ly_min << " " << ly_max << endl;
      cout << "MinMax m: " << mx_min << " " << mx_max << " " <<  my_min << " " << my_max << endl;
      cout << "MinMax r: " << rx_min << " " << rx_max << " " <<  ry_min << " " << ry_max << endl;






    //cout << "h";
   // mid_priority_ids_[PRIO_0_P1_AND_P2_AND_D1_AND_D2];
   // right_priority_ids_[PRIO_0_P1_AND_P2_AND_D1_AND_D2];

    float left_mean_direction = 0;
    float mid_mean_direction = 0;
    float right_mean_direction = 0;

    float left_mean_x = 0;
    float left_mean_y = 0;

    float mid_mean_x = 0;
    float mid_mean_y = 0;

    float right_mean_x = 0;
    float right_mean_y = 0;


    int PRIO = PRIO_2_P1_AND_P2;
*/
    /*
    vector<float> left_directions;
    vector<float> mid_directions;
    vector<float> right_directions;

    vector<float> p_left_mid_right_directions;

    vector<float> p_mid_left_right_directions;
    vector<float> p_mid_left_right_unique_directions;
    vector<float> p_right_left_mid_directions;
    vector<float> p_right_left_mid_unique_directions;

    vector<float> p_left_mid_directions;
    vector<float> p_left_mid_unique_directions;

    vector<float> p_mid_left_directions;
    vector<float> p_mid_left_unique_directions;

    vector<float> p_right_left_directions;
    vector<float> p_right_left_unique_directions;


    vector<float> p_left_right_directions;
    vector<float> p_left_right_unique_directions;

    vector<float> p_mid_right_directions;
    vector<float> p_mid_right_unique_directions;

    vector<float> p_right_mid_directions;
    vector<float> p_right_mid_unique_directions;


    vector<float> f_left_mid_right_directions;
    vector<float> f_left_mid_right_unique_directions;
    vector<float> f_mid_left_right_directions;
    vector<float> f_mid_left_right_unique_directions;
    vector<float> f_right_left_mid_directions;
    vector<float> f_right_left_mid_unique_directions;

    vector<float> f_left_mid_directions;
    vector<float> f_left_mid_unique_directions;

    vector<float> f_mid_left_directions;
    vector<float> f_mid_left_unique_directions;

    vector<float> f_right_left_directions;
    vector<float> f_right_left_unique_directions;


    vector<float> f_left_right_directions;
    vector<float> f_left_right_unique_directions;

    vector<float> f_mid_right_directions;
    vector<float> f_mid_right_unique_directions;

    vector<float> f_right_mid_directions;
    vector<float> f_right_mid_unique_directions;


*/




    vector<float> p_left_mid_right_unique_directions = GetUniqueDirectionsInRect(left_line_direction_in_range_, left_line_points_in_rect_ids_, left_priority_ids_, PRIO_2_P1_AND_P2);
    vector<float> p_left_mid_unique_directions       = GetUniqueDirectionsInRect(left_line_direction_in_range_, left_line_points_in_rect_ids_, left_priority_ids_, PRIO_9_P1);
    vector<float> p_left_right_unique_directions     = GetUniqueDirectionsInRect(left_line_direction_in_range_, left_line_points_in_rect_ids_, left_priority_ids_, PRIO_10_P2);

    vector<float> p_mid_left_right_unique_directions = GetUniqueDirectionsInRect(mid_line_direction_in_range_, mid_line_points_in_rect_ids_, mid_priority_ids_, PRIO_2_P1_AND_P2);
    vector<float> p_mid_left_unique_directions       = GetUniqueDirectionsInRect(mid_line_direction_in_range_, mid_line_points_in_rect_ids_, mid_priority_ids_, PRIO_9_P1);
    vector<float> p_mid_right_unique_directions      = GetUniqueDirectionsInRect(mid_line_direction_in_range_, mid_line_points_in_rect_ids_, mid_priority_ids_, PRIO_10_P2);

    vector<float> p_right_left_mid_unique_directions = GetUniqueDirectionsInRect(right_line_direction_in_range_, right_line_points_in_rect_ids_, right_priority_ids_, PRIO_2_P1_AND_P2);
    vector<float> p_right_left_unique_directions     = GetUniqueDirectionsInRect(right_line_direction_in_range_, right_line_points_in_rect_ids_, right_priority_ids_, PRIO_9_P1);
    vector<float> p_right_mid_unique_directions      = GetUniqueDirectionsInRect(right_line_direction_in_range_, right_line_points_in_rect_ids_, right_priority_ids_, PRIO_10_P2);

    vector<float> f_left_mid_right_unique_directions = GetUniqueDirectionsInRect(left_line_direction_in_range_, left_line_points_in_rect_ids_, left_priority_ids_, PRIO_11_FP1_AND_FP2);
    vector<float> f_left_mid_unique_directions       = GetUniqueDirectionsInRect(left_line_direction_in_range_, left_line_points_in_rect_ids_, left_priority_ids_, PRIO_12_FP1);
    vector<float> f_left_right_unique_directions     = GetUniqueDirectionsInRect(left_line_direction_in_range_, left_line_points_in_rect_ids_, left_priority_ids_, PRIO_13_FP2);

    vector<float> f_mid_left_right_unique_directions = GetUniqueDirectionsInRect(mid_line_direction_in_range_, mid_line_points_in_rect_ids_, mid_priority_ids_, PRIO_11_FP1_AND_FP2);
    vector<float> f_mid_left_unique_directions       = GetUniqueDirectionsInRect(mid_line_direction_in_range_, mid_line_points_in_rect_ids_, mid_priority_ids_, PRIO_12_FP1);
    vector<float> f_mid_right_unique_directions      = GetUniqueDirectionsInRect(mid_line_direction_in_range_, mid_line_points_in_rect_ids_, mid_priority_ids_, PRIO_13_FP2);

    vector<float> f_right_left_mid_unique_directions = GetUniqueDirectionsInRect(right_line_direction_in_range_, right_line_points_in_rect_ids_, right_priority_ids_, PRIO_11_FP1_AND_FP2);
    vector<float> f_right_left_unique_directions     = GetUniqueDirectionsInRect(right_line_direction_in_range_, right_line_points_in_rect_ids_, right_priority_ids_, PRIO_12_FP1);
    vector<float> f_right_mid_unique_directions      = GetUniqueDirectionsInRect(right_line_direction_in_range_, right_line_points_in_rect_ids_, right_priority_ids_, PRIO_13_FP2);


    int p_left_mid_right_count = p_left_mid_right_unique_directions.size();
    int p_left_mid_count   = p_left_mid_unique_directions.size();
    int p_left_right_count = p_left_right_unique_directions.size();

    int p_mid_left_right_count = p_mid_left_right_unique_directions.size();
    int p_mid_left_count   = p_mid_left_unique_directions.size();
    int p_mid_right_count  = p_mid_right_unique_directions.size();

    int p_right_left_mid_count = p_right_left_mid_unique_directions.size();
    int p_right_left_count = p_right_left_unique_directions.size();
    int p_right_mid_count  = p_right_mid_unique_directions.size();


    int f_left_mid_right_count = f_left_mid_right_unique_directions.size();
    int f_left_mid_count       = f_left_mid_unique_directions.size();
    int f_left_right_count     = f_left_right_unique_directions.size();

    int f_mid_left_right_count = f_mid_left_right_unique_directions.size();
    int f_mid_left_count       = f_mid_left_unique_directions.size();
    int f_mid_right_count      = f_mid_right_unique_directions.size();

    int f_right_left_mid_count = f_right_left_mid_unique_directions.size();
    int f_right_left_count     = f_right_left_unique_directions.size();
    int f_right_mid_count      = f_right_mid_unique_directions.size();




    if(p_left_mid_right_count>0 && p_mid_left_right_count>0 && p_right_left_mid_count>0)
    {
        cout<<"0"<<endl;

        float left_direction   = p_left_mid_right_unique_directions[p_left_mid_right_count-1];
        float right_direction = p_right_left_mid_unique_directions[p_right_left_mid_count-1];
        float mid_direction   = p_mid_left_right_unique_directions[p_mid_left_right_count-1];

        mean_direction = (left_direction + mid_direction + right_direction) / 3;

    }
    else if(p_left_mid_right_count>0 && p_mid_left_right_count>0)
    {
        cout<<"1"<<endl;

        float left_direction  = p_left_mid_right_unique_directions[p_left_mid_right_count-1];
        float mid_direction   = p_mid_left_right_unique_directions[p_mid_left_right_count-1];

        mean_direction = (left_direction + mid_direction) / 2;
    }
    else if(p_right_left_mid_count>0 && p_left_mid_right_count>0)
    {
        cout<<"2"<<endl;

        float left_direction    = p_left_mid_right_unique_directions[p_left_mid_right_count-1];
        float right_direction   = p_right_left_mid_unique_directions[p_right_left_mid_count-1];

        mean_direction = (left_direction + right_direction) / 2;
    }
    else if(p_mid_left_right_count>0 && p_right_left_mid_count>0)
    {
        cout<<"3"<<endl;

        float mid_direction    = p_mid_left_right_unique_directions[p_mid_left_right_count-1];
        float right_direction   = p_right_left_mid_unique_directions[p_right_left_mid_count-1];

        mean_direction = (mid_direction + right_direction) / 2;
    }
    else if(p_left_mid_right_count>0)
    {
        cout<<"4"<<endl;
         float left_direction    = p_left_mid_right_unique_directions[p_left_mid_right_count-1];
         mean_direction = left_direction;

    }
    else if(p_mid_left_right_count>0)
    {
        cout<<"5"<<endl;
        float mid_direction    = p_mid_left_right_unique_directions[p_mid_left_right_count-1];
        mean_direction = mid_direction;
    }
    else if(p_right_left_mid_count>0)
    {
        cout<<"6"<<endl;
        float right_direction    = p_right_left_mid_unique_directions[p_right_left_mid_count-1];
        mean_direction = right_direction;
    }
    else if(p_left_mid_count>0 && p_mid_left_count>0 && f_left_right_count>0 && f_mid_right_count>0)
    {
        cout<<"7"<<endl;
        float left_direction    = p_left_mid_unique_directions[p_left_mid_count-1];
        float mid_direction    =  p_mid_left_unique_directions[p_mid_left_count-1];
        mean_direction = (left_direction + mid_direction) / 2;

    }
    else if(p_right_mid_count>0 && p_mid_right_count>0 && f_right_left_count>0 && f_mid_left_count>0)
    {
        cout<<"8"<<endl;
        float right_direction    = p_right_mid_unique_directions[p_right_mid_count-1];
        float mid_direction    =  p_mid_right_unique_directions[p_mid_right_count-1];
        mean_direction = (right_direction + mid_direction) / 2;

    }
    else if(p_left_right_count>0 && p_right_left_count>0 && f_left_mid_count>0 && f_right_mid_count>0)
    {
        cout<<"9"<<endl;
        float left_direction    = p_left_right_unique_directions[p_left_right_count-1];
        float right_direction    =  p_right_left_unique_directions[p_right_left_count-1];
        mean_direction = (left_direction + right_direction) / 2;

    }
    else if(p_left_mid_count>0 && p_mid_left_count>0 && (f_left_right_count>0 || f_mid_right_count>0))
    {
        cout<<"10"<<endl;
        float left_direction     =  p_left_mid_unique_directions[p_left_mid_count-1];
        float mid_direction    =  p_mid_left_unique_directions[p_mid_left_count-1];
        mean_direction = (left_direction + mid_direction) / 2;

    }
    else if(p_right_mid_count>0 && p_mid_right_count>0 && (f_right_left_count>0 || f_mid_left_count>0))
    {
        cout<<"11"<<endl;
        float right_direction    = p_right_mid_unique_directions[p_right_mid_count-1];
        float mid_direction    =  p_mid_right_unique_directions[p_mid_right_count-1];
        mean_direction = (right_direction + mid_direction) / 2;
    }
    else if(p_left_right_count>0 && p_right_left_count>0 && (f_left_mid_count>0 || f_right_mid_count>0))
    {
        cout<<"12"<<endl;
        float left_direction    = p_left_right_unique_directions[p_left_right_count-1];
        float right_direction    =  p_right_left_unique_directions[p_right_left_count-1];
        mean_direction = (left_direction + right_direction) / 2;
    }
    else if(p_left_mid_count>0 && p_mid_left_count>0)
    {
        cout<<"13"<<endl;
        float left_direction     =  p_left_mid_unique_directions[p_left_mid_count-1];
        float mid_direction    =  p_mid_left_unique_directions[p_mid_left_count-1];
        mean_direction = (left_direction + mid_direction) / 2;
    }
    else if(p_right_mid_count>0 && p_mid_right_count>0)
    {
        cout<<"14"<<endl;
        float right_direction    = p_right_mid_unique_directions[p_right_mid_count-1];
        float mid_direction    =  p_mid_right_unique_directions[p_mid_right_count-1];
        mean_direction = (right_direction + mid_direction) / 2;
    }
    else if(p_left_right_count>0 && p_right_left_count>0)
    {
        cout<<"15"<<endl;
        float left_direction    = p_left_right_unique_directions[p_left_right_count-1];
        float right_direction    =  p_right_left_unique_directions[p_right_left_count-1];
        mean_direction = (left_direction + right_direction) / 2;
    }

    else if(p_left_mid_count>0 && f_left_right_count>0)
    {
        cout<<"16"<<endl;
        float left_direction     =  p_left_mid_unique_directions[p_left_mid_count-1];
        mean_direction = left_direction;
    }

    else if(p_left_right_count>0 && f_left_mid_count>0)
    {
        cout<<"17"<<endl;
        float left_direction    = p_left_right_unique_directions[p_left_right_count-1];
        mean_direction = left_direction;
    }
    else if(p_mid_left_count>0 && f_mid_right_count>0)
    {
        cout<<"18"<<endl;
        float mid_direction    =  p_mid_left_unique_directions[p_mid_left_count-1];
        mean_direction = mid_direction;
    }
    else if(p_mid_right_count>0 && f_mid_left_count>0)
    {
        cout<<"19"<<endl;
        float mid_direction    =  p_mid_right_unique_directions[p_mid_right_count-1];
        mean_direction = mid_direction;
    }
    else if(p_right_left_count>0 && f_right_mid_count>0)
    {
        cout<<"20"<<endl;
        float right_direction    =  p_right_left_unique_directions[p_right_left_count-1];
        mean_direction = right_direction;
    }
    else if(p_right_mid_count>0 && f_right_left_count>0)
    {
        cout<<"21"<<endl;
        float right_direction    = p_right_mid_unique_directions[p_right_mid_count-1];
        mean_direction = right_direction;
    }
    else if(p_left_mid_count>0)
    {
        cout<<"22"<<endl;
        float left_direction     =  p_left_mid_unique_directions[p_left_mid_count-1];
        mean_direction = left_direction;
    }
    else if(p_left_right_count>0)
    {
        cout<<"23"<<endl;
        float left_direction    = p_left_right_unique_directions[p_left_right_count-1];
        mean_direction = left_direction;
    }
    else if(p_mid_left_count>0)
    {
        cout<<"24"<<endl;
        float mid_direction    =  p_mid_left_unique_directions[p_mid_left_count-1];
        mean_direction = mid_direction;
    }
    else if(p_mid_right_count>0)
    {
        cout<<"25"<<endl;
        float mid_direction    =  p_mid_right_unique_directions[p_mid_right_count-1];
        mean_direction = mid_direction;
    }
    else if(p_right_left_count>0)
    {
        cout<<"26"<<endl;
        float right_direction    =  p_right_left_unique_directions[p_right_left_count-1];
        mean_direction = right_direction;
    }
    else if(p_right_mid_count>0)
    {
        cout<<"27"<<endl;
        float right_direction    = p_right_mid_unique_directions[p_right_mid_count-1];
        mean_direction = right_direction;
    }
    else if(f_left_mid_count>0 && f_left_right_count>0 && f_mid_left_count>0 &&
            f_mid_right_count>0 && f_right_left_count>0 && f_right_mid_count>0)
    {
        cout<<"28"<<endl;
        float left_mid_direction   = f_left_mid_unique_directions[f_left_mid_count-1];
        float left_right_direction =  f_left_right_unique_directions[f_left_right_count-1];
        float mid_left_direction   =  f_mid_left_unique_directions[f_mid_left_count-1];
        float mid_right_direction   =  f_mid_right_unique_directions[f_mid_right_count-1];
        float right_left_direction   =  f_right_left_unique_directions[f_right_left_count-1];
        float right_mid_direction   =  f_right_mid_unique_directions[f_right_mid_count-1];

        mean_direction = (left_mid_direction + left_right_direction +mid_left_direction+mid_right_direction+right_left_direction+right_mid_direction)/6;

    }

    else if(f_left_mid_count>0 && f_left_right_count>0)
    {
        cout<<"29"<<endl;
        float left_mid_direction   = f_left_mid_unique_directions[f_left_mid_count-1];
        float left_right_direction =  f_left_right_unique_directions[f_left_right_count-1];
        mean_direction = (left_mid_direction + left_right_direction)/2;
    }
    else if(f_mid_left_count>0 && f_mid_right_count>0)
    {
        cout<<"30"<<endl;
        float mid_left_direction   =  f_mid_left_unique_directions[f_mid_left_count-1];
        float mid_right_direction   =  f_mid_right_unique_directions[f_mid_right_count-1];
         mean_direction = (mid_left_direction+mid_right_direction)/2;
    }
    else if(f_right_left_count>0 && f_right_mid_count>0)
    {
        cout<<"31"<<endl;
        float right_left_direction   =  f_right_left_unique_directions[f_right_left_count-1];
        float right_mid_direction   =  f_right_mid_unique_directions[f_right_mid_count-1];
        mean_direction = (right_left_direction+right_mid_direction)/2;
    }

    else if(f_left_mid_count>0 && f_mid_left_count>0)
    {
        cout<<"32"<<endl;
        float left_mid_direction   = f_left_mid_unique_directions[f_left_mid_count-1];
        float mid_left_direction   =  f_mid_left_unique_directions[f_mid_left_count-1];
        mean_direction = (left_mid_direction+mid_left_direction)/2;
    }

    else if(f_right_left_count>0 && f_left_right_count>0)
    {
        cout<<"33"<<endl;
        float left_right_direction =  f_left_right_unique_directions[f_left_right_count-1];
        float right_left_direction   =  f_right_left_unique_directions[f_right_left_count-1];
        mean_direction = (left_right_direction+right_left_direction)/2;

    }
    else if(f_mid_right_count>0 && f_right_mid_count>0)
    {
        cout<<"34"<<endl;
        float mid_right_direction   =  f_mid_right_unique_directions[f_mid_right_count-1];
        float right_mid_direction   =  f_right_mid_unique_directions[f_right_mid_count-1];
        mean_direction = (mid_right_direction+right_mid_direction)/2;

    }

    else if(f_left_mid_count>0)
    {
        cout<<"35"<<endl;
        float left_mid_direction   = f_left_mid_unique_directions[f_left_mid_count-1];
        mean_direction = left_mid_direction;
    }
    else if(f_left_right_count>0)
    {
        cout<<"36"<<endl;
        float left_right_direction =  f_left_right_unique_directions[f_left_right_count-1];
        mean_direction = left_right_direction;
    }

    else if(f_mid_left_count>0)
    {
        cout<<"37"<<endl;
        float mid_left_direction   =  f_mid_left_unique_directions[f_mid_left_count-1];
        mean_direction = mid_left_direction;
    }

    else if(f_mid_right_count>0)
    {
        cout<<"38"<<endl;
        float mid_right_direction   =  f_mid_right_unique_directions[f_mid_right_count-1];
        mean_direction = mid_right_direction;
    }
    else if(f_right_left_count>0)
    {
        cout<<"39"<<endl;
        float right_left_direction   =  f_right_left_unique_directions[f_right_left_count-1];
        mean_direction = right_left_direction;
    }
    else if(f_right_mid_count>0)
    {
        cout<<"40"<<endl;
        float right_mid_direction   =  f_right_mid_unique_directions[f_right_mid_count-1];
        mean_direction = f_right_mid_count;
    }

    else if(left_line_points_in_rect_ids_.size()>0)
    {
        cout<<"41"<<endl;
        float left_direction   = left_line_direction_in_range_[left_line_points_in_rect_ids_[(left_line_points_in_rect_ids_.size()-1)]].GetDirection();
        mean_direction = left_direction;
    }
    else if(mid_line_points_in_rect_ids_.size()>0)
    {
        cout<<"42"<<endl;
        float mid_direction   = mid_line_direction_in_range_[mid_line_points_in_rect_ids_[(mid_line_points_in_rect_ids_.size()-1)]].GetDirection();
        mean_direction = mid_direction;
    }
    else if(right_line_points_in_rect_ids_.size()>0)
    {
        cout<<"43"<<endl;
        float right_direction   = right_line_direction_in_range_[right_line_points_in_rect_ids_[(right_line_points_in_rect_ids_.size()-1)]].GetDirection();
        mean_direction = right_direction;
    }
    else{

        cout<<"nothing"<<endl;
        mean_direction = -1;

        return;
    }


/*
    if(p_left_mid_right_count>0 && p_mid_left_right_count>0 && p_right_left_mid_count>0)
    {
        float left_direction   = p_left_mid_right_unique_directions[(p_left_mid_right_unique_directions.size()-1)];
        float right_direction = p_right_left_mid_unique_directions[(p_right_left_mid_unique_directions.size()-1)];
        float mid_direction   = p_mid_left_right_unique_directions[(p_mid_left_right_unique_directions.size()-1)];


        mean_direction = (left_direction + mid_direction + right_direction) / 3;
    }
    else if(p_left_mid_right_count>0 && p_mid_left_right_count>0)
    {
        float left_direction   = p_left_mid_right_unique_directions[(p_left_mid_right_unique_directions.size()-1)];
        float mid_direction   = p_mid_left_right_unique_directions[(p_mid_left_right_unique_directions.size()-1)];
        mean_direction = (left_direction + mid_direction)/2;
    }
    else if(p_left_mid_right_count>0 && p_right_left_mid_count>0)
    {
        float left_direction   = p_left_mid_right_unique_directions[(p_left_mid_right_unique_directions.size()-1)];
        float right_direction = p_right_left_mid_unique_directions[(p_right_left_mid_unique_directions.size()-1)];

        mean_direction = (left_direction + right_direction)/2;
    }
    else if(p_mid_left_right_count>0 && p_right_left_mid_count>0)
    {
        float mid_direction   = p_mid_left_right_unique_directions[(p_mid_left_right_unique_directions.size()-1)];
        float right_direction = p_right_left_mid_unique_directions[(p_right_left_mid_unique_directions.size()-1)];

        mean_direction = (mid_direction + right_direction)/2;
    }
    else if(p_left_mid_count>0 && p_mid_left_count>0)
    {
        float left_direction   = p_left_mid_unique_directions[(p_left_mid_unique_directions.size()-1)];
        float mid_direction = p_mid_left_unique_directions[(p_mid_left_unique_directions.size()-1)];

        mean_direction = (left_direction + mid_direction)/2;
    }






    else if(right_line_points_in_rect_ids_.size()>0)
    {
        float right_direction   = right_line_direction_in_range_[right_line_points_in_rect_ids_[(right_line_points_in_rect_ids_.size()-1)]].GetDirection();
        mean_direction = right_direction;
    }
    else {

            mean_direction = -1;
            mean_point = Point(-1,-1);
            return;

    }

*/



//cout << "";
/*
 *
 *
 *    if(p_left_mid_right_count>0 && p_mid_left_right_count>0 && p_right_left_mid_count>0)cout<<"0"<<endl;

    else if(p_left_mid_right_count>0 && p_mid_left_right_count>0)cout<<"1"<<endl;

    else if(p_right_left_mid_count>0 && p_left_mid_right_count>0)cout<<"2"<<endl;

    else if(p_mid_left_right_count>0 && p_right_left_mid_count>0)cout<<"3"<<endl;

    else if(p_left_mid_right_count>0)cout<<"4"<<endl;

    else if(p_mid_left_right_count>0)cout<<"5"<<endl;

    else if(p_right_left_mid_count>0)cout<<"6"<<endl;

    else if(p_left_mid_count>0 && p_mid_left_count>0 && f_left_right_count>0 && f_mid_right_count>0)cout<<"7"<<endl;

    else if(p_right_mid_count>0 && p_mid_right_count>0 && f_right_left_count>0 && f_mid_left_count>0)cout<<"8"<<endl;

    else if(p_left_right_count>0 && p_right_left_count>0 && f_left_mid_count>0 && f_right_mid_count>0)cout<<"9"<<endl;

    else if(p_left_mid_count>0 && p_mid_left_count>0 && (f_left_right_count>0 || f_mid_right_count>0))cout<<"10"<<endl;

    else if(p_right_mid_count>0 && p_mid_right_count>0 && (f_right_left_count>0 || f_mid_left_count>0))cout<<"11"<<endl;

    else if(p_left_right_count>0 && p_right_left_count>0 && (f_left_mid_count>0 || f_right_mid_count>0))cout<<"12"<<endl;

    else if(p_left_mid_count>0 && p_mid_left_count>0)cout<<"13"<<endl;

    else if(p_right_mid_count>0 && p_mid_right_count>0)cout<<"14"<<endl;

    else if(p_left_right_count>0 && p_right_left_count>0)cout<<"15"<<endl;

    else if(p_left_mid_count>0 && f_left_right_count>0)cout<<"16"<<endl;

    else if(p_left_right_count>0 && f_left_mid_count>0)cout<<"17"<<endl;

    else if(p_mid_left_count>0 && f_mid_right_count>0)cout<<"18"<<endl;

    else if(p_mid_right_count>0 && f_mid_left_count>0)cout<<"19"<<endl;

    else if(p_right_left_count>0 && f_right_mid_count>0)cout<<"20"<<endl;

    else if(p_right_mid_count>0 && f_right_left_count>0)cout<<"21"<<endl;

    else if(p_left_mid_count>0)cout<<"22"<<endl;

    else if(p_left_right_count>0)cout<<"23"<<endl;

    else if(p_mid_left_count>0)cout<<"24"<<endl;

    else if(p_mid_right_count>0)cout<<"25"<<endl;

    else if(p_right_left_count>0)cout<<"26"<<endl;

    else if(p_right_mid_count>0)cout<<"27"<<endl;

    else if(f_left_mid_count>0 && f_left_right_count>0 && f_mid_left_count>0 &&
            f_mid_right_count>0 && f_right_left_count>0 && f_right_mid_count>0)cout<<"28"<<endl;

    else if(f_left_mid_count>0 && f_left_right_count>0)cout<<"29"<<endl;

    else if(f_mid_left_count>0 && f_mid_right_count>0)cout<<"30"<<endl;

    else if(f_right_left_count>0 && f_right_mid_count>0)cout<<"31"<<endl;

    else if(f_left_mid_count>0 && f_mid_left_count>0)cout<<"32"<<endl;

    else if(f_right_left_count>0 && f_left_right_count>0)cout<<"33"<<endl;

    else if(f_mid_right_count>0 && f_right_mid_count>0)cout<<"34"<<endl;

    else if(f_left_mid_count>0)cout<<"35"<<endl;

    else if(f_left_right_count>0)cout<<"36"<<endl;

    else if(f_mid_left_count>0)cout<<"37"<<endl;

    else if(f_mid_right_count>0)    cout<<"38"<<endl;

    else if(f_right_left_count>0)  cout<<"39"<<endl;

    else if(f_right_mid_count>0) cout<<"40"<<endl;

    else if(left_line_points_in_rect_ids_.size()>0)    cout<<"41"<<endl;

    else if(mid_line_points_in_rect_ids_.size()>0) cout<<"42"<<endl;

    else if(right_line_points_in_rect_ids_.size()>0)  cout<<"43"<<endl;

    else cout<<"nothing"<<endl;
 *
 *
    for(int i=0; i<left_priority_ids_[PRIO].size(); i++)
    {
        float direction = left_line_direction_in_range_[left_priority_ids_[PRIO][i]].GetDirection();
        //Point origin = left_line_direction_in_range_[left_priority_ids_[PRIO][i]].GetOriginPoint();

        //left_mean_x += origin.x;
        //left_mean_y += origin.y;

        //left_mean_direction += direction;

        left_directions.push_back(direction);

       //cout << "left: " << direction << endl;
    }


    for(int i=0; i<mid_priority_ids_[PRIO].size(); i++)
    {
        float direction = mid_line_direction_in_range_[mid_priority_ids_[PRIO][i]].GetDirection();
        //Point origin = mid_line_direction_in_range_[mid_priority_ids_[PRIO][i]].GetOriginPoint();

        //mid_mean_x += origin.x;
        //mid_mean_y += origin.y;

        //mid_mean_direction += direction;

        mid_directions.push_back(direction);

       //cout << "mid: " << direction << endl;
    }

    for(int i=0; i<right_priority_ids_[PRIO].size(); i++)
    {
        float direction = right_line_direction_in_range_[right_priority_ids_[PRIO][i]].GetDirection();
        //Point origin = right_line_direction_in_range_[right_priority_ids_[PRIO][i]].GetOriginPoint();

        //right_mean_x += origin.x;
        //right_mean_y += origin.y;


        //right_mean_direction += direction;

        right_directions.push_back(direction);


       //cout << "right: " << direction << endl;
    }







    if(left_priority_ids_[PRIO].size() > 0 &&
       mid_priority_ids_[PRIO].size() > 0 &&
       right_priority_ids_[PRIO].size() > 0     )
    {

        std::sort(left_directions.begin(), left_directions.end());
        auto last_left = std::unique(left_directions.begin(), left_directions.end());
        left_directions.erase(last_left, left_directions.end());

        //cout << "left: " << endl;
        for(auto it: left_directions) cout << it << " ";
        cout << endl;


        std::sort(mid_directions.begin(), mid_directions.end());
        auto last_mid = std::unique(mid_directions.begin(), mid_directions.end());
        mid_directions.erase(last_mid, mid_directions.end());

        //cout << "mid: " << endl;
        for(auto it: mid_directions) cout << it << " ";
        cout << endl;


        std::sort(right_directions.begin(), right_directions.end());
        auto last_right = std::unique(right_directions.begin(), right_directions.end());
        right_directions.erase(last_right, right_directions.end());

        //cout << "right: " << endl;
        for(auto it: right_directions) cout << it << " ";
        cout << endl;



        left_mean_direction /= left_priority_ids_[PRIO].size();
        mid_mean_direction /= mid_priority_ids_[PRIO].size();
        right_mean_direction /= right_priority_ids_[PRIO].size();

        left_mean_x /= left_priority_ids_[PRIO].size();
        left_mean_y /= left_priority_ids_[PRIO].size();

        mid_mean_x /= mid_priority_ids_[PRIO].size();
        mid_mean_y /= mid_priority_ids_[PRIO].size();

        right_mean_x /= right_priority_ids_[PRIO].size();
        right_mean_y /= right_priority_ids_[PRIO].size();

        //cout << "all directions" << endl;
//cout << left_mean_direction << " "<< mid_mean_direction<<" "<< right_mean_direction << endl;


        float mean_x =  (left_mean_x + mid_mean_x + right_mean_x) / 3;
        float mean_y =  (left_mean_y + mid_mean_y + right_mean_y) / 3;

        mean_point = Point(1, 1);

        //mean_direction =  (left_mean_direction + mid_mean_direction + right_mean_direction) / 3;

        mean_direction =  *max_element(left_directions.begin(), left_directions.end());;



    }else{
        mean_point = Point(-1,-1);
        mean_direction = -1;
    }

*/
}


void ValidLinePointSearch::DrawPointsInRect(Mat &rgb)
{
    for(auto it:left_line_points_in_rect_ids_)
    {
         circle(rgb, left_line_validation_table_[it].GetOriginPoint(), 1, Scalar(0, 0, 255),CV_FILLED );
    }

    for(auto it:mid_line_points_in_rect_ids_)
    {
         circle(rgb, mid_line_validation_table_[it].GetOriginPoint(), 1, Scalar(0, 255, 0),CV_FILLED );
    }


    for(auto it:right_line_points_in_rect_ids_)
    {
         circle(rgb, right_line_validation_table_[it].GetOriginPoint(), 1, Scalar(255, 0, 0),CV_FILLED );
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
    ExamineValidationTable(LEFT_TO_RIGHT, left_line_validation_table_);

    ExamineValidationTable(MID_TO_LEFT, mid_line_validation_table_);
    ExamineValidationTable(MID_TO_RIGHT, mid_line_validation_table_);

    ExamineValidationTable(RIGHT_TO_LEFT, right_line_validation_table_);
    ExamineValidationTable(RIGHT_TO_MID, right_line_validation_table_);



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
           r_info.emplace_back(ValidPoints{RIGHT_LINE, origin_point, true, true, false, next_direction_distance, search_direction});

            //l.emplace_back(left_point);
           // m.emplace_back(mid_point);
          r.emplace_back(origin_point);
       }
       /*else if(left && mid && !left_near && mid_near)
       {
           l.emplace_back(left_point);
           //m.emplace_back(mid_point);
           r.emplace_back(origin_point);
       }*/
       else if(left && left_near)
       {
           r_info.emplace_back(ValidPoints{RIGHT_LINE, origin_point, true, false, false, next_direction_distance, search_direction});

           //l.emplace_back(left_point);
           r.emplace_back(origin_point);

       }
       else if(mid && mid_near)
       {
           r_info.emplace_back(ValidPoints{RIGHT_LINE, origin_point, false, true, false, next_direction_distance, search_direction});

           //m.emplace_back(mid_point);
           r.emplace_back(origin_point);
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
            l_info.emplace_back(ValidPoints{LEFT_LINE, origin_point, false, true, true, next_direction_distance, search_direction});
            l.emplace_back(origin_point);
            //m.emplace_back(mid_point);
            //r.emplace_back(right_point);
       }
       else if(right && right_near)
       {
           l_info.emplace_back(ValidPoints{LEFT_LINE, origin_point, false, false, true, next_direction_distance, search_direction});
           l.emplace_back(origin_point);
           //r.emplace_back(right_point);
       }
       else if(mid && mid_near)
       {
           l_info.emplace_back(ValidPoints{LEFT_LINE, origin_point, false, true, false, next_direction_distance, search_direction});
           l.emplace_back(origin_point);
           //m.emplace_back(mid_point);
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
           m_info.emplace_back(ValidPoints{MID_LINE, origin_point, true, false, true, next_direction_distance, search_direction});
            //l.emplace_back(left_point);
            m.emplace_back(origin_point);
            //r.emplace_back(right_point);
       }
       else if(right && right_near)
       {
           m_info.emplace_back(ValidPoints{MID_LINE, origin_point, false, false, true, next_direction_distance, search_direction});
           m.emplace_back(origin_point);
           //r.emplace_back(right_point);

       }
       else if(left && left_near)
       {
            m_info.emplace_back(ValidPoints{MID_LINE, origin_point, true, false, false, next_direction_distance, search_direction});
           //l.emplace_back(left_point);
           m.emplace_back(origin_point);

       }

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
        if(left_line_validation_table_[i].GetMidPrediction() && left_line_validation_table_[i].GetRightPrediction())
        {
            circle(rgb, left_line_validation_table_[i].GetOriginPoint(), 1, Scalar(0, 0, 255),CV_FILLED );
       }
    }

    for(int i=0; i<mid_line_validation_table_.size(); i++)
    {
        if(mid_line_validation_table_[i].GetLeftPrediction() && mid_line_validation_table_[i].GetRightPrediction())
        {
            circle(rgb, mid_line_validation_table_[i].GetOriginPoint(), 1, Scalar(0, 255, 0),CV_FILLED );
        }
    }

    for(int i=0; i<right_line_validation_table_.size(); i++)
    {
        if(right_line_validation_table_[i].GetLeftPrediction() && right_line_validation_table_[i].GetMidPrediction())
        {
            circle(rgb, right_line_validation_table_[i].GetOriginPoint(), 1, Scalar(255, 0, 0),CV_FILLED );
        }
    }
}


void ValidLinePointSearch::DrawMinMaxFromDirectionInRange(Mat &rgb)
{
    if(left_line_minmax_elements_.initialized)
    {
        circle(rgb, left_line_minmax_elements_.x_min, 7, Scalar(0, 0, 255),CV_FILLED);
        circle(rgb, left_line_minmax_elements_.x_max, 7, Scalar(0, 255, 255),CV_FILLED);
        circle(rgb, left_line_minmax_elements_.y_min, 7, Scalar(255, 0, 255),CV_FILLED);
        circle(rgb, left_line_minmax_elements_.y_max, 7, Scalar(0, 255, 0),CV_FILLED);
    }

    if(mid_line_minmax_elements_.initialized)
    {
        circle(rgb, mid_line_minmax_elements_.x_min, 7, Scalar(0, 0, 255),CV_FILLED);
        circle(rgb, mid_line_minmax_elements_.x_max, 7, Scalar(0, 255, 255),CV_FILLED);
        circle(rgb, mid_line_minmax_elements_.y_min, 7, Scalar(255, 0, 255),CV_FILLED);
        circle(rgb, mid_line_minmax_elements_.y_max, 7, Scalar(0, 255, 0),CV_FILLED);
    }
    if(right_line_minmax_elements_.initialized)
    {
        circle(rgb, right_line_minmax_elements_.x_min, 7, Scalar(0, 0, 255),CV_FILLED);
        circle(rgb, right_line_minmax_elements_.x_max, 7, Scalar(0, 255,255),CV_FILLED);
        circle(rgb, right_line_minmax_elements_.y_min, 7, Scalar(255, 0, 255),CV_FILLED);
        circle(rgb, right_line_minmax_elements_.y_max, 7, Scalar(0, 255, 0),CV_FILLED);
    }
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

    rect_info_.clear();

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

    examined_regions_.clear();

    l.clear();
    m.clear();
    r.clear();


    l_info.clear();
   m_info.clear();
    r_info.clear();


    rect_mid_points_.clear();

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


void ValidLinePointSearch::DrawLastAdjacentPointMatch(Mat &rgb)
{

    if(left_line_last_adjacent_point_match_.mid_set)
    {
        circle(rgb, left_line_last_adjacent_point_match_.last_mid_point, 10, Scalar(255, 0, 255),CV_FILLED );
    }


    if(left_line_last_adjacent_point_match_.right_set)
    {
        circle(rgb, left_line_last_adjacent_point_match_.last_right_point, 10, Scalar(0, 255, 255),CV_FILLED );
    }


    if(mid_line_last_adjacent_point_match_.left_set)
    {
        circle(rgb, mid_line_last_adjacent_point_match_.last_left_point, 10, Scalar(255, 0, 255),CV_FILLED );
    }


    if(mid_line_last_adjacent_point_match_.right_set)
    {
        circle(rgb, mid_line_last_adjacent_point_match_.last_right_point, 10, Scalar(0, 255, 255),CV_FILLED );
    }


    if(right_line_last_adjacent_point_match_.left_set)
    {
        circle(rgb, right_line_last_adjacent_point_match_.last_left_point, 10, Scalar(255, 0, 255),CV_FILLED );
    }


    if(right_line_last_adjacent_point_match_.mid_set)
    {
        circle(rgb, right_line_last_adjacent_point_match_.last_mid_point, 10, Scalar(0, 255, 255) );
    }



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
