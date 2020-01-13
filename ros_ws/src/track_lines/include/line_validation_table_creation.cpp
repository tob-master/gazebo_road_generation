#include "line_validation_table_creation.h"

LineValidationTableCreation::LineValidationTableCreation(int image_height, int image_width, LineValidationTableCreationInitializationParameters init):
kImageWidth_(image_width),
kImageHeight_(image_height),
kMinLeftToRightLineDistance_(init.min_left_to_right_line_distance),
kMaxLeftToRightLineDistance_(init.max_left_to_right_line_distance),
kMinLeftToMidLineDistance_(init.min_left_to_mid_line_distance),
kMaxLeftToMidLineDistance_(init.max_left_to_mid_line_distance),
kMinRightToLeftLineDistance_(init.min_right_to_left_line_distance),
kMaxRightToLeftLineDistance_(init.max_right_to_left_line_distance),
kMinRightToMidLineDistance_(init.min_right_to_mid_line_distance),
kMaxRightToMidLineDistance_(init.max_right_to_mid_line_distance),
kMinMidToLeftLineDistance_(init.min_mid_to_left_line_distance),
kMaxMidToLeftLineDistance_(init.max_mid_to_left_line_distance),
kMinMidToRightLineDistance_(init.min_mid_to_right_line_distance),
kMaxMidToRightLineDistance_(init.max_mid_to_right_line_distance),
kMinLeftToRightPixelIntensity_(init.min_left_to_right_pixel_intensity),
kMinLeftToMidPixelIntensity_(init.min_left_to_mid_pixel_intensity),
kMinRightToLeftPixelIntensity_(init.min_right_to_left_pixel_intensity),
kMinRightToMidPixelIntensity_(init.min_right_to_mid_pixel_intensity),
kMinMidToLeftPixelIntensity_(init.min_mid_to_left_pixel_intensity),
kMinMidToRightPixelIntensity_(init.min_mid_to_right_pixel_intensity),
kMinLeftToRightLineWidth_(init.min_left_to_right_line_width),
kMaxLeftToRightLineWidth_(init.max_left_to_right_line_width),
kMinLeftToMidLineWidth_(init.min_left_to_mid_line_width),
kMaxLeftToMidLineWidth_(init.max_left_to_mid_line_width),
kMinRightToLeftLineWidth_(init.min_right_to_left_line_width),
kMaxRightToLeftLineWidth_(init.max_right_to_left_line_width),
kMinRightToMidLineWidth_(init.min_right_to_mid_line_width),
kMaxRightToMidLineWidth_(init.max_right_to_mid_line_width),
kMinMidToLeftLineWidth_(init.min_mid_to_left_line_width),
kMaxMidToLeftLineWidth_(init.max_mid_to_left_line_width),
kMinMidToRightLineWidth_(init.min_mid_to_right_line_width),
kMaxMidToRightLineWidth_(init.max_mid_to_right_line_width)
{

}


void LineValidationTableCreation::ClearMemory()
{
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

    left_line_points_in_drive_direction_.clear();
    mid_line_points_in_drive_direction_.clear();
    right_line_points_in_drive_direction_.clear();

}

void LineValidationTableCreation::SetImage(Mat image)
{
    image_ = image;
}

void LineValidationTableCreation::FindValidPointsFromLineFollow(vector<PointInDirection> line_directions, int SEARCH_LINE_CODE)
{
    FindValidPoints(line_directions, SEARCH_LINE_CODE);
}


void LineValidationTableCreation::FindValidPointsFromMidLineSearch(vector<vector<PointInDirection>> mid_line_directions_clusters, int SEARCH_LINE_CODE)
{

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



void LineValidationTableCreation::FindValidPoints(vector<PointInDirection> line_directions, int SEARCH_LINE_CODE)
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

            SearchOrthogonalValues(image_,
                                   point_in_search_direction_x,
                                   point_in_search_direction_y,
                                   orthogonal_angle,
                                   orthogonal_line_activations,
                                   orthogonal_line_points,
                                   kImageWidth_,kImageHeight_,
                                   kMinLeftToMidLineDistance_,kMaxLeftToMidLineDistance_,kMinLeftToRightLineDistance_,kMaxLeftToRightLineDistance_,kMinRightToMidLineDistance_,kMaxRightToMidLineDistance_,
                                   kMinRightToLeftLineDistance_,kMaxRightToLeftLineDistance_,kMinMidToLeftLineDistance_,kMaxMidToLeftLineDistance_,kMinMidToRightLineDistance_,kMaxMidToRightLineDistance_,
                                   kMinLeftToMidPixelIntensity_,kMinLeftToRightPixelIntensity_,kMinRightToMidPixelIntensity_,kMinRightToLeftPixelIntensity_,kMinMidToRightPixelIntensity_,kMinMidToLeftPixelIntensity_,
                                   SEARCH_LINE_CODE);

            SegmentStartIDAndWidth line_match;

            bool is_matched = CheckLineMatch(orthogonal_line_activations,
                                             line_match,
                                             kMinLeftToMidLineWidth_,
                                             kMaxLeftToMidLineWidth_,
                                             kMinLeftToRightLineWidth_,
                                             kMaxLeftToRightLineWidth_,
                                             kMinRightToMidLineWidth_,
                                             kMaxRightToMidLineWidth_,
                                             kMinRightToLeftLineWidth_,
                                             kMaxRightToLeftLineWidth_,
                                             kMinMidToRightLineWidth_,
                                             kMaxMidToRightLineWidth_,
                                             kMinMidToLeftLineWidth_,
                                             kMaxMidToLeftLineWidth_,
                                             SEARCH_LINE_CODE);

            SafeLinePoint(line_match,
                          orthogonal_line_points,
                          is_matched,
                          point_in_search_direction_x,
                          point_in_search_direction_y,
                          current_to_next_point_distance,
                          angle_to_next_point,
                          left_to_mid_search_info_,
                          left_to_right_search_info_,
                          right_to_mid_search_info_,
                          right_to_left_search_info_,
                          mid_to_right_search_info_,
                          mid_to_left_search_info_,
                          SEARCH_LINE_CODE);

        }
    }
}


float LineValidationTableCreation::GetOrthogonalAngle(float angle, int SEARCH_LINE_CODE)
{
    int  angle_ = 0;

    if(SEARCH_LINE_CODE == LEFT_TO_MID || SEARCH_LINE_CODE == LEFT_TO_RIGHT || SEARCH_LINE_CODE == MID_TO_RIGHT) angle_ = (angle - 90);

    if(SEARCH_LINE_CODE == RIGHT_TO_MID || SEARCH_LINE_CODE == RIGHT_TO_LEFT || SEARCH_LINE_CODE == MID_TO_LEFT) angle_ = (angle + 90);

    if(angle_ > 359) angle_ = angle_ % 360;

    if(angle_ < 0) angle_ =  360 - abs(angle_);

    return float(angle_);
}


void LineValidationTableCreation::SearchOrthogonalValues( Mat image,
                                                          const int point_in_search_direction_x,
                                                          const int point_in_search_direction_y,
                                                          float orthogonal_angle,
                                                          vector<int>& orthogonal_line_activations,
                                                          vector<Point>& orthogonal_line_points,
                                                          const int kImageWidth,
                                                          const int kImageHeight,
                                                          const int kMinLeftToMidLineDistance,
                                                          const int MaxLeftToMidLineDistance,
                                                          const int kMinLeftToRightLineDistance,
                                                          const int kMaxLeftToRightLineDistance,
                                                          const int kMinRightToMidLineDistance,
                                                          const int kMaxRightToMidLineDistance,
                                                          const int kMinRightToLeftLineDistance,
                                                          const int kMaxRightToLeftLineDistance,
                                                          const int kMinMidToLeftLineDistance,
                                                          const int kMaxMidToLeftLineDistance,
                                                          const int kMinMidToRightLineDistance,
                                                          const int kMaxMidToRightLineDistance,
                                                          const int kMinLeftToMidPixelIntensity,
                                                          const int kMinLeftToRightPixelIntensity,
                                                          const int kMinRightToMidPixelIntensity,
                                                          const int kMinRightToLeftPixelIntensity,
                                                          const int kMinMidToRightPixelIntensity,
                                                          const int kMinMidToLeftPixelIntensity,
                                                          const int SEARCH_LINE_CODE)
{

     SearchLineDistanceThresholds search_line_distance_thresholds = GetSearchLineDistanceThresholds(kMinLeftToMidLineDistance,
                                                                                                    MaxLeftToMidLineDistance,
                                                                                                    kMinLeftToRightLineDistance,
                                                                                                    kMaxLeftToRightLineDistance,
                                                                                                    kMinRightToMidLineDistance,
                                                                                                    kMaxRightToMidLineDistance,
                                                                                                    kMinRightToLeftLineDistance,
                                                                                                    kMaxRightToLeftLineDistance,
                                                                                                    kMinMidToLeftLineDistance,
                                                                                                    kMaxMidToLeftLineDistance,
                                                                                                    kMinMidToRightLineDistance,
                                                                                                    kMaxMidToRightLineDistance,
                                                                                                    SEARCH_LINE_CODE);
     int min_pixel_intensity_threshold = GetMinPixelIntensityThreshold(kMinLeftToMidPixelIntensity,
                                                                       kMinLeftToRightPixelIntensity,
                                                                       kMinRightToMidPixelIntensity,
                                                                       kMinRightToLeftPixelIntensity,
                                                                       kMinMidToRightPixelIntensity,
                                                                       kMinMidToLeftPixelIntensity,
                                                                       SEARCH_LINE_CODE);

     for(int current_distance=search_line_distance_thresholds.min; current_distance < search_line_distance_thresholds.max; current_distance++)
     {
         int orthogonal_point_in_search_direction_x  = point_in_search_direction_x + current_distance * cos(orthogonal_angle*PI/180);
         int orthogonal_point_in_search_direction_y  = point_in_search_direction_y - current_distance * sin(orthogonal_angle*PI/180);

         Point current_point = Point(orthogonal_point_in_search_direction_x,
                                     orthogonal_point_in_search_direction_y);

         if(orthogonal_point_in_search_direction_x < kImageWidth && orthogonal_point_in_search_direction_x >= 0 &&
            orthogonal_point_in_search_direction_y < kImageHeight && orthogonal_point_in_search_direction_y >= 0)
         {
             int pixel_value = GetPixelValue(image,current_point);

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


SearchLineDistanceThresholds LineValidationTableCreation::GetSearchLineDistanceThresholds(  const int kMinLeftToMidLineDistance,
                                                                                            const int kMaxLeftToMidLineDistance,
                                                                                            const int kMinLeftToRightLineDistance,
                                                                                            const int kMaxLeftToRightLineDistance,
                                                                                            const int kMinRightToMidLineDistance,
                                                                                            const int kMaxRightToMidLineDistance,
                                                                                            const int kMinRightToLeftLineDistance,
                                                                                            const int kMaxRightToLeftLineDistance,
                                                                                            const int kMinMidToLeftLineDistance,
                                                                                            const int kMaxMidToLeftLineDistance,
                                                                                            const int kMinMidToRightLineDistance,
                                                                                            const int kMaxMidToRightLineDistance,
                                                                                            const int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return SearchLineDistanceThresholds{kMinLeftToMidLineDistance,kMaxLeftToMidLineDistance};

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) return SearchLineDistanceThresholds{kMinLeftToRightLineDistance,kMaxLeftToRightLineDistance};

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return SearchLineDistanceThresholds{kMinRightToMidLineDistance,kMaxRightToMidLineDistance};

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return SearchLineDistanceThresholds{kMinRightToLeftLineDistance,kMaxRightToLeftLineDistance};

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return SearchLineDistanceThresholds{kMinMidToLeftLineDistance,kMaxMidToLeftLineDistance};

    if(SEARCH_LINE_CODE == MID_TO_RIGHT)return SearchLineDistanceThresholds{kMinMidToRightLineDistance,kMaxMidToRightLineDistance};
}



int LineValidationTableCreation::GetMinPixelIntensityThreshold(const int kMinLeftToMidPixelIntensity,
                                                               const int  kMinLeftToRightPixelIntensity,
                                                               const int  kMinRightToMidPixelIntensity,
                                                               const int  kMinRightToLeftPixelIntensity,
                                                               const int  kMinMidToRightPixelIntensity,
                                                               const int  kMinMidToLeftPixelIntensity,
                                                               const int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return kMinLeftToMidPixelIntensity;

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT)return kMinLeftToRightPixelIntensity;

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return kMinRightToMidPixelIntensity;

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return kMinRightToLeftPixelIntensity;

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return kMinMidToRightPixelIntensity;

    if(SEARCH_LINE_CODE == MID_TO_LEFT)return kMinMidToLeftPixelIntensity;
}


int LineValidationTableCreation::GetPixelValue(Mat image, int x, int y)
{
    return (int)image.at<uchar>(Point(x,y));
}

int LineValidationTableCreation::GetPixelValue(Mat image, Point point)
{
    return (int)image.at<uchar>(point);
}

bool LineValidationTableCreation::CheckLineMatch(vector<int> orthogonal_line_activations,
                                                 SegmentStartIDAndWidth& line_match,
                                                 const int kMinLeftToMidLineWidth,
                                                 const int kMaxLeftToMidLineWidth,
                                                 const int kMinLeftToRightLineWidth,
                                                 const int kMaxLeftToRightLineWidth,
                                                 const int kMinRightToMidLineWidth,
                                                 const int kMaxRightToMidLineWidth,
                                                 const int kMinRightToLeftLineWidth,
                                                 const int kMaxRightToLeftLineWidth,
                                                 const int kMinMidToRightLineWidth,
                                                 const int kMaxMidToRightLineWidth,
                                                 const int kMinMidToLeftLineWidth,
                                                 const int kMaxMidToLeftLineWidth,
                                                 const int SEARCH_LINE_CODE)
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
    SearchLineWidthThresholds search_line_width_thresholds = GetSearchLineWidthThresholds(kMinLeftToMidLineWidth,
                                                                                          kMaxLeftToMidLineWidth,
                                                                                          kMinLeftToRightLineWidth,
                                                                                          kMaxLeftToRightLineWidth,
                                                                                          kMinRightToMidLineWidth,
                                                                                          kMaxRightToMidLineWidth,
                                                                                          kMinRightToLeftLineWidth,
                                                                                          kMaxRightToLeftLineWidth,
                                                                                          kMinMidToRightLineWidth,
                                                                                          kMaxMidToRightLineWidth,
                                                                                          kMinMidToLeftLineWidth,
                                                                                          kMaxMidToLeftLineWidth,
                                                                                          SEARCH_LINE_CODE);

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



SearchLineWidthThresholds LineValidationTableCreation::GetSearchLineWidthThresholds(const int kMinLeftToMidLineWidth,
                                                                                    const int kMaxLeftToMidLineWidth,
                                                                                    const int kMinLeftToRightLineWidth,
                                                                                    const int kMaxLeftToRightLineWidth,
                                                                                    const int kMinRightToMidLineWidth,
                                                                                    const int kMaxRightToMidLineWidth,
                                                                                    const int kMinRightToLeftLineWidth,
                                                                                    const int kMaxRightToLeftLineWidth,
                                                                                    const int kMinMidToRightLineWidth,
                                                                                    const int kMaxMidToRightLineWidth,
                                                                                    const int kMinMidToLeftLineWidth,
                                                                                    const int kMaxMidToLeftLineWidth,
                                                                                    const int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return SearchLineWidthThresholds{kMinLeftToMidLineWidth,kMaxLeftToMidLineWidth};

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT)return SearchLineWidthThresholds{kMinLeftToRightLineWidth,kMaxLeftToRightLineWidth};

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return SearchLineWidthThresholds{kMinRightToMidLineWidth,kMaxRightToMidLineWidth};

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return SearchLineWidthThresholds{kMinRightToLeftLineWidth,kMaxRightToLeftLineWidth};

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return SearchLineWidthThresholds{kMinMidToRightLineWidth,kMaxMidToRightLineWidth};

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return SearchLineWidthThresholds{kMinMidToLeftLineWidth,kMaxMidToLeftLineWidth};
}

void LineValidationTableCreation::SafeLinePoint(SegmentStartIDAndWidth line_match,
                                                vector<Point> orthogonal_line_points,
                                                const bool is_matched,
                                                const int point_in_search_direction_x,
                                                const int point_in_search_direction_y,
                                                const int current_to_next_point_distance,
                                                const float angle_to_next_point,
                                                vector<ValidLinePointSearchInfo> &left_to_mid_search_info,
                                                vector<ValidLinePointSearchInfo> &left_to_right_search_info,
                                                vector<ValidLinePointSearchInfo> &right_to_mid_search_info,
                                                vector<ValidLinePointSearchInfo> &right_to_left_search_info,
                                                vector<ValidLinePointSearchInfo> &mid_to_right_search_info,
                                                vector<ValidLinePointSearchInfo> &mid_to_left_search_info,
                                                const int SEARCH_LINE_CODE)
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

    if(SEARCH_LINE_CODE == LEFT_TO_MID) left_to_mid_search_info.emplace_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) left_to_right_search_info.emplace_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) right_to_mid_search_info.emplace_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) right_to_left_search_info.emplace_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) mid_to_right_search_info.emplace_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});

    if(SEARCH_LINE_CODE == MID_TO_LEFT) mid_to_left_search_info.emplace_back(ValidLinePointSearchInfo{origin, search_direction, distance_to_next_direction, matched_point});
}






LineValidationTableCreationReturnInfo LineValidationTableCreation::CreateLineValidationTables()
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

    CheckDistanceFromPredictedToAdjacentPoint(left_line_validation_table_,
                           left_line_validation_table_,
                           mid_line_validation_table_,
                           right_line_validation_table_,
                           EMPTY_POINT_,
                           kMaxDistanceOfPredictedToAdjacentPoint_,
                           LEFT_TO_MID);

    CheckDistanceFromPredictedToAdjacentPoint(left_line_validation_table_,
                           left_line_validation_table_,
                           mid_line_validation_table_,
                           right_line_validation_table_,
                           EMPTY_POINT_,
                           kMaxDistanceOfPredictedToAdjacentPoint_,
                           LEFT_TO_RIGHT);

    CheckDistanceFromPredictedToAdjacentPoint(mid_line_validation_table_,
                           left_line_validation_table_,
                           mid_line_validation_table_,
                           right_line_validation_table_,
                           EMPTY_POINT_,
                           kMaxDistanceOfPredictedToAdjacentPoint_,
                           MID_TO_LEFT);

    CheckDistanceFromPredictedToAdjacentPoint(mid_line_validation_table_,
                           left_line_validation_table_,
                           mid_line_validation_table_,
                           right_line_validation_table_,
                           EMPTY_POINT_,
                           kMaxDistanceOfPredictedToAdjacentPoint_,
                           MID_TO_RIGHT);

    CheckDistanceFromPredictedToAdjacentPoint(right_line_validation_table_,
                           left_line_validation_table_,
                           mid_line_validation_table_,
                           right_line_validation_table_,
                           EMPTY_POINT_,
                           kMaxDistanceOfPredictedToAdjacentPoint_,
                           RIGHT_TO_LEFT);

    CheckDistanceFromPredictedToAdjacentPoint(right_line_validation_table_,
                           left_line_validation_table_,
                           mid_line_validation_table_,
                           right_line_validation_table_,
                           EMPTY_POINT_,
                           kMaxDistanceOfPredictedToAdjacentPoint_,
                           RIGHT_TO_MID);


    return GetReturnInfo(left_line_validation_table_,mid_line_validation_table_,right_line_validation_table_);

}

void LineValidationTableCreation::CheckDistanceFromPredictedToAdjacentPoint(vector<LineValidationTable>&table,
                                                         vector<LineValidationTable> left_line_validation_table,
                                                         vector<LineValidationTable> mid_line_validation_table,
                                                         vector<LineValidationTable> right_line_validation_table,
                                                         Point EMPTY_POINT,
                                                         const int kMaxPointDistance,
                                                         int SEARCH_LINE_CODE)
{


  for(int i=0; i<table.size(); i++)
    {
        Point adjacent_point_prediction = table[i].GetAdjacentPointPrediction(SEARCH_LINE_CODE);

        if(adjacent_point_prediction == EMPTY_POINT){ continue;}
        else{table[i].SetAdjacentPointPredictionFound(SEARCH_LINE_CODE,true);}

        if(AdjacentValidationTableIsEmpty(left_line_validation_table,
                                          mid_line_validation_table,
                                          right_line_validation_table,
                                          SEARCH_LINE_CODE))
                                          {continue;}

        Point min_distance_adjacent_point;
        int min_distance_adjacent_point_id;
        int min_distance;

        FindMinDistanceFromPredictionToAdjacentPoint(left_line_validation_table,
                                                     mid_line_validation_table,
                                                     right_line_validation_table,
                                                     adjacent_point_prediction,
                                                     min_distance_adjacent_point,
                                                     min_distance_adjacent_point_id,
                                                     min_distance,
                                                     SEARCH_LINE_CODE);

        if(min_distance < kMaxPointDistance)
        {
            table[i].SetAdjacentPointPrediction(SEARCH_LINE_CODE,true);
            table[i].SetAdjacentPointId(SEARCH_LINE_CODE,min_distance_adjacent_point_id);
        }
        else {continue;}

        // TODO: For more performance skip this
        // Check if direction of points is in range
        /*
        int adjacent_point_direction = GetAdjacentPointDirection(SEARCH_LINE_CODE,min_distance_adjacent_point_id);
        int origin_point_direction   = table[i].GetDirection();

        int direction_difference = abs(adjacent_point_direction - origin_point_direction);

        if(direction_difference < kMaxDirectionDifferenceOfLinePointsInDriveDirection_ || (360 - kMaxDirectionDifferenceOfLinePointsInDriveDirection_) < direction_difference)
        {
            table[i].SetAdjacentPointDirectionInRange(SEARCH_LINE_CODE,true);
        }

        Point origin_prediction = GetAdjacentPointOriginPrediction(SEARCH_LINE_CODE,min_distance_adjacent_point_id);

        if(origin_prediction == EMPTY_POINT_){ continue; }

        Point origin  = table[i].GetOriginPoint();
        double point_distance = Distance2d(origin,origin_prediction);

        if(point_distance < kMaxDistanceOfPredictedToAdjacentPoint_)
        {
            table[i].SetAdjacentPointOriginPrediction(SEARCH_LINE_CODE,true);
        }
        */

    }
}

bool LineValidationTableCreation::AdjacentValidationTableIsEmpty(vector<LineValidationTable> left_line_validation_table,
                                                                 vector<LineValidationTable> mid_line_validation_table,
                                                                 vector<LineValidationTable> right_line_validation_table,
                                                                 const int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return mid_line_validation_table.empty();

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) return right_line_validation_table.empty();

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return left_line_validation_table.empty();

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return right_line_validation_table.empty();

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return left_line_validation_table.empty();

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return mid_line_validation_table.empty();
}


void LineValidationTableCreation::FindMinDistanceFromPredictionToAdjacentPoint(vector<LineValidationTable> left_line_validation_table,
                                                                                vector<LineValidationTable> mid_line_validation_table,
                                                                                vector<LineValidationTable> right_line_validation_table,
                                                                                Point adjacent_point_prediction,
                                                                                Point &min_distance_adjacent_point,
                                                                                int &min_distance_adjacent_point_id,
                                                                                int &min_distance,
                                                                                const int SEARCH_LINE_CODE)
{

     vector<LineValidationTable> table;

    if(SEARCH_LINE_CODE == LEFT_TO_MID) table = mid_line_validation_table;

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) table = right_line_validation_table;

    if(SEARCH_LINE_CODE == MID_TO_LEFT) table = left_line_validation_table;

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) table = right_line_validation_table;

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) table = left_line_validation_table;

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) table = mid_line_validation_table;



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


double LineValidationTableCreation::Distance2d(const Point& p, LineValidationTable  hs)
{
    return sqrt(pow(p.x-hs.GetOriginPoint().x,2) + pow(p.y-hs.GetOriginPoint().y,2));
}

double LineValidationTableCreation::Distance2d(const Point p1, const Point p2)
{
    return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));
}

LineValidationTableCreationReturnInfo LineValidationTableCreation::GetReturnInfo(vector<LineValidationTable> left_line_validation_table,
                                                    vector<LineValidationTable> mid_line_validation_table,
                                                    vector<LineValidationTable> right_line_validation_table)
{
    int left_found_both_points_and_predictions = 0;
    int left_found_both_predictions = 0;
    int left_found_mid_prediction = 0;
    int left_found_right_prediction = 0;
    int left_found_both_points = 0;
    int left_found_mid_point = 0;
    int left_found_right_point = 0;

    for(auto it: left_line_validation_table)
    {
        if(it.GetFoundMidPoint() && it.GetFoundRightPoint() && it.GetMidPrediction() && it.GetRightPrediction())
        {
            left_found_both_points_and_predictions++;
        }
        else if(it.GetMidPrediction() && it.GetRightPrediction())
        {
            left_found_both_predictions++;
        }
        else if(it.GetMidPrediction())
        {
            left_found_mid_prediction++;
        }
        else if(it.GetRightPrediction())
        {
            left_found_right_prediction++;
        }
        else if(it.GetFoundMidPoint() && it.GetFoundRightPoint())
        {
            left_found_both_points++;
        }
        else if(it.GetFoundMidPoint())
        {
            left_found_mid_point++;
        }
        else if(it.GetFoundRightPoint())
        {
            left_found_right_point++;
        }
    }

    int mid_found_both_points_and_predictions = 0;
    int mid_found_both_predictions = 0;
    int mid_found_left_prediction = 0;
    int mid_found_right_prediction = 0;
    int mid_found_both_points = 0;
    int mid_found_left_point = 0;
    int mid_found_right_point = 0;

    for(auto it: mid_line_validation_table)
    {
        if(it.GetFoundLeftPoint() && it.GetFoundRightPoint() && it.GetLeftPrediction() && it.GetRightPrediction())
        {
            mid_found_both_points_and_predictions++;
        }
        else if(it.GetLeftPrediction() && it.GetRightPrediction())
        {
            mid_found_both_predictions++;
        }
        else if(it.GetLeftPrediction())
        {
            mid_found_left_prediction++;
        }
        else if(it.GetRightPrediction())
        {
            mid_found_right_prediction++;
        }
        else if(it.GetFoundLeftPoint() && it.GetFoundRightPoint())
        {
            mid_found_both_points++;
        }
        else if(it.GetFoundLeftPoint())
        {
            mid_found_left_point++;
        }
        else if(it.GetFoundRightPoint())
        {
            mid_found_right_point++;
        }
    }

cout << "xxxx" << mid_line_validation_table.size() << endl;
    int right_found_both_points_and_predictions = 0;
    int right_found_both_predictions = 0;
    int right_found_left_prediction = 0;
    int right_found_mid_prediction = 0;
    int right_found_both_points = 0;
    int right_found_left_point = 0;
    int right_found_mid_point = 0;

    for(auto it: right_line_validation_table)
    {
        if(it.GetFoundLeftPoint() && it.GetFoundMidPoint() && it.GetLeftPrediction() && it.GetMidPrediction())
        {
            right_found_both_points_and_predictions++;
        }
        else if(it.GetLeftPrediction() && it.GetMidPrediction())
        {
            right_found_both_predictions++;
        }
        else if(it.GetLeftPrediction())
        {
            right_found_left_prediction++;
        }
        else if(it.GetMidPrediction())
        {
            right_found_mid_prediction++;
        }
        else if(it.GetFoundLeftPoint() && it.GetFoundMidPoint())
        {
            right_found_both_points++;
        }
        else if(it.GetFoundLeftPoint())
        {
            right_found_left_point++;
        }
        else if(it.GetFoundMidPoint())
        {
            right_found_mid_point++;
        }
    }


    return LineValidationTableCreationReturnInfo{   left_found_both_points_and_predictions,
                                                    left_found_both_predictions,
                                                    left_found_mid_prediction,
                                                    left_found_right_prediction,
                                                    left_found_both_points,
                                                    left_found_mid_point,
                                                    left_found_right_point,
                                                    (int)left_line_validation_table.size(),
                                                    mid_found_both_points_and_predictions,
                                                    mid_found_both_predictions,
                                                    mid_found_left_prediction,
                                                    mid_found_right_prediction,
                                                    mid_found_both_points,
                                                    mid_found_left_point,
                                                    mid_found_right_point,
                                                    (int)mid_line_validation_table.size(),
                                                    right_found_both_points_and_predictions,
                                                    right_found_both_predictions,
                                                    right_found_left_prediction,
                                                    right_found_mid_prediction,
                                                    right_found_both_points,
                                                    right_found_left_point,
                                                    right_found_mid_point,
                                                    (int)right_line_validation_table.size()
                                                };

}


void LineValidationTableCreation::GetLineValidationTables(vector<LineValidationTable> &left_line_validation_table, vector<LineValidationTable> &mid_line_validation_table, vector<LineValidationTable>& right_line_validation_table)
{

    left_line_validation_table = left_line_validation_table_;
    mid_line_validation_table  = mid_line_validation_table_;
    right_line_validation_table = right_line_validation_table_;

}


void LineValidationTableCreation::GetLinePointsInDriveDirection(vector<LineValidationTable> &left_line_points_in_drive_direction,
                                                                vector<LineValidationTable> &mid_line_points_in_drive_direction,
                                                                vector<LineValidationTable> &right_line_points_in_drive_direction)
{


    ExtractLinePointsInDriveDirection(left_line_validation_table_,
                                      left_line_points_in_drive_direction,
                                      kMinStartDirectionOfLinePointsInDriveDirection_,
                                      kMaxStartDirectionOfLinePointsInDriveDirection_,
                                      kMaxDirectionDifferenceOfLinePointsInDriveDirection_);

    ExtractLinePointsInDriveDirection(mid_line_validation_table_,
                                      mid_line_points_in_drive_direction,
                                      kMinStartDirectionOfLinePointsInDriveDirection_,
                                      kMaxStartDirectionOfLinePointsInDriveDirection_,
                                      kMaxDirectionDifferenceOfLinePointsInDriveDirection_);

    ExtractLinePointsInDriveDirection(right_line_validation_table_,
                                      right_line_points_in_drive_direction,
                                      kMinStartDirectionOfLinePointsInDriveDirection_,
                                      kMaxStartDirectionOfLinePointsInDriveDirection_,
                                      kMaxDirectionDifferenceOfLinePointsInDriveDirection_);

}


void LineValidationTableCreation::ExtractLinePointsInDriveDirection(vector<LineValidationTable> line_validation_table,
                                                                    vector<LineValidationTable>& line_points_in_drive_direction,
                                                                    const int kMinStartDirection,
                                                                    const int kMaxStartDirection,
                                                                    const int kMaxDirectionDifference)
{
    if(line_validation_table.size()>0)
    {
        int cut_off_id = 0;
        bool cut_off = false;

        int tmp_direction = line_validation_table[0].GetDirection();

        if(tmp_direction > kMinStartDirection && tmp_direction < kMaxStartDirection)
        {
            for(int i=0; i<line_validation_table.size(); i++)
            {
                int current_direction =  line_validation_table[i].GetDirection();
                int direction_difference = abs(current_direction - tmp_direction);

                if(direction_difference > kMaxDirectionDifference || (360 - kMaxDirectionDifference) < direction_difference)
                {
                    cut_off_id =  i;
                    cut_off = true;
                    break;
                }
                tmp_direction = current_direction;
            }

            if(cut_off)
            {
                for(int i=0; i<cut_off_id; i++) line_points_in_drive_direction.emplace_back(line_validation_table[i]);
            }
            else{
                    copy(line_validation_table.begin(), line_validation_table.end(), back_inserter(line_points_in_drive_direction));
            }
        }
    }
}







/*

Point LineValidationTableCreation::GetAdjacentPointOriginPrediction(int SEARCH_LINE_CODE, int min_distance_adjacent_point_id)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id].GetLeftLinePointPrediction();

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id].GetLeftLinePointPrediction();

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id].GetMidLinePointPrediction();

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id].GetMidLinePointPrediction();

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id].GetRightLinePointPrediction();

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id].GetRightLinePointPrediction();
}



int LineValidationTableCreation::GetAdjacentPointDirection(int SEARCH_LINE_CODE, int min_distance_adjacent_point_id)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == MID_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == MID_TO_RIGHT) return right_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT) return left_line_validation_table_[min_distance_adjacent_point_id].GetDirection();

    if(SEARCH_LINE_CODE == RIGHT_TO_MID) return mid_line_validation_table_[min_distance_adjacent_point_id].GetDirection();
}

void LineValidationTableCreation::DrawSpline(Mat &rgb)
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


void LineValidationTableCreation::DrawValidatedPointsOnDriveWay(Mat &rgb)
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
*/

