#include "valid_line_point_search.h"

ValidLinePointSearch::ValidLinePointSearch()
{

}



float ValidLinePointSearch::GetOrthogonalAngle(float angle, int SEARCH_LINE_CODE)
{
    int  angle_ = 0;

    if(SEARCH_LINE_CODE == LEFT_TO_MID || SEARCH_LINE_CODE == LEFT_TO_RIGHT)
    {
        angle_ = (angle - 90);

    }

    if(SEARCH_LINE_CODE == RIGHT_TO_MID || SEARCH_LINE_CODE == RIGHT_TO_LEFT)
    {
        angle_ = (angle + 90);

    }


    if(angle_ > 359)
    {
        angle_ = angle_ % 360;
    }

    if(angle_ < 0)
    {
        angle_ =  360 - abs(angle_);

    }

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


    if(SEARCH_LINE_CODE == LEFT_TO_MID)
    {
        return SearchLineDistanceThresholds{kMinLeftToMidLineDistance_,kMaxLeftToMidLineDistance_};
    }

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT)
    {
        return SearchLineDistanceThresholds{kMinLeftToRightLineDistance_,kMaxLeftToRightLineDistance_};
    }


    if(SEARCH_LINE_CODE == RIGHT_TO_MID)
    {
        return SearchLineDistanceThresholds{kMinRightToMidLineDistance_,kMaxRightToMidLineDistance_};
    }

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT)
    {
        return SearchLineDistanceThresholds{kMinRightToLeftLineDistance_,kMaxRightToLeftLineDistance_};
    }

}



int ValidLinePointSearch::GetMinPixelIntensityThreshold(int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID)
    {
        return kMinLeftToMidPixelIntensity_;
    }

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT)
    {
        return kMinLeftToRightPixelIntensity_;
    }

    if(SEARCH_LINE_CODE == RIGHT_TO_MID)
    {
        return kMinRightToMidPixelIntensity_;
    }

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT)
    {
        return kMinRightToLeftPixelIntensity_;
    }
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
    if(SEARCH_LINE_CODE == LEFT_TO_MID)
    {
        return SearchLineWidthThresholds{kMinLeftToMidLineWidth_,kMaxLeftToMidLineWidth_};
    }

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT)
    {
        return SearchLineWidthThresholds{kMinLeftToRightLineWidth_,kMaxLeftToRightLineWidth_};
    }


    if(SEARCH_LINE_CODE == RIGHT_TO_MID)
    {
        return SearchLineWidthThresholds{kMinRightToMidLineWidth_,kMaxRightToMidLineWidth_};
    }

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT)
    {
        return SearchLineWidthThresholds{kMinRightToLeftLineWidth_,kMaxRightToLeftLineWidth_};
    }
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

void ValidLinePointSearch::SetLine(vector<PointInDirection> line_directions, int START_LINE_CODE)
{

    if(START_LINE_CODE == LEFT_LINE)
    {
        left_line_directions_ = line_directions;
    }

    if(START_LINE_CODE == RIGHT_LINE)
    {
        right_line_directions_ = line_directions;
    }

    if(START_LINE_CODE != LEFT_LINE && START_LINE_CODE != RIGHT_LINE)
    {
        cout << "Wrong Line!" << endl;
    }
}



vector<PointInDirection> ValidLinePointSearch::GetLineDirections(int SEARCH_LINE_CODE)
{

    if(SEARCH_LINE_CODE == LEFT_TO_MID || SEARCH_LINE_CODE == LEFT_TO_RIGHT)
    {
        return left_line_directions_;
    }
    else if(SEARCH_LINE_CODE == RIGHT_TO_MID || SEARCH_LINE_CODE == RIGHT_TO_LEFT)
    {
        return right_line_directions_;
    }
    else
    {
       cout << "False Search Line Code!" << endl;
    }


}



void ValidLinePointSearch::FindValidPointsFromMidLineSearch()
{
   ClearMemory(SEARCH_LINE_CODE);

   vector<vector<PointInDirection>>

}



void ValidLinePointSearch::FindValidPointsFromLineFollow(int SEARCH_LINE_CODE)
{
    ClearMemory(SEARCH_LINE_CODE);


    vector<PointInDirection> line_directions = GetLineDirections(SEARCH_LINE_CODE);

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

            if(is_matched)
            {
                SafeLinePoint(line_match, orthogonal_line_points,SEARCH_LINE_CODE);
            }
        }
    }
}


void ValidLinePointSearch::SafeLinePoint(SegmentStartIDAndWidth line_match, vector<Point> orthogonal_line_points, int SEARCH_LINE_CODE)
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


    if(SEARCH_LINE_CODE == LEFT_TO_MID)
    {
        left_line_follow_mid_line_points_.push_back(Point(x_mean,y_mean));
    }

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT)
    {
        left_line_follow_right_line_points_.push_back(Point(x_mean,y_mean));
    }


    if(SEARCH_LINE_CODE == RIGHT_TO_MID)
    {
        right_line_follow_mid_line_points_.push_back(Point(x_mean,y_mean));
    }

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT)
    {
        right_line_follow_left_line_points_.push_back(Point(x_mean,y_mean));
    }



}

void ValidLinePointSearch::DrawLinePoints(Mat &rgb, int SEARCH_LINE_CODE)
{ 
    if(SEARCH_LINE_CODE == LEFT_TO_MID)
    {
        for(auto it: left_line_follow_mid_line_points_)
            circle(rgb, it, 4, Scalar(0, 0, 255),CV_FILLED );
    }

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT)
    {
        for(auto it: left_line_follow_right_line_points_)
            circle(rgb, it, 4, Scalar(0, 0, 255),CV_FILLED );
    }


    if(SEARCH_LINE_CODE == RIGHT_TO_MID)
    {
        for(auto it: right_line_follow_mid_line_points_)
            circle(rgb, it, 2, Scalar(0, 255, 0),CV_FILLED );
    }

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT)
    {
        for(auto it: right_line_follow_left_line_points_)
            circle(rgb, it, 4, Scalar(0, 255, 0),CV_FILLED );
    }

}


void ValidLinePointSearch::SetImage(Mat image)
{
    current_image_ = image;
}


void ValidLinePointSearch::ClearMemory(int SEARCH_LINE_CODE)
{
    if(SEARCH_LINE_CODE == LEFT_TO_MID)
    {
        left_line_follow_mid_line_points_.clear();
    }

    if(SEARCH_LINE_CODE == LEFT_TO_RIGHT)
    {
        left_line_follow_right_line_points_.clear();
    }


    if(SEARCH_LINE_CODE == RIGHT_TO_MID)
    {
        right_line_follow_mid_line_points_.clear();
    }

    if(SEARCH_LINE_CODE == RIGHT_TO_LEFT)
    {
        right_line_follow_left_line_points_.clear();
    }

}
