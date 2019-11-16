#include "valid_line_point_search.h"

ValidLinePointSearch::ValidLinePointSearch()
{

}



float ValidLinePointSearch::GetOrthogonalAngle(float angle, int line)
{
    if(line == LEFT_LINE)
    {
        int angle_ = (angle - 90);

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
}


int ValidLinePointSearch::GetPixelValue(int x, int y)
{
    return (int)current_image_.at<uchar>(Point(x,y));
}

int ValidLinePointSearch::GetPixelValue(Point point)
{
    return (int)current_image_.at<uchar>(point);
}


void ValidLinePointSearch::SearchOrthogonalValues(int point_in_search_direction_x,
                                                  int point_in_search_direction_y,
                                                  float orthogonal_angle,
                                                  vector<int>& orthogonal_line_activations,
                                                  vector<Point>& orthogonal_line_points)
{

         for(int current_distance=kMinMidLineDistance_; current_distance<kMaxMidLineDistance_; current_distance++)
         {
             int orthogonal_point_in_search_direction_x  = point_in_search_direction_x + current_distance * cos(orthogonal_angle*PI/180);
             int orthogonal_point_in_search_direction_y  = point_in_search_direction_y - current_distance * sin(orthogonal_angle*PI/180);


             Point current_point = Point(orthogonal_point_in_search_direction_x,
                                         orthogonal_point_in_search_direction_y);


             if(orthogonal_point_in_search_direction_x < kImageWidth_ && orthogonal_point_in_search_direction_x >= 0 &&
                orthogonal_point_in_search_direction_y < kImageHeight_ && orthogonal_point_in_search_direction_y >= 0)
             {


                 int pixel_value = GetPixelValue(current_point);

                 if(pixel_value > kMinMidLineIntensity_)
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



bool ValidLinePointSearch::SearchMidLineMatch(vector<int> orthogonal_line_activations, pair<int,int>& mid_line_match)
{

    vector<pair<int,int>> segments;

    int segment_width = 0;


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

            segments.push_back(make_pair(start_id,segment_width));
        }
    }



    if(segments.size() == 1)
    {

        int width = segments[0].second;

        if(width >= kMinMidLineWidth_ && width <= kMaxMidLineWidth_)
        {
           mid_line_match = segments[0];
           return true;
        }
        else
        {
            mid_line_match = make_pair(0,0);
            return false;
        }

    }
    else
    {
        mid_line_match = make_pair(0,0);
        return false;
    }
/*
    for(auto it:segments)
    {

        cout << it << " ";
    }
    if(segments.size()>0)
    {
        cout << endl;
    }
*/
}

void ValidLinePointSearch::FindValidPointsFromLeftLineFollow(Mat image, vector<ReducedPointDirection> left_line_directions)
{
    ClearMemory();
    SetImage(image);

    left_line_directions_ = left_line_directions;


    for(int i=0; i<left_line_directions_.size(); i++)
    {
        int   current_point_x       = left_line_directions_[i].x;
        int   current_point_y       = left_line_directions_[i].y;
        float angle_to_next_point   = left_line_directions_[i].angle;
        int   length_to_next_point  = left_line_directions_[i].length;


        float orthogonal_angle = GetOrthogonalAngle(angle_to_next_point, LEFT_LINE);

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
                                   orthogonal_line_points);


            //for(auto i:orthogonal_line_activations) cout << i << " ";


            //if(orthogonal_line_activations.size()!=25) cout << "si " << orthogonal_line_activations.size() << endl;

            pair<int,int> mid_line_match;

            bool is_matched = SearchMidLineMatch(orthogonal_line_activations,mid_line_match);


            if(is_matched)
            {
                int width = mid_line_match.second;
                int pos   = mid_line_match.first;


                int x_mean = 0;
                int y_mean = 0;

                for(int i=pos; i<pos+width; i++)
                {
                    x_mean += orthogonal_line_points[i].x;
                    y_mean += orthogonal_line_points[i].y;
                }

                x_mean /= width;
                y_mean /= width;


                //cout << x_mean << " " << y_mean << endl;

                left_line_follow_mid_line_points_.push_back(Point(x_mean,y_mean));

            }


        }

    }
}


void ValidLinePointSearch::DrawMidLinePoints(Mat &rgb)
{
    for(auto it: left_line_follow_mid_line_points_)
        circle(rgb, it, 7, Scalar(0, 255, 255),CV_FILLED );
}


void ValidLinePointSearch::SetImage(Mat image)
{
    current_image_ = image;
}


void ValidLinePointSearch::ClearMemory()
{
    left_line_follow_mid_line_points_.clear();
}
