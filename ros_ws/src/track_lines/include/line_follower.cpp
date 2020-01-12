#include "line_follower.h"

LineFollow::LineFollow(int image_height, int image_width, LineFollowerInitializationParameters init):
kImageWidth_(image_width),
kImageHeight_(image_height),
kMaxIterations_(init.max_iterations),
kSearchRadius_(init.search_radius),
kMaxWeightDirectionScaler_(init.max_weight_direction_scaler),
kFieldOfView_(init.field_of_view),
kMaxConsecutiveBackSteps_(init.max_consecutive_back_steps),
kMinTravelDistanceToNotGotStuck_(init.min_travel_distance_to_not_got_stuck),
kMaxGotStuckCounts_(init.max_got_stuck_counts),
kStartAngleFieldOfView_((kFieldOfView_/2 ) * (PI/180)),
kEndAngleFieldOfView_((kFieldOfView_/2 ) * (PI/180) - 0.001),
kStepFieldOfView_((kFieldOfView_/ 4) * (PI/180))
{

}


void LineFollow::SetImage(Mat image)
{
    image_ = image;
}


void LineFollow::SetStartParameters(StartParameters start_parameters)
{
    start_left_x_ = start_parameters.left_x;
    start_left_y_ = start_parameters.left_y;
    start_angle_left_ = start_parameters.left_angle  * (PI/180);
    found_left_line_ = start_parameters.found_left_line;

    start_right_x_ = start_parameters.right_x;
    start_right_y_ = start_parameters.right_y;
    start_angle_right_ = start_parameters.right_angle  * (PI/180);
    found_right_line_ = start_parameters.found_right_line;
}


void LineFollow::ClearMemory()
{
    left_line_max_iterations_exceeded_ = false;
    left_line_search_radius_out_of_image_= false;
    left_line_has_got_stuck_= false;
    left_line_is_walking_backwards_= false;
    left_line_iterations_counter_ = 0;
    left_line_got_stuck_counter_ = 0;
    left_line_walked_backwards_counter_ = 0;

    right_line_max_iterations_exceeded_= false;
    right_line_search_radius_out_of_image_= false;
    right_line_has_got_stuck_= false;
    right_line_is_walking_backwards_= false;
    right_line_iterations_counter_ = 0;
    right_line_got_stuck_counter_ = 0;
    right_line_walked_backwards_counter_ = 0;

    left_line_points_and_directions_.clear();
    right_line_points_and_directions_.clear();

    found_left_line_ = false;
    found_right_line_ = false;

}


void LineFollow::ResetCounters(int &iterations_counter, int &got_stuck_counter, int &walked_backwards_counter)
{
    iterations_counter = 0;
    got_stuck_counter = 0;
    walked_backwards_counter = 0;
}







int LineFollow::FollowLine(int x, int y, float search_direction, int line_type)
{
    if(MaxIterationsExceeded(iterations_counter_,left_line_max_iterations_exceeded_,right_line_max_iterations_exceeded_,kMaxIterations_,line_type)
    ||
    SearchRadiusIsNotInImage(x,y,left_line_search_radius_out_of_image_,right_line_search_radius_out_of_image_, kSearchRadius_,kImageWidth_,kImageHeight_,line_type))
    { return 0; }

    SetSearchDirectionParameters(search_direction,start_of_search_,end_of_search_,kStartAngleFieldOfView_ ,kEndAngleFieldOfView_);

    int otsu_threshold = GetOtsuThreshold(x, y,image_,start_of_search_, end_of_search_,kStepFieldOfView_,kSearchRadius_);

    vector<ScannedMoments> scanned_moments = GetScannedMoments(x,y,image_,otsu_threshold,start_of_search_,end_of_search_,kStepFieldOfView_,kSearchRadius_);

    Point center_of_gravity = GetCenterOfGravity(x, y, scanned_moments);

    Point new_start_point = ChangeToBrightestCoordinateWithinReach(image_,center_of_gravity);

    float new_angle = GetNewAngle(x, y, new_start_point);

    if(HasGotStuck(x,y,new_start_point,left_line_has_got_stuck_,right_line_has_got_stuck_,got_stuck_counter_,kMinTravelDistanceToNotGotStuck_,kMaxGotStuckCounts_,line_type)
    ||
       IsWalkingBackwards(y,new_start_point,left_line_is_walking_backwards_,right_line_is_walking_backwards_,walked_backwards_counter_,kMaxConsecutiveBackSteps_,line_type))
    { return 0; }

    AddIteration(new_start_point,left_line_points_and_directions_,right_line_points_and_directions_,iterations_counter_,new_angle,line_type);

    FollowLine(new_start_point.x, new_start_point.y, new_angle, line_type);





    //TODO: check if angle may change only a little is better
    //############################
    /*
       if( new_angle > (search_direction+kMaxChangeInDegreePerIteration_* PI/180) || new_angle < (search_direction-kMaxChangeInDegreePerIteration_* PI/180)  )
        {
            //cout << "an " << new_angle*180/PI << " " << search_direction*180/PI  << " " << search_direction*180/PI+kMaxChangeInDegreePerIteration_ <<
            //        " " << search_direction*180/PI-kMaxChangeInDegreePerIteration_ << endl;
            new_angle = search_direction;
        }
    */
    //####################



}



Point LineFollow::GetPolarCoordinate(int x, int y, float angle, int radius)
{
    x = x + radius * cos(angle) + 0.5;
    y = y - radius * sin(angle) + 0.5;

    return Point(x,y);
}

Point LineFollow::ChangeToBrightestCoordinateWithinReach(Mat image,Point center_of_gravity)
{

    int x =  center_of_gravity.x;
    int y =  center_of_gravity.y;

    int left      = GetPixelValue(image,Point(x-1,y));
    int right     = GetPixelValue(image,Point(x+1,y));
    int mid       = GetPixelValue(image,Point(x,y));
    int top       = GetPixelValue(image,Point(x,y-1));
    int bottom    = GetPixelValue(image,Point(x,y+1));

    int intensities[] = {left,right,mid,top,bottom};

    int max_id = distance(intensities, max_element(intensities, intensities+5));

    switch(max_id)
    {
        case 0:
                return Point(x-1,y);
                break;
        case 1:
                return Point(x+1,y);
                break;
        case 2:
                return Point(x,y);
                break;
        case 3:
                return Point(x,y-1);
                break;
        case 4:
                return Point(x,y+1);
                break;
        default:
                 cout << "something went wrong??" << endl;
                 return Point(x,y);
                 break;
    }

}

int LineFollow::GetPixelValue(Mat image, Point point)
{
    return (int)image.at<uchar>(point);
}


vector<int> LineFollow::ScanIntensitiesInSearchDirection(int x,
                                                         int y,
                                                         Mat image,
                                                         const float start_of_search,
                                                         const float end_of_search,
                                                         const float kStepFieldOfView,
                                                         const int kSearchRadius)
{
    vector<int> scanned_intensities;

    for (float angle=start_of_search; angle>=end_of_search; angle-=kStepFieldOfView)
    {
        for (int current_search_radius=0; current_search_radius<kSearchRadius; current_search_radius++)
        {
            Point current_point = GetPolarCoordinate(x,y,angle,current_search_radius);
            int intensity = GetPixelValue(image,current_point);
            scanned_intensities.push_back(intensity);
        }
    }

    return scanned_intensities;
}


int LineFollow::GetOtsuThreshold(int x,
                                 int y,
                                 Mat image,
                                 const float start_of_search,
                                 const float end_of_search,
                                 const float kStepFieldOfView,
                                 const int kSearchRadius)
{
    vector<int> scanned_intensities = ScanIntensitiesInSearchDirection(x,
                                                                       y,
                                                                       image,
                                                                       start_of_search,
                                                                       end_of_search,
                                                                       kStepFieldOfView,
                                                                       kSearchRadius);

    Mat scanned_intensities_mat( 1,scanned_intensities.size(), CV_32SC1,scanned_intensities.data());
    scanned_intensities_mat.convertTo(scanned_intensities_mat, CV_8UC1);

    int otsu_threshold = threshold(scanned_intensities_mat, scanned_intensities_mat, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    return otsu_threshold;
}

vector<ScannedMoments> LineFollow::GetScannedMoments(int x,
                                                     int y,
                                                     Mat image,
                                                     int otsu_threshold,
                                                     const float  start_of_search,
                                                     const float end_of_search,
                                                     const float kStepFieldOfView,
                                                     const int kSearchRadius)
{
    vector<ScannedMoments> scanned_moments;

    for (float angle=start_of_search; angle>=end_of_search; angle-=kStepFieldOfView)
    {
        int intensity_sum = 0;
        bool scan_terminated = false;


        for (int current_search_radius=0; current_search_radius<kSearchRadius; current_search_radius++)
        {
            Point current_point = GetPolarCoordinate(x,y,angle,current_search_radius);
            int intensity = GetPixelValue(image,current_point);



            if(intensity <= otsu_threshold)
            {
                Point current_point = GetPolarCoordinate(x,y,angle,current_search_radius-1);
                scanned_moments.push_back(ScannedMoments{current_point.x,current_point.y,intensity_sum});
                scan_terminated = true;
                break;
            }

            intensity_sum += intensity;

        }

        if(!scan_terminated)
        {
            Point current_point = GetPolarCoordinate(x,y,angle,kSearchRadius-1);
            scanned_moments.push_back(ScannedMoments{current_point.x,current_point.y,intensity_sum});
        }
    }

    return scanned_moments;
}





void LineFollow::CoutReturnInfo()
{
    LineFollowerReturnInfo info = GetReturnInfo();

    cout << "___LineFollow ReturnInfo___" << endl;
    cout << "LEFT: " << endl;
    cout << "max it: \t\t" <<std::boolalpha <<info.left_line_max_iterations_exceeded << endl;
    cout << "out range: \t\t" <<std::boolalpha << info.left_line_search_radius_out_of_image << endl;
    cout << "has got stuck: \t\t" << std::boolalpha << info.left_line_has_got_stuck << endl;
    cout << "walked backwards: \t" << std::boolalpha << info.left_line_is_walking_backwards << endl;
    cout << "it count: \t\t" << info.left_line_iterations_counter << endl;
    cout << "got stuck count: \t" << info.left_line_got_stuck_counter << endl;
    cout << "walked backwards count: " << info.left_line_walked_backwards_counter << endl<< endl;

    cout << "RIGHT: " << endl;
    cout << "max it: \t\t" <<std::boolalpha <<info.right_line_max_iterations_exceeded << endl;
    cout << "out range: \t\t" <<std::boolalpha << info.right_line_search_radius_out_of_image << endl;
    cout << "has got stuck: \t\t" << std::boolalpha << info.right_line_has_got_stuck << endl;
    cout << "walked backwards: \t" << std::boolalpha << info.right_line_is_walking_backwards << endl;
    cout << "it count: \t\t" << info.right_line_iterations_counter << endl;
    cout << "got stuck count: \t" << info.right_line_got_stuck_counter << endl;
    cout << "walked backwards count: " << info.right_line_walked_backwards_counter << endl;
        cout << "#######################################" << endl;

}


void LineFollow::SearchMaxWeightMoment(SummedMoments &summed_moments, vector<ScannedMoments> scanned_moments)
{
    for (int id=0; id<scanned_moments.size(); id++)
    {
        if(summed_moments.max_weight < scanned_moments[id].intensity_sum)
        {
            summed_moments.max_weight = scanned_moments[id].intensity_sum;
            summed_moments.max_weight_id = id;
        }
    }
}

void LineFollow::SumUpMoments(SummedMoments &summed_moments, vector<ScannedMoments> scanned_moments)
{
    for (int id=0; id<scanned_moments.size(); id++)
    {
        int weight = scanned_moments[id].intensity_sum;
        int x = scanned_moments[id].x;
        int y = scanned_moments[id].y;

        if(id == summed_moments.max_weight_id) weight *= kMaxWeightDirectionScaler_;

        summed_moments.moment_weight += weight;
        summed_moments.moment_x      += weight * x;
        summed_moments.moment_y      += weight * y;
    }
}


Point LineFollow::GetCenterOfGravity(int x, int y, vector<ScannedMoments> scanned_moments)
{

    SummedMoments summed_moments = {};

    SearchMaxWeightMoment(summed_moments, scanned_moments);
    SumUpMoments(summed_moments, scanned_moments);

    int center_of_gravity_x = 0;
    int center_of_gravity_y = 0;

    // TODO: when zero weight found there are no pixels so break out of loop ?
    if(summed_moments.moment_weight <= 0)
    {
        center_of_gravity_x = x;
        center_of_gravity_y = y;
    }
    else
    {
        center_of_gravity_x = summed_moments.moment_x / summed_moments.moment_weight;
        center_of_gravity_y = summed_moments.moment_y / summed_moments.moment_weight;
    }

    return Point(center_of_gravity_x,center_of_gravity_y);

}

float LineFollow::GetNewAngle(int x, int y, Point new_start_point)
{

    int opposite =  y - new_start_point.y;
    int adjacent =  new_start_point.x - x;

    return CalculateAngle4Quadrants(opposite, adjacent) * PI/180;
}


bool LineFollow::MaxIterationsExceeded(const int iterations_counter,
                                         bool &left_line_max_iterations_exceeded,
                                         bool &right_line_max_iterations_exceeded,
                                         const int kMaxIterations,
                                         const int line_type)
{
    if(iterations_counter >= kMaxIterations)
    {
        if(line_type == LEFT_LINE) left_line_max_iterations_exceeded = true;
        if(line_type == RIGHT_LINE) right_line_max_iterations_exceeded = true;

        return true;
    }
    else
    {
        return false;
    }
}

bool LineFollow::SearchRadiusIsNotInImage(int x,
                                            int y,
                                            bool &left_line_search_radius_out_of_image,
                                            bool &right_line_search_radius_out_of_image,
                                            const int kSearchRadius,
                                            const int kImageWidth,
                                            const int kImageHeight,
                                            int line_type)
{
    if(x < kSearchRadius || y < kSearchRadius ||(kImageWidth  - kSearchRadius)  < x || (kImageHeight - kSearchRadius)  < y)
    {
        if(line_type == LEFT_LINE) left_line_search_radius_out_of_image = true;
        if(line_type == RIGHT_LINE) right_line_search_radius_out_of_image = true;

        return true;
    }
    else
    {
        return false;
    }

}


void LineFollow::SetSearchDirectionParameters(float search_direction,
                                              float &start_of_search,
                                              float &end_of_search,
                                              const float kStartAngleFieldOfView,
                                              const float kEndAngleFieldOfView)
{
    start_of_search =  search_direction + kStartAngleFieldOfView;
    end_of_search   =  search_direction - kEndAngleFieldOfView;
}

bool LineFollow::IsWalkingBackwards(int y,
                                    Point new_start_point,
                                    bool &left_line_is_walking_backwards,
                                    bool &right_line_is_walking_backwards,
                                    int  &walked_backwards_counter,
                                    const int kMaxConsecutiveBackSteps,
                                    const int line_type)
{
    if(new_start_point.y > y)
    {
        walked_backwards_counter++;
    }
    else {
        walked_backwards_counter = 0;
    }

    if(walked_backwards_counter > kMaxConsecutiveBackSteps)
    {
        if(line_type == LEFT_LINE) left_line_is_walking_backwards = true;
        if(line_type == RIGHT_LINE) right_line_is_walking_backwards = true;

        return true;
    }
    else {
        return false;
    }

}

bool LineFollow::HasGotStuck(int x,
                             int y,
                             const Point new_start_point,
                             bool &left_line_has_got_stuck,
                             bool &right_line_has_got_stuck,
                             int &got_stuck_counter,
                             const int kMinTravelDistanceToNotGotStuck,
                             const int kMaxGotStuckCounts,
                             const int line_type)
{
    int distance = sqrt(pow(x-new_start_point.x,2) + pow(y -new_start_point.y,2));

    if(distance < kMinTravelDistanceToNotGotStuck)
    {
        got_stuck_counter++;
    }
    else {

        got_stuck_counter = 0;
    }

    if(got_stuck_counter>kMaxGotStuckCounts){

         if(line_type == LEFT_LINE) left_line_has_got_stuck = true;
         if(line_type == RIGHT_LINE) right_line_has_got_stuck = true;

         return true;
    }
    else {
        return false;
    }
}


void LineFollow::AddIteration(Point new_start_point,
                              vector<PointAndDirection> &left_line_points_and_directions,
                              vector<PointAndDirection> &right_line_points_and_directions,
                              int &iterations_counter,
                              const float new_angle,
                              const int line_type)
{
    if(line_type == LEFT_LINE)
        left_line_points_and_directions.push_back(PointAndDirection{new_start_point.x,new_start_point.y,new_angle});

    if(line_type == RIGHT_LINE)
        right_line_points_and_directions.push_back(PointAndDirection{new_start_point.x,new_start_point.y,new_angle});

    iterations_counter++;
}


void LineFollow::DrawLinePoints(Mat &rgb, int line)
{
    if(line == LEFT_LINE)
    {
        for(auto &it : left_line_points_and_directions_)
        {
            circle(rgb, Point(it.x,it.y), 7, Scalar(0, 255, 255));
        }
    }

    if(line == RIGHT_LINE)
    {
        for(auto &it : right_line_points_and_directions_)
        {
            circle(rgb, Point(it.x,it.y), 7, Scalar(255, 255, 0));
        }
    }
}


void LineFollow::SaveCounterValuesToReturnInfo(const int iterations_counter,
                                                 const int got_stuck_counter,
                                                 const int walked_backwards_counter,
                                                 int &left_line_iterations_counter,
                                                 int &left_line_got_stuck_counter,
                                                 int &left_line_walked_backwards_counter,
                                                 int &right_line_iterations_counter,
                                                 int &right_line_got_stuck_counter,
                                                 int &right_line_walked_backwards_counter,
                                                 int line_type)
{
    if(line_type == LEFT_LINE)
    {
        left_line_iterations_counter = iterations_counter;
        left_line_got_stuck_counter  = got_stuck_counter;
        left_line_walked_backwards_counter = walked_backwards_counter;
    }

    if(line_type == RIGHT_LINE)
    {
        right_line_iterations_counter = iterations_counter;
        right_line_got_stuck_counter  = got_stuck_counter;
        right_line_walked_backwards_counter = walked_backwards_counter;
    }
}


LineFollowerReturnInfo LineFollow::GetReturnInfo()
{
    return LineFollowerReturnInfo{  left_line_max_iterations_exceeded_,
                                    left_line_search_radius_out_of_image_,
                                    left_line_has_got_stuck_,
                                    left_line_is_walking_backwards_,
                                    left_line_iterations_counter_,
                                    left_line_got_stuck_counter_,
                                    left_line_walked_backwards_counter_,
                                    right_line_max_iterations_exceeded_,
                                    right_line_search_radius_out_of_image_,
                                    right_line_has_got_stuck_,
                                    right_line_is_walking_backwards_,
                                    right_line_iterations_counter_,
                                    right_line_got_stuck_counter_,
                                    right_line_walked_backwards_counter_};
}

LineFollowerReturnInfo LineFollow::FollowLines()
{
    ResetCounters(iterations_counter_, got_stuck_counter_, walked_backwards_counter_);

    if(found_left_line_)
    {
        FollowLine(start_left_x_, start_left_y_, start_angle_left_, LEFT_LINE);
    }
    SaveCounterValuesToReturnInfo(iterations_counter_,
                                  got_stuck_counter_,
                                  walked_backwards_counter_,
                                  left_line_iterations_counter_,
                                  left_line_got_stuck_counter_,
                                  left_line_walked_backwards_counter_,
                                  right_line_iterations_counter_,
                                  right_line_got_stuck_counter_,
                                  right_line_walked_backwards_counter_,
                                  LEFT_LINE);

    ResetCounters(iterations_counter_, got_stuck_counter_, walked_backwards_counter_);

    if(found_right_line_)
    {
        FollowLine(start_right_x_, start_right_y_, start_angle_right_, RIGHT_LINE);
    }
    SaveCounterValuesToReturnInfo(iterations_counter_,
                                  got_stuck_counter_,
                                  walked_backwards_counter_,
                                  left_line_iterations_counter_,
                                  left_line_got_stuck_counter_,
                                  left_line_walked_backwards_counter_,
                                  right_line_iterations_counter_,
                                  right_line_got_stuck_counter_,
                                  right_line_walked_backwards_counter_,
                                  RIGHT_LINE);


    return GetReturnInfo();
}

void LineFollow::GetLine(vector<PointAndDirection> &_line, int line)
{
    if(line == LEFT_LINE)
    {
        _line = left_line_points_and_directions_;
    }

    if(line == RIGHT_LINE)
    {
        _line = right_line_points_and_directions_;
    }
}

