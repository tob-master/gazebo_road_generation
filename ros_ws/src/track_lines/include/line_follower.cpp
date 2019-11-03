#include "line_follower.h"

LineFollower::LineFollower()
{

}


void LineFollower::SetImage(Mat image)
{
    image_ = image;
    //image_width_ = image_.cols;
    //image_height_ = image_.rows;
}


Point LineFollower::GetPolarCoordinate(int x, int y, float angle, int radius)
{
    x = x + radius * cos(angle) + 0.5;
    y = y - radius * sin(angle) + 0.5;

    return Point(x,y);
}

Point LineFollower::ChangeToBrightestCoordinateWithinReach(Point center_of_gravity)
{

    int x =  center_of_gravity.x;
    int y =  center_of_gravity.y;

    int left      = GetPixelValue(Point(x-1,y));
    int right     = GetPixelValue(Point(x+1,y));
    int mid       = GetPixelValue(Point(x,y));
    int top       = GetPixelValue(Point(x,y-1));
    int bottom    = GetPixelValue(Point(x,y+1));

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

int LineFollower::GetPixelValue(Point point)
{
    return (int)image_.at<uchar>(point);
}


vector<int> LineFollower::ScanIntensitiesInSearchDirection(int x, int y)
{
    vector<int> scanned_intensities;

    for (float angle=start_of_search_; angle>=end_of_search_; angle-=kStepFieldOfView_)
    {
        for (int current_search_radius=0; current_search_radius<kSearchRadius_; current_search_radius++)
        {
            Point current_point = GetPolarCoordinate(x,y,angle,current_search_radius);
            int intensity = GetPixelValue(current_point);
            scanned_intensities.push_back(intensity);
        }
    }

    return scanned_intensities;
}


int LineFollower::GetOtsuThreshold(int x, int y)
{
    vector<int> scanned_intensities = ScanIntensitiesInSearchDirection(x,y);

    Mat scanned_intensities_mat( 1,scanned_intensities.size(), CV_32SC1,scanned_intensities.data());
    scanned_intensities_mat.convertTo(scanned_intensities_mat, CV_8UC1);

    int otsu_threshold = threshold(scanned_intensities_mat, scanned_intensities_mat, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    return otsu_threshold;
}

vector<ScannedMoments> LineFollower::GetScannedMoments(int otsu_threshold, int x, int y)
{
    vector<ScannedMoments> scanned_moments;

    for (float angle=start_of_search_; angle>=end_of_search_; angle-=kStepFieldOfView_)
    {
        int intensity_sum = 0;
        bool scan_terminated = false;


        for (int current_search_radius=0; current_search_radius<kSearchRadius_; current_search_radius++)
        {
            Point current_point = GetPolarCoordinate(x,y,angle,current_search_radius);
            int intensity = GetPixelValue(current_point);



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
            Point current_point = GetPolarCoordinate(x,y,angle,kSearchRadius_-1);
            scanned_moments.push_back(ScannedMoments{current_point.x,current_point.y,intensity_sum});
        }
    }

    return scanned_moments;
}


void LineFollower::SearchMaxWeightMoment(SummedMoments &summed_moments, vector<ScannedMoments> scanned_moments)
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

void LineFollower::SumUpMoments(SummedMoments &summed_moments, vector<ScannedMoments> scanned_moments)
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


Point LineFollower::GetCenterOfGravity(int x, int y, vector<ScannedMoments> scanned_moments)
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

float LineFollower::GetNewAngle(int x, int y, Point new_start_point)
{

    int opposite =  y - new_start_point.y;
    int adjacent =  new_start_point.x - x;

    return CalculateAngle4Quadrants(opposite, adjacent) * PI/180;
}


bool LineFollower::MaxIterationsExceeded()
{
    return iterations_counter_ >= kMaxIterations_;
}

bool LineFollower::SearchRadiusIsNotInImage(int x, int y)
{
    return (x < kSearchRadius_ || y < kSearchRadius_ ||(image_width_  - kSearchRadius_)  < x || (image_height_ - kSearchRadius_)  < y);
}


void LineFollower::SetSearchDirectionParameters(float search_direction)
{
    start_of_search_ =  search_direction + kStartAngleFieldOfView_;
    end_of_search_   =  search_direction - kEndAngleFieldOfView_;
}

bool LineFollower::IsWalkingBackwards(int y, Point new_start_point)
{
    if(new_start_point.y > y)
    {
        walked_backwards_counter_++;
    }
    else {
        walked_backwards_counter_ = 0;
    }

    if(walked_backwards_counter_ > kMaxConsecutiveBackSteps_)
    {
        return true;
    }
    else {
        return false;
    }

}

bool LineFollower::HasGotStuck(int x, int y, Point new_start_point)
{
    int distance = sqrt(pow(x-new_start_point.x,2) + pow(y -new_start_point.y,2));

    if(distance < kMinTravelDistanceToNotGotStuck_)
    {
        got_stuck_counter_++;
    }
    else {

        got_stuck_counter_ = 0;
    }

    if(got_stuck_counter_>kMaxGotStuckCounts_){

         return true;
    }
    else {
        return false;
    }
}


void LineFollower::AddIteration(Point new_start_point, int new_angle, int line)
{
    if(line == LEFT_LINE)
        left_line_points_and_directions_.push_back(PointAndDirection{new_start_point.x,new_start_point.y,new_angle});

    if(line == RIGHT_LINE)
        right_line_points_and_directions_.push_back(PointAndDirection{new_start_point.x,new_start_point.y,new_angle});

    iterations_counter_++;
}

int LineFollower::FollowLine(int x, int y, float search_direction, int line)
{
    if(MaxIterationsExceeded() || SearchRadiusIsNotInImage(x, y)){ return 0; }

    SetSearchDirectionParameters(search_direction);

    int otsu_threshold = GetOtsuThreshold(x, y);

    vector<ScannedMoments> scanned_moments = GetScannedMoments(otsu_threshold, x, y);

    Point center_of_gravity = GetCenterOfGravity(x, y, scanned_moments);

    Point new_start_point = ChangeToBrightestCoordinateWithinReach(center_of_gravity);

    float new_angle = GetNewAngle(x, y, new_start_point);

    if(HasGotStuck(x,y,new_start_point) || IsWalkingBackwards(y,new_start_point)){ return 0; }

    AddIteration(new_start_point, new_angle, line);

    FollowLine(new_start_point.x, new_start_point.y, new_angle, line);

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







//######################################
    //if(new_angle > 220 &&  new_angle < 320) new_angle = 90;

   // cout << new_angle << " " << start_x << " " << start_y << " " << new_start_point << endl;


    //if(start_y < new_start_y) return 0;


    //if(new_angle == 270) new_angle = 90;

    //if(new_angle > 220 && new_angle < 320) new_angle = 90;










}



void LineFollower::SetStartParameters(StartParameters start_parameters)
{
    start_left_x_ = start_parameters.left_x;
    start_left_y_ = start_parameters.left_y;
    start_angle_left_ = start_parameters.left_angle  * (PI/180);

    start_right_x_ = start_parameters.right_x;
    start_right_y_ = start_parameters.right_y;
    start_angle_right_ = start_parameters.right_angle  * (PI/180);
}

void LineFollower::ResetCounters()
{
    iterations_counter_ = 0;
    got_stuck_counter_ = 0;
    walked_backwards_counter_ = 0;
}


void LineFollower::ClearMemory()
{
    left_line_points_and_directions_.clear();
    right_line_points_and_directions_.clear();
}


void LineFollower::DrawLinePoints(Mat &rgb)
{
    for(auto &it : left_line_points_and_directions_)
    {
        circle(rgb, Point(it.x,it.y), 7, Scalar(0, 255, 255));
    }

    for(auto &it : right_line_points_and_directions_)
    {
        circle(rgb, Point(it.x,it.y), 7, Scalar(255, 0, 255));
    }
}

void LineFollower::FollowLines(Mat image, StartParameters start_parameters)
{

    SetImage(image);
    SetStartParameters(start_parameters);

    ResetCounters();
    ClearMemory();

    FollowLine(start_left_x_, start_left_y_, start_angle_left_, LEFT_LINE);
    ResetCounters();
    FollowLine(start_right_x_, start_right_y_, start_angle_right_, RIGHT_LINE);

    /*
        //cout << "ls: " << left_line_points_and_directions_.size() << endl;
        for(auto &it: left_line_points_and_directions_)
        {



            left_line_points_for_rdp_.push_back(RDP_Point(double(it.x),double(it.y)));

        }




        RamerDouglasPeucker(left_line_points_for_rdp_, 10.0, left_line_rdp_reduced_);


        for(size_t i=0;i< left_line_rdp_reduced_.size();i++)
        {


            circle(rgb, Point(left_line_rdp_reduced_[i].first,left_line_rdp_reduced_[i].second), 7, Scalar(0, 255, 255));

            //cout << pointListOut[i].first << "," << pointListOut[i].second << endl;
        }



        iterations_counter_ = 0;
        got_stuck_counter_ = 0;
        walked_backwards_counter_=0;

        FollowLine(grey, start_right_x_, start_right_y_, start_angle_right_, RIGHT_LINE);

        //cout << "rs: " << right_line_points_and_directions_.size() << endl;




        for(auto &it: right_line_points_and_directions_)
        {
            right_line_points_for_rdp_.push_back(RDP_Point(double(it.x),double(it.y)));
            //cout << "right("<<it.x<<","<<it.y<<") -> "<< it.angle * 180/PI << endl;
            //circle(rgb, Point(it.x,it.y), 7, Scalar(255, 0, 255));
        }
        RamerDouglasPeucker(right_line_points_for_rdp_, 10.0, right_line_rdp_reduced_);



        for(size_t i=0;i< right_line_rdp_reduced_.size();i++)
        {


            circle(rgb, Point(right_line_rdp_reduced_[i].first,right_line_rdp_reduced_[i].second), 7, Scalar(255, 0, 255));

            //cout << pointListOut[i].first << "," << pointListOut[i].second << endl;
        }







        vector<float> left_angles;

        vector<float> right_angles;

        vector<tuple<int,int,int,int>> right_line_pointers;
        vector<tuple<int,int,int,int>> left_line_pointers;

        if(left_line_rdp_reduced_.size() > 1)
        {
            for(auto i=0; i<left_line_rdp_reduced_.size()-1; i++)
            {
                int x_bottom = left_line_rdp_reduced_[i].first;
                int y_bottom = left_line_rdp_reduced_[i].second;

                int x_top = left_line_rdp_reduced_[i+1].first;
                int y_top = left_line_rdp_reduced_[i+1].second;

                int opposite =  y_bottom - y_top;
                int adjacent =  x_top - x_bottom;

                int length = sqrt(pow(adjacent,2)+pow(opposite,2));

                int angle =  CalculateAngle4Quadrants(opposite, adjacent);

                left_line_pointers.push_back(make_tuple(x_bottom,y_bottom,length, angle));
            }
        }
        else{

            left_line_pointers.push_back(make_tuple(0,0,0,0));

        }


        if(right_line_rdp_reduced_.size() > 1)
        {
            for(auto i=0; i<right_line_rdp_reduced_.size()-1; i++)
            {
                int x_bottom = right_line_rdp_reduced_[i].first;
                int y_bottom = right_line_rdp_reduced_[i].second;

                int x_top = right_line_rdp_reduced_[i+1].first;
                int y_top = right_line_rdp_reduced_[i+1].second;

                int opposite =  y_bottom - y_top;
                int adjacent =  x_top - x_bottom;

                int length = sqrt(pow(adjacent,2)+pow(opposite,2));

                int angle =  CalculateAngle4Quadrants(opposite, adjacent);

                right_line_pointers.push_back(make_tuple(x_bottom,y_bottom,length, angle));

            }
        }
        else{

            right_line_pointers.push_back(make_tuple(0,0,0,0));

        }*/
}
