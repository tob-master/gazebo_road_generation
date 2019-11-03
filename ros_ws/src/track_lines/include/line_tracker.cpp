#include "line_tracker.h"





void LineTracker::LoadAllClassInitializationParameters()
{


    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/start_of_lines_search_init.yaml";
    const char *command = str.c_str();
    system(command);


    n.getParam("/start_of_lines_search_init/top_row", start_of_lines_search_init.top_row);
    n.getParam("/start_of_lines_search_init/mid_row", start_of_lines_search_init.mid_row);
    n.getParam("/start_of_lines_search_init/bottom_row", start_of_lines_search_init.bottom_row);
    n.getParam("/start_of_lines_search_init/min_line_width", start_of_lines_search_init.min_line_width);
    n.getParam("/start_of_lines_search_init/max_line_width", start_of_lines_search_init.max_line_width);
    n.getParam("/start_of_lines_search_init/min_track_width", start_of_lines_search_init.min_track_width);
    n.getParam("/start_of_lines_search_init/max_track_width", start_of_lines_search_init.max_track_width);
    n.getParam("/start_of_lines_search_init/window_size_for_line_search", start_of_lines_search_init.window_size_for_line_search);
    n.getParam("/start_of_lines_search_init/line_threshold", start_of_lines_search_init.line_threshold);
    n.getParam("/start_of_lines_search_init/mid_line_threshold", start_of_lines_search_init.mid_line_threshold);
    n.getParam("/start_of_lines_search_init/image_height", start_of_lines_search_init.image_height);
    n.getParam("/start_of_lines_search_init/image_width", start_of_lines_search_init.image_width);
    n.getParam("/start_of_lines_search_init/window_size_for_mid_line_search", start_of_lines_search_init.window_size_for_mid_line_search);
    n.getParam("/start_of_lines_search_init/max_distance_between_adjacent_row_pairs", start_of_lines_search_init.max_distance_between_adjacent_row_pairs);
    n.getParam("/start_of_lines_search_init/car_position_in_frame", start_of_lines_search_init.car_position_in_frame);
    n.getParam("/start_of_lines_search_init/road_model_left_line", start_of_lines_search_init.road_model_left_line);
    n.getParam("/start_of_lines_search_init/road_model_right_line", start_of_lines_search_init.road_model_right_line);
    n.getParam("/start_of_lines_search_init/line_to_car_distance_threshold", start_of_lines_search_init.line_to_car_distance_threshold);


}



void LineTracker::initBirdseye()
{
  // default values of the homography parameters
  int alpha_=34;
  int  beta_=90;
  int gamma_=90;
  int f_ = 211;
  int dist_ = 65;



  //double f, dist;
  //double alpha, beta, gamma;
  double alpha = ((double)alpha_ - 90.)*PI/180;
  double beta = ((double)beta_ - 90.)*PI/180;
  double gammma = ((double)gamma_ - 90.)*PI/180;
  double f = (double) f_;
  double dist = (double) dist_;


 double w = 1280., h = 720.;

  // Projection 2D -> 3D matrix
  Mat A1 = (Mat_<double>(4,3) <<
    1, 0, -w/2,
    0, 1, -h/2,
    0, 0,    0,
    0, 0,    1);

  // Rotation matrices around the X,Y,Z axis
  Mat RX = (Mat_<double>(4, 4) <<
    1,          0,           0, 0,
    0, cos(alpha), -sin(alpha), 0,
    0, sin(alpha),  cos(alpha), 0,
    0,          0,           0, 1);

  Mat RY = (Mat_<double>(4, 4) <<
    cos(beta), 0, -sin(beta), 0,
            0, 1,          0, 0,
    sin(beta), 0,  cos(beta), 0,
            0, 0,          0, 1);

  Mat RZ = (Mat_<double>(4, 4) <<
    cos(gammma), -sin(gammma), 0, 0,
    sin(gammma),  cos(gammma), 0, 0,
    0,          0,           1, 0,
    0,          0,           0, 1);

  // Composed rotation matrix with (RX,RY,RZ)
  Mat R = RX * RY * RZ;

  // Translation matrix on the Z axis change dist will change the height
  Mat T = (Mat_<double>(4, 4) <<           1, 0, 0, 0,           0, 1, 0, 0,           0, 0, 1, dist,           0, 0, 0, 1);       // Camera Intrisecs matrix 3D -> 2D
  Mat A2 = (Mat_<double>(3,4) <<
    f, 0, w/2, 0,
    0, f, h/2, 0,
    0, 0,   1, 0);

  // Final and overall transformation matrix
  transfo = A2 * (T * (R * A1));
}

Point LineTracker::PolarCoordinate(int x, int y, float a, int l)
{
    x = x + l * cos(a) + 0.5;
    y = y - l * sin(a) + 0.5;

    return Point(x,y);
}

Point LineTracker::ChangeToBrightestCoordinateWithinReach(Mat image, Point center_of_gravity)
{

    int x =  center_of_gravity.x;
    int y =  center_of_gravity.y;

    int cog_left   = (int)image.at<uchar>(Point(x-1,y));
    int cog_right  = (int)image.at<uchar>(Point(x+1,y));
    int cog        = (int)image.at<uchar>(Point(x,y));
    int cog_top    = (int)image.at<uchar>(Point(x,y-1));
    int cog_bottom = (int)image.at<uchar>(Point(x,y+1));

    int cog_in_reach_intensities[] = {cog_left,cog_right,cog,cog_top,cog_bottom};



    int max_id = distance(cog_in_reach_intensities, max_element(cog_in_reach_intensities, cog_in_reach_intensities+5));



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




int LineTracker::GetOtsuTheshold(Mat grey, int start_x, int start_y, float start_angle, float end_angle, float step)
{
    vector<int> scanned_intensities_for_otsu;

    for (float angle=start_angle; angle>=end_angle; angle-=step)
    {
        for (int current_search_length_=0; current_search_length_<search_length_; current_search_length_++)
        {
            Point current_point = PolarCoordinate(start_x,start_y,angle,current_search_length_);
            int intensity = (int)grey.at<uchar>(current_point);
            scanned_intensities_for_otsu.push_back(intensity);
        }
    }

    Mat scanned_intensities_for_otsu_mat( 1,scanned_intensities_for_otsu.size(), CV_32SC1,scanned_intensities_for_otsu.data());
    scanned_intensities_for_otsu_mat.convertTo(scanned_intensities_for_otsu_mat, CV_8UC1);

    return threshold(scanned_intensities_for_otsu_mat, scanned_intensities_for_otsu_mat, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
}

vector<LineSearchMoments> LineTracker::GetScannedMoments(Mat grey, int otsu_threshold, int start_x, int start_y, float start_angle, float end_angle, float step)
{
    vector<LineSearchMoments> scanned_moments;

    for (float angle=start_angle; angle>=end_angle; angle-=step)
    {
        int intensity_sum = 0;
        bool scan_terminated = false;


        for (int current_search_length_=0; current_search_length_<search_length_; current_search_length_++)
        {
            Point current_point = PolarCoordinate(start_x,start_y,angle,current_search_length_);
            int intensity = (int)grey.at<uchar>(current_point);



            if(intensity <= otsu_threshold)
            {
                Point current_point = PolarCoordinate(start_x,start_y,angle,current_search_length_-1);
                scanned_moments.push_back(LineSearchMoments{current_point.x,current_point.y,intensity_sum});
                scan_terminated = true;
                break;
            }

            intensity_sum += intensity;

        }

        if(!scan_terminated)
        {
            Point current_point = PolarCoordinate(start_x,start_y,angle,search_length_-1);
            scanned_moments.push_back(LineSearchMoments{current_point.x,current_point.y,intensity_sum});
        }
    }

    return scanned_moments;
}


Point LineTracker::GetCenterOfGravity(int start_x, int start_y, vector<LineSearchMoments> scanned_moments)
{
    float moment_weight = 0;
    float moment_x = 0;
    float moment_y = 0;

    int max_weight_id = 0;
    int max_weight = 0;

    for (int id=0; id<scanned_moments.size(); id++)
    {
        if(max_weight < scanned_moments[id].sum)
        {
            max_weight = scanned_moments[id].sum;
            max_weight_id = id;
        }
    }


    for (int id=0; id<scanned_moments.size(); id++)
    {
        int weight = scanned_moments[id].sum;
        int x = scanned_moments[id].x;
        int y = scanned_moments[id].y;

        if(id == max_weight_id) weight *= 6;

        moment_weight += weight;
        moment_x      += weight * x;
        moment_y      += weight * y;
    }

    int center_of_gravity_x = 0;
    int center_of_gravity_y = 0;

    if(moment_weight <= 0)
    {
        center_of_gravity_x = start_x;
        center_of_gravity_y = start_y;
    }
    else
    {
        center_of_gravity_x = moment_x / moment_weight;
        center_of_gravity_y = moment_y / moment_weight;
    }

    return Point(center_of_gravity_x,center_of_gravity_y);

}

float LineTracker::GetNewAngle(int start_x, int start_y, Point new_start_point)
{

    int opposite =  start_y - new_start_point.y;
    int adjacent =  new_start_point.x - start_x;

    return CalculateAngle4Quadrants(opposite, adjacent) * PI/180;
}

int LineTracker::FollowLine(Mat grey, int start_x, int start_y, float search_direction, int line)
{
    if(line_follow_iterations_counter_ == 150)
    {
        return 0;
    }
    if(start_x < search_length_ || start_y < search_length_ || (grey.cols-search_length_) < start_x || (grey.rows-search_length_) < start_y) return 0;

    float start_angle =  (field_of_view_/2 ) * (PI/180) + search_direction;
    float end_angle   =  search_direction - (field_of_view_/2) * (PI/180) - 0.001;
    float step        =  ((field_of_view_/ 4)* (PI/180)) ;


    int otsu_threshold = GetOtsuTheshold(grey, start_x, start_y, start_angle, end_angle, step);

    vector<LineSearchMoments> scanned_moments = GetScannedMoments(grey, otsu_threshold, start_x, start_y, start_angle, end_angle, step);

    Point center_of_gravity = GetCenterOfGravity(start_x, start_y, scanned_moments);

    Point new_start_point = ChangeToBrightestCoordinateWithinReach(grey, center_of_gravity);

    float new_angle = GetNewAngle(start_x, start_y, new_start_point);



    //TODO change to class variables
    //############################
/*
   if( new_angle > (search_direction+kMaxChangeInDegreePerIteration_* PI/180) || new_angle < (search_direction-kMaxChangeInDegreePerIteration_* PI/180)  )
    {
        //cout << "an " << new_angle*180/PI << " " << search_direction*180/PI  << " " << search_direction*180/PI+kMaxChangeInDegreePerIteration_ <<
        //        " " << search_direction*180/PI-kMaxChangeInDegreePerIteration_ << endl;
        new_angle = search_direction;
    }
*/



    if(new_start_point.y > start_y)
    {
        backwards_counter++;
    }
    else {
        backwards_counter = 0;
    }

    if(backwards_counter > 5)
    {
        //cout << "retback" << endl;
        return 0;
    }


    if(sqrt(pow(start_x-new_start_point.x,2) + pow(start_y -new_start_point.y,2)) < kMinDistanceToNotGotStuck_)
    {
        got_stuck_counter_++;
    }
    else {

        got_stuck_counter_ = 0;
    }



     if(got_stuck_counter_>6){
         //cout << "retstuck" << endl;
         return 0;
     }
//######################################
    //if(new_angle > 220 &&  new_angle < 320) new_angle = 90;

   // cout << new_angle << " " << start_x << " " << start_y << " " << new_start_point << endl;


    //if(start_y < new_start_y) return 0;


    //if(new_angle == 270) new_angle = 90;

    //if(new_angle > 220 && new_angle < 320) new_angle = 90;



    if(line == LEFT_LINE)
        found_points_and_directions_left_line_.push_back(LineSearchFoundPointAndDirection{new_start_point.x,new_start_point.y,new_angle});
    if(line == RIGHT_LINE)
        found_points_and_directions_right_line_.push_back(LineSearchFoundPointAndDirection{new_start_point.x,new_start_point.y,new_angle});



    line_follow_iterations_counter_++;




    FollowLine(grey, new_start_point.x, new_start_point.y, new_angle, line);
}





double PerpendicularDistance(const RDP_Point &pt, const RDP_Point &lineStart, const RDP_Point &lineEnd)
{
    double dx = lineEnd.first - lineStart.first;
    double dy = lineEnd.second - lineStart.second;

    //Normalise
    double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
    if(mag > 0.0)
    {
        dx /= mag; dy /= mag;
    }

    double pvx = pt.first - lineStart.first;
    double pvy = pt.second - lineStart.second;

    //Get dot product (project pv onto normalized direction)
    double pvdot = dx * pvx + dy * pvy;

    //Scale line direction vector
    double dsx = pvdot * dx;
    double dsy = pvdot * dy;

    //Subtract this from pv
    double ax = pvx - dsx;
    double ay = pvy - dsy;

    return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}

void RamerDouglasPeucker(const vector<RDP_Point> &pointList, double epsilon, vector<RDP_Point> &out)
{
    if(pointList.size()<2)
        throw invalid_argument("Not enough points to simplify");

    // Find the point with the maximum distance from line between start and end
    double dmax = 0.0;
    size_t index = 0;
    size_t end = pointList.size()-1;
    for(size_t i = 1; i < end; i++)
    {
        double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
        if (d > dmax)
        {
            index = i;
            dmax = d;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if(dmax > epsilon)
    {
        // Recursive call
        vector<RDP_Point> recResults1;
        vector<RDP_Point> recResults2;
        vector<RDP_Point> firstLine(pointList.begin(), pointList.begin()+index+1);
        vector<RDP_Point> lastLine(pointList.begin()+index, pointList.end());
        RamerDouglasPeucker(firstLine, epsilon, recResults1);
        RamerDouglasPeucker(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end()-1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
        if(out.size()<2)
            throw runtime_error("Problem assembling output");
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList[0]);
        out.push_back(pointList[end]);
    }
}



void LineTracker::FollowLinePoints(Mat grey, StartParameters start_parameters)
{



    left_line_points_for_rdp_.clear();
    right_line_points_for_rdp_.clear();

    left_line_rdp_reduced_.clear();
    right_line_rdp_reduced_.clear();





    int start_left_x_ = start_parameters.left_x;
    int start_left_y_ = start_parameters.left_y;
    float start_angle_left_ = start_parameters.left_angle  * (PI/180);

    int start_right_x_ = start_parameters.right_x;
    int start_right_y_ = start_parameters.right_y;
    float start_angle_right_ = start_parameters.right_angle  * (PI/180);


        line_follow_iterations_counter_ = 0;
        got_stuck_counter_ = 0;
        backwards_counter = 0;
        found_points_and_directions_left_line_.clear();
        FollowLine(grey, start_left_x_, start_left_y_, start_angle_left_, LEFT_LINE);

        //cout << "ls: " << found_points_and_directions_left_line_.size() << endl;
        for(auto &it: found_points_and_directions_left_line_)
        {



            left_line_points_for_rdp_.push_back(RDP_Point(double(it.x),double(it.y)));

        }




        RamerDouglasPeucker(left_line_points_for_rdp_, 10.0, left_line_rdp_reduced_);


        for(size_t i=0;i< left_line_rdp_reduced_.size();i++)
        {


            circle(rgb, Point(left_line_rdp_reduced_[i].first,left_line_rdp_reduced_[i].second), 7, Scalar(0, 255, 255));

            //cout << pointListOut[i].first << "," << pointListOut[i].second << endl;
        }



        line_follow_iterations_counter_ = 0;
        got_stuck_counter_ = 0;
        backwards_counter=0;
        found_points_and_directions_right_line_.clear();
        FollowLine(grey, start_right_x_, start_right_y_, start_angle_right_, RIGHT_LINE);

        //cout << "rs: " << found_points_and_directions_right_line_.size() << endl;




        for(auto &it: found_points_and_directions_right_line_)
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



       // right_line_rdp_reduced_



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

        }

/*
        for (auto&it:left_line_pointers)
        {
            cout << "l(" << get<0>(it) << "," << get<1>(it) << ")" << " " << get<2>(it) << " " << get<3>(it) << endl;
        }

        for (auto&it:right_line_pointers)
        {
            cout << "r(" << get<0>(it) << "," << get<1>(it) << ")" << " " << get<2>(it) << " " << get<3>(it) << endl;
        }
*/



    /*

    vector<pair<int,int>> line_follow_scanner;

    int search_width = 9;

    int field_of_view_ = 144;

    int start_x = start_parameters[0].left_x;
    int start_y = start_parameters[0].left_y;
    float search_direction = start_parameters[0].left_angle;




    if(search_direction == 0)
    {
        float start_angle =  (field_of_view_/2 ) * (PI/180) + PI/2;
        float end_angle   =  PI/2 - (field_of_view_/2) * (PI/180) - 0.001;
        float step        =  ((field_of_view_/ 4)* (PI/180)) ;

        for (float angle=start_angle; angle>=end_angle; angle-=step)
        {
          cout << start_angle << " " << end_angle << " " << angle << " " << step << endl;

          float sin_ = sin(angle);
          float cos_ = cos(angle);

          int x = cos_*search_width+0.5;
          int y = sin_*search_width+0.5;


          if (!(std::find(line_follow_scanner.begin(), line_follow_scanner.end(), pair<int,int>{x,y}) != line_follow_scanner.end()))
          {
            line_follow_scanner.push_back({x,y});
          }

         }


        vector<int> scanned_pixels_for_otsu;


        for(auto &direction_to_scan :  line_follow_scanner)
        {
            int end_x = start_x + direction_to_scan.first;
            int end_y = start_y - direction_to_scan.second;

            Point start_point(start_x,start_y);
            Point end_point(end_x,end_y);

            LineIterator it(grey, start_point, end_point, 4);



            cout << start_point << " " << end_point << endl;

            for(int i = 0; i < it.count; i++, ++it)
            {
                int pixel_intensity = (int)grey.at<uchar>(it.pos());
                scanned_pixels_for_otsu.push_back(pixel_intensity);

                cout << "it: " << it.pos() << endl;

            }
        }


        Mat m2( 1,scanned_pixels_for_otsu.size(), CV_32SC1,scanned_pixels_for_otsu.data());

        m2.convertTo(m2, CV_8UC1);
        //cout << m2 << endl;
        int otsu_threshold = threshold(m2, m2, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);



        vector<LineSearchMoments> scanned_moments;

        for(auto &direction_to_scan :  line_follow_scanner)
        {
            int end_x = start_x + direction_to_scan.first;
            int end_y = start_y - direction_to_scan.second;

            int intensity_sum = 0;

            Point start_point(start_x,start_y);
            Point end_point(end_x,end_y);

            LineIterator it(grey, start_point, end_point, 4);

            Point last_pos;
            Point current_pos;
            bool scan_terminated = false;

            for(int i = 0; i < it.count; i++, ++it)
            {

                int pixel_intensity = (int)grey.at<uchar>(it.pos());

                current_pos.x = it.pos().x;
                current_pos.y = it.pos().y;

                if(pixel_intensity <= otsu_threshold)
                {

                    scanned_moments.push_back(LineSearchMoments{last_pos.x,last_pos.y,intensity_sum});
                    scan_terminated = true;
                }

                last_pos.x = current_pos.x;
                last_pos.y = current_pos.y;

                intensity_sum += pixel_intensity;

            }
        }



        //cout << otsu_threshold << endl;

        Mat matrix= Mat::zeros(scanned_pixels_for_otsu.size(), 1, CV_32SC1);

        matrix.col(0).copyTo(scanned_pixels_for_otsu);


        //
           for(auto &it : scanned_pixels_for_otsu)
           {
                cout << it << endl;
           }


        int end_x = start_x + line_follow_scanner[0].first;
        int end_y = start_y - line_follow_scanner[0].second;

        Point start_point(start_x,start_y);
        Point end_point(end_x,end_y);

        LineIterator it(grey, start_point, end_point, 4);

        //vector<uchar> buf(it.count);

        cout << start_point << " " << end_point << endl;

        for(int i = 0; i < it.count; i++, ++it)
        {
            cout << "it: " << it.pos() << endl;
            //uchar val = grey.at<uchar>(it.pos());
            //CV_Assert(buf[i] == val);
        }


    for (auto &it: line_follow_scanner) {

        cout << it.first << " " << it.second << endl;
    }

    cout << "_" << endl;

*/

}

void LineTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try {



          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");






          warpPerspective(cv_ptr->image, grey, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);




          grey= grey(Rect(0,0,1280,417));



           //Mat otsu;
           //GaussianBlur( grey, otsu, Size( 5, 5 ), 0, 0 );
           //int otsu_threshold = cv::threshold(grey, otsu, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

           // cout << otsu_threshold << endl;
/*
           imshow("grey", grey);
           imshow("otsu", otsu);
           waitKey(30);




**//*
            Mat kk;
           threshold(grey, kk, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

           Mat labels;
               Mat stats;
               Mat centroids;
               cv::connectedComponentsWithStats(kk, labels, stats, centroids);

             imshow("labels", labels);
         cout << "labels: " << labels << endl;
         // cout << "stats: " << stats << endl;
          cout << "centroids: " << centroids << endl;
            cout << endl;

     */
            //Mat kk;

            //kk = grey;

            //Mat imBin;
            //threshold(kk,imBin,0,255,THRESH_BINARY| CV_THRESH_OTSU);


            //Mat stats, centroids, labelImage;
            //int nLabels = connectedComponentsWithStats(imBin, labelImage, stats, centroids, 8, CV_32S);

            //cout << stats << endl;
/*
            std::vector<Vec3b> colors(nLabels);
            colors[0] = Vec3b(0, 0, 0);//background


            for (int label = 1; label < nLabels; ++label)
            {

                colors[label] = Vec3b((rand() & 255), (rand() & 255), (rand() & 255));

            }

            Mat dst(kk.size(), CV_8UC3,Scalar(0,0,0));

            Mat maxMid =stats.col(4)<200;
            Mat minMid = stats.col(4)>50;

            for (int i = 1; i < nLabels; i++)
            {

                Mat mask(labelImage.size(), CV_8UC1, Scalar(0));
                if (maxMid.at<uchar>(i, 0) && minMid.at<uchar>(i,0))
                {
                    mask = mask | (labelImage==i);

                    //cout << mask << endl;


                    for (int r = 0; r < dst.rows; ++r){
                        for (int c = 0; c < dst.cols; ++c){

                            int t = mask.at<uchar>(r, c);

                            if(t==255)
                            {
                                dst.at<Vec3b>(r, c) = colors[i];

                            }
                        }
                    }


                }
            }

            //Mat r(kk.size(), CV_8UC1, Scalar(0));
            //kk.copyTo(r,mask);
            //imshow("Result", r);
            //imshow("imbin", imBin);
            imshow("colorccl", dst);

*/

       cv::cvtColor(grey, rgb, CV_GRAY2BGR);


          clock_t begin = clock();




          if(StartOfLinesSearcher_->FindStartParameters(grey))
          {
              StartOfLinesSearcher_->DrawStartParameters(rgb);
              LineFollower_->FollowLines(grey,StartOfLinesSearcher_->GetStartParametersForLineSearch());
              LineFollower_->DrawLinePoints(rgb);

          }


          clock_t end = clock();
          double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
          cout << "fps: " << 1/elapsed_secs << endl;


          imshow("input",cv_ptr->image);
          imshow("output", rgb);
          waitKey(0);

/*
          if(found_start_parameters)
          {

              //StartOfLinesSearcher_.DrawStartParameters(rgb);

              StartParameters start_parameters = StartOfLinesSearcher_.GetStartParametersForLineSearch();

              FollowLinePoints(grey, start_parameters);



          }

*/





          //MidLineSearcher.FindMidLineClusters(grey);
          //MidLineSearcher.DrawMidLineClusters(rgb);
          //MidLineSearcher.DrawConnectedClusters(rgb);

          /*
           * MidLineSearcher.ScanImageToFindMidLineClusters(grey);

          vector<pair<int,int>> midline_cluster_coordinates = MidLineSearcher.GetMidLineClustersCenterOfGravity();


            MidLineSearcher.FindConnectedClusters();

           for (auto const& cluster : midline_cluster_coordinates)
           {
             //cout << cluster.second << endl;
             circle(rgb, Point(cluster.first,cluster.second), 10, Scalar(0, 255, 255));

           }
*/


          //imshow("img",rgb);


          //cout << "eltime: " << elapsed_secs << endl;





/*
 * //Canny(grey, grey, 100, 200,3);
           grey(Rect(620, 380, 40, 20)) = 0;


          Mat roi;
          grey(Rect(0, 350, 1280, 1)).copyTo(roi);


          roi/=255;




          Mat m = Mat(1, 128, CV_32F);
          m=-1;

          m.at<float>(0)     = 2;
          m.at<float>(1)     = 2;
          m.at<float>(2)     = 2;
          m.at<float>(3)     = 2;

          m.at<float>(63)    = 2;
          m.at<float>(64)    = 2;
          m.at<float>(65)    = 2;

          m.at<float>(124)   = 2;
          m.at<float>(125)   = 2;
          m.at<float>(126)   = 2;
          m.at<float>(127)   = 2;

          roi.convertTo(roi, CV_32FC1);
          Mat res;
          filter2D(roi, res, -1 , m,Point(-1,-1));

          //res.convertTo(res, CV_32SC1);
  //        threshold( res, res, 9, 0,THRESH_TOZERO );
          //cout << res << endl;
          for(int i=0; i<1280; i++)
          {
              if((int)res.at<float>(i) > 0)
                cout << i << " " << (int)res.at<float>(i) << endl;
          }
           // cout << endl;


          for(int i=0; i<1280; i++)
          {
              if(roi.at<uchar>(i)>0)
                cout << i << " " << (int)roi.at<uchar>(i) << endl;
          }


           // cout << m << endl;

          Mat res;





          roi.convertTo(roi, CV_32FC1);

          matchTemplate( roi, m, res, CV_TM_SQDIFF_NORMED );
            normalize( res, res, 0, 1, NORM_MINMAX, -1, Mat() );

         // cout << roi << endl;

roi.convertTo(roi, CV_32FC1);

          filter2D(roi, res, -1 , m,Point(-1,-1));

cout << res << endl;

           threshold( res, res, 250, 0,THRESH_TOZERO );
            res.convertTo(res, CV_8UC1);

          for(int i=0; i<1280; i++)
          {
              if(res.at<uchar>(i)>0)
                cout << i << " " << (int)res.at<uchar>(i) << endl;
          }
           //cout << endl;
        //  imshow("grey",grey);
         // imshow("roi",roi);
          //imshow("filter",res);
         // waitKey(20);
          //memcpy(m.data, vec.data(), vec.size()*sizeof(uchar));



          HoughLine.ApplyCannyEdge(grey, 100, 200,3);




          int min_line_length = 1;
          int max_line_gap = 3;


          HoughLine.ApplyHoughLines(1, CV_PI/180, 10, min_line_length, max_line_gap);
          HoughLine.ShowHoughLines();

          //cv::cvtColor(grey, rgb, CV_GRAY2BGR);


          MidLineSearcher.ScanImageToFindMidLineClusters(grey);

          vector<pair<int,int>> midline_cluster_coordinates = MidLineSearcher.GetMidLineClustersCenterOfGravity();


            cout << "JOOO" << endl;
          MidLineSearcher.FindConnectedClusters();

         // cout << midline_cluster_coordinates.size() << endl;

           for (auto const& cluster : midline_cluster_coordinates)
           {
             circle(rgb, Point(cluster.second,cluster.first), 10, Scalar(0, 255, 0));

           }

           //StartOfLinesSearcher_.FilterRows(grey);


           vector<tuple<int,int,int,int,int,int>> matched_pattern_coordinates = StartOfLinesSearcher_.SearchLineFeatures(grey);

           for (int i=0; i<matched_pattern_coordinates.size(); i++)
           {



           circle(rgb, Point(get<0>(matched_pattern_coordinates.at(i)),380), 7, Scalar(0, 0, 255));
           circle(rgb, Point(get<1>(matched_pattern_coordinates.at(i)),380), 7, Scalar(0, 0, 255));
           circle(rgb, Point(get<4>(matched_pattern_coordinates.at(i)),380), 7, Scalar(255, 0, 0));

           circle(rgb, Point(get<2>(matched_pattern_coordinates.at(i)),360), 7, Scalar(0, 255, 0));
           circle(rgb, Point(get<3>(matched_pattern_coordinates.at(i)),360), 7, Scalar(0, 255, 0));
           circle(rgb, Point(get<5>(matched_pattern_coordinates.at(i)),360), 7, Scalar(255, 0, 0));


           circle(rgb, Point(get<5>(matched_pattern_coordinates.at(i)),340), 7, Scalar(255, 0, 0));

          }
          cv::imshow("Result", rgb);
          cv::waitKey(1);

            */

  } catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("Could not convert from '%s' to 'mono8'.",
              msg->encoding.c_str());
  }

};

LineTracker::LineTracker(ros::NodeHandle* nh_):n(*nh_),it(*nh_),taille(1280.,720.)
{

  initBirdseye();
  image_sub = it.subscribe("/rrbot/camera1/image_raw", 1, &LineTracker::imageCallback, this);
  search_length_ = 10;

  field_of_view_ = 144;
  line_follow_iterations_counter_ = 0;





  LoadAllClassInitializationParameters();

  StartOfLinesSearcher_ = new StartOfLinesSearch(start_of_lines_search_init);
  LineFollower_         = new LineFollower;


};
