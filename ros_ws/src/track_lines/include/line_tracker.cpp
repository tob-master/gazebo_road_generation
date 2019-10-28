#include "line_tracker.h"



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

float LineTracker::CalculateAngle4Quadrants(int opposite, int adjacent)
{
    float angle = 0;

    if(adjacent != 0 && opposite != 0)
    {
        angle = atan(float(abs(opposite)/abs(adjacent)));
        angle = angle * 180/PI;

        if (adjacent > 0 && opposite > 0)
        ;
        else if (adjacent < 0 && opposite > 0)
        {
            angle = 180 - angle;
        }
        else if (adjacent < 0 && opposite < 0)
        {
            angle = 180 + angle;
        }
        else if (adjacent > 0 && opposite < 0)
        {
            angle = 360 - angle;
        }
        else {
            cout << "something went wrong??" << endl;
        }
    }
    else if(adjacent > 0 && opposite == 0)
    {
            angle = 0;
    }
    else if(adjacent < 0 && opposite == 0)
    {
            angle = 180;
    }
    else if(adjacent == 0 && opposite > 0)
    {
            angle = 90;
    }
    else if(adjacent == 0 && opposite < 0)
    {
            angle = 270;
    }
    else
    {
        angle = 0;
    }

    return angle;
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

    return CalculateAngle4Quadrants(opposite, adjacent);
}

int LineTracker::FollowLine(Mat grey, int start_x, int start_y, float search_direction, int line)
{
    if(line_follow_iterations_counter_ == 40

            )
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


   // cout << new_angle << " " << start_x << " " << start_y << " " << new_start_point << endl;


    //if(start_y < new_start_y) return 0;


    //if(new_angle == 270) new_angle = 90;

    if(new_angle > 180 && new_angle < 360) return 0;  new_angle = 90;



    if(line == LEFT_LINE)
        found_points_and_directions_left_line_.push_back(LineSearchFoundPointAndDirection{new_start_point.x,new_start_point.y,new_angle});
    if(line == RIGHT_LINE)
        found_points_and_directions_right_line_.push_back(LineSearchFoundPointAndDirection{new_start_point.x,new_start_point.y,new_angle});



    line_follow_iterations_counter_++;

    FollowLine(grey, new_start_point.x, new_start_point.y, new_angle, line);
}



void LineTracker::FollowLinePoints(Mat grey, vector<LineSearchStartParameters> line_search_start_parameters)
{


    int start_left_x = line_search_start_parameters[0].left_x;
    int start_left_y = line_search_start_parameters[0].left_y;
    float search_direction_left = line_search_start_parameters[0].left_angle  * (PI/180);

    int start_right_x = line_search_start_parameters[0].right_x;
    int start_right_y = line_search_start_parameters[0].right_y;
    float search_direction_right = line_search_start_parameters[0].right_angle  * (PI/180);


        line_follow_iterations_counter_ = 0;
        found_points_and_directions_left_line_.clear();
        FollowLine(grey, start_left_x, start_left_y, search_direction_left, LEFT_LINE);

        cout << "ls: " << found_points_and_directions_left_line_.size() << endl;
        for(auto &it: found_points_and_directions_left_line_)
        {

            circle(rgb, Point(it.x,it.y), 7, Scalar(0, 255, 255));
        }
        line_follow_iterations_counter_ = 0;
        found_points_and_directions_right_line_.clear();
        FollowLine(grey, start_right_x, start_right_y, search_direction_right, RIGHT_LINE);

        cout << "rs: " << found_points_and_directions_right_line_.size() << endl;
        for(auto &it: found_points_and_directions_right_line_)
        {

            circle(rgb, Point(it.x,it.y), 7, Scalar(255, 0, 255));
        }


    /*

    vector<pair<int,int>> line_follow_scanner;

    int search_width = 9;

    int field_of_view_ = 144;

    int start_x = line_search_start_parameters[0].left_x;
    int start_y = line_search_start_parameters[0].left_y;
    float search_direction = line_search_start_parameters[0].left_angle;




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


        imshow("d",cv_ptr->image);
          warpPerspective(cv_ptr->image, grey, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);

          grey= grey(Rect(0,0,1280,417));

           clock_t begin = clock();

           //Mat otsu;
           //GaussianBlur( grey, otsu, Size( 5, 5 ), 0, 0 );
           //int otsu_threshold = cv::threshold(grey, otsu, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

           // cout << otsu_threshold << endl;
/*
           imshow("grey", grey);
           imshow("otsu", otsu);
           waitKey(30);
**/
          cv::cvtColor(grey, rgb, CV_GRAY2BGR);

          //vector<LineSearchStartParameters> line_search_start_parameters;
/*
          LineClassifier.FindStartParametersForLineTracking(grey,line_search_start_parameters);
          LineClassifier.DrawStartParameters(rgb, line_search_start_parameters);


          if(!line_search_start_parameters.empty())
          {
              //cv::cvtColor(grey, grey, CV_BGR2GRAY);
              FollowLinePoints(grey, line_search_start_parameters);

          }

*/

          MidLineSearcher.FindMidLineClusters(grey);
          //MidLineSearcher.DrawMidLineClusters(rgb);
          MidLineSearcher.DrawConnectedClusters(rgb);

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
          clock_t end = clock();
          double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            cout << "fps: " << 1/elapsed_secs << endl;
          imshow("img",rgb);
          waitKey(0);

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

           //LineClassifier.FilterRows(grey);


           vector<tuple<int,int,int,int,int,int>> matched_pattern_coordinates = LineClassifier.SearchLineFeatures(grey);

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
  search_length_ = 9;

  field_of_view_ = 144;
  line_follow_iterations_counter_ = 0;

};
