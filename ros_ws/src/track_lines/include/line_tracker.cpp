#include "line_tracker.h"


void LineTracker::LoadStartOfLinesSearchInitializationParameters()
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
    n.getParam("/start_of_lines_search_init/window_size_for_mid_line_search", start_of_lines_search_init.window_size_for_mid_line_search);
    n.getParam("/start_of_lines_search_init/max_distance_between_adjacent_row_pairs", start_of_lines_search_init.max_distance_between_adjacent_row_pairs);
    n.getParam("/start_of_lines_search_init/car_position_in_frame", start_of_lines_search_init.car_position_in_frame);
    n.getParam("/start_of_lines_search_init/road_model_left_line", start_of_lines_search_init.road_model_left_line);
    n.getParam("/start_of_lines_search_init/road_model_right_line", start_of_lines_search_init.road_model_right_line);
    n.getParam("/start_of_lines_search_init/line_to_car_distance_threshold", start_of_lines_search_init.line_to_car_distance_threshold);

}

void LineTracker::LoadLineFollowerInitializationParameters()
{

    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/line_follower_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/line_follower_init/max_iterations", line_follower_init.max_iterations);
    n.getParam("/line_follower_init/search_radius", line_follower_init.search_radius);
    n.getParam("/line_follower_init/max_weight_direction_scaler", line_follower_init.max_weight_direction_scaler);
    n.getParam("/line_follower_init/field_of_view", line_follower_init.field_of_view);
    n.getParam("/line_follower_init/max_consecutive_back_steps", line_follower_init.max_consecutive_back_steps);
    n.getParam("/line_follower_init/min_travel_distance_to_not_got_stuck", line_follower_init.min_travel_distance_to_not_got_stuck);
    n.getParam("/line_follower_init/max_got_stuck_counts", line_follower_init.max_got_stuck_counts);

}

void LineTracker::LoadBirdseyeInitializationParameters()
{
    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/birdseye_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/birdseye_init/alpha", birdseye_init.alpha);
    n.getParam("/birdseye_init/beta", birdseye_init.beta);
    n.getParam("/birdseye_init/gamma", birdseye_init.gamma);
    n.getParam("/birdseye_init/fov", birdseye_init.fov);
    n.getParam("/birdseye_init/distance", birdseye_init.distance);

};




void LineTracker::LoadAllInitializationParameters()
{

    LoadStartOfLinesSearchInitializationParameters();
    LoadLineFollowerInitializationParameters();
    LoadBirdseyeInitializationParameters();
}



void LineTracker::InitializeBirdseyeTransformationMatrix()
{
  // default values of the homography parameters




  //double f, dist;
  //double alpha, beta, gamma;
  double alpha = ((double)birdseye_init.alpha - 90.)*PI/180;
  double beta = ((double)birdseye_init.beta - 90.)*PI/180;
  double gammma = ((double)birdseye_init.gamma - 90.)*PI/180;
  double f = (double) birdseye_init.fov;
  double dist = (double) birdseye_init.distance;


 double w = kInputImageWidth_, h = kInputImageHeight_;

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

  // Final and overall birdseye_transformation_matrix_rmation matrix
  birdseye_transformation_matrix_ = A2 * (T * (R * A1));
}













/*

void LineTracker::FollowLinePoints(Mat image_mono_, StartParameters start_parameters)
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
        FollowLine(image_mono_, start_left_x_, start_left_y_, start_angle_left_, LEFT_LINE);

        //cout << "ls: " << found_points_and_directions_left_line_.size() << endl;
        for(auto &it: found_points_and_directions_left_line_)
        {



            left_line_points_for_rdp_.push_back(RDP_Point(double(it.x),double(it.y)));

        }




        RamerDouglasPeucker(left_line_points_for_rdp_, 10.0, left_line_rdp_reduced_);


        for(size_t i=0;i< left_line_rdp_reduced_.size();i++)
        {


            circle(image_rgb_, Point(left_line_rdp_reduced_[i].first,left_line_rdp_reduced_[i].second), 7, Scalar(0, 255, 255));

            //cout << pointListOut[i].first << "," << pointListOut[i].second << endl;
        }



        line_follow_iterations_counter_ = 0;
        got_stuck_counter_ = 0;
        backwards_counter=0;
        found_points_and_directions_right_line_.clear();
        FollowLine(image_mono_, start_right_x_, start_right_y_, start_angle_right_, RIGHT_LINE);

        //cout << "rs: " << found_points_and_directions_right_line_.size() << endl;




        for(auto &it: found_points_and_directions_right_line_)
        {
            right_line_points_for_rdp_.push_back(RDP_Point(double(it.x),double(it.y)));
            //cout << "right("<<it.x<<","<<it.y<<") -> "<< it.angle * 180/PI << endl;
            //circle(image_rgb_, Point(it.x,it.y), 7, Scalar(255, 0, 255));
        }
        RamerDouglasPeucker(right_line_points_for_rdp_, 10.0, right_line_rdp_reduced_);



        for(size_t i=0;i< right_line_rdp_reduced_.size();i++)
        {


            circle(image_rgb_, Point(right_line_rdp_reduced_[i].first,right_line_rdp_reduced_[i].second), 7, Scalar(255, 0, 255));

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



}
*/
void LineTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try {

          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");

          warpPerspective(cv_ptr->image, image_mono_, birdseye_transformation_matrix_, kInputImageSize_, INTER_CUBIC | WARP_INVERSE_MAP);

          image_mono_ = image_mono_(Rect(0,0,image_width_,image_height_));

          cv::cvtColor(image_mono_, image_rgb_, CV_GRAY2BGR);

          clock_t begin = clock();

          if(StartOfLinesSearcher_->FindStartParameters(image_mono_))
          {
              //StartOfLinesSearcher_->DrawStartParameters(image_rgb_);
              LineFollower_->FollowLines(image_mono_,StartOfLinesSearcher_->GetStartParameters());

              vector<PointAndDirection> left_line, right_line;

              LineFollower_->GetLines(left_line,right_line);
              LineFollower_->DrawLinePoints(image_rgb_);

              double max_distance = 10;

              LinePointsReducer_->ReduceLinePoints(left_line,right_line,max_distance);
              LinePointsReducer_->DrawReducedLinePoints(image_rgb_);

              vector<ReducedPoints> left_line_points_reduced, right_line_points_reduced;
              vector<LengthAndDirectionFromConsecutiveReducedLinePoints> left_line_points_reduced_length_direction, right_line_points_reduced_length_direction;

              LinePointsReducer_->GetReducedLinePoints(left_line_points_reduced,right_line_points_reduced);
              LinePointsReducer_->GetLengthAndDirectionFromConsecutiveReducedLinePoints(left_line_points_reduced_length_direction,
                                                                                        right_line_points_reduced_length_direction);

              LinePointsReducer_->CoutLengthAndDirectionFromConsecutiveReducedLinePoints();







          }

          MidLineSearcher.FindMidLineClusters(image_mono_);
          MidLineSearcher.DrawClusters(image_rgb_);

          /*
           * TODO: Midline clean code
           *       Ramer douglas as class or in LineFollow ?
           *       CCL implementation
           */


          clock_t end = clock();
          double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
          cout << "fps: " << 1/elapsed_secs << endl;

          imshow("input",cv_ptr->image);
          imshow("output", image_rgb_);
          waitKey(0);

  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("Could not convert from '%s' to 'mono8'.",
              msg->encoding.c_str());
  }

};

LineTracker::LineTracker(ros::NodeHandle* nh_):n(*nh_),it(*nh_)
{
  camera_subscriber_ = it.subscribe("/rrbot/camera1/image_raw", 1, &LineTracker::imageCallback, this);

  LoadAllInitializationParameters();
  InitializeBirdseyeTransformationMatrix();

  StartOfLinesSearcher_ = new StartOfLinesSearch(image_height_,image_width_,start_of_lines_search_init);
  LineFollower_         = new LineFollower(image_height_,image_width_,line_follower_init);
  LinePointsReducer_    = new LinePointsReducer;

};


/*
            Mat kk;
           threshold(image_mono_, kk, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

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

            //kk = image_mono_;

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
