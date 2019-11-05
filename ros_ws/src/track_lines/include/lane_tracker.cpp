#include "lane_tracker.h"


void LaneTracker::LoadStartOfLinesSearchInitializationParameters()
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

void LaneTracker::LoadLineFollowerInitializationParameters()
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

void LaneTracker::LoadBirdseyeInitializationParameters()
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

void LaneTracker::LoadMidLineSearchInitializationParameters()
{
    string str = "rosparam load /home/tb/gazebo_road_generation/ros_ws/src/track_lines/initialization/mid_line_search_init.yaml";
    const char *command = str.c_str();
    system(command);

    n.getParam("/mid_line_search_init/min_pixel_value_for_clustering", mid_line_search_init.min_pixel_value_for_clustering);
    n.getParam("/mid_line_search_init/max_radial_scan_out_of_cluster_value", mid_line_search_init.max_radial_scan_out_of_cluster_value);
    n.getParam("/mid_line_search_init/radial_scan_scaling_factor", mid_line_search_init.radial_scan_scaling_factor);
    n.getParam("/mid_line_search_init/mid_line_length", mid_line_search_init.mid_line_length);
    n.getParam("/mid_line_search_init/min_valuable_cluster_size", mid_line_search_init.min_valuable_cluster_size);
    n.getParam("/mid_line_search_init/max_connected_cluster_distance", mid_line_search_init.max_connected_cluster_distance);
}




void LaneTracker::LoadAllInitializationParameters()
{

    LoadStartOfLinesSearchInitializationParameters();
    LoadLineFollowerInitializationParameters();
    LoadBirdseyeInitializationParameters();
    LoadMidLineSearchInitializationParameters();
}



void LaneTracker::InitializeBirdseyeTransformationMatrix()
{
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


void LaneTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        warpPerspective(cv_ptr->image, image_mono_, birdseye_transformation_matrix_, kInputImageSize_, INTER_CUBIC | WARP_INVERSE_MAP);
        image_mono_ = image_mono_(Rect(0,0,image_width_,image_height_));
        cv::cvtColor(image_mono_, image_rgb_, CV_GRAY2BGR);

        clock_t begin = clock();

        /*
        VanashingPoint.FindVanashingPoint(cv_ptr->image);
        VanashingPoint.ApplyCannyEdge();
        //VanashingPoint.ShowCannyEdgeImage();
        VanashingPoint.ApplyHoughLines();
        VanashingPoint.ComputeIntersections();
        //VanashingPoint.ShowHoughLines();
        */

        StartOfLinesSearchReturnInfo start_of_lines_search_return_info = StartOfLinesSearcher_->FindStartParameters(image_mono_);

        if(start_of_lines_search_return_info.has_found_start_parameters)
        {
            StartOfLinesSearcher_->DrawStartParameters(image_rgb_);

            LineFollowerReturnInfo line_follower_return_info = LineFollower_->FollowLines(image_mono_,StartOfLinesSearcher_->GetStartParameters());
            LineFollower_->CoutReturnInfo();

            vector<PointAndDirection> left_line, right_line;

            if(line_follower_return_info.left_line_iterations_counter >= 2)
            {
                LineFollower_->GetLine(left_line, LEFT_LINE);
                LineFollower_->DrawLinePoints(image_rgb_,LEFT_LINE);
            }

            if(line_follower_return_info.right_line_iterations_counter >= 2)
            {
                LineFollower_->GetLine(right_line, RIGHT_LINE);
                LineFollower_->DrawLinePoints(image_rgb_,RIGHT_LINE);
            }

            double max_distance = 10;

            LinePointsReducerReturnInfo line_points_reducer_return_info = LinePointsReducer_->ReduceLinePoints(left_line,right_line,max_distance);

            vector<ReducedPoints> left_line_points_reduced, right_line_points_reduced;
            vector<LengthAndDirectionFromConsecutiveReducedLinePoints> left_line_points_reduced_length_direction,
                                                                       right_line_points_reduced_length_direction;

            if(line_points_reducer_return_info.left_line_is_reduced)
            {
                LinePointsReducer_->DrawReducedLinePoints(image_rgb_,LEFT_LINE);
                LinePointsReducer_->GetReducedLinePoints(left_line_points_reduced,LEFT_LINE);
                LinePointsReducer_->GetLengthAndDirectionFromConsecutiveReducedLinePoints(left_line_points_reduced_length_direction, LEFT_LINE);
            }

            if(line_points_reducer_return_info.right_line_is_reduced)
            {
                LinePointsReducer_->DrawReducedLinePoints(image_rgb_,RIGHT_LINE);
                LinePointsReducer_->GetReducedLinePoints(right_line_points_reduced,RIGHT_LINE);
                LinePointsReducer_->GetLengthAndDirectionFromConsecutiveReducedLinePoints(right_line_points_reduced_length_direction,RIGHT_LINE);
            }

            LinePointsReducer_->CoutLengthAndDirectionFromConsecutiveReducedLinePoints();

        }

        MidLineSearchReturnInfo mid_line_search_return_info = MidLineSearcher->FindMidLineClusters(image_mono_);

        if(mid_line_search_return_info.has_found_mid_line_clusters)
        {
            //MidLineSearcher->DrawClusters(image_rgb_);

            if(mid_line_search_return_info.has_found_group)
            {
                MidLineSearcher->DrawConnectedClusters(image_rgb_);
                MidLineSearcher->CoutLengthAndDirectionOfConnectedClusters();
            }
        }

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

LaneTracker::LaneTracker(ros::NodeHandle* nh_):n(*nh_),it(*nh_)
{
  camera_subscriber_ = it.subscribe("/rrbot/camera1/image_raw", 1, &LaneTracker::imageCallback, this);

  LoadAllInitializationParameters();
  InitializeBirdseyeTransformationMatrix();

  StartOfLinesSearcher_ = new StartOfLinesSearch(image_height_,image_width_,start_of_lines_search_init);
  LineFollower_         = new LineFollower(image_height_,image_width_,line_follower_init);
  LinePointsReducer_    = new LinePointsReducer;
  MidLineSearcher       = new MidLineSearch(image_height_,image_width_,mid_line_search_init);

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
