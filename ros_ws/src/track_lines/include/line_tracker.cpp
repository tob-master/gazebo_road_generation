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

void LineTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try {
          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");




          warpPerspective(cv_ptr->image, grey, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);

          grey= grey(Rect(0,0,1280,417));


          cv::cvtColor(grey, rgb, CV_GRAY2BGR);


          MidLineSearcher.FindMidlinesInFrame(grey);

          vector<pair<int,int>> midline_cluster_coordinates = MidLineSearcher.GetMidlinePoints(7);


            cout << "JOOO" << endl;
          MidLineSearcher.CheckClusterConnections();

         // cout << midline_cluster_coordinates.size() << endl;

           for (auto const& cluster : midline_cluster_coordinates)
           {
             circle(rgb, Point(cluster.second,cluster.first), 10, Scalar(0, 255, 0));

           }

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

  ;
};
