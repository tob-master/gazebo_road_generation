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

           clock_t begin = clock();

           Mat lc_im;

          LineClassifier.FilterRowSegments(grey);
          LineClassifier.FindStartAndWidthOfRowSegments();
          if(LineClassifier.CheckMidRowMatch())
          {
              LineClassifier.RejectFalseWidthRowSegments();
              LineClassifier.RejectFalseDistantRowSegments();
              LineClassifier.RejectFalseDistantMidAndBottomRowSegments();
              LineClassifier.RejectFalseMidLineSegments();

          }


            lc_im= LineClassifier.DrawMatches();



          clock_t end = clock();
          double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

          //cout << "eltime: " << elapsed_secs << endl;

          imshow("grey",lc_im);

          waitKey(20);


           //Canny(grey, grey, 100, 200,3);
/*
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
*/
          //res.convertTo(res, CV_32SC1);
  //        threshold( res, res, 9, 0,THRESH_TOZERO );
          //cout << res << endl;
    /*      for(int i=0; i<1280; i++)
          {
              if((int)res.at<float>(i) > 0)
                cout << i << " " << (int)res.at<float>(i) << endl;
          }
      */     // cout << endl;

/*
          for(int i=0; i<1280; i++)
          {
              if(roi.at<uchar>(i)>0)
                cout << i << " " << (int)roi.at<uchar>(i) << endl;
          }
*//*

           // cout << m << endl;

          Mat res;



*/
          /*
          roi.convertTo(roi, CV_32FC1);

          matchTemplate( roi, m, res, CV_TM_SQDIFF_NORMED );
            normalize( res, res, 0, 1, NORM_MINMAX, -1, Mat() );
*/
         // cout << roi << endl;
/*
roi.convertTo(roi, CV_32FC1);

          filter2D(roi, res, -1 , m,Point(-1,-1));

cout << res << endl;

           threshold( res, res, 250, 0,THRESH_TOZERO );
            res.convertTo(res, CV_8UC1);
*/
          /*
 *
 *
 *
          for(int i=0; i<1280; i++)
          {
              if(res.at<uchar>(i)>0)
                cout << i << " " << (int)res.at<uchar>(i) << endl;
          }
          */  //cout << endl;
        //  imshow("grey",grey);
         // imshow("roi",roi);
          //imshow("filter",res);
         // waitKey(20);
          //memcpy(m.data, vec.data(), vec.size()*sizeof(uchar));


/*
          HoughLine.ApplyCannyEdge(grey, 100, 200,3);




          int min_line_length = 1;
          int max_line_gap = 3;


          HoughLine.ApplyHoughLines(1, CV_PI/180, 10, min_line_length, max_line_gap);
          HoughLine.ShowHoughLines();
*/
          //cv::cvtColor(grey, rgb, CV_GRAY2BGR);
        /*

          MidLineSearcher.ScanImageToFindMidLineClusters(grey);

          vector<pair<int,int>> midline_cluster_coordinates = MidLineSearcher.GetMidLineClustersCenterOfGravity();


            cout << "JOOO" << endl;
          MidLineSearcher.FindConnectedClusters();

         // cout << midline_cluster_coordinates.size() << endl;

           for (auto const& cluster : midline_cluster_coordinates)
           {
             circle(rgb, Point(cluster.second,cluster.first), 10, Scalar(0, 255, 0));

           }
*/
           //LineClassifier.FilterRows(grey);

           /*
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

  ;
};
