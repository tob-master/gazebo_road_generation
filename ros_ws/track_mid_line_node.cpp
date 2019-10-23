#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <stdio.h>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <image_transport/image_transport.h>
#include <ctime>



#define PI 3.14

using namespace cv;
using namespace std;


const int kLineWitdhMin = 3;
const int kLineWidthMax = 9;

const int kTrackWidthMax = 143;
const int kTrackWidthMin = 117;


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
Size taille(w,h);
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
Mat transfo = A2 * (T * (R * A1));


// images
cv::Mat warped;
//cv::Mat cropped;





bool checkLineWidth(int w)
{
  if(w >= kLineWitdhMin && w <= kLineWidthMax) { return true; }
  else { return false; }
}


void getTruePoints(int row_id, std::multimap<int,std::tuple<int, int>> &row_elements, std::vector<pair<int,int>> &tp)
{
  std::pair < std::multimap<int,std::tuple<int, int>>::iterator,  std::multimap<int,std::tuple<int, int>>::iterator> ret;
  ret = row_elements.equal_range(row_id);


  std::vector<int> row_points;

  for (std::multimap<int,std::tuple<int, int>>::iterator it=ret.first; it!=ret.second; ++it)
  {
    //std::cout << "row: " << it->first << " id: " << std::get<0>(it->second) << " length: " << std::get<1>(it->second) << endl;

    int row = it->first;
    int idx = std::get<0>(it->second);
    int width = std::get<1>(it->second);


    bool is_in_range = checkLineWidth(width);

    if(is_in_range)
    {
      row_points.push_back(idx+width/2);
    }
    /*else {
      row_points.push_back(idx);
    }*/

  }


    vector<string> used_permutations;
    std::vector<string>::iterator it;

    for(int i=0; i<row_points.size(); i++)
    {
      for(int j=0; j<row_points.size(); j++)
      {
          if(i==j) continue;


          string permutation_ij = std::to_string(i) + std::to_string(j);

          it = find (used_permutations.begin(), used_permutations.end(), permutation_ij);

          if (it != used_permutations.end())
          {
            //std::cout << "Element found in myvector: " << *it << '\n';
            continue;
          }
          else
          {
            //std::cout << "Element not found in myvector\n";
            string permutation_ji = std::to_string(j) + std::to_string(i);
            used_permutations.push_back(permutation_ij);
            used_permutations.push_back(permutation_ji);
          }

          int w = abs(row_points.at(j)-row_points.at(i));
          //cout <<"row: "<< row_id<< " " << w << " " << row_points.at(j)<< " " << row_points.at(i)<< endl;
          if(w>=kTrackWidthMin && w<=kTrackWidthMax)
          {
            tp.push_back(make_pair(row_points.at(i),row_points.at(j)));
          }
      }
    }

   // cout << "finished TP" << endl;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {


    try {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");




      //Apply matrix transformation
      warpPerspective(cv_ptr->image, warped, transfo, taille, INTER_CUBIC | WARP_INVERSE_MAP);

      // Output

      cv::Mat image= warped(Rect(0,0,1280,417));
    cv::Mat rgb;
    cv::cvtColor(image, rgb, CV_GRAY2BGR);

            clock_t begin = clock();



      vector<pair<int,int>> radial_scan1_, radial_scan2_;

      float fac = 1.5;

      const int intensity_threshold = 240;
      const int max_candidate_intensity = 120;

      int midline_length = 30;

      const int pointerLen  = ((midline_length * fac)/2);
      const int pointerLen2 = ((midline_length * fac)/2) + 1;

      float start_angle = PI;
      float end_angle   = start_angle+2*PI;

      for (float angle=start_angle; angle<end_angle; angle+=0.002)
      {
        float sin_ = sin(angle);
        float cos_ = cos(angle);

        int xC = cos_*pointerLen+0.5;
        int yC = sin_*pointerLen+0.5;
        int xC2 = cos_*pointerLen2+0.5;
        int yC2 = sin_*pointerLen2+0.5;

        if (!(std::find(radial_scan1_.begin(), radial_scan1_.end(), pair<int,int>{xC,yC}) != radial_scan1_.end()))
        {
          radial_scan1_.push_back({xC,yC});
        }

        if (!(std::find(radial_scan2_.begin(), radial_scan2_.end(), pair<int,int>{xC2,yC2}) != radial_scan2_.end()))
        {
          radial_scan2_.push_back({xC2,yC2});
        }

      }

      map<pair<int,int>,int> candidates_count_hashmap;
      map<pair<int,int>,int> candidates_xmass_hashmap;
      map<pair<int,int>,int> candidates_ymass_hashmap;

      for (int i=pointerLen2; i<image.rows-pointerLen2; i++)
      {
        for (int j=pointerLen2; j<image.cols-pointerLen2; j++)
        {



          int M = (int)image.at<uchar>(Point(j,i));
          if (M > intensity_threshold)
          {
            bool is_candidate_scan = true;
            bool is_candidate_rescan = true;

            for (auto &it : radial_scan1_)
            {
              int x = j + it.first;
              int y = i + it.second;
              if((int)image.at<uchar>(y,x) >= max_candidate_intensity) is_candidate_scan = false;
            }
            for (auto &it : radial_scan2_)
            {
              int x = j + it.first;
              int y = i + it.second;
              if((int)image.at<uchar>(y,x) >= max_candidate_intensity) is_candidate_rescan = false;
            }

            if(is_candidate_scan && is_candidate_rescan)
            {


              int i_key = i / (pointerLen2*2);
              int j_key = j / (pointerLen2*2);

              cout << i_key << " " << j_key << endl;

              if ( candidates_count_hashmap.find(make_pair(i_key,j_key)) == candidates_count_hashmap.end() )
              {
                  candidates_count_hashmap[make_pair(i_key,j_key)] = 1;

                  candidates_xmass_hashmap[make_pair(i_key,j_key)] = i;
                  candidates_ymass_hashmap[make_pair(i_key,j_key)] = j;

                  for (auto const& hash : candidates_count_hashmap)
                  {
                      std::cout << "new: " << get<0>(hash.first) << " " << get<1>(hash.first) << " " << hash.second << std::endl ;
                  }

              }
              else
              {
                int count = candidates_count_hashmap.at(make_pair(i_key,j_key)) + 1;
                candidates_count_hashmap.at(make_pair(i_key,j_key)) = count;

                candidates_xmass_hashmap[make_pair(i_key,j_key)] += i;
                candidates_ymass_hashmap[make_pair(i_key,j_key)] += j;

                for (auto const& hash : candidates_count_hashmap)
                {
                    std::cout << "add: " << get<0>(hash.first)<< " " << get<1>(hash.first) << " " << hash.second << std::endl ;
                }

              }


              //circle(rgb, Point(j,i), 10, Scalar(0, 0, 255));

            }
         }






        }
      }

      for (auto const& hash : candidates_count_hashmap)
      {
          //std::cout << get<0>(hash.first)*pointerLen2 << " " << get<1>(hash.first)*pointerLen2 << " " << hash.second << std::endl ;

          if(hash.second >= 7)
          {

            int xx = candidates_xmass_hashmap.at(hash.first) / hash.second;
            int yy = candidates_ymass_hashmap.at(hash.first) / hash.second;

            cout << xx << " " << yy << " " << candidates_xmass_hashmap.at(hash.first) << " " << candidates_ymass_hashmap.at(hash.first) << endl;

            //int i = get<0>(hash.first)*pointerLen2*2;
            //int j = get<1>(hash.first)*pointerLen2*2;
            circle(rgb, Point(yy,xx), 10, Scalar(0, 255, 0));
          }
      }


      /*
      for (auto &it : radial_scan2_)
      {
        cout << it.first << " " << it.second << endl;
      }



      clock_t end = clock();
      double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
      std::cout << elapsed_secs << std::endl;
*/

      cv::imshow("Result", rgb);
      cv::waitKey(0);

  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'mono8'.",
              msg->encoding.c_str());
  }



}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "track_mid_line_node");
  ros::NodeHandle nh;

cv::namedWindow("Result", 1);


image_transport::ImageTransport it(nh);
image_transport::Subscriber sub = it.subscribe("/rrbot/camera1/image_raw", 1, imageCallback);


ros::spin();
cv::destroyWindow("Result");

ros::shutdown();

    return 0;
}


/*


for ( auto &t : true_points )
{
    std::cout << t.first << " " << t.second << endl;
}





for ( auto &t : true_points )
{
    std::cout << t.first << " " << t.second << endl;
}

*/

/*
    if(artefacts_info.count(row1) >= 3)
    {

      std::vector<pair<int,int>> true_points;

      if(artefacts_info.count(row1) == 3)
      {

        getTruePoints(row1,artefacts_info, true_points);


      }



}
*/

/*
      }
      else if (artefacts_info.count(row0) == 1) {

      }
      else if (artefacts_info.count(row0) == 0) {

      }
      else {

      }





      ret = artefacts_info.equal_range(row1);


      for (std::multimap<int,std::tuple<int, int>>::iterator it=ret.first; it!=ret.second; ++it)
      {
        std::cout << "row: " << it->first << " id: " << std::get<0>(it->second) << " length: " << std::get<1>(it->second) << endl;
      }


      ret = artefacts_info.equal_range(row2);


      for (std::multimap<int,std::tuple<int, int>>::iterator it=ret.first; it!=ret.second; ++it)
      {
        std::cout << "row: " << it->first << " id: " << std::get<0>(it->second) << " length: " << std::get<1>(it->second) << endl;
      }

*/
/*
      for ( auto &artefact : artefacts_info ) {

        if(get<0>(artefact) == row0)
        {
          get<0>(artefact);

        }

      }
*/





    /*
    for(int i=0;i<kImgCols;i++)
    {
      cout << (int)image.at<uchar>(400,i) << " ";
    }
*/
    /*
    for ( auto &t : row_spikes ) {
        std::cout << t << " ";

    }*/

  //  cout << row_spikes.size() << endl;

/*
  for (int j=0;j<kImgRows;j++)
  {
    for (int i=0; i<kImgCols; i++)
    {
      //std::cout << (int)image.at<uchar>(400,i) << " "; // std::endl;
      //if((int)image.at<uchar>(j,i);
    }
  }





  int yy = 0;

  const int* p = image.ptr<int>(400);
  std::vector<int> v(p, p + image.cols);

  for ( auto &i : v ) {
      //std::cout << i << std::endl;
      yy = i;
  }
*/
/*
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << elapsed_secs << std::endl;
*/
  //cv::imshow("Result", destination);
  //cv::waitKey(30);



    //ros::spin();


    //ros::shutdown();
