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
      const int kLineThreshold    = 200;
      const int kSearchWindowSize = 5;

      cv::Mat destination;


      //cv::Mat image = cv::imread("/home/tb/gazebo_road_generation/ros_ws/src/track_annotation/images/straight.png", CV_LOAD_IMAGE_GRAYSCALE);


      cv::Size image_size = image.size();
      const int kImgRows = image_size.height;
      const int kImgCols = image_size.width;

      const int kRowStride = 5;
      const int kIntensityThreshold = 200;

      vector<int> row_spikes(image.cols,0);

      clock_t begin = clock();

       // std::vector<std::tuple<int, int, int>> artefacts_info;

        std::vector<pair<int,int>> artefacts_count;

        /*
        int row0 = 357;
        int row1 = 329;
        int row2 = 300;
*/
        int row0 = 380;
        int row1 = 360;
        int row2 = 340;



        //std::multimap<std::vector<std::tuple<int, int, int>>,std::vector<pair<int,int>>> mm;

        std::multimap<int,std::tuple<int, int>> artefacts_info;





        //artefacts_info.emplace_back(row,artefact_length,artefact_start_id);
    /*
        mm.insert(pair<string,int>("b", 3));
        mm.insert(pair<string,int>("b", 3));
        mm.insert(pair<string,int>("a", 1));
        mm.insert(pair<string,int>("a", 2));
    */



    //mm.insert(make_pair("hello world"), i);

        artefacts_count.emplace_back(row0,0);
        artefacts_count.emplace_back(row1,0);
        artefacts_count.emplace_back(row2,0);


        vector<int> line_search_regions = {row0,row1,row2};


        for ( auto &row : line_search_regions )
        {

          fill(row_spikes.begin(), row_spikes.end(), 0);


          for (int i=((kRowStride-1)/2); i<kImgCols-((kRowStride-1)/2); i++)
          {
            for (int k= -((kRowStride-1)/2); k<=((kRowStride-1)/2); k++)
            {

              if((int)image.at<uchar>(row,i+k) >= kIntensityThreshold)
              {
                row_spikes.at(i) = 1;
              }

            }
          }

          int artefact_length=0;


          for (int i=0;i<kImgCols;i++)
          {
            if(row_spikes.at(i)==1)
            {

              int artefact_start_id = i;

              while(row_spikes.at(i) == 1 && i<kImgCols)
              {
                artefact_length++;
                i++;
              }

              artefacts_info.insert(pair<int,std::tuple<int,int>>(row, make_tuple(artefact_start_id,artefact_length)));
              //cout << "row: " << row << "  id: " << artefact_start_id << "  lenght: " << artefact_length << endl;
              artefact_length=0;

            }

          }




        }


        //int midline_artefacts_counter = 0;


        //vec.erase(vec.begin() + index);

    /*
        for ( auto &artefact : artefacts_info ) {

          if(get<0>(artefact) == row1)
          {
            midline_artefacts_counter++;
          }

        }
    */


    if (artefacts_info.count(row1) >= 3)
    {

    std::vector<pair<int,int>> true_points_row0;
    std::vector<pair<int,int>> true_points_row1;
    std::vector<pair<int,int>> true_points_row2;

    getTruePoints(row0,artefacts_info, true_points_row0);
    getTruePoints(row1,artefacts_info, true_points_row1);
    getTruePoints(row2,artefacts_info, true_points_row2);


    //artefacts_info.begin()

     //cout << "hi" << endl;


    const int kMidlineSearchSpace = 7;

    const int maxDistance = 15;

    vector<tuple<int,int,int,int>> matched_points;

     if(!true_points_row0.empty() && !true_points_row1.empty())
     {


       for(int i=0; i<true_points_row0.size();i++)
       {
         for(int j=0; j<true_points_row1.size();j++)
         {

           int g = row0 - row1;

           int a1 = true_points_row0.at(i).first - true_points_row1.at(j).first;

           int a2 = true_points_row0.at(i).second - true_points_row1.at(j).second;


           if(abs(a1) < maxDistance && abs(a2) < maxDistance)
           {
             matched_points.push_back(make_tuple(true_points_row0.at(i).first,
                                                 true_points_row0.at(i).second,
                                                 true_points_row1.at(j).first,
                                                 true_points_row1.at(j).second));
           }
           //cout << a1 << " " << a2 << endl;

            /*
           float val1 = 0, val2 = 0;

           float angle1 = 0;
           float angle2 = 0;

           if(a1 != 0)
           {
             float val1 = g/a1;
             angle1 = atan(val1) * 180/PI ;


             cout << "a1 " << atan(val1) * 180/PI << endl;
           }
           else
           {
             cout << "a1 straight" << endl;
             angle1 = 90;
           }



           if (a2 != 0)
           {
             float val2 = g/a2;
             angle2 = atan(val2) * 180/PI ;
             cout << "a2 " <<  atan(val2) * 180/PI << endl;
           }
           else
           {
             angle2 = 90;
             cout << "a2 straight" << endl;
           }


          */


         }
       }




      // cout << "tpsize: " << true_points_row0.at(0).first << endl;




          for ( auto &t : true_points_row0 )
          {
              std::cout <<"r0: " << t.first << " " << t.second << endl;
          }



      for ( auto &t : true_points_row1 )
      {
          std::cout <<"r1: " << t.first << " " << t.second << endl;
      }


      for ( auto &t : matched_points )
      {


        int mid_row0 = 0;
        int mid_row1 = 0;


        int k1_row0 = get<0>(t);
        int k2_row0 = get<1>(t);
        mid_row0 = (k1_row0 + k2_row0)/2;

        int k1_row1 = get<2>(t);
        int k2_row1 = get<3>(t);
        mid_row1 = (k1_row1 + k2_row1)/2;

        bool mid_pattern_row0 = false;
        bool mid_pattern_row1 = false;
        bool mid_pattern_row2 = false;

        for (int k= -((kMidlineSearchSpace-1)/2); k<=((kMidlineSearchSpace-1)/2); k++)
        {
            if((int)image.at<uchar>(row0,mid_row0+k)<= kIntensityThreshold) mid_pattern_row0 = true;
            if((int)image.at<uchar>(row1,mid_row1+k)>= kIntensityThreshold) mid_pattern_row1 = true;
            if((int)image.at<uchar>(row2,mid_row1+k)<= kIntensityThreshold) mid_pattern_row2 = true;
        }
/*
         circle(rgb, Point(get<0>(t),row0), 8, Scalar(0, 0, 255));
         circle(rgb, Point(get<1>(t),row0), 8, Scalar(0, 0, 255));

         circle(rgb, Point(get<2>(t),row1), 8, Scalar(255, 0, 0));
         circle(rgb, Point(get<3>(t),row1), 8, Scalar(255, 0, 0));
*/
         //cout << "row0: " << mid_pattern_row0 << " " << "row1: " << mid_pattern_row1 << " " << "row2: " << mid_pattern_row2 << endl;
         if(mid_pattern_row0 && mid_pattern_row1 && mid_pattern_row2)
         {



           circle(rgb, Point(get<0>(t),row0), 8, Scalar(0, 0, 255));
           circle(rgb, Point(get<1>(t),row0), 8, Scalar(0, 0, 255));
           circle(rgb, Point(mid_row0,row0), 8, Scalar(255, 0, 0));

           circle(rgb, Point(get<2>(t),row1), 8, Scalar(0, 255, 0));
           circle(rgb, Point(get<3>(t),row1), 8, Scalar(0, 255, 0));
           circle(rgb, Point(mid_row1,row1), 8, Scalar(255, 0, 0));


           circle(rgb, Point(mid_row1,row2), 8, Scalar(255, 0, 0));

         }

      }


/*
    if( matched_points.size() > 1)
      cout << "mps: " << endl;

    int mid_row0 = 0;
    int mid_row1 = 0;


    int k1_row0 = (true_points_row0.begin())->first;
    int k2_row0 = (true_points_row0.begin())->second;
    mid_row0 = (k1_row0 + k2_row0)/2;




    int k1_row1 = (true_points_row1.begin())->first;
    int k2_row1 = (true_points_row1.begin())->second;
    mid_row1 = (k1_row1 + k2_row1)/2;



    circle(rgb, Point((true_points_row1.begin())->first,row1), 8, Scalar(0, 255, 0));
    circle(rgb, Point((true_points_row1.begin())->second,row1), 8, Scalar(0, 255, 0));
    circle(rgb, Point(mid_row1,row1), 8, Scalar(255, 0, 0));


    circle(rgb, Point(mid_row1,row2), 8, Scalar(255, 0, 0));
    bool mid_pattern_row0 = false;
    bool mid_pattern_row1 = false;
    bool mid_pattern_row2 = false;




      for (int k= -((kMidlineSearchSpace-1)/2); k<=((kMidlineSearchSpace-1)/2); k++)
      {
          if((int)image.at<uchar>(row0,mid_row0+k)<= kIntensityThreshold) mid_pattern_row0 = true;
          if((int)image.at<uchar>(row1,mid_row1+k)>= kIntensityThreshold) mid_pattern_row1 = true;
          if((int)image.at<uchar>(row2,mid_row1+k)<= kIntensityThreshold) mid_pattern_row2 = true;
      }


    cout << "row0: " << mid_pattern_row0 << " " << "row1: " << mid_pattern_row1 << " " << "row2: " << mid_pattern_row2 << endl;
    if(mid_pattern_row0 && mid_pattern_row1 && mid_pattern_row2)
    {
      circle(rgb, Point((true_points_row0.begin())->first,row0), 8, Scalar(0, 0, 255));
      circle(rgb, Point((true_points_row0.begin())->second,row0), 8, Scalar(0, 0, 255));
      circle(rgb, Point(mid_row0,row0), 8, Scalar(255, 0, 0));

      circle(rgb, Point((true_points_row1.begin())->first,row1), 8, Scalar(0, 255, 0));
      circle(rgb, Point((true_points_row1.begin())->second,row1), 8, Scalar(0, 255, 0));
      circle(rgb, Point(mid_row1,row1), 8, Scalar(255, 0, 0));


      circle(rgb, Point(mid_row1,row2), 8, Scalar(255, 0, 0));

    }*/

}

}
else{
    line( rgb, Point( 0, row0 ), Point( 1279, row0), Scalar( 255, 0, 0 ),  1, 8 );
    line( rgb, Point( 0, row1 ), Point( 1279, row1), Scalar( 0, 255, 0 ),  1, 8 );
    line( rgb, Point( 0, row2 ), Point( 1279, row2), Scalar( 0, 0, 255 ),  1, 8 );
}
      cv::imshow("Result", rgb);
      cv::waitKey(1);

  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'mono8'.",
              msg->encoding.c_str());
  }



}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "track_lines_node");
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
