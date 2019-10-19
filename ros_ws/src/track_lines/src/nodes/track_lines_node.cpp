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

    for(int i=0; i<row_points.size(); i++)
    {
      for(int j=0; j<row_points.size(); j++)
      {
          if(i==j) continue;

          int w = abs(row_points.at(j)-row_points.at(i));
          //cout << w << " " << row_points.at(j)<< " " << row_points.at(i)<< endl;
          if(w>=kTrackWidthMin && w<=kTrackWidthMax)
          {
            tp.push_back(make_pair(row_points.at(j),row_points.at(i)));
          }
      }
    }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "track_lines_node");
  ros::NodeHandle nh;


  const int kLineThreshold    = 200;
  const int kSearchWindowSize = 5;

  cv::Mat destination;


  cv::Mat image = cv::imread("/home/tb/gazebo_road_generation/ros_ws/src/track_annotation/images/straight.png", CV_LOAD_IMAGE_GRAYSCALE);


  cv::Size image_size = image.size();
  const int kImgRows = image_size.height;
  const int kImgCols = image_size.width;

  const int kRowStride = 5;
  const int kIntensityThreshold = 200;

  vector<int> row_spikes(image.cols,0);

  clock_t begin = clock();

   // std::vector<std::tuple<int, int, int>> artefacts_info;

    std::vector<pair<int,int>> artefacts_count;

    int row0 = 357;
    int row1 = 329;
    int row2 = 300;


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
          cout << "row: " << row << "  id: " << artefact_start_id << "  lenght" << artefact_length << endl;
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

std::vector<pair<int,int>> true_points_row0;
std::vector<pair<int,int>> true_points_row1;
std::vector<pair<int,int>> true_points_row2;

getTruePoints(row0,artefacts_info, true_points_row0);
getTruePoints(row1,artefacts_info, true_points_row1);
getTruePoints(row2,artefacts_info, true_points_row2);


//artefacts_info.begin()




for ( auto &t : true_points_row1 )
{
    std::cout << t.first << " " << t.second << endl;
}

int k1 = (true_points_row1.begin())->first;
int k2 = (true_points_row1.begin())->second;

int mid = (k1 + k2)/2;



for (int k= -((kRowStride-1)/2); k<=((kRowStride-1)/2); k++)
{
    cout << mid+k << " " << (int)image.at<uchar>(row1,mid+k) << endl;
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

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << elapsed_secs << std::endl;

  //cv::imshow("Result", destination);
  //cv::waitKey(30);



    //ros::spin();


    //ros::shutdown();
    return 0;
}
