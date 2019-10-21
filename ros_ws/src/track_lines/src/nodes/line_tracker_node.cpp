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


class MidLineSearch
{
    private:
      vector<pair<int,int>> scan, rescan;

      const float kSearchScalingFactor;

      const int kIntensityThreshold;
      const int kMaxCandidateIntensity;



      const int kMidlineLength;

      const int kSearchRadius1;
      const int kSearchRadius2;

      map<pair<int,int>,vector<pair<int,int>>> cluster_coordinates_hashmap;




      map<pair<int,int>,int> candidates_count_hashmap;
      map<pair<int,int>,int> candidates_xmass_hashmap;
      map<pair<int,int>,int> candidates_ymass_hashmap;

    public:
      MidLineSearch();
      bool CheckLineThickness(int thickness);
      void InitClusterScanners();
      void FindMidlinesInFrame(Mat image);
      void SortClusters();
      void CheckClusterConnections();


      vector<pair<int,int>> GetMidlinePoints(int min_cluster_size);
};


MidLineSearch::MidLineSearch():
  kIntensityThreshold(240),
  kMaxCandidateIntensity(120),
  kSearchScalingFactor(1.5),
  kMidlineLength(30),

  kSearchRadius1((kMidlineLength * kSearchScalingFactor)/2),
  kSearchRadius2(((kMidlineLength * kSearchScalingFactor)/2) + 1)

{
  InitClusterScanners();
}



void MidLineSearch::CheckClusterConnections()
{

    auto end = --cluster_coordinates_hashmap.rend();


    for (auto it = cluster_coordinates_hashmap.rbegin(); it != end; ++it )
    {


      auto nx =  std::next(it,1);


      int y_start = it->second.begin()->first;
      int y_next_end = nx->second.rbegin()->first;

      int x_start = it->second.begin()->second;
      int x_next_end = nx->second.rbegin()->second;

      cout <<"xdist: " <<  x_start - x_next_end << endl;
      cout <<"ydist: " <<  y_start - y_next_end << endl;
      cout <<"pow: " << sqrt(pow(x_start - x_next_end,2) + pow(y_start - y_next_end,2)) << endl;

      /*
      cout <<"top " <<  it->second.begin()->first << " " << it->second.begin()->second << endl;


      cout <<"bottom " <<  nx->second.rbegin()->first << " " << nx->second.rbegin()->second << endl;

*/


      //cout << "next " << nx->second.begin()->first<< endl;






      /*
      for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2)
      {
        std::cout << it2->first << " " << it2->second << std::endl ;
      }
      */

    }
    cout << "_____________" << endl;

}



void MidLineSearch::SortClusters()
{

/*
  for (auto const& hash : cluster_coordinates_hashmap)
  {
    for (int i=0;i<(cluster_coordinates_hashmap.at(hash.first)).size();i++)
    {

      int y = ((cluster_coordinates_hashmap.at(hash.first)).at(i)).first;
      int x = ((cluster_coordinates_hashmap.at(hash.first)).at(i)).second;

      cout << x << " " << y << endl;

    }
  }

 for (auto const& hash : cluster_coordinates_hashmap)
  {
      //sort(hash.second.begin(),hash.second.end());

   vector<pair<int,int>> go(hash.second.begin(),hash.second.end());

    cout << "before" << endl;
   for (auto const& k : go) cout << k.first << " " << k.second << endl;

    sort(go.begin(),go.end());

    cout << "after" << endl;
    for (auto const& k : go) cout << k.first << " " << k.second << endl;

  }

cout << "after" << endl;

for (auto const& hash : cluster_coordinates_hashmap)
{
  for (int i=0;i<(cluster_coordinates_hashmap.at(hash.first)).size();i++)
  {

    int y = ((cluster_coordinates_hashmap.at(hash.first)).at(i)).first;
    int x = ((cluster_coordinates_hashmap.at(hash.first)).at(i)).second;

    cout << x << " " << y << endl;

  }
}
*/
}


void MidLineSearch::InitClusterScanners()
{
  float start_angle = PI;
  float end_angle   = start_angle+2*PI;

  for (float angle=start_angle; angle<end_angle; angle+=0.002)
  {
    float sin_ = sin(angle);
    float cos_ = cos(angle);

    int xC = cos_*kSearchRadius1+0.5;
    int yC = sin_*kSearchRadius1+0.5;
    int xC2 = cos_*kSearchRadius2+0.5;
    int yC2 = sin_*kSearchRadius2+0.5;

    if (!(std::find(scan.begin(), scan.end(), pair<int,int>{xC,yC}) != scan.end()))
    {
      scan.push_back({xC,yC});
    }

    if (!(std::find(rescan.begin(), rescan.end(), pair<int,int>{xC2,yC2}) != rescan.end()))
    {
      rescan.push_back({xC2,yC2});
    }

  }

}

void MidLineSearch::FindMidlinesInFrame(Mat image)
{

  candidates_count_hashmap.clear();
  candidates_xmass_hashmap.clear();
  candidates_ymass_hashmap.clear();
  cluster_coordinates_hashmap.clear();

  for (int i=kSearchRadius1; i<image.rows-kSearchRadius1; i++)
  {
    for (int j=kSearchRadius2; j<image.cols-kSearchRadius2; j++)
    {




      int M = (int)image.at<uchar>(Point(j,i));
      if (M > kIntensityThreshold)
      {
        bool is_candidate_scan = true;
        bool is_candidate_rescan = true;

        for (auto &it : scan)
        {
          int x = j + it.first;
          int y = i + it.second;
          if((int)image.at<uchar>(y,x) >= kMaxCandidateIntensity) is_candidate_scan = false;
        }
        for (auto &it : rescan)
        {
          int x = j + it.first;
          int y = i + it.second;
          if((int)image.at<uchar>(y,x) >= kMaxCandidateIntensity) is_candidate_rescan = false;
        }

        if(is_candidate_scan && is_candidate_rescan)
        {


          int i_key = i / (kSearchRadius2*2);
          int j_key = j / (kSearchRadius2*2);

          //cout << i_key << " " << j_key << endl;

          if ( candidates_count_hashmap.find(make_pair(i_key,j_key)) == candidates_count_hashmap.end() )
          {
              candidates_count_hashmap[make_pair(i_key,j_key)] = 1;

              candidates_xmass_hashmap[make_pair(i_key,j_key)] = i;
              candidates_ymass_hashmap[make_pair(i_key,j_key)] = j;


              cluster_coordinates_hashmap[make_pair(i_key,j_key)].push_back(make_pair(i,j));

              /*
              for (auto const& hash : candidates_count_hashmap)
              {
                  std::cout << "new: " << get<0>(hash.first) << " " << get<1>(hash.first) << " " << hash.second << std::endl ;
              }
              */
          }
          else
          {
            int count = candidates_count_hashmap.at(make_pair(i_key,j_key)) + 1;
            candidates_count_hashmap.at(make_pair(i_key,j_key)) = count;

            candidates_xmass_hashmap[make_pair(i_key,j_key)] += i;
            candidates_ymass_hashmap[make_pair(i_key,j_key)] += j;

            cluster_coordinates_hashmap[make_pair(i_key,j_key)].push_back(make_pair(i,j));

             /*
            for (auto const& hash : candidates_count_hashmap)
            {
                std::cout << "add: " << get<0>(hash.first)<< " " << get<1>(hash.first) << " " << hash.second << std::endl ;
            }
            */
          }

        }
     }






    }
  }
}


vector<pair<int,int>> MidLineSearch::GetMidlinePoints(int min_cluster_size)
{

  vector<pair<int,int>> cluster_coordinates;


  // cout << "AllSize "  << cluster_coordinates_hashmap.size() << " " << candidates_count_hashmap.size() << endl;

  for (auto const& hash : candidates_count_hashmap)
  {

      if(hash.second >= min_cluster_size)
      {

        int x = candidates_xmass_hashmap.at(hash.first) / hash.second;
        int y = candidates_ymass_hashmap.at(hash.first) / hash.second;




        cluster_coordinates.push_back(make_pair(x,y));


    //  cout << "OneSize "  << cluster_coordinates_hashmap.at(hash.first).size() << endl;
/*
        for (int i=0;i<(cluster_coordinates_hashmap.at(hash.first)).size();i++)
        {

            int y = ((cluster_coordinates_hashmap.at(hash.first)).at(i)).first;
            int x = ((cluster_coordinates_hashmap.at(hash.first)).at(i)).second;

     //       cout << x << " " << y << endl;
        }
*/
        //cout << xx << " " << yy << " " << candidates_xmass_hashmap.at(hash.first) << " " << candidates_ymass_hashmap.at(hash.first) << endl;

        //int i = get<0>(hash.first)*pointerLen2*2;
        //int j = get<1>(hash.first)*pointerLen2*2;
        //circle(rgb, Point(yy,xx), 10, Scalar(0, 255, 0));

      }
      else
      {
        cluster_coordinates_hashmap.erase(hash.first);
      }

  }
   //cout << "AfterSize "  << cluster_coordinates_hashmap.size() << " " << candidates_count_hashmap.size() << endl;

  return cluster_coordinates;

}







class LineClassification
{
private:
  const int kLineThicknessMin;
  const int kLineThicknessMax;

  const int kTrackWidthMax;
  const int kTrackWidthMin;

  const int kRowStride;
  const int kIntensityThreshold;

  const int kImgRows;
  const int kImgCols;

  const int row0;
  const int row1;
  const int row2;

  const int kMidlineSearchSpace;

  const int kMaxColumnDistanceForRowPoints;


  std::vector<pair<int,int>> correct_features_row0;
  std::vector<pair<int,int>> correct_features_row1;
  std::vector<pair<int,int>> correct_features_row2;

      vector<tuple<int,int,int,int>> correct_features;


      vector<tuple<int,int,int,int,int,int>> matched_pattern_coordinates;

public:
  LineClassification();
  bool CheckLineThickness(int thickness);
vector<tuple<int,int,int,int,int,int>> SearchLineFeatures(Mat image);
void CheckThicknessAndDistancesPerRow(int row_id, std::multimap<int,std::tuple<int, int>> &row_elements, std::vector<pair<int,int>> &correct_features);
  vector<int> row_spikes;

     std::multimap<int,std::tuple<int, int>> artefacts_info;

    std::vector<pair<int,int>> artefacts_count;
    vector<int> line_search_regions = {row0,row1,row2};

};


LineClassification::LineClassification():
  row0(380),
  row1(360),
  row2(340),
  kLineThicknessMin(3),
  kLineThicknessMax(9),
  kTrackWidthMin(117),
  kTrackWidthMax(143),
  kRowStride(5),
  kIntensityThreshold(200),
  kImgRows(417),
  kImgCols(1280),
  row_spikes(kImgCols,0),
  kMidlineSearchSpace(7),
  kMaxColumnDistanceForRowPoints(15)


{



}


vector<tuple<int,int,int,int,int,int>> LineClassification::SearchLineFeatures(Mat image)
{
  artefacts_count.emplace_back(row0,0);
  artefacts_count.emplace_back(row1,0);
  artefacts_count.emplace_back(row2,0);
  artefacts_info.clear();
   correct_features.clear();

        matched_pattern_coordinates.clear();


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

  if (artefacts_info.count(row1) >= 3)
  {

    correct_features_row0.clear();
    correct_features_row1.clear();
    correct_features_row2.clear();

    CheckThicknessAndDistancesPerRow(row0,artefacts_info,correct_features_row0);
    CheckThicknessAndDistancesPerRow(row1,artefacts_info,correct_features_row1);
    CheckThicknessAndDistancesPerRow(row2,artefacts_info,correct_features_row2);








     if(!correct_features_row0.empty() && !correct_features_row1.empty())
     {


       for(int i=0; i<correct_features_row0.size();i++)
       {
         for(int j=0; j<correct_features_row1.size();j++)
         {

           //int g = row0 - row1;

           int a1 = correct_features_row0.at(i).first - correct_features_row1.at(j).first;

           int a2 = correct_features_row0.at(i).second - correct_features_row1.at(j).second;


           if(abs(a1) < kMaxColumnDistanceForRowPoints && abs(a2) < kMaxColumnDistanceForRowPoints)
           {
             correct_features.push_back(make_tuple(correct_features_row0.at(i).first,
                                                 correct_features_row0.at(i).second,
                                                 correct_features_row1.at(j).first,
                                                 correct_features_row1.at(j).second));
           }



         }
       }




      // cout << "tpsize: " << true_points_row0.at(0).first << endl;



/*
          for ( auto &t : correct_features_row0 )
          {
              std::cout <<"r0: " << t.first << " " << t.second << endl;
          }



      for ( auto &t : correct_features_row1 )
      {
          std::cout <<"r1: " << t.first << " " << t.second << endl;
      }

*/






      for ( auto &t : correct_features )
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

         //cout << "row0: " << mid_pattern_row0 << " " << "row1: " << mid_pattern_row1 << " " << "row2: " << mid_pattern_row2 << endl;
         if(mid_pattern_row0 && mid_pattern_row1 && mid_pattern_row2)
         {

            matched_pattern_coordinates.push_back(make_tuple(get<0>(t),get<1>(t),get<2>(t),get<3>(t), mid_row0, mid_row1));




         }

      }



  }


}

  return matched_pattern_coordinates;

}
void LineClassification::CheckThicknessAndDistancesPerRow(int row_id, std::multimap<int,std::tuple<int, int>> &row_elements, std::vector<pair<int,int>> &correct_features)
{
  std::pair < std::multimap<int,std::tuple<int, int>>::iterator,  std::multimap<int,std::tuple<int, int>>::iterator> ret;
  ret = row_elements.equal_range(row_id);


  std::vector<int> row_points;

  for (std::multimap<int,std::tuple<int, int>>::iterator it=ret.first; it!=ret.second; ++it)
  {
    //std::cout << "row: " << it->first << " id: " << std::get<0>(it->second) << " length: " << std::get<1>(it->second) << endl;

    int row = it->first;
    int idx = std::get<0>(it->second);
    int thickness = std::get<1>(it->second);


    bool is_in_range = CheckLineThickness(thickness);

    if(is_in_range)
    {
      row_points.push_back(idx+thickness/2);
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

          int distance = abs(row_points.at(j)-row_points.at(i));
          //cout <<"row: "<< row_id<< " " << w << " " << row_points.at(j)<< " " << row_points.at(i)<< endl;
          if(distance>=kTrackWidthMin && distance<=kTrackWidthMax)
          {
            correct_features.push_back(make_pair(row_points.at(i),row_points.at(j)));
          }
      }
    }

   // cout << "finished TP" << endl;
}




bool LineClassification::CheckLineThickness(int thickness)
{
  if(thickness >= kLineThicknessMin && thickness <= kLineThicknessMax) { return true; }
  else { return false; }
}


class LineTracker
{

private:
    ros::NodeHandle n;
    image_transport::ImageTransport it;

    const int kLineWitdhMin = 3;
    const int kLineWidthMax = 9;

    const int kTrackWidthMax = 143;
    const int kTrackWidthMin = 117;

    Mat transfo;

    Mat grey;
    Mat rgb;



    Size taille;

    MidLineSearch MidLineSearcher;
    LineClassification LineClassifier;


public:
  LineTracker(ros::NodeHandle* nh_);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void initBirdseye();
  image_transport::Subscriber image_sub;
};


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




int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_tracker_node");
  ros::NodeHandle nh;

  LineTracker t(&nh);
ros::spin();
ros::shutdown();

    return 0;
}
