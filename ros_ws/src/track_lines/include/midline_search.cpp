

#include "midline_search.h"

MidLineSearch::MidLineSearch():
  kMinPixelValueForClustering_(240),
  kMaxRadialScanOutOfClusterValue_(120),
  kRadialScanScalingFactor_(1.5),
  kMidLineLength_(30),
  kMinValuableClusterSize_(7),
  kRadialScanRadius1_((kMidLineLength_ * kRadialScanScalingFactor_)/2),
  kRadialScanRadius2_(((kMidLineLength_ * kRadialScanScalingFactor_)/2) + 1)

{
  InitRadialScanners();
}



void MidLineSearch::FindConnectedClusters()
{

    // stop iteration at last but one adress
    auto end = --midline_clusters_coordinates_.rend();

    // rbegin is used to iterate from bottom to the top of the image
    // because at the bottom the midline clusters should be almost at
    // the same position
    for (auto it = midline_clusters_coordinates_.rbegin(); it != end; ++it )
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




void MidLineSearch::InitRadialScanners()
{
  float start_angle = PI;
  float end_angle   = start_angle+2*PI;

  for (float angle=start_angle; angle<end_angle; angle+=0.002)
  {
    float sin_ = sin(angle);
    float cos_ = cos(angle);

    int x = cos_*kRadialScanRadius1_+0.5;
    int y = sin_*kRadialScanRadius1_+0.5;
    int x2 = cos_*kRadialScanRadius2_+0.5;
    int y2 = sin_*kRadialScanRadius2_+0.5;

    if (!(std::find(radial_scan1_.begin(), radial_scan1_.end(), pair<int,int>{x,y}) != radial_scan1_.end()))
    {
      radial_scan1_.push_back({x,y});
    }

    if (!(std::find(radial_scan2_.begin(), radial_scan2_.end(), pair<int,int>{x2,y2}) != radial_scan2_.end()))
    {
      radial_scan2_.push_back({x2,y2});
    }

  }

}

void MidLineSearch::ScanImageToFindMidLineClusters(Mat image)
{

  midline_clusters_size_.clear();
  midline_clusters_xweight_.clear();
  midline_clusters_yweight_.clear();
  midline_clusters_coordinates_.clear();

  // padding of borders with kRadialScanRadius2 to avoid a memory adress which is out of bounds
  for (int x=kRadialScanRadius2_; x<image.rows-kRadialScanRadius2_; x++)
  {
    for (int y=kRadialScanRadius2_; y<image.cols-kRadialScanRadius2_; y++)
    {

      // mat.at<type>(row,column) or mat.at<type>(cv::Point(x,y)) to access the same point if x=column and y=row
      int pixel_value = (int)image.at<uchar>(Point(x,y));

      if (pixel_value > kMinPixelValueForClustering_)
      {
        bool is_midline_scan1 = true;
        bool is_midline_scan2 = true;

        for (auto &it : radial_scan1_)
        {
          int xR = x + it.first;
          int yR = y + it.second;

          if((int)image.at<uchar>(Point(xR,yR)) >= kMaxRadialScanOutOfClusterValue_) is_midline_scan1 = false;
        }
        for (auto &it : radial_scan2_)
        {
          int xR = y + it.first;
          int yR = y + it.second;

          if((int)image.at<uchar>(Point(xR,yR)) >= kMaxRadialScanOutOfClusterValue_) is_midline_scan2 = false;
        }

        if(is_midline_scan1 && is_midline_scan2)
        {

          int x_cluster_key = x / (kRadialScanRadius2_*2);
          int y_cluster_key = y / (kRadialScanRadius2_*2);

          //cout << x_cluster_key << " " << y_cluster_key << endl;

          if ( midline_clusters_size_.find(make_pair(x_cluster_key,y_cluster_key)) == midline_clusters_size_.end() )
          {
              midline_clusters_size_[make_pair(x_cluster_key,y_cluster_key)] = 1;

              midline_clusters_xweight_[make_pair(x_cluster_key,y_cluster_key)] = x;
              midline_clusters_yweight_[make_pair(x_cluster_key,y_cluster_key)] = y;


              midline_clusters_coordinates_[make_pair(x_cluster_key,y_cluster_key)].push_back(make_pair(x,y));

              /*
              for (auto const& hash : midline_clusters_size_)
              {
                  std::cout << "new: " << get<0>(hash.first) << " " << get<1>(hash.first) << " " << hash.second << std::endl ;
              }
              */
          }
          else
          {
            int cluster_size = midline_clusters_size_.at(make_pair(x_cluster_key,y_cluster_key)) + 1;
            midline_clusters_size_.at(make_pair(x_cluster_key,y_cluster_key)) = cluster_size;

            midline_clusters_xweight_[make_pair(x_cluster_key,y_cluster_key)] += x;
            midline_clusters_yweight_[make_pair(x_cluster_key,y_cluster_key)] += y;

            midline_clusters_coordinates_[make_pair(x_cluster_key,y_cluster_key)].push_back(make_pair(x,y));

             /*
            for (auto const& hash : midline_clusters_size_)
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


vector<pair<int,int>> MidLineSearch::GetMidLineClustersCenterOfGravity()
{

  vector<pair<int,int>> midline_clusters_centers_of_gravity;


  // cout << "AllSize "  << midline_clusters_coordinates_.size() << " " << midline_clusters_size_.size() << endl;

  for (auto const& it : midline_clusters_size_)
  {

      if(it.second >= kMinValuableClusterSize_)
      {

        int x = midline_clusters_xweight_.at(it.first) / it.second;
        int y = midline_clusters_yweight_.at(it.first) / it.second;




        midline_clusters_centers_of_gravity.push_back(make_pair(x,y));


    //  cout << "OneSize "  << midline_clusters_coordinates_.at(hash.first).size() << endl;
/*
        for (int i=0;i<(midline_clusters_coordinates_.at(hash.first)).size();i++)
        {

            int y = ((midline_clusters_coordinates_.at(hash.first)).at(i)).first;
            int x = ((midline_clusters_coordinates_.at(hash.first)).at(i)).second;

     //       cout << x << " " << y << endl;
        }
*/
        //cout << xx << " " << yy << " " << midline_clusters_xweight_.at(hash.first) << " " << midline_clusters_yweight_.at(hash.first) << endl;

        //int i = get<0>(hash.first)*pointerLen2*2;
        //int j = get<1>(hash.first)*pointerLen2*2;
        //circle(rgb, Point(yy,xx), 10, Scalar(0, 255, 0));

      }
      else
      {
        midline_clusters_coordinates_.erase(it.first);
      }

  }
   //cout << "AfterSize "  << midline_clusters_coordinates_.size() << " " << midline_clusters_size_.size() << endl;

  return midline_clusters_centers_of_gravity;

}


/* TODO: Sorting is not neccessary
 *
void MidLineSearch::SortClusters()
{


  for (auto const& hash : midline_clusters_coordinates_)
  {
    for (int i=0;i<(midline_clusters_coordinates_.at(hash.first)).size();i++)
    {

      int y = ((midline_clusters_coordinates_.at(hash.first)).at(i)).first;
      int x = ((midline_clusters_coordinates_.at(hash.first)).at(i)).second;

      cout << x << " " << y << endl;

    }
  }

 for (auto const& hash : midline_clusters_coordinates_)
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

for (auto const& hash : midline_clusters_coordinates_)
{
  for (int i=0;i<(midline_clusters_coordinates_.at(hash.first)).size();i++)
  {

    int y = ((midline_clusters_coordinates_.at(hash.first)).at(i)).first;
    int x = ((midline_clusters_coordinates_.at(hash.first)).at(i)).second;

    cout << x << " " << y << endl;

  }
}

}
*/
