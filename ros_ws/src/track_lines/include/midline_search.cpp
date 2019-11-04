

#include "midline_search.h"

MidLineSearch::MidLineSearch(int image_height, int image_width, MidLineSearchInitializationParameters init):
kImageWidth_(image_width),
kImageHeight_(image_height),
kMinPixelValueForClustering_(init.min_pixel_value_for_clustering),
kMaxRadialScanOutOfClusterValue_(init.max_radial_scan_out_of_cluster_value),
kRadialScanScalingFactor_(init.radial_scan_scaling_factor),
kMidLineLength_(init.mid_line_length),
kMinValuableClusterSize_(init.min_valuable_cluster_size),
kMaxConnectedClusterDistance_(init.max_connected_cluster_distance),
kClusterBinSize((((init.mid_line_length * init.radial_scan_scaling_factor)/2) + 1) * 2),
kImageBorderPadding_(((init.mid_line_length * init.radial_scan_scaling_factor)/2) + 1),
kRadialScanRadius1_((init.mid_line_length * init.radial_scan_scaling_factor)/2),
kRadialScanRadius2_(((init.mid_line_length * init.radial_scan_scaling_factor)/2) + 1)
{
   InitRadialScanners();
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

void MidLineSearch::SetImage(Mat image)
{
    current_image_ = image;
}

void MidLineSearch::ClearMemory()
{
    midline_clusters_size_.clear();
    midline_clusters_xweight_.clear();
    midline_clusters_yweight_.clear();
    midline_clusters_coordinates_.clear();
    centers_of_gravity.clear();
    connected_clusters.clear();
    connected_cluster_keys.clear();
    found_graphs.clear();
}

void MidLineSearch::GroupValueablePointsInClusterBins()
{
    for (int y=kImageBorderPadding_; y<kImageHeight_ - kImageBorderPadding_; y++)
    {
      for (int x=kImageBorderPadding_; x<kImageWidth_ - kImageBorderPadding_; x++)
      {
        if (HasMinPixelValueForClustering(GetPixelValue(x,y)))
        {
          if(IsAClusterPoint(x,y))
          {
            AddPointToClusterBin(x,y);
          }
        }
      }
    }
}

int MidLineSearch::GetPixelValue(int x, int y)
{
    return (int)current_image_.at<uchar>(Point(x,y));
}

bool MidLineSearch::HasMinPixelValueForClustering(int pixel_value)
{
    return (pixel_value > kMinPixelValueForClustering_);
}

bool MidLineSearch::IsAClusterPoint(int x, int y)
{
    return RadialScanPoint(x,y);
}

bool MidLineSearch::RadialScanPoint(int x, int y)
{
    for (auto &it : radial_scan1_)
    {
      int xR = x + it.first;
      int yR = y + it.second;

      if(GetPixelValue(xR, yR) >= kMaxRadialScanOutOfClusterValue_) return false;
    }

    for (auto &it : radial_scan2_)
    {
      int xR = y + it.first;
      int yR = y + it.second;

      if(GetPixelValue(xR, yR) >= kMaxRadialScanOutOfClusterValue_) return false;
    }

    return true;
}

void MidLineSearch::AddPointToClusterBin(int x, int y)
{
    int x_cluster_bin_key = x / kClusterBinSize;
    int y_cluster_bin_key = y / kClusterBinSize;

    if (IsNewKey(x_cluster_bin_key,y_cluster_bin_key))
    {
        AddNewClusterBin(x,y,x_cluster_bin_key,y_cluster_bin_key);
    }
    else
    {
        AppendClusterBin(x,y,x_cluster_bin_key,y_cluster_bin_key);
    }
}

bool MidLineSearch::IsNewKey(int x_cluster_bin_key, int y_cluster_bin_key)
{
    return midline_clusters_size_.find(make_pair(x_cluster_bin_key,y_cluster_bin_key)) == midline_clusters_size_.end();
}

void MidLineSearch::AddNewClusterBin(int x, int y, int x_cluster_bin_key, int y_cluster_bin_key)
{
    midline_clusters_size_[make_pair(x_cluster_bin_key,y_cluster_bin_key)] = 1;

    midline_clusters_xweight_[make_pair(x_cluster_bin_key,y_cluster_bin_key)] = x;
    midline_clusters_yweight_[make_pair(x_cluster_bin_key,y_cluster_bin_key)] = y;

    midline_clusters_coordinates_[make_pair(x_cluster_bin_key,y_cluster_bin_key)].push_back(make_pair(x,y));
}

void MidLineSearch::AppendClusterBin(int x, int y, int x_cluster_bin_key, int y_cluster_bin_key)
{
    int cluster_size = midline_clusters_size_.at(make_pair(x_cluster_bin_key,y_cluster_bin_key)) + 1;
    midline_clusters_size_.at(make_pair(x_cluster_bin_key,y_cluster_bin_key)) = cluster_size;

    midline_clusters_xweight_[make_pair(x_cluster_bin_key,y_cluster_bin_key)] += x;
    midline_clusters_yweight_[make_pair(x_cluster_bin_key,y_cluster_bin_key)] += y;

    midline_clusters_coordinates_[make_pair(x_cluster_bin_key,y_cluster_bin_key)].push_back(make_pair(x,y));
}

void MidLineSearch::RejectClustersUnderSizeThreshold()
{
    vector<pair<int,int>> cluster_ids_to_remove;

    for (auto const& it : midline_clusters_size_)
    {
        if(it.second <= kMinValuableClusterSize_)
        {
            cluster_ids_to_remove.push_back(it.first);
        }
    }

    for( auto& it: cluster_ids_to_remove)
    {
        midline_clusters_size_.erase(it);
        midline_clusters_xweight_.erase(it);
        midline_clusters_yweight_.erase(it);
        midline_clusters_coordinates_.erase(it);
    }
}

void MidLineSearch::ComputeClustersCenterOfGravity()
{
    for (auto const& it : midline_clusters_size_)
    {
      int x = midline_clusters_xweight_.at(it.first) / it.second;
      int y = midline_clusters_yweight_.at(it.first) / it.second;

      centers_of_gravity[it.first].push_back(make_pair(x,y));

    }
}




 void MidLineSearch::FindMidLineClusters(Mat image)
 {
        SetImage(image);
        ClearMemory();
        GroupValueablePointsInClusterBins();
        RejectClustersUnderSizeThreshold();
        ComputeClustersCenterOfGravity();


        // TODO: Clean this mess
        //FindConnectedClusters();
        //ComputeConnectedClusterSlopes();
}


 /*
#include <algorithm>
#include <numeric>
 double slope(const std::vector<double>& x, const std::vector<double>& y) {
    const auto n    = x.size();
    const auto s_x  = std::accumulate(x.begin(), x.end(), 0.0);
    const auto s_y  = std::accumulate(y.begin(), y.end(), 0.0);
    const auto s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
    const auto s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
    const auto a    = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
    return a;
 }
*/


 void MidLineSearch::ComputeConnectedClusterSlopes()
 {



     for (auto& it: found_graphs)
     {

         if(it.size() > 1)
         {
             vector<double> x;
             vector<double> y;

             for(int i=0; i<it.size()-1; i++)
             {


                 int y_bottom = centers_of_gravity[it[i]].begin()->second;
                 int y_top    = centers_of_gravity[it[i+1]].begin()->second;

                 int x_bottom = centers_of_gravity[it[i]].begin()->first;
                 int x_top    = centers_of_gravity[it[i+1]].begin()->first;

                 int opposite =  y_bottom - y_top;

                 int adjacent =  x_top - x_bottom;


                 float angle = CalculateAngle4Quadrants(opposite, adjacent);

                /*
                 x.push_back(double(centers_of_gravity[it[i]].begin()->first));
                 y.push_back(double(centers_of_gravity[it[i]].begin()->second));

                 x.push_back(double(centers_of_gravity[it[i+1]].begin()->first));
                 y.push_back(double(centers_of_gravity[it[i+1]].begin()->second));

                 cout << "slope: " << atan(slope(x,y)) * 180/PI << endl;
                 */

                 //cout <<  "op: " <<  opposite  << " adj: " << adjacent << " xb " << x_bottom << " yb " << y_bottom << " xt " << x_top << " yt " << y_top << " angle: " << angle << endl;

             }


         }
         //cout << "nx_cluster" << endl;
     }





 }



void MidLineSearch::FindConnectedClusters()
{
    vector<pair<int,int>> single_graph;




    if (midline_clusters_coordinates_.size() >= 2)
    {
        // stop iteration at last but one adress
        auto end = --midline_clusters_coordinates_.rend();
        auto p = --midline_clusters_coordinates_.rend();
        // rbegin is used to iterate from bottom to the top of the image
        // because at the bottom the midline clusters should be almost at
        // the same position




        auto last_iteration = --p;

        bool previous_connection = false;

        //bool initial_run = true;


map<pair<int,int>,vector<pair<int,int>>> midline_clusters_coordinates_;
        vector<vector<pair<int,int>>> found_graphs;

        for (auto it = midline_clusters_coordinates_.rbegin(); it != end; ++it )
        {


          auto it_next =  next(it,1);

          int x_start = it->second.begin()->first;
          int y_start = it->second.begin()->second;

          int x_next_end = it_next->second.rbegin()->first;
          int y_next_end = it_next->second.rbegin()->second;


          //cout <<"co " <<  x_start << " " << x_next_end << " " <<  y_start << " " << y_next_end << endl;

          float distance = sqrt(pow(x_start - x_next_end,2) + pow(y_start - y_next_end,2));

          if(distance <= kMaxConnectedClusterDistance_)
          {

    /*
                int itf = it->first.first;
                int its = it->first.second;

                int x = centers_of_gravity[it->first].begin()->first;
                int y = centers_of_gravity[it->first].begin()->second;

                int x_nx = centers_of_gravity[it_next->first].begin()->first;
                int y_nx = centers_of_gravity[it_next->first].begin()->second;

                //connected_clusters.push_back(TwoConnectedClusters{x,y,x_nx,y_nx});
                cout <<"cog " <<  itf << " " << its << " " << x << " " << y << " " << x_nx << " "  << y_nx << endl;
      */

                int key_x_it = it->first.first;
                int key_y_it = it->first.second;

                int key_x_it_next = it_next->first.first;
                int key_y_it_next = it_next->first.second;

                if(!previous_connection)
                {
                    single_graph.push_back(make_pair(key_x_it,key_y_it));
                    single_graph.push_back(make_pair(key_x_it_next,key_y_it_next));
                }
                else
                {
                    single_graph.push_back(make_pair(key_x_it_next,key_y_it_next));
                }


                //connected_cluster_keys.push_back(ConnectedClusterKeys{key_x_it,key_y_it,key_x_it_next,key_y_it_next});

                previous_connection = true;

                if(last_iteration == it)
                {
                    found_graphs.push_back(single_graph);
                    continue;
                }
                else
                {
                    continue;
                }


          }

          //cout <<"xdist: " <<  x_start - x_next_end << endl;
          //cout <<"ydist: " <<  y_start - y_next_end << endl;
          //cout <<"pow: " << sqrt(pow(x_start - x_next_end,2) + pow(y_start - y_next_end,2)) << endl;

          if(single_graph.size() == 0 && !(last_iteration==it))
          {
              single_graph.push_back(make_pair(it->first.first,it->first.second));
              found_graphs.push_back(single_graph);
          }
          else if(last_iteration == it && previous_connection)
          {
              found_graphs.push_back(single_graph);
              single_graph.clear();
              single_graph.push_back(make_pair(it_next->first.first,it_next->first.second));
              found_graphs.push_back(single_graph);

          }
          else if(last_iteration == it && !previous_connection)
          {

              single_graph.push_back(make_pair(it->first.first,it->first.second));
              found_graphs.push_back(single_graph);

              single_graph.clear();
              single_graph.push_back(make_pair(it_next->first.first,it_next->first.second));
              found_graphs.push_back(single_graph);

          }
          else{
              found_graphs.push_back(single_graph);
              single_graph.clear();
          }
          single_graph.clear();
          previous_connection = false;


        }
}
else if(midline_clusters_size_.size() == 1)
{

   int x_start = midline_clusters_coordinates_.begin()->first.first;
   int y_start = midline_clusters_coordinates_.begin()->first.second;

   single_graph.push_back(make_pair(x_start,y_start));
   found_graphs.push_back(single_graph);
   single_graph.clear();
}

}






















void MidLineSearch::DrawClusters(Mat &rgb)
{
    for (auto const& cluster : centers_of_gravity)
    {
        int x = cluster.second.begin()->first;
        int y = cluster.second.begin()->second;

      circle(rgb, Point(x,y), 10, Scalar(255, 0, 255));

    }
}






void MidLineSearch::DrawConnectedClusters(Mat &rgb)
{

#include <stdio.h>
#include <stdlib.h>
#include <time.h>




        srand (time(NULL));

    for (auto& it: found_graphs)
    {

        int B = rand() % 255;

        int G = rand() % 255;

        int R = rand() % 255;
        //cout << "allg: " << found_graphs.size() << "  g: " << it.size() << endl;
        for(int i=0; i<it.size(); i++)
        {
            int x = centers_of_gravity[it[i]].begin()->first;
            int y = centers_of_gravity[it[i]].begin()->second;

            circle(rgb, Point(x,y), 10, Scalar(B, G, R));



        }
    }

    //cout << "__" << endl;


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
