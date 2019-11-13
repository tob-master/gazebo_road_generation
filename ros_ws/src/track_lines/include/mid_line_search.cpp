

#include "mid_line_search.h"

MidLineSearch::MidLineSearch(int image_height, int image_width, MidLineSearchInitializationParameters init):
kImageWidth_(image_width),
kImageHeight_(image_height),
kMinPixelValueForClustering_(init.min_pixel_value_for_clustering),
kMaxRadialScanOutOfClusterValue_(init.max_radial_scan_out_of_cluster_value),
kRadialScanScalingFactor_(init.radial_scan_scaling_factor),
kMidLineLength_(init.mid_line_length),
kMinValuableClusterSize_(init.min_valuable_cluster_size),
kMinMidLineClusterDistance_(init.min_cluster_distance),
kMaxMidLineClusterDistance_(init.max_cluster_distance),
kCarPosition_(init.car_position_x,init.car_position_y ),
kClusterBinSize((((init.mid_line_length * init.radial_scan_scaling_factor)/2) + 1) * 2),
kImageBorderPadding_(((init.mid_line_length * init.radial_scan_scaling_factor)/2) + 1),
kRadialScanRadius1_((init.mid_line_length * init.radial_scan_scaling_factor)/2),
kRadialScanRadius2_(((init.mid_line_length * init.radial_scan_scaling_factor)/2) + 1)
{
   InitRadialScanners();
}


MidLineSearchReturnInfo MidLineSearch::FindMidLineClusters(Mat image)
{
       SetImage(image);
       ClearMemory();
       GroupValueablePointsInClusterBins();
       RejectClustersUnderSizeThreshold();

       if(HasFoundMidLineClusters())
       {
           ComputeClustersCentroid();
           GroupClusters();

           if(HasFoundGroup())
           {
              ComputeLengthAndDirectionOfConnectedClusters();
           }
       }

       return GetReturnInfo();
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
    has_found_mid_line_clusters_ = false;
    has_found_group_ = false;

    midline_clusters_size_.clear();
    midline_clusters_xweight_.clear();
    midline_clusters_yweight_.clear();
    midline_clusters_coordinates_.clear();
    centers_of_gravity.clear();
    grouped_clusters_length_and_direction_.clear();
    connected_clusters_length_and_direction_.clear();

    //connected_cluster_bin_keys_.clear();
    //grouped_cluster_bin_keys_.clear();

    //sorted_centroid_groups_.clear();

    cluster_centroids_.clear();
    used_permutations_.clear();
    //grouped_mid_line_clusters_.clear();

    mid_line_cluster_groups_.clear();
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

void MidLineSearch::ComputeClustersCentroid()
{
    for (auto const& it : midline_clusters_size_)
    {



 //       cout << it.first.first << " , " << it.first.second << endl;

      int x = midline_clusters_xweight_.at(it.first) / it.second;
      int y = midline_clusters_yweight_.at(it.first) / it.second;

      cluster_centroids_.push_back(Point(x,y));

    }
}


bool MidLineSearch::HasFoundMidLineClusters()
{
    has_found_mid_line_clusters_ = (midline_clusters_coordinates_.size() > 0);
    return (midline_clusters_coordinates_.size() > 0);
}

bool MidLineSearch::HasFoundGroup()
{
    for (auto& it: mid_line_cluster_groups_)
    {
        if(it.size() > 1)
        {
            has_found_group_ = true;
            return true;
        }
    }

    has_found_group_ = false;
    return false;
}


MidLineSearchReturnInfo MidLineSearch::GetReturnInfo()
{
    return MidLineSearchReturnInfo{has_found_mid_line_clusters_,has_found_group_};
}


 void MidLineSearch::ComputeLengthAndDirectionOfConnectedClusters()
 {
     for (auto& it: mid_line_cluster_groups_)
     {
         for(int i=0; i<it.size()-1; i++)
         {
             Point current_cluster_center_of_gravity = it[i];
             Point next_cluster_center_of_gravity    = it[i+1];

             int opposite =  current_cluster_center_of_gravity.y - next_cluster_center_of_gravity.y;
             int adjacent =  next_cluster_center_of_gravity.x - current_cluster_center_of_gravity.x;

             int length = sqrt(pow(adjacent,2)+pow(opposite,2));

             float angle = CalculateAngle4Quadrants(opposite, adjacent);

             connected_clusters_length_and_direction_.push_back(LengthAndDirectionFromConnectedClusters{current_cluster_center_of_gravity.x,
                                                                                                       current_cluster_center_of_gravity.y,
                                                                                                       length,
                                                                                                       angle});
         }
         grouped_clusters_length_and_direction_.push_back(connected_clusters_length_and_direction_);
         connected_clusters_length_and_direction_.clear();

     }
 }


 vector<vector<LengthAndDirectionFromConnectedClusters>> MidLineSearch::GetGroupedMidLineClustersLengthAndDirection()
 {
     return grouped_clusters_length_and_direction_;
 }


 void MidLineSearch::CoutLengthAndDirectionOfConnectedClusters()
 {
         cout << "___MidLineSearch LengthAndDirectionOfConnectedClusters___" << endl;
     for(int i=0; i<grouped_clusters_length_and_direction_.size(); i++)
     {
         for(int j=0; j<grouped_clusters_length_and_direction_[i].size(); j++)
         {
             cout << "Cluster " << i << ": \tPoint(" << grouped_clusters_length_and_direction_[i][j].x
                                             << ","  << grouped_clusters_length_and_direction_[i][j].y
                                             << ") \t Direction: "  <<grouped_clusters_length_and_direction_[i][j].angle
                                             << "° \tLength: "  << grouped_clusters_length_and_direction_[i][j].length <<"px" <<  endl;
         }
     }
     cout << "#######################################" << endl;
 }






bool MidLineSearch::IsPermuted(int i, int j, vector<string> &used_permutations)
{
    if(i==j){ return true;}

    string permutation_hash12 = to_string(i) + to_string(j);

    auto it = find (used_permutations.begin(), used_permutations.end(), permutation_hash12);

    if (it != used_permutations.end())
    {
        return true;
    }
    else
    {
      string permutation_hash21 = std::to_string(j) + std::to_string(i);
      used_permutations.push_back(permutation_hash12);
      used_permutations.push_back(permutation_hash21);
      return false;
    }
}








double MidLineSearch::Distance2d(const Point& lhs, const Point& rhs)
{
    return sqrt(pow(lhs.x-rhs.x,2) + pow(lhs.y-rhs.y,2));
}


bool MidLineSearch::IsConnected(float cluster_distance)
{
    return cluster_distance > kMinMidLineClusterDistance_ && cluster_distance < kMaxMidLineClusterDistance_;
}

void MidLineSearch::FindClusterConnections(vector<pair<int,int>> &cluster_connections)
{
    for(int i=0; i<cluster_centroids_.size(); i++)
    {
        int clusters_ix = cluster_centroids_[i].x;
        int clusters_iy = cluster_centroids_[i].y;

        for(int j=0; j<cluster_centroids_.size(); j++)
        {
            if(IsPermuted(i,j,used_permutations_)) continue;

            int clusters_jx = cluster_centroids_[j].x;
            int clusters_jy = cluster_centroids_[j].y;

            float cluster_distance = sqrt(pow(clusters_jx-clusters_ix,2) + pow(clusters_jy-clusters_iy,2));

            if(IsConnected(cluster_distance))
            {
                cluster_connections.push_back(make_pair(i,j));
            }
        }
    }
}

void MidLineSearch::GroupClusters()
{

    vector<pair<int,int>> cluster_connections;

    FindClusterConnections(cluster_connections);


    int num_vertices = cluster_centroids_.size();

    DepthFirstSearch ConnectedMidLineGroupFinder(num_vertices);

    for(auto it:cluster_connections){ ConnectedMidLineGroupFinder.addEdge(it.first, it.second); }

    vector<vector<int>> connected_mid_line_groups = ConnectedMidLineGroupFinder.connectedComponents();

   vector<Point> connected_cluster_centroids;



   for(int i=0; i<connected_mid_line_groups.size(); ++i)
   {
       for(int j=0; j<connected_mid_line_groups[i].size(); ++j)
       {
           connected_cluster_centroids.push_back(cluster_centroids_[connected_mid_line_groups[i][j]]);
       }

       std::sort(begin(connected_cluster_centroids),
                 end(connected_cluster_centroids),
                 [&](const Point& lhs, const Point& rhs){ return Distance2d(kCarPosition_, lhs) < Distance2d(kCarPosition_, rhs); });


       mid_line_cluster_groups_.push_back(connected_cluster_centroids);
       connected_cluster_centroids.clear();
   }

    used_permutations_.clear();
}



void MidLineSearch::DrawClusters(Mat &rgb)
{

    for (int i=0; i<cluster_centroids_.size(); i++)
    {
        int x = cluster_centroids_[i].x;
        int y = cluster_centroids_[i].y;

        string text = to_string(i);
         putText(rgb, text, Point(x,y),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,0), 1, CV_AA);

    }

}

void MidLineSearch::DrawConnectedClusters(Mat &rgb)
{
    std::vector<Vec3b> colors(mid_line_cluster_groups_.size());

    for (int label = 0; label < mid_line_cluster_groups_.size(); ++label)
    {
        colors[label] = Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
    }

    for ( int i=0; i<mid_line_cluster_groups_.size(); ++i)
    {


        for(int j=0; j<mid_line_cluster_groups_[i].size(); j++)
        {


            circle(rgb,mid_line_cluster_groups_[i][j], 10, colors[i],CV_FILLED);

            string text = to_string(i) + " " + to_string(j);
            putText(rgb, text, Point(mid_line_cluster_groups_[i][j].x + 30,mid_line_cluster_groups_[i][j].y),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);

        }
    }
}






void MidLineSearch::DrawGroupedMidLineClustersDirections(Mat &rgb)
{

    std::vector<Vec3b> colors(grouped_clusters_length_and_direction_.size());

    for (int label = 0; label < grouped_clusters_length_and_direction_.size(); ++label)
    {
        colors[label] = Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
    }

    for(int i=0;i<grouped_clusters_length_and_direction_.size();i++ )
    {
        for (int j=0;j<grouped_clusters_length_and_direction_[i].size();j++ ) {

            int x_start = grouped_clusters_length_and_direction_[i][j].x;
            int y_start = grouped_clusters_length_and_direction_[i][j].y;

            int length = grouped_clusters_length_and_direction_[i][j].length;
            float angle  = grouped_clusters_length_and_direction_[i][j].angle * PI/180;

            int x_offset = cos(angle) * length;
            int y_offset = sin(angle) * length;

            int x_end = x_start + x_offset;
            int y_end = y_start - y_offset;

            line( rgb,Point(x_start,y_start), Point(x_end,y_end), colors[i], 3, CV_AA);

            circle(rgb, Point(x_start,y_start), 7, Scalar(0,255,0),CV_FILLED);
            circle(rgb, Point(x_end,y_end), 10, Scalar(0,0,255),2);
        }
    }



}

/* TODO: Sorting is not neccessary
 *
 *
 *
 *
 * void MidLineSearch::DrawGroupedMidLineClusters(Mat &rgb)
{
    std::vector<Vec3b> colors(grouped_mid_line_clusters_.size());

    for (int label = 0; label < grouped_mid_line_clusters_.size(); ++label)
    {
        colors[label] = Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
    }


    for (int i=0;i<grouped_mid_line_clusters_.size();i++)
    {
        for (int j=0;j<grouped_mid_line_clusters_[i].size();j++)
        {

            int x = cluster_centroids_[grouped_mid_line_clusters_[i][j]].x;
            int y = cluster_centroids_[grouped_mid_line_clusters_[i][j]].y;


            circle(rgb, Point(x,y), 7, colors[i],CV_FILLED);
        }
    }

}
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

void MidLineSearch::ComputeClustersCentroid()
{
    for (auto const& it : midline_clusters_size_)
    {

      cout << it.first.first << " , " << it.first.second << endl;

      int x = midline_clusters_xweight_.at(it.first) / it.second;
      int y = midline_clusters_yweight_.at(it.first) / it.second;

      centers_of_gravity[it.first].push_back(make_pair(x,y));


    }
}


 void MidLineSearch::ComputeLengthAndDirectionOfConnectedClustersNearest()
 {
     for (auto& it: grouped_cluster_bin_keys_)
     {
         if(it.size() > 1)
         {
             //vector<double> x;
             //vector<double> y;

             for(int i=0; i<it.size()-1; i++)
             {
                 //cout << it[i].x_cluster_bin_key << " " << it[i].y_cluster_bin_key<< endl;

                 Point current_cluster_center_of_gravity = GetCenterOfGravityPointFromClusterBinKey(it[i]);
                 Point next_cluster_center_of_gravity    = GetCenterOfGravityPointFromClusterBinKey(it[i+1]);

                 int opposite =  current_cluster_center_of_gravity.y - next_cluster_center_of_gravity.y;
                 int adjacent =  next_cluster_center_of_gravity.x - current_cluster_center_of_gravity.x;

                 int length = sqrt(pow(adjacent,2)+pow(opposite,2));

                 float angle = CalculateAngle4Quadrants(opposite, adjacent);

                 connected_clusters_length_and_direction_.push_back(LengthAndDirectionFromConnectedClusters{current_cluster_center_of_gravity.x,
                                                                                                           current_cluster_center_of_gravity.y,
                                                                                                           length,
                                                                                                           angle});
                     }
             grouped_clusters_length_and_direction_.push_back(connected_clusters_length_and_direction_);
             connected_clusters_length_and_direction_.clear();
         }
     }
 }


 void MidLineSearch::ComputeLengthAndDirectionOfConnectedClusters()
 {
     for (auto& it: grouped_mid_line_clusters_)
     {
         if(it.size() > 1)
         {
             //vector<double> x;
             //vector<double> y;

             for(int i=0; i<it.size()-1; i++)
             {
                 //cout << it[i].x_cluster_bin_key << " " << it[i].y_cluster_bin_key<< endl;

                 Point current_cluster_center_of_gravity = cluster_centroids_[it[i]];
                 Point next_cluster_center_of_gravity    = cluster_centroids_[it[i+1]];

                 int opposite =  current_cluster_center_of_gravity.y - next_cluster_center_of_gravity.y;
                 int adjacent =  next_cluster_center_of_gravity.x - current_cluster_center_of_gravity.x;

                 int length = sqrt(pow(adjacent,2)+pow(opposite,2));

                 float angle = CalculateAngle4Quadrants(opposite, adjacent);

                 connected_clusters_length_and_direction_.push_back(LengthAndDirectionFromConnectedClusters{current_cluster_center_of_gravity.x,
                                                                                                           current_cluster_center_of_gravity.y,
                                                                                                           length,
                                                                                                           angle});
                     }
             grouped_clusters_length_and_direction_.push_back(connected_clusters_length_and_direction_);
             connected_clusters_length_and_direction_.clear();
         }
     }
 }






 Point MidLineSearch::GetCenterOfGravityPointFromClusterBinKey(ClusterBinKey cluster_bin_key)
 {
     pair<int,int> key = make_pair(cluster_bin_key.x_cluster_bin_key,cluster_bin_key.y_cluster_bin_key);

    int x = centers_of_gravity[key].begin()->first;
    int y = centers_of_gravity[key].begin()->second;

    return Point(x,y);
 }


 void MidLineSearch::SortClusterGroupCentroids()
 {
     for (auto& it: grouped_cluster_bin_keys_)
     {
         vector<int> x_centers_group;
         vector<int> y_centers_group;

         for(int i=0; i<it.size(); i++)
         {
            pair<int,int> key = make_pair(it[i].x_cluster_bin_key,it[i].y_cluster_bin_key);

            int x = centers_of_gravity[key].begin()->first;
            int y = centers_of_gravity[key].begin()->second;

            x_centers_group.push_back(x);
            y_centers_group.push_back(y);

         }

         int group_size = y_centers_group.size();

         Mat y_centers_group_mat = Mat(1, group_size, CV_32SC1);
         Mat x_centers_group_mat = Mat(1, group_size, CV_32SC1);

         memcpy(y_centers_group_mat.data, y_centers_group.data(), group_size*sizeof(int));
         memcpy(x_centers_group_mat.data, x_centers_group.data(), group_size*sizeof(int));

         Mat sorted_y_center_group_ids;
         sortIdx(y_centers_group_mat, sorted_y_center_group_ids, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING);

         vector<Point> sorted_centroids_group;

         for (int i=0;i<group_size;i++)
         {
             int id = sorted_y_center_group_ids.at<int>(i);
             int x = x_centers_group_mat.at<int>(id);
             int y = y_centers_group_mat.at<int>(id);
             Point centroid = Point(x,y);

             sorted_centroids_group.push_back(centroid);


         }

         sorted_centroid_groups_.push_back(sorted_centroids_group);
       }



 }

 void MidLineSearch::ComputeLengthAndDirectionOfConnectedClustersBottomToTop()
 {

     SortClusterGroupCentroids();

     for (int i=0; i<sorted_centroid_groups_.size(); i++)
     {
         if(sorted_centroid_groups_[i].size() > 1)
         {
             for(int j=0; j<sorted_centroid_groups_[i].size()-1; j++)
             {

                     Point current_cluster_center_of_gravity = sorted_centroid_groups_[i][j];
                     Point next_cluster_center_of_gravity    = sorted_centroid_groups_[i][j+1];

                     int opposite =  current_cluster_center_of_gravity.y - next_cluster_center_of_gravity.y;
                     int adjacent =  next_cluster_center_of_gravity.x - current_cluster_center_of_gravity.x;

                     int length = sqrt(pow(adjacent,2)+pow(opposite,2));

                     float angle = CalculateAngle4Quadrants(opposite, adjacent);

                     connected_clusters_length_and_direction_.push_back(LengthAndDirectionFromConnectedClusters{current_cluster_center_of_gravity.x,
                                                                                                               current_cluster_center_of_gravity.y,
                                                                                                               length,
                                                                                                               angle});
              }
              grouped_clusters_length_and_direction_.push_back(connected_clusters_length_and_direction_);
              connected_clusters_length_and_direction_.clear();
             }
         }

 }

Point MidLineSearch::GetStartPointOfCurrentCluster(ReverseMidLineCoordinatesIterator it)
{

 int x = it->second.begin()->first;
 int y = it->second.begin()->second;

 return Point(x,y);
}


Point MidLineSearch::GetEndPointOfNextCluster(ReverseMidLineCoordinatesIterator it)
{
  auto it_next =  next(it,1);

  int x = it_next->second.rbegin()->first;
  int y = it_next->second.rbegin()->second;

  return Point(x,y);
}



float MidLineSearch::GetCurrentStartToNextEndClusterDistance(Point start_point_of_current_cluster, Point end_point_of_next_cluster)
{
   float distance = sqrt(pow(start_point_of_current_cluster.x - end_point_of_next_cluster.x,2) +
                         pow(start_point_of_current_cluster.y - end_point_of_next_cluster.y,2));

   return distance;
}

bool MidLineSearch::ClustersAreConnected(float distance)
{
  return (distance <= kMaxConnectedClusterDistance_);
}



ClusterBinKey MidLineSearch::GetClusterBinKeyOfCurrentClusterFromIterator(ReverseMidLineCoordinatesIterator it)
{
  int x_cluster_bin_key = it->first.first;
  int y_cluster_bin_key = it->first.second;

  return ClusterBinKey{x_cluster_bin_key,y_cluster_bin_key};
}

ClusterBinKey MidLineSearch::GetClusterBinKeyOfNextClusterFromIterator(ReverseMidLineCoordinatesIterator it)
{
  auto it_next =  next(it,1);

  int x_cluster_bin_key = it_next->first.first;
  int y_cluster_bin_key = it_next->first.second;

  return ClusterBinKey{x_cluster_bin_key,y_cluster_bin_key};
}

ClusterBinKey MidLineSearch::GetSingleClusterBinKey()
{
  int x_cluster_bin_key =  midline_clusters_coordinates_.begin()->first.first;
  int y_cluster_bin_key =  midline_clusters_coordinates_.begin()->first.second;

  return ClusterBinKey{x_cluster_bin_key,y_cluster_bin_key};
}


bool MidLineSearch::IsNewGroup()
{
  return connected_cluster_bin_keys_.empty();
}


void MidLineSearch::AddClusterBinKeyToGroup(ClusterBinKey cluster_bin_key)
{
  connected_cluster_bin_keys_.push_back(cluster_bin_key);
  }

void MidLineSearch::SafeGroup()
{
    grouped_cluster_bin_keys_.push_back(connected_cluster_bin_keys_);
}

void MidLineSearch::NewGroup()
{
    connected_cluster_bin_keys_.clear();
}


bool MidLineSearch::IsGroupAble()
{
    return midline_clusters_coordinates_.size() >= 2;
}


bool MidLineSearch::HasSingleCluster()
{
    return midline_clusters_size_.size() == 1;
}


vector<int> MidLineSearch::LinkConnectedClusters()
{

    vector<int> linked_clusters(cluster_centroids_.size());
    fill(linked_clusters.begin(),linked_clusters.end(),-1);

    for(int i=0; i<cluster_centroids_.size(); i++)
    {
        int clusters_ix = cluster_centroids_[i].x;
        int clusters_iy = cluster_centroids_[i].y;

        for(int j=0; j<cluster_centroids_.size(); j++)
        {
            if(IsPermuted(i,j,used_permutations_)) continue;

            int clusters_jx = cluster_centroids_[j].x;
            int clusters_jy = cluster_centroids_[j].y;

            float cluster_distance = sqrt(pow(clusters_jx-clusters_ix,2) + pow(clusters_jy-clusters_iy,2));

            if(cluster_distance > kMinMidLineClusterDistance_ && cluster_distance < kMaxMidLineClusterDistance_)
            {
                linked_clusters.at(i) = j;
            }
        }
    }

    for(int i=0; i<linked_clusters.size();i++)
    {
        cout << i << " " << linked_clusters.at(i) << "\t "<<  cluster_centroids_[i].x << " " << cluster_centroids_[i].y<< endl;
    }

    return linked_clusters;

}


bool MidLineSearch::IdAlreadyConnected(int id)
{

   for(auto it:grouped_mid_line_clusters_)
   {
       auto iter = find(it.begin(), it.end(), id);

       if (iter != it.end()){ return true;}
   }
   return false;
}


void MidLineSearch::GroupMidLineClusters()
{



    vector<int> linked_clusters = LinkConnectedClusters();
    vector<int> resolved_linkage_clusters;

    for(int i=0; i<cluster_centroids_.size();i++)
    {
        int id = linked_clusters.at(i);

        if(IdAlreadyConnected(id)) continue;

        resolved_linkage_clusters.clear();


        if(id!=kEndOfLinkageMarker_)
        {
            resolved_linkage_clusters.push_back(i);
            resolved_linkage_clusters.push_back(id);
        }

        while(id!=kEndOfLinkageMarker_)
        {
            id = linked_clusters.at(id);

            if(id!=kEndOfLinkageMarker_)
            {
                resolved_linkage_clusters.push_back(id);
            }
        }

        if(!resolved_linkage_clusters.empty())
        {
            grouped_mid_line_clusters_.push_back(resolved_linkage_clusters);
        }

    }
}

/*
void MidLineSearch::GroupClusters()
{
    if (IsGroupAble())
    {
        auto it_start = midline_clusters_coordinates_.rbegin();
        auto it_end = next(midline_clusters_coordinates_.rend(),-1);
        auto last_iteration = next(midline_clusters_coordinates_.rend(),-2);

        for (auto it = it_start; it != it_end; ++it )
        {
            float distance = GetCurrentStartToNextEndClusterDistance(GetStartPointOfCurrentCluster(it), GetEndPointOfNextCluster(it));

            if(ClustersAreConnected(distance))
            {
                if(IsNewGroup())
                {
                    AddClusterBinKeyToGroup(GetClusterBinKeyOfCurrentClusterFromIterator(it));
                    AddClusterBinKeyToGroup(GetClusterBinKeyOfNextClusterFromIterator(it));
                }
                else
                {
                    AddClusterBinKeyToGroup(GetClusterBinKeyOfNextClusterFromIterator(it));
                }

                if(last_iteration == it) SafeGroup();

                continue;
            }
            else
            {
                if(IsNewGroup())
                {
                    if(last_iteration == it)
                    {
                        AddClusterBinKeyToGroup(GetClusterBinKeyOfCurrentClusterFromIterator(it));
                        SafeGroup();
                        NewGroup();

                        AddClusterBinKeyToGroup(GetClusterBinKeyOfNextClusterFromIterator(it));
                        SafeGroup();
                        NewGroup();
                    }
                    else
                    {
                        AddClusterBinKeyToGroup(GetClusterBinKeyOfCurrentClusterFromIterator(it));
                        SafeGroup();
                        NewGroup();
                    }
                }
                else
                {
                    if(last_iteration == it)
                    {
                        SafeGroup();
                        NewGroup();

                        AddClusterBinKeyToGroup(GetClusterBinKeyOfNextClusterFromIterator(it));
                        SafeGroup();
                        NewGroup();
                    }
                    else
                    {
                        SafeGroup();
                        NewGroup();
                    }
                }
            }
        }
    }
    else if(HasSingleCluster())
    {
        AddClusterBinKeyToGroup(GetSingleClusterBinKey());
        SafeGroup();
        NewGroup();
    }
    else
    {
        cout << "no clusters found " << endl;
        system("pause");
    }
}
*/


