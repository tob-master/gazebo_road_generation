

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
    has_found_mid_line_clusters_ = false;
    has_found_group_ = false;

    midline_clusters_size_.clear();
    midline_clusters_xweight_.clear();
    midline_clusters_yweight_.clear();
    midline_clusters_coordinates_.clear();
    centers_of_gravity.clear();
    grouped_clusters_length_and_direction.clear();
    connected_clusters_length_and_direction.clear();

    connected_cluster_bin_keys_.clear();
    grouped_cluster_bin_keys_.clear();
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


bool MidLineSearch::HasFoundMidLineClusters()
{
    has_found_mid_line_clusters_ = (midline_clusters_coordinates_.size() > 0);
    return (midline_clusters_coordinates_.size() > 0);
}

bool MidLineSearch::HasFoundGroup()
{
    for (auto& it: grouped_cluster_bin_keys_)
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

 MidLineSearchReturnInfo MidLineSearch::FindMidLineClusters(Mat image)
 {
        SetImage(image);
        ClearMemory();
        GroupValueablePointsInClusterBins();
        RejectClustersUnderSizeThreshold();

        if(HasFoundMidLineClusters())
        {
            ComputeClustersCenterOfGravity();
            GroupClusters();

            if(HasFoundGroup())
            {
               ComputeLengthAndDirectionOfConnectedClusters();
            }
        }

        return GetReturnInfo();
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


 Point MidLineSearch::GetCenterOfGravityPointFromClusterBinKey(ClusterBinKey cluster_bin_key)
 {
     pair<int,int> key = make_pair(cluster_bin_key.x_cluster_bin_key,cluster_bin_key.y_cluster_bin_key);

    int x = centers_of_gravity[key].begin()->first;
    int y = centers_of_gravity[key].begin()->second;

    return Point(x,y);
 }

 void MidLineSearch::ComputeLengthAndDirectionOfConnectedClusters()
 {
     for (auto& it: grouped_cluster_bin_keys_)
     {
         if(it.size() > 1)
         {
             //vector<double> x;
             //vector<double> y;

             for(int i=0; i<it.size()-1; i++)
             {
                 Point current_cluster_center_of_gravity = GetCenterOfGravityPointFromClusterBinKey(it[i]);
                 Point next_cluster_center_of_gravity    = GetCenterOfGravityPointFromClusterBinKey(it[i+1]);

                 int opposite =  current_cluster_center_of_gravity.y - next_cluster_center_of_gravity.y;
                 int adjacent =  next_cluster_center_of_gravity.x - current_cluster_center_of_gravity.x;

                 int length = sqrt(pow(adjacent,2)+pow(opposite,2));

                 float angle = CalculateAngle4Quadrants(opposite, adjacent) * PI/180;

                 connected_clusters_length_and_direction.push_back(LengthAndDirectionFromConnectedClusters{current_cluster_center_of_gravity.x,
                                                                                                           current_cluster_center_of_gravity.y,
                                                                                                           length,
                                                                                                           angle});
                /*
                 x.push_back(double(centers_of_gravity[it[i]].begin()->first));
                 y.push_back(double(centers_of_gravity[it[i]].begin()->second));

                 x.push_back(double(centers_of_gravity[it[i+1]].begin()->first));
                 y.push_back(double(centers_of_gravity[it[i+1]].begin()->second));

                 cout << "slope: " << atan(slope(x,y)) * 180/PI << endl;
                 */
                 //cout <<  "op: " <<  opposite  << " adj: " << adjacent << " xb " << x_bottom << " yb " << y_bottom << " xt " << x_top << " yt " << y_top << " angle: " << angle << endl;
             }
             grouped_clusters_length_and_direction.push_back(connected_clusters_length_and_direction);
             connected_clusters_length_and_direction.clear();
         }
     }
 }

 void MidLineSearch::CoutLengthAndDirectionOfConnectedClusters()
 {
     for(int i=0; i<grouped_clusters_length_and_direction.size(); i++)
     {
         for(int j=0; j<grouped_clusters_length_and_direction[i].size(); j++)
         {
             cout << "Cluster " << i << ": " << grouped_clusters_length_and_direction[i][j].x
                                             << " "  << grouped_clusters_length_and_direction[i][j].y
                                             << " "  << grouped_clusters_length_and_direction[i][j].length
                                             << " "  << grouped_clusters_length_and_direction[i][j].angle * 180/PI << endl;
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
    srand (time(NULL));

    for (auto& it: grouped_cluster_bin_keys_)
    {
        int B = rand() % 255;
        int G = rand() % 255;
        int R = rand() % 255;

        for(int i=0; i<it.size(); i++)
        {
            int x = centers_of_gravity[make_pair(it[i].x_cluster_bin_key,it[i].y_cluster_bin_key)].begin()->first;
            int y = centers_of_gravity[make_pair(it[i].x_cluster_bin_key,it[i].y_cluster_bin_key)].begin()->second;

            circle(rgb, Point(x,y), 10, Scalar(B, G, R));
        }
    }
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
