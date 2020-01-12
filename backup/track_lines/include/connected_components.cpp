#include "connected_components.h"

ConnectedComponentsSearch::ConnectedComponentsSearch(int image_height, int image_width, ConnectedComponentsSearchInitializationParameters init):
kImageHeight_(image_height),
kImageWidth_(image_width),
kImageSize_(image_width,image_height),
kConnectionCount_(init.connection_count),
kMaxMidLineComponentSize_(init.max_mid_line_component_size),
kMinMidLineComponentSize_(init.min_mid_line_component_size),
kMaxMidLineComponentVolume_(init.max_mid_line_component_volume),
kMinMidLineComponentVolume_(init.min_mid_line_component_volume),
kMinMidLineComponentsDistance_(init.min_mid_line_component_distance),
kMaxMidLineComponentsDistance_(init.max_mid_line_component_distance),
kEndOfLinkageMarker_(init.end_of_linkage_marker),
kMaxROICenterToCentroidDistance_(init.max_roi_center_to_centroid_distance),
kCarPosition_(init.car_position_x,init.car_position_y)
{


}

bool ConnectedComponentsSearch::HasFoundMidLineComponents()
{
   has_found_mid_line_components_ = (mid_line_components_.size() > 0);

   return has_found_mid_line_components_;
}


bool ConnectedComponentsSearch::HasFoundGroup()
{
    for (auto& it: mid_line_cluster_groups_)
    {
        if(it.size() > 1)
        {
            has_found_mid_line_group_ = true;
            return true;
        }
    }

    has_found_mid_line_group_ = false;
    return false;
}


ConnectedComponentsSearchReturnInfo ConnectedComponentsSearch::GetReturnInfo()
{
   return ConnectedComponentsSearchReturnInfo{has_found_mid_line_components_,has_found_mid_line_group_};
}

ConnectedComponentsSearchReturnInfo ConnectedComponentsSearch::FindConnectedComponents(Mat image)
{
    SetImage(image);
    ClearMemory();
    ApplyConnectedComponents();
    FilterMidLineComponents();

    if(HasFoundMidLineComponents())
    {
        GroupMidLineComponents();

        if(HasFoundGroup())
        {
            ComputeLengthAndDirectionOfConnectedMidLineComponents();
        }
    }

    return GetReturnInfo();
}


void ConnectedComponentsSearch::FilterMidLineComponents()
{
    Mat under_max_size_components_truth_table = components_stats_.col(COMPONENT_SIZE_COLUMN)<kMaxMidLineComponentSize_;
    Mat over_min_size_components_truth_table  = components_stats_.col(COMPONENT_SIZE_COLUMN)>kMinMidLineComponentSize_;

    Mat components_stats_float;
    components_stats_.convertTo(components_stats_float, CV_32F);


    Mat components_width  = components_stats_float.col(ROI_WIDTH_COLUMN);
    Mat components_height = components_stats_float.col(ROI_HEIGHT_COLUMN);
    Mat components_volume = components_width.mul(components_height);

    Mat under_max_volume_components_truth_table = components_volume<kMaxMidLineComponentVolume_;
    Mat over_min_volume_components_truth_table  = components_volume>kMinMidLineComponentVolume_;

    Mat components_roi_x = components_stats_float.col(ROI_X_COLUMN);
    Mat components_roi_y = components_stats_float.col(ROI_Y_COLUMN);

    Mat components_roi_width  = components_stats_float.col(ROI_WIDTH_COLUMN);
    Mat components_roi_height = components_stats_float.col(ROI_HEIGHT_COLUMN);


    components_roi_width/=2;
    components_roi_height/=2;

    components_roi_x += components_roi_width;
    components_roi_y += components_roi_height;

    components_centroids_.convertTo(components_centroids_, CV_32F);

    Mat components_centroids_x = components_centroids_.col(CENTROIDS_X_COLUMN);
    Mat components_centroids_y = components_centroids_.col(CENTROIDS_Y_COLUMN);

    Mat dx,dx2,dy,dy2,distance, distance2;

    subtract(components_roi_x, components_centroids_x, dx);
    subtract(components_roi_y, components_centroids_y, dy);
    pow(dx, 2, dx2);
    pow(dy, 2, dy2);
    add(dx2,dy2,distance2);
    pow(distance2, 0.5, distance);

    Mat under_max_roi_center_to_centroid_distance_components_truth_table =  distance < kMaxROICenterToCentroidDistance_;

    Mat descending_sorted_y_centroid_component_ids;
    sortIdx(components_centroids_y, descending_sorted_y_centroid_component_ids, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING);

    under_max_size_components_truth_table.convertTo(under_max_size_components_truth_table, CV_8UC1);
    over_min_size_components_truth_table.convertTo(over_min_size_components_truth_table, CV_8UC1);
    under_max_volume_components_truth_table.convertTo(under_max_volume_components_truth_table, CV_8UC1);
    over_min_volume_components_truth_table.convertTo(over_min_volume_components_truth_table, CV_8UC1);
    under_max_roi_center_to_centroid_distance_components_truth_table.convertTo(under_max_roi_center_to_centroid_distance_components_truth_table, CV_8UC1);

    for (int component_id = 0; component_id < components_count_; component_id++)
    {
        int id = descending_sorted_y_centroid_component_ids.at<int>(component_id,0);

        if (under_max_size_components_truth_table.at<uchar>(id, 0)&&
            over_min_size_components_truth_table.at<uchar>(id, 0)&&
            under_max_volume_components_truth_table.at<uchar>(id, 0)&&
            over_min_volume_components_truth_table.at<uchar>(id, 0)&&
            under_max_roi_center_to_centroid_distance_components_truth_table.at<uchar>(id, 0)    )
        {
            int component_centroid_x = components_centroids_.at<float>(id,CENTROIDS_X_COLUMN);
            int component_centroid_y = components_centroids_.at<float>(id,CENTROIDS_Y_COLUMN);
            int component_roi_x      = components_stats_.at<int>(id,ROI_X_COLUMN);
            int component_roi_y      = components_stats_.at<int>(id,ROI_Y_COLUMN);
            int component_roi_width  = components_stats_.at<int>(id,ROI_WIDTH_COLUMN);
            int component_roi_height = components_stats_.at<int>(id,ROI_HEIGHT_COLUMN);
            int component_size       = components_stats_.at<int>(id,COMPONENT_SIZE_COLUMN);

            mid_line_components_.push_back(ConnectedComponent{component_centroid_x,
                                                              component_centroid_y,
                                                              component_roi_x,
                                                              component_roi_y,
                                                              component_roi_width,
                                                              component_roi_height,
                                                              component_size});

            cluster_centroids_.push_back(Point(component_centroid_x,component_centroid_y));

        }
    }
}


bool ConnectedComponentsSearch::IsPermuted(int i, int j, vector<string> &used_permutations)
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




vector<pair<int,int>> ConnectedComponentsSearch::FindClusterConnections(vector<pair<int,int>> &cluster_connections)
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

            float component_distance = sqrt(pow(clusters_jx-clusters_ix,2) + pow(clusters_jy-clusters_iy,2));

            if(component_distance > kMinMidLineComponentsDistance_ && component_distance < kMaxMidLineComponentsDistance_)
            {
                cluster_connections.push_back(make_pair(i,j));
            }
        }
    }
}


double ConnectedComponentsSearch::Distance2d(const Point& lhs, const Point& rhs)
{
    return sqrt(pow(lhs.x-rhs.x,2) + pow(lhs.y-rhs.y,2));
}

void ConnectedComponentsSearch::GroupMidLineComponents()
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


void ConnectedComponentsSearch::ComputeLengthAndDirectionOfConnectedMidLineComponents()
{
    for (auto& it: mid_line_cluster_groups_)
    {
            for(int i=0; i<it.size()-1; i++)
            {
                int x_bottom =  it[i].x;
                int y_bottom =  it[i].y;
                int x_top    =  it[i+1].x;
                int y_top    =  it[i+1].y;

                int opposite = y_bottom - y_top;
                int adjacent = x_top - x_bottom;

                int length = sqrt(pow(adjacent,2)+pow(opposite,2));
                float angle = CalculateAngle4Quadrants(opposite, adjacent);

                connected_mid_line_clusters_length_and_direction_.push_back(LengthAndDirectionFromConnectedComponents{x_bottom,
                                                                                                                      y_bottom,
                                                                                                                      length,
                                                                                                                      angle});
            }
            grouped_connected_mid_line_clusters_length_and_direction_.push_back(connected_mid_line_clusters_length_and_direction_);
            connected_mid_line_clusters_length_and_direction_.clear();
        }

}

void ConnectedComponentsSearch::DrawGroupedMidLineComponentsDirections(Mat &rgb)
{

    std::vector<Vec3b> colors(mid_line_cluster_groups_.size());

    for (int label = 0; label < mid_line_cluster_groups_.size(); ++label)
    {
        colors[label] = Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
    }

    for(int i=0;i<grouped_connected_mid_line_clusters_length_and_direction_.size();i++ )
    {
        for (int j=0;j<grouped_connected_mid_line_clusters_length_and_direction_[i].size();j++ ) {

            int x_start = grouped_connected_mid_line_clusters_length_and_direction_[i][j].x;
            int y_start = grouped_connected_mid_line_clusters_length_and_direction_[i][j].y;

            int length = grouped_connected_mid_line_clusters_length_and_direction_[i][j].length;
            float angle  = grouped_connected_mid_line_clusters_length_and_direction_[i][j].angle * PI/180;

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

void ConnectedComponentsSearch::DrawGroupedMidLineComponents(Mat &rgb)
{
    std::vector<Vec3b> colors(mid_line_cluster_groups_.size());

    for (int label = 0; label < mid_line_cluster_groups_.size(); ++label)
    {
        colors[label] = Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
    }


    for (int i=0;i<mid_line_cluster_groups_.size();i++)
    {
        for (int j=0;j<mid_line_cluster_groups_[i].size();j++)
        {

            int x = mid_line_cluster_groups_[i][j].x;
            int y = mid_line_cluster_groups_[i][j].y;


            circle(rgb, Point(x,y), 7, colors[i],CV_FILLED);
        }
    }

}

void ConnectedComponentsSearch::SetImage(Mat image)
{
    current_image_ = image;
}


void ConnectedComponentsSearch::ApplyConnectedComponents()
{
    components_count_ = connectedComponentsWithStats(current_image_, labeled_image_, components_stats_, components_centroids_, kConnectionCount_, CV_32S);
}



void ConnectedComponentsSearch::DrawMidLineComponentsRect(Mat &rgb)
{
    for(auto &it:mid_line_components_)
    {
       rectangle(rgb, Rect(it.roi_x,it.roi_y,it.roi_width,it.roi_height), Scalar(0, 0, 255), 1, 8 );
    }
}

void ConnectedComponentsSearch::ClearMemory()
{
    has_found_mid_line_components_ = false;
    has_found_mid_line_group_ =  false;


    mid_line_components_.clear();
    grouped_mid_line_components_.clear();
    connected_mid_line_clusters_length_and_direction_.clear();
    grouped_connected_mid_line_clusters_length_and_direction_.clear();
    used_permutations_.clear();
    cluster_centroids_.clear();
    mid_line_cluster_groups_.clear();
}

/*
vector<int> ConnectedComponentsSearch::LinkInDistanceComponents()
{
    vector<int> linked_components(mid_line_components_.size());
    fill(linked_components.begin(),linked_components.end(),kEndOfLinkageMarker_);

    for(int i=0; i<mid_line_components_.size(); i++)
    {
        int component_ix = mid_line_components_[i].centroid_x;
        int component_iy = mid_line_components_[i].centroid_y;

        for(int j=0; j<mid_line_components_.size(); j++)
        {
            if(IsPermuted(i,j,used_permutations_)) continue;

            int component_jx = mid_line_components_[j].centroid_x;
            int component_jy = mid_line_components_[j].centroid_y;

            float component_distance = sqrt(pow(component_jx-component_ix,2) + pow(component_jy-component_iy,2));

            if(component_distance > kMinMidLineComponentsDistance_ && component_distance < kMaxMidLineComponentsDistance_)
            {
                linked_components.at(i) = j;
            }
        }
    }

    return linked_components;
}


bool ConnectedComponentsSearch::IdAlreadyConnected(int id)
{

   for(auto it:grouped_mid_line_components_)
   {
       auto iter = find(it.begin(), it.end(), id);

       if (iter != it.end()){ return true;}
   }
   return false;
}


void ConnectedComponentsSearch::GroupMidLineComponents()
{



    vector<int> linked_components = LinkInDistanceComponents();
    vector<int> resolved_linkage_components;

    for(int i=0; i<mid_line_components_.size();i++)
    {
        int id = linked_components.at(i);

        if(IdAlreadyConnected(id)) continue;

        resolved_linkage_components.clear();


        if(id!=kEndOfLinkageMarker_)
        {
            resolved_linkage_components.push_back(i);
            resolved_linkage_components.push_back(id);
        }

        while(id!=kEndOfLinkageMarker_)
        {
            id = linked_components.at(id);

            if(id!=kEndOfLinkageMarker_)
            {
                resolved_linkage_components.push_back(id);
            }
        }

        if(!resolved_linkage_components.empty())
        {
            grouped_mid_line_components_.push_back(resolved_linkage_components);
        }

    }


}

void ConnectedComponentsSearch::ComputeLengthAndDirectionOfConnectedMidLineComponents()
{
    for (auto& it: grouped_mid_line_components_)
    {
            for(int i=0; i<it.size()-1; i++)
            {
                int x_bottom =  mid_line_components_[it[i]].centroid_x;
                int y_bottom =  mid_line_components_[it[i]].centroid_y;
                int x_top    =  mid_line_components_[it[i+1]].centroid_x;
                int y_top    =  mid_line_components_[it[i+1]].centroid_y;

                int opposite = y_bottom - y_top;
                int adjacent = x_top - x_bottom;

                int length = sqrt(pow(adjacent,2)+pow(opposite,2));
                float angle = CalculateAngle4Quadrants(opposite, adjacent);

                connected_mid_line_clusters_length_and_direction_.push_back(LengthAndDirectionFromConnectedComponents{x_bottom,
                                                                                                                      y_bottom,
                                                                                                                      length,
                                                                                                                      angle});
            }
            grouped_connected_mid_line_clusters_length_and_direction_.push_back(connected_mid_line_clusters_length_and_direction_);
            connected_mid_line_clusters_length_and_direction_.clear();
        }

}

*/
