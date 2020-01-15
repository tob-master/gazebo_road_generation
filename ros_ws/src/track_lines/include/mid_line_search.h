#ifndef MID_LINE_SEARCH_H
#define MID_LINE_SEARCH_H

//#pragma once

#include <iostream>
#include <stdio.h>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <numeric>


#include "datatypes.h"
#include "utils.h"
#include "defines.h"
#include "depth_first_search.h"



//using namespace std;
//using namespace cv;
using namespace mid_line_search;

class MidLineSearch
{
    private:

    Mat image_;
    const int kImageWidth_;
    const int kImageHeight_;


    int kRadialScanRadius1_;
    int kRadialScanRadius2_;
    vector<pair<int,int>> radial_scan1_, radial_scan2_;

    const int kImageBorderPadding_;

    const int kMidLineLength_;
    const float kRadialScanScalingFactor_;

    const int kMinPixelValueForClustering_;
    const int kMinValuableClusterSize_;

    const int kMaxRadialScanOutOfClusterValue_;
    const int kClusterBinSize_;

    const int kMinMidLineClusterDistance_;
    const int kMaxMidLineClusterDistance_;

    const Point kCarPosition_;

    bool has_found_mid_line_clusters_ ;
    bool has_found_group_;

    map<pair<int,int>,int> midline_clusters_size_;
    map<pair<int,int>,int> midline_clusters_xweight_;
    map<pair<int,int>,int> midline_clusters_yweight_;
    map<pair<int,int>,vector<pair<int,int>>> midline_clusters_coordinates_;
    map<pair<int,int>,vector<pair<int,int>>> centers_of_gravity;

    vector<Point> cluster_centroids_;

    vector<SingleCluster> single_clusters_;
    vector<vector<PointInDirection>> grouped_clusters_length_and_direction_;

    vector<vector<Point>> mid_line_cluster_groups_;

    void InitRadialScanners(
    vector<pair<int,int>> &radial_scan1,
    vector<pair<int,int>> &radial_scan2,
    const int kRadialScanRadius1,
    const int kRadialScanRadius2);

    void GroupValueablePointsInClusterBins(
    Mat image,
    vector<pair<int,int>> radial_scan1,
    vector<pair<int,int>> radial_scan2,
    map<pair<int,int>,int> &midline_clusters_size,
    map<pair<int,int>,int> &midline_clusters_xweight,
    map<pair<int,int>,int> &midline_clusters_yweight,
    map<pair<int,int>,vector<pair<int,int>>> &midline_clusters_coordinates,
    const int kImageHeight,
    const int kImageWidth,
    const int kImageBorderPadding,
    const int kMinPixelValueForClustering,
    const int kMaxRadialScanOutOfClusterValue,
    const int kClusterBinSize);

    int  GetPixelValue(
    Mat image,
    int x,
    int y);

    bool HasMinPixelValueForClustering(
    const int pixel_value,
    const int kMinPixelValueForClustering);

    bool IsAClusterPoint(
    Mat image,
    const int x,
    const int y,
    vector<pair<int,int>> radial_scan1,
    vector<pair<int,int>> radial_scan2,
    const int kMaxRadialScanOutOfClusterValue);

    void AddPointToClusterBin(
    const int x,
    const int y,
    map<pair<int,int>,int> &midline_clusters_size,
    map<pair<int,int>,int> &midline_clusters_xweight,
    map<pair<int,int>,int> &midline_clusters_yweight,
    map<pair<int,int>,vector<pair<int,int>>> &midline_clusters_coordinates,
    const int kClusterBinSize);

    bool IsNewKey(
    int x_cluster_bin_key,
    int y_cluster_bin_key ,
    map<pair<int,int>,int> midline_clusters_size);

    void AddNewClusterBin(
    const int x,
    const int y,
    const int x_cluster_bin_key,
    const int y_cluster_bin_key,
    map<pair<int,int>,int> &midline_clusters_size,
    map<pair<int,int>,int> &midline_clusters_xweight,
    map<pair<int,int>,int> &midline_clusters_yweight,
    map<pair<int,int>,vector<pair<int,int>>> &midline_clusters_coordinates);

    void AppendClusterBin(
    const int x,
    const int y,
    const int x_cluster_bin_key,
    const int y_cluster_bin_key,
    map<pair<int,int>,int> &midline_clusters_size,
    map<pair<int,int>,int> &midline_clusters_xweight,
    map<pair<int,int>,int> &midline_clusters_yweight,
    map<pair<int,int>,vector<pair<int,int>>> &midline_clusters_coordinates);

    void RejectClustersUnderSizeThreshold(
    map<pair<int,int>,int> &midline_clusters_size,
    map<pair<int,int>,int> &midline_clusters_xweight,
    map<pair<int,int>,int> &midline_clusters_yweight,
    map<pair<int,int>,vector<pair<int,int>>> &midline_clusters_coordinates,
    const int kMinValuableClusterSize);

    void ComputeClustersCentroid(
    map<pair<int,int>,int> midline_clusters_size,
    map<pair<int,int>,int> midline_clusters_xweight,
    map<pair<int,int>,int> midline_clusters_yweight,
    vector<Point> &cluster_centroids);

    double Distance2d(
    const Point& lhs,
    const Point& rhs);

    bool HasFoundMidLineClusters(
    map<pair<int,int>,
    vector<pair<int,int>>> midline_clusters_coordinates,
    bool &has_found_mid_line_clusters);

    bool HasFoundGroup(
    vector<vector<Point>> mid_line_cluster_groups,
    bool &has_found_group);

    MidLineSearchReturnInfo GetReturnInfo();

    void ComputeLengthAndDirectionOfConnectedClusters(
    vector<vector<Point>> mid_line_cluster_groups,
    vector<vector<PointInDirection>> &grouped_clusters_length_and_direction);

    bool IsPermuted(
    int i,
    int j,
    vector<string> &used_permutations);

    void GroupClusters(
    vector<Point> cluster_centroids,
    vector<vector<Point>> &mid_line_cluster_groups,
    const Point kCarPosition);

    bool IsConnected(float cluster_distance);

    void FindClusterConnections(
    vector<Point> cluster_centroids,
    vector<pair<int,int>> &cluster_connections);

    void FindOrientationForSingleClusters();


    public:

    void SetImage(Mat image);
    void ClearMemory();

    MidLineSearch(
    int image_height,
    int image_width,
    MidLineSearchInitializationParameters init);

    MidLineSearchReturnInfo  FindMidLineClusters();
    void CoutLengthAndDirectionOfConnectedClusters();

    vector<vector<PointInDirection>> GetGroupedMidLineClustersLengthAndDirection();
    vector<SingleCluster> GetSingleClusters();

    void DrawClusters(Mat &rgb);
    void DrawConnectedClusters(Mat &rgb);
    void DrawGroupedMidLineClustersDirections(Mat &rgb);
    void DrawGroupedMidLineClusters(Mat &rgb);
    void DrawSingleClusters(Mat &rgb);

};

#endif // MID_LINE_SEARCH_H
