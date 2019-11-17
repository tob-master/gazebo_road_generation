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

using namespace std;
using namespace cv;
using namespace mid_line_search;

class MidLineSearch
{
    private:

        Mat current_image_;
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
        const int kClusterBinSize;

        //const int kMaxConnectedClusterDistance_;

        const int kMinMidLineClusterDistance_;
        const int kMaxMidLineClusterDistance_;

        const Point kCarPosition_;

        //const int kEndOfGroupClustersageMarker_ = -1;


        bool has_found_mid_line_clusters_ ;
        bool has_found_group_;

        vector<string> used_permutations_;

        map<pair<int,int>,int> midline_clusters_size_;
        map<pair<int,int>,int> midline_clusters_xweight_;
        map<pair<int,int>,int> midline_clusters_yweight_;
        map<pair<int,int>,vector<pair<int,int>>> midline_clusters_coordinates_;
        map<pair<int,int>,vector<pair<int,int>>> centers_of_gravity;


        //vector<ClusterBinKey> connected_cluster_bin_keys_;
        //vector<vector<ClusterBinKey>> grouped_cluster_bin_keys_;


        //vector<vector<int>> grouped_mid_line_clusters_;

        //vector<vector<Point>> sorted_centroid_groups_;
        vector<Point> cluster_centroids_;

        vector<SingleCluster> single_clusters_;
        vector<vector<PointInDirection>> grouped_clusters_length_and_direction_;
        vector<PointInDirection> connected_clusters_length_and_direction_;


       vector<vector<Point>> mid_line_cluster_groups_;





        void InitRadialScanners();
        void SetImage(Mat image);
        void ClearMemory();
        void GroupValueablePointsInClusterBins();
        int  GetPixelValue(int x, int y);
        bool HasMinPixelValueForClustering(int pixel_value);
        bool IsAClusterPoint(int x, int y);
        bool RadialScanPoint(int x, int y);
        void AddPointToClusterBin(int x, int y);
        bool IsNewKey(int x_cluster_bin_key, int y_cluster_bin_key);
        void AddNewClusterBin(int x, int y, int x_cluster_bin_key, int y_cluster_bin_key);
        void AppendClusterBin(int x, int y, int x_cluster_bin_key, int y_cluster_bin_key);
        void RejectClustersUnderSizeThreshold();
        void ComputeClustersCentroid();
        double Distance2d(const Point& lhs, const Point& rhs);


        //Point GetStartPointOfCurrentCluster(ReverseMidLineCoordinatesIterator it);
        //Point GetEndPointOfNextCluster(ReverseMidLineCoordinatesIterator it);
        //float GetCurrentStartToNextEndClusterDistance(Point start_point_of_current_cluster, Point end_point_of_next_cluster);
        //bool  ClustersAreConnected(float distance);
        //ClusterBinKey GetClusterBinKeyOfCurrentClusterFromIterator(ReverseMidLineCoordinatesIterator it);
        //ClusterBinKey GetClusterBinKeyOfNextClusterFromIterator(ReverseMidLineCoordinatesIterator it);
        //bool IsNewGroup();

        //void AddClusterBinKeyToGroup(ClusterBinKey cluster_bin_key);
        //void SafeGroup();
        //void NewGroup();
        //bool IsGroupAble();
        //bool HasSingleCluster();
        //ClusterBinKey GetSingleClusterBinKey();
        //Point GetCenterOfGravityPointFromClusterBinKey(ClusterBinKey cluster_bin_key);
        //void GroupClusters();
        //void ComputeLengthAndDirectionOfConnectedClustersNearest();

        bool HasFoundMidLineClusters();
        bool HasFoundGroup();
        MidLineSearchReturnInfo GetReturnInfo();
        //void SortClusterGroupCentroids();
        //void ComputeLengthAndDirectionOfConnectedClustersBottomToTop();

        //void GroupMidLineClusters();


        void ComputeLengthAndDirectionOfConnectedClusters();
        //vector<int> GroupClustersConnectedClusters();
        bool IsPermuted(int i, int j, vector<string> &used_permutations);
        //bool IdAlreadyConnected(int id);


        void GroupClusters();
        bool IsConnected(float cluster_distance);
        void FindClusterConnections(vector<pair<int,int>> &cluster_connections);
        void FindOrientationForSingleClusters();


    public:

      MidLineSearch(int image_height, int image_width, MidLineSearchInitializationParameters init);
      MidLineSearchReturnInfo  FindMidLineClusters(Mat image);
      void CoutLengthAndDirectionOfConnectedClusters();

      vector<vector<PointInDirection>> GetGroupedMidLineClustersLengthAndDirection();
      vector<SingleCluster> GetSingleClusters();

      void DrawClusters(Mat &rgb);
      void DrawConnectedClusters(Mat &rgb);
      void DrawGroupedMidLineClustersDirections(Mat &rgb);
      void DrawGroupedMidLineClusters(Mat &rgb);
      void DrawSingleClusters(Mat &rgb);

      //float CalculateAngle(int opposite, int adjacent);
      //void SortClusters();

};

#endif // MID_LINE_SEARCH_H
