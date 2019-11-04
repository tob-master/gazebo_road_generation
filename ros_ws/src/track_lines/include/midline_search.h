#ifndef MIDLINE_SEARCH_H
#define MIDLINE_SEARCH_H

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

#include "own_datatypes.h"
#include "own_utils.h"
#include "own_defines.h"

using namespace std;
using namespace cv;

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

        const int kMaxConnectedClusterDistance_;

        map<pair<int,int>,int> midline_clusters_size_;
        map<pair<int,int>,int> midline_clusters_xweight_;
        map<pair<int,int>,int> midline_clusters_yweight_;
        map<pair<int,int>,vector<pair<int,int>>> midline_clusters_coordinates_;
        map<pair<int,int>,vector<pair<int,int>>> centers_of_gravity;

        vector<vector<pair<int,int>>> found_graphs;

        vector<TwoConnectedClusters> connected_clusters;
        vector<ConnectedClusterKeys> connected_cluster_keys;

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
        void ComputeClustersCenterOfGravity();

    public:

      MidLineSearch(int image_height, int image_width, MidLineSearchInitializationParameters init);
      void FindMidLineClusters(Mat image);





      void FindConnectedClusters();
      vector<pair<int,int>> GetMidLineClustersCenterOfGravity();

      void DrawClusters(Mat &rgb);
      void DrawConnectedClusters(Mat &rgb);
      void ComputeConnectedClusterSlopes();
      //float CalculateAngle(int opposite, int adjacent);


      //void SortClusters();

};

#endif
