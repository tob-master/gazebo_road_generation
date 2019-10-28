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

using namespace std;
using namespace cv;

#define PI 3.14

class MidLineSearch
{
    private:

      vector<pair<int,int>> radial_scan1_, radial_scan2_;

      const float kRadialScanScalingFactor_;
      int radial_scan_radius_1_;
      int radial_scan_radius_2_;

      Mat current_image_;

      const int kMinValuableClusterSize_;

      const int kMidLineLength_;
      const int kMinPixelValueForClustering_;
      const int kMaxRadialScanOutOfClusterValue_;
      const int kMaxClusterDistance_;

      map<pair<int,int>,int> midline_clusters_size_;
      map<pair<int,int>,int> midline_clusters_xweight_;
      map<pair<int,int>,int> midline_clusters_yweight_;
      map<pair<int,int>,vector<pair<int,int>>> midline_clusters_coordinates_;
      map<pair<int,int>,vector<pair<int,int>>> centers_of_gravity;

      vector<vector<pair<int,int>>> found_graphs;

      vector<TwoConnectedClusters> connected_clusters;
      vector<ConnectedClusterKeys> connected_cluster_keys;

    public:

      MidLineSearch();

      void InitRadialScanners();
      void FindMidLineClusters(Mat grey);

      void ScanImageToFindMidLineClusters(Mat image);
      void FindConnectedClusters();
      vector<pair<int,int>> GetMidLineClustersCenterOfGravity();
      void RejectClustersUnderSizeThreshold();
      void AddPointToClustering(int x, int y, int x_cluster_key, int y_cluster_key);
      void ClearMemory();
      bool RadialScanPoint(int x, int y);
      void ComputeClustersCenterOfGravity();
      void DrawMidLineClusters(Mat &rgb);

      void DrawConnectedClusters(Mat &rgb);


      //void SortClusters();

};

#endif
