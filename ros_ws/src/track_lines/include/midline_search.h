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

using namespace std;
using namespace cv;

#define PI 3.14

class MidLineSearch
{
    private:

      vector<pair<int,int>> radial_scan1_, radial_scan2_;

      const float kRadialScanScalingFactor_;
      const int kRadialScanRadius1_;
      const int kRadialScanRadius2_;

      const int kMinValuableClusterSize_;

      const int kMidLineLength_;
      const int kMinPixelValueForClustering_;
      const int kMaxRadialScanOutOfClusterValue_;

      map<pair<int,int>,int> midline_clusters_size_;
      map<pair<int,int>,int> midline_clusters_xweight_;
      map<pair<int,int>,int> midline_clusters_yweight_;
      map<pair<int,int>,vector<pair<int,int>>> midline_clusters_coordinates_;

    public:

      MidLineSearch();

      void InitRadialScanners();
      void ScanImageToFindMidLineClusters(Mat image);
      void FindConnectedClusters();
      vector<pair<int,int>> GetMidLineClustersCenterOfGravity();


      //void SortClusters();

};

#endif
