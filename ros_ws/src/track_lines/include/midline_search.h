#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <string>


#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <image_transport/image_transport.h>
#include <ctime>

using namespace std;
using namespace cv;

#define PI 3.14

class MidLineSearch
{
    private:
      vector<pair<int,int>> scan, rescan;

      const float kSearchScalingFactor;

      const int kIntensityThreshold;
      const int kMaxCandidateIntensity;



      const int kMidlineLength;

      const int kSearchRadius1;
      const int kSearchRadius2;

      map<pair<int,int>,vector<pair<int,int>>> cluster_coordinates_hashmap;




      map<pair<int,int>,int> candidates_count_hashmap;
      map<pair<int,int>,int> candidates_xmass_hashmap;
      map<pair<int,int>,int> candidates_ymass_hashmap;

    public:
      MidLineSearch();
      bool CheckLineThickness(int thickness);
      void InitClusterScanners();
      void FindMidlinesInFrame(Mat image);
      void SortClusters();
      void CheckClusterConnections();


      vector<pair<int,int>> GetMidlinePoints(int min_cluster_size);
};
