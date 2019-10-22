#ifndef LINE_CLASSIFICATION_H
#define LINE_CLASSIFICATION_H

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


class LineClassification
{
private:
  const int kLineThicknessMin;
  const int kLineThicknessMax;

  const int kTrackWidthMax;
  const int kTrackWidthMin;

  const int kRowStride;
  const int kIntensityThreshold;

  const int kImgRows;
  const int kImgCols;

  const int row0;
  const int row1;
  const int row2;

  const int kMidlineSearchSpace;

  const int kMaxColumnDistanceForRowPoints;


  std::vector<pair<int,int>> correct_features_row0;
  std::vector<pair<int,int>> correct_features_row1;
  std::vector<pair<int,int>> correct_features_row2;

      vector<tuple<int,int,int,int>> correct_features;


      vector<tuple<int,int,int,int,int,int>> matched_pattern_coordinates;

public:
  LineClassification();
  bool CheckLineThickness(int thickness);
vector<tuple<int,int,int,int,int,int>> SearchLineFeatures(Mat image);
void CheckThicknessAndDistancesPerRow(int row_id, std::multimap<int,std::tuple<int, int>> &row_elements, std::vector<pair<int,int>> &correct_features);
  vector<int> row_spikes;

     std::multimap<int,std::tuple<int, int>> artefacts_info;

    std::vector<pair<int,int>> artefacts_count;
    vector<int> line_search_regions = {row0,row1,row2};

};


#endif // LINE_CLASSIFICATION_H
