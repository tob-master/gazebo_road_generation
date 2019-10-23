#ifndef LINE_CLASSIFICATION_H
#define LINE_CLASSIFICATION_H

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


enum ROWS {BOTTOM_ROW, MID_ROW, TOP_ROW};

class LineClassification
{
private:

  const int kMinLineThickness_;
  const int kMaxLineThickness_;

  const int kMaxLaneWidth_;
  const int kMinLaneWidth_;

  const int kWindowSizeForLineSearch_;
  const int kLineThreshold_;
  const int kMidLineThreshold_;

  const int kImageHeight_;
  const int kImageWidth_;

  const int bottom_row_;
  const int mid_row_;
  const int top_row_;

  const int kWindowSizeForMidLineSearch_;

  const int kMaxColumnDistanceForBottomAndMidPoints_;


  map<int,vector<int>> row_filter_activations;


  std::vector<pair<int,int>> correct_features_row0;
  std::vector<pair<int,int>> correct_features_row1;
  std::vector<pair<int,int>> correct_features_row2;

      vector<tuple<int,int,int,int>> correct_features;


      vector<tuple<int,int,int,int,int,int>> matched_pattern_coordinates;

      vector<int> row_spikes;

         std::multimap<int,std::tuple<int, int>> artefacts_info;

        std::vector<pair<int,int>> artefacts_count;

        vector<int> rows_to_search_for_lines_ = {bottom_row_,mid_row_,top_row_};


        bool CheckLineThickness(int thickness);
public:
    LineClassification();

    vector<tuple<int,int,int,int,int,int>> SearchLineFeatures(Mat image);

    void CheckThicknessAndDistancesPerRow(int row_id, std::multimap<int,std::tuple<int, int>> &row_elements, std::vector<pair<int,int>> &correct_features);

    void FilterRows(Mat image);
};


#endif // LINE_CLASSIFICATION_H
