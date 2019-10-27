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

#include "own_datatypes.h"

using namespace std;
using namespace cv;

#define PI 3.14159265359


enum ROWS {BOTTOM_ROW, MID_ROW, TOP_ROW};

class LineClassification
{
private:

    Mat current_image_;

    bool mid_row_is_matched_;

  const int kMinLineWidth_;
  const int kMaxLineWidth_;

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

  vector<int> rows_to_search_for_lines_ = {bottom_row_,mid_row_};

  const int kWindowSizeForMidLineSearch_;

  const int kMaxColumnDistanceForBottomAndMidPoints_;


  map<int,vector<int>> row_filter_activations_;
  multimap<int,tuple<int, int>> row_segments_start_and_width_;

  bool found_mid_row_match_;

  multimap<int,int> row_segments_true_width_ids_;


  multimap<int,pair<int,int>> row_segments_true_width_and_distance_ids_;

  vector<tuple<int,int,int,int>> row_segments_true_mid_and_bottom_;

   vector<tuple<int,int,int,int,int,int,int>> matched_pattern_positions;





  //std::vector<int> row_segments_true_width_ids;

  std::vector<pair<int,int>> correct_features_row0;
  std::vector<pair<int,int>> correct_features_row1;
  std::vector<pair<int,int>> correct_features_row2;

      vector<tuple<int,int,int,int>> correct_features;




      vector<int> row_spikes;

         std::multimap<int,std::tuple<int, int>> artefacts_info;



        std::vector<pair<int,int>> artefacts_count;




        bool CheckLineThickness(int thickness);


        bool CheckRowSegmentWidth(int width);

         float CalculateAngle(int opposite, int adjacent);

         void ClearMemory();

public:
    LineClassification();

    //vector<tuple<int,int,int,int,int,int>> SearchLineFeatures(Mat image);

    void CheckThicknessAndDistancesPerRow(int row_id, std::multimap<int,std::tuple<int, int>> &row_elements, std::vector<pair<int,int>> &correct_features);

    void SetImage(Mat image);
    void FilterRowSegments();
    void FindStartAndWidthOfRowSegments();
    bool CheckMidRowMatch();
   // void CheckWidthAndDistancesOfRowSegments();

    void RejectFalseWidthRowSegments();
    void RejectFalseDistantRowSegments();
    void RejectFalseDistantMidAndBottomRowSegments();
    void RejectFalseMidLineSegments();
    void GetStartPointsAndAngles(vector<LineSearchStartParameters> &line_search_start_parameters);

    void FindStartParametersForLineTracking(Mat image,
                                            vector<LineSearchStartParameters> &line_search_start_parameters);

    void DrawStartParameters(Mat &rgb, vector<LineSearchStartParameters> &line_search_start_parameters);

    Mat DrawMatches();

};


#endif // LINE_CLASSIFICATION_H
