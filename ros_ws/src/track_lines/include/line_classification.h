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
#include "own_utils.h"
#include "own_defines.h"

using namespace std;
using namespace cv;
using namespace start_of_lines_search;

class StartOfLinesSearch
{
    private:

        Mat image_;
        const int kImageHeight_;
        const int kImageWidth_;

        const int kTopRow_;
        const int kMidRow_;
        const int kBottomRow_;
        vector<int> rows_to_search_for_lines_ = {kBottomRow_,kMidRow_};

        const int kLineThreshold_;
        const int kMidLineThreshold_;

        const int kWindowSizeForLineSearch_;
        const int kWindowSizeForMidLineSearch_;

        const int kMinLineWidth_;
        const int kMaxLineWidth_;

        const int kMaxTrackWidth_;
        const int kMinTrackWidth_;

        const int kMaxDistanceBetweenAdjacentRowPairs_;

        const int kCarPositionInFrame_;
        const int kRoadModelLeftLine_;
        const int kRoadModelRightLine_;

        const int kLeftLineToCarDistance_;
        const int kRightLineToCarDistance_;

        const int kLineToCarDistanceThreshold_;

        const int kMinLeftLineToCarDistance_;
        const int kMaxLeftLineToCarDistance_;

        const int kMinRightLineToCarDistance_;
        const int kMaxRightLineToCarDistance_;

        bool mid_row_is_matched_;

        int pattern_matches_count_;
        int pattern_to_car_matches_count_;

        map<int,vector<int>> row_filter_activations_;

        multimap<int,StartAndWidth> row_segments_raw_;
        multimap<int,TrueLineWidthRowId> row_segments_true_line_width_;
        multimap<int,TrueTrackWidthRowPairIds> row_segments_true_track_width_;
        vector<TrueAdjacentTrackWidthRowPairIds> row_segments_true_adjacent_track_width_;

        vector<PatternMatchIds> pattern_matches_;
        vector<PatternMatchIds> pattern_to_car_matches_;

        StartParameters start_parameters_;


        void ClearRowFilterActivations();
        int GetPixelValue(int x, int y);
        void SetRowFilterActivation(int row, int index);
        bool RowFilterIndexActivated(int row, int index);
        void MeasureSegment(int row, int &index, int &start_id, int &width);
        bool HasCorrectLineWidth(int width);
        bool IsPermuted(int it1_pos, int &it2_pos, vector<string> &used_permutations);
        bool HasCorrectTrackWidth(int id1, int id2);
        bool FoundAdjacentRowPairs();
        bool HasCorrectAlignment(int bottom_row_left_id, int bottom_row_right_id, int mid_row_left_id, int mid_row_right_id);
        int GetMidId(int left_row_id, int right_row_id);
        int GetTopRowMidId(int bottom_row_mid_id, int mid_row_mid_id);
        bool PatternHasMatched(int bottom_row_mid_id, int mid_row_mid_id, int top_row_mid_id);
        bool HasCorrectPatternToCarDistance(int bottom_row_left_id, int bottom_row_right_id);


        void SetImage(Mat image);
        void FilterRowsForActivations();
        void FindStartAndWidthOfRowSegments();
        bool CheckMidRowMatch();
        void RejectFalseLineWidth();
        void RejectFalseTrackWidth();
        void RejectFalseAlignedAdjacentRowPairs();
        void CheckPatternMatch();
        void CountPatternMatches();
        void CheckPatternToCarMatch();
        void CountPatternToCarMatches();
        void SetStartParameters();
        void ClearMemory();


    public:

        StartOfLinesSearch(int image_height,int image_width,StartOfLinesSearchInitializationParameters init);
        bool FindStartParameters(Mat image);
        StartParameters GetStartParameters();
        void DrawStartParameters(Mat &rgb);


};


#endif // LINE_CLASSIFICATION_H
