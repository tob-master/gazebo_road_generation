#ifndef START_OF_LINES_SEARCH_H
#define START_OF_LINES_SEARCH_H

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

#include "datatypes.h"
#include "utils.h"
#include "defines.h"

//using namespace std;
//using namespace cv;
using namespace start_of_lines_search;



/*! This class searches for a pattern in 3 distinct rows of the image.
 * The mid row must find 3 segments which have a correct line width and the outer line must have the corrct track width.
 * The bottom and top row must find 2 segments which have the correct track with.
 * In the mid of the segments (resembled as the mid of the lane) white pixels must be found in the mid row and no white segments in the bottom and top row.
 */
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
        bool found_adjacent_row_pairs_;

        int pattern_matches_count_;
        int pattern_to_car_matches_count_;

        map<int,vector<int>> row_filter_activations_;

        multimap<int,SegmentStartIDAndWidth> row_segments_raw_;
        multimap<int,TrueLineWidthRowId> row_segments_true_line_width_;
        multimap<int,TrueTrackWidthRowPairIds> row_segments_true_track_width_;
        vector<TrueAdjacentTrackWidthRowPairIds> row_segments_true_adjacent_track_width_;

        vector<PatternMatchIds> pattern_matches_;
        vector<PatternMatchIds> pattern_to_car_matches_;

        StartParameters start_parameters_;
        bool has_found_start_parameters_;


        void ClearRowFilterActivations(vector<int> rows_to_search_for_lines,
                                       map<int,vector<int>> &row_filter_activations);

        int GetPixelValue(Mat image,
                          int x,
                          int y);

        void SetRowFilterActivation(map<int,vector<int>> &row_filter_activations,
                                    int row,
                                    int index);

        bool RowFilterIndexActivated(map<int,vector<int>> row_filter_activations,
                                     int row,
                                     int index);

        void MeasureSegment(map<int,vector<int>> row_filter_activations,
                            const int kImageWidth,
                            int row,
                            int &index,
                            int &start_id,
                            int &width);

        bool HasCorrectLineWidth(int width,
                                 int kMinLineWidth,
                                 const int kMaxLineWidth);

        bool IsPermuted(int it1_pos,
                        int &it2_pos,
                        vector<string> &used_permutations);

        bool HasCorrectTrackWidth(int id1,
                                  int id2,
                                  const int kMinTrackWidth,
                                  const int kMaxTrackWidth);

        bool CheckIfFoundAdjacentRowPairs(multimap<int,TrueTrackWidthRowPairIds> row_segments_true_track_width,
                                          bool &found_adjacent_row_pairs,
                                          const int kBottomRow,
                                          const int kMidRow);

        bool HasCorrectAlignment(int bottom_row_left_id,
                                 int bottom_row_right_id,
                                 int mid_row_left_id,
                                 int mid_row_right_id,
                                 const int kMaxDistanceBetweenAdjacentRowPairs);

        int GetMidId(int left_row_id,
                     int right_row_id);

        int GetTopRowMidId(int bottom_row_mid_id,
                           int mid_row_mid_id);

        bool PatternHasMatched(Mat image,
                               int bottom_row_mid_id,
                               int mid_row_mid_id,
                               int top_row_mid_id);

        bool HasCorrectPatternToCarDistance(int bottom_row_left_id,
                                            int bottom_row_right_id,
                                            const int kCarPositionInFrame,
                                            const int kMinLeftLineToCarDistance,
                                            const int kMaxLeftLineToCarDistance,
                                            const int kMinRightLineToCarDistance,
                                            const int kMaxRightLineToCarDistance);



        void FilterRowsForActivations(Mat image,
                                      vector<int> rows_to_search_for_lines,
                                      map<int,vector<int>> &row_filter_activations,
                                      const int kImageWidth,
                                      const int kWindowSizeForLineSearch,
                                      const int kLineThreshold);

        void FindStartAndWidthOfRowSegments(vector<int> rows_to_search_for_lines,
                                            map<int,vector<int>> row_filter_activations,
                                            multimap<int,SegmentStartIDAndWidth> &row_segments_raw,
                                            const int kImageWidth);

        bool CheckMidRowMatch(multimap<int,
                              SegmentStartIDAndWidth> row_segments_raw,
                              bool &mid_row_is_matched,
                              const int kMidRow);

        void RejectFalseLineWidth(vector<int> rows_to_search_for_lines,
                                  multimap<int,SegmentStartIDAndWidth> row_segments_raw,
                                  multimap<int,TrueLineWidthRowId> &row_segments_true_line_width,
                                  const int kMinLineWidth,
                                  const int kMaxLineWidth);

        void RejectFalseTrackWidth(vector<int> rows_to_search_for_lines,
                                   multimap<int,TrueLineWidthRowId> row_segments_true_line_width,
                                   multimap<int,TrueTrackWidthRowPairIds> &row_segments_true_track_width,
                                   const int kMinTrackWidth,
                                   const int kMaxTrackWidth);

        void RejectFalseAlignedAdjacentRowPairs(multimap<int,TrueTrackWidthRowPairIds> row_segments_true_track_width_,
                                                vector<TrueAdjacentTrackWidthRowPairIds> &row_segments_true_adjacent_track_width_,
                                                const int kBottomRow,
                                                const int kMidRow,
                                                const int kMaxDistanceBetweenAdjacentRowPairs);

        void CheckPatternMatch(Mat image,
                               vector<TrueAdjacentTrackWidthRowPairIds> row_segments_true_adjacent_track_width,
                               vector<PatternMatchIds> &pattern_matches);

        void CountPatternMatches(vector<PatternMatchIds> pattern_matches,
                                 int &pattern_matches_count);

        void CheckPatternToCarMatch(vector<PatternMatchIds> pattern_matches,
                                    vector<PatternMatchIds> &pattern_to_car_matches,
                                    const int kCarPositionInFrame,
                                    const int kMinLeftLineToCarDistance,
                                    const int kMaxLeftLineToCarDistance,
                                    const int kMinRightLineToCarDistance,
                                    const int kMaxRightLineToCarDistance);

        void CountPatternToCarMatches(vector<PatternMatchIds> pattern_to_car_matches,
                                      int &pattern_to_car_matches_count);

        void SetStartParameters(vector<PatternMatchIds> pattern_to_car_matches,
                                StartParameters &start_parameters,
                                int pattern_matches_count,
                                const int kBottomRow,
                                const int kMidRow);


        StartOfLinesSearchReturnInfo GetReturnInfo();


    public:

        StartOfLinesSearch(int image_height,int image_width,StartOfLinesSearchInitializationParameters init);
        StartOfLinesSearchReturnInfo FindStartParameters();
        StartParameters GetStartParameters();
        void DrawStartParameters(Mat &rgb);
        void ClearMemory();
        void SetImage(Mat image);
};


#endif // START_OF_LINES_SEARCH_H
