#include "start_of_lines_search.h"



StartOfLinesSearch::StartOfLinesSearch(
int image_height,
int image_width,
StartOfLinesSearchInitializationParameters init):
kImageHeight_(image_height),
kImageWidth_(image_width),
kBottomRow_(init.bottom_row),
kMidRow_(init.mid_row),
kTopRow_(init.top_row),
kMinLineWidth_(init.min_line_width),
kMaxLineWidth_(init.max_line_width),
kMinTrackWidth_(init.min_track_width),
kMaxTrackWidth_(init.max_track_width),
kWindowSizeForLineSearch_(init.window_size_for_line_search),
kLineThreshold_(init.line_threshold),
kMidLineThreshold_(init.mid_line_threshold),
kWindowSizeForMidLineSearch_(init.window_size_for_mid_line_search),
kMaxDistanceBetweenAdjacentRowPairs_(init.max_distance_between_adjacent_row_pairs),
kCarPositionInFrame_(init.car_position_in_frame),
kRoadModelLeftLine_(init.road_model_left_line),
kRoadModelRightLine_(init.road_model_right_line),
kLineToCarDistanceThreshold_(init.line_to_car_distance_threshold),
kLeftLineToCarDistance_(kCarPositionInFrame_ - kRoadModelLeftLine_),
kRightLineToCarDistance_(kRoadModelRightLine_ - kCarPositionInFrame_),
kMinLeftLineToCarDistance_(kLeftLineToCarDistance_ - kLineToCarDistanceThreshold_),
kMaxLeftLineToCarDistance_(kLeftLineToCarDistance_ + kLineToCarDistanceThreshold_),
kMinRightLineToCarDistance_(kRightLineToCarDistance_ - kLineToCarDistanceThreshold_),
kMaxRightLineToCarDistance_(kRightLineToCarDistance_ + kLineToCarDistanceThreshold_)
{
row_filter_activations_.insert(make_pair(kBottomRow_,vector<int>(kImageWidth_)));
row_filter_activations_.insert(make_pair(kMidRow_,vector<int>(kImageWidth_)));
row_filter_activations_.insert(make_pair(kTopRow_,vector<int>(kImageWidth_)));

mid_row_is_matched_ = false;
pattern_matches_count_ = 0;
pattern_to_car_matches_count_ = 0;
}

/*! returns if start parameters were found */
StartOfLinesSearchReturnInfo StartOfLinesSearch::GetReturnInfo()
{
    return StartOfLinesSearchReturnInfo{has_found_start_parameters_};
}


/*! Entry Point of the class.
 * Searches for a pattern in 3 rows
  */
StartOfLinesSearchReturnInfo StartOfLinesSearch::FindStartParameters()
{
    ClearRowFilterActivations(rows_to_search_for_lines_,row_filter_activations_);

    FilterRowsForActivations(image_,rows_to_search_for_lines_,row_filter_activations_,kImageWidth_,kWindowSizeForLineSearch_,kLineThreshold_);

    FindStartAndWidthOfRowSegments(rows_to_search_for_lines_,row_filter_activations_,row_segments_raw_,kImageWidth_);

    CheckMidRowMatch(row_segments_raw_, mid_row_is_matched_, kMidRow_);

    if(mid_row_is_matched_)
    {
        RejectFalseLineWidth(rows_to_search_for_lines_,row_segments_raw_,row_segments_true_line_width_,kMinLineWidth_,kMaxLineWidth_);

        RejectFalseTrackWidth(rows_to_search_for_lines_,row_segments_true_line_width_,row_segments_true_track_width_,kMinTrackWidth_,kMaxTrackWidth_);

        CheckIfFoundAdjacentRowPairs(row_segments_true_track_width_,found_adjacent_row_pairs_,kBottomRow_,kMidRow_);

        if(found_adjacent_row_pairs_)
        {
            RejectFalseAlignedAdjacentRowPairs(row_segments_true_track_width_,row_segments_true_adjacent_track_width_, kBottomRow_,kMidRow_,kMaxDistanceBetweenAdjacentRowPairs_);

            CheckPatternMatch(image_,row_segments_true_adjacent_track_width_,pattern_matches_);
            CountPatternMatches(pattern_matches_, pattern_matches_count_);

            if(pattern_matches_count_ >= 1)
            {
                // TODO: implement case when driving on right site
                CheckPatternToCarMatch(pattern_matches_,pattern_to_car_matches_,kCarPositionInFrame_,kMinLeftLineToCarDistance_,kMaxLeftLineToCarDistance_,kMinRightLineToCarDistance_,kMaxRightLineToCarDistance_);

                CountPatternToCarMatches(pattern_to_car_matches_, pattern_to_car_matches_count_);

                if(pattern_to_car_matches_count_ >= 1)
                {
                   has_found_start_parameters_ = true;
                   SetStartParameters(pattern_to_car_matches_,start_parameters_, pattern_matches_count_, kBottomRow_, kMidRow_);
                }
            }
        }
    }

    return GetReturnInfo();
}


void StartOfLinesSearch::SetImage(
Mat image)
{
    image_ = image;
}

/*! Slides a Kernel (1xkWindowSizeForLineSearch_) over 3 image rows and searches for white pixels*/
void StartOfLinesSearch::FilterRowsForActivations(
Mat image,
vector<int> rows_to_search_for_lines,
map<int,vector<int>> &row_filter_activations,
const int kImageWidth,
const int kWindowSizeForLineSearch,
const int kLineThreshold)
{
    int filtering_start = ((kWindowSizeForLineSearch-1)/2);
    int filtering_end   = kImageWidth-((kWindowSizeForLineSearch-1)/2);

    int filter_start = -((kWindowSizeForLineSearch-1)/2);
    int filter_end   = ((kWindowSizeForLineSearch-1)/2);

    for ( auto &row : rows_to_search_for_lines )
    {
      for (int i=filtering_start; i<filtering_end; i++)
      {
        for (int k=filter_start; k<=filter_end; k++)
        {
          int x = i+k;
          int y = row;

          if( GetPixelValue(image, x,y) >= kLineThreshold)
          {
            SetRowFilterActivation(row_filter_activations,row,i);
          }
        }
      }
    }
  }

/*! Searches for the found white pixels and meassures their length*/
void StartOfLinesSearch::FindStartAndWidthOfRowSegments(
vector<int> rows_to_search_for_lines,
map<int,vector<int>> row_filter_activations,
multimap<int,SegmentStartIDAndWidth> &row_segments_raw,
const int kImageWidth)
{
    for ( auto &row : rows_to_search_for_lines )
    {
        for (int i=0;i<kImageWidth;i++)
        {
          if(RowFilterIndexActivated(row_filter_activations,row,i))
          {
            int width = 0;
            int start_id = 0;

            MeasureSegment(row_filter_activations, kImageWidth,row,i,start_id,width);

            row_segments_raw.insert(pair<int,SegmentStartIDAndWidth>(row, SegmentStartIDAndWidth{start_id,width}));

          }
        }
    }
}
/*! Checks if 3 activation in mid row were found this is the pattern to look for*/
bool StartOfLinesSearch::CheckMidRowMatch(multimap<int,SegmentStartIDAndWidth> row_segments_raw, bool &mid_row_is_matched, const int kMidRow)
{
    if(row_segments_raw.count(kMidRow) >= 3)
    {
        mid_row_is_matched = true;
    }
    else {
        mid_row_is_matched = false;
    }
}

/*! Meassures the width of the found segments and rejects segments which have not the width of a line*/
void StartOfLinesSearch::RejectFalseLineWidth(vector<int> rows_to_search_for_lines,
                                              multimap<int,SegmentStartIDAndWidth> row_segments_raw,
                                              multimap<int,TrueLineWidthRowId> &row_segments_true_line_width,
                                              const int kMinLineWidth,
                                              const int kMaxLineWidth)
{

    for ( auto &row : rows_to_search_for_lines )
    {
        //pair<multimap<int,SegmentStartIDAndWidth>::iterator,multimap<int,SegmentStartIDAndWidth>::iterator> ret;
        auto ret = row_segments_raw.equal_range(row);

        for (auto it=ret.first; it!=ret.second; ++it)
        {
              int start_id = it->second.start_id;
              int width    = it->second.width;

              if(HasCorrectLineWidth(width,kMinLineWidth,kMaxLineWidth))
              {
                int mid_of_segment = start_id+width/2;
                row_segments_true_line_width.insert(pair<int,TrueLineWidthRowId>(row, TrueLineWidthRowId{mid_of_segment}));
              }
        }
    }
}

/*! Combines row_ids per row which together have a correct track distance and rejects false segments*/
void StartOfLinesSearch::RejectFalseTrackWidth(
vector<int> rows_to_search_for_lines,
multimap<int,TrueLineWidthRowId> row_segments_true_line_width,
multimap<int,TrueTrackWidthRowPairIds> &row_segments_true_track_width,
const int kMinTrackWidth,
const int kMaxTrackWidth)
{
    for ( auto &row : rows_to_search_for_lines )
    {
        vector<string> used_permutations;

        auto ret = row_segments_true_line_width.equal_range(row);

        int it1_pos = 0;
        int it2_pos = 0;

        for (auto it1=ret.first; it1!=ret.second; ++it1)
        {
            it2_pos=0;
            for (auto it2=ret.first; it2!=ret.second; ++it2)
            {
                if(IsPermuted(it1_pos,it2_pos,used_permutations)) continue;

                int row_id1 = it1->second.row_id;
                int row_id2 = it2->second.row_id;

                if(HasCorrectTrackWidth(row_id1,row_id2, kMinTrackWidth, kMaxTrackWidth))
                {
                  row_segments_true_track_width.insert(pair<int,TrueTrackWidthRowPairIds>(row, TrueTrackWidthRowPairIds{row_id1,row_id2}));
                }
                it2_pos++;
            }
            it1_pos++;
        }
    }
}

/*! Checks if row segments on distinct rows (bottom and mid) are aligned (form a line together)*/
void StartOfLinesSearch::RejectFalseAlignedAdjacentRowPairs(
multimap<int,TrueTrackWidthRowPairIds> row_segments_true_track_width,
vector<TrueAdjacentTrackWidthRowPairIds> &row_segments_true_adjacent_track_width,
const int kBottomRow,
const int kMidRow,
const int kMaxDistanceBetweenAdjacentRowPairs)
{

    auto ret_bottom_row = row_segments_true_track_width.equal_range(kBottomRow);
    auto ret_mid_row = row_segments_true_track_width.equal_range(kMidRow);

    for (auto itb=ret_bottom_row.first; itb!=ret_bottom_row.second; ++itb)
    {
        for (auto itm=ret_mid_row.first; itm!=ret_mid_row.second; ++itm)
        {
            int bottom_row_left_id = itb->second.row_id1;
            int bottom_row_right_id = itb->second.row_id2;

            int mid_row_left_id = itm->second.row_id1;
            int mid_row_right_id = itm->second.row_id2;

            if(HasCorrectAlignment(
               bottom_row_left_id,
               bottom_row_right_id,
               mid_row_left_id,
               mid_row_right_id,
               kMaxDistanceBetweenAdjacentRowPairs))
            {
                row_segments_true_adjacent_track_width.push_back(TrueAdjacentTrackWidthRowPairIds{
                                                                 bottom_row_left_id,
                                                                 bottom_row_right_id,
                                                                 mid_row_left_id,
                                                                 mid_row_right_id});
            }
        }
    }

}

/*! Checks if the given pattern has white pixels in the mid of the mid row and black pixels in the mid of bottom and top row
    this is the pattern to look for
*/
void StartOfLinesSearch::CheckPatternMatch(
Mat image,
vector<TrueAdjacentTrackWidthRowPairIds> row_segments_true_adjacent_track_width,
vector<PatternMatchIds> &pattern_matches)
{
    for ( auto &pattern_it : row_segments_true_adjacent_track_width )
    {
        int bottom_row_left_id  = pattern_it.bottom_row_left_id;
        int bottom_row_right_id = pattern_it.bottom_row_right_id;

        int mid_row_left_id  = pattern_it.mid_row_left_id;
        int mid_row_right_id = pattern_it.mid_row_right_id;

        int bottom_row_mid_id = GetMidId(bottom_row_left_id,bottom_row_right_id);
        int mid_row_mid_id    = GetMidId(mid_row_left_id,mid_row_right_id);
        int top_row_mid_id    = GetTopRowMidId(bottom_row_mid_id,mid_row_mid_id);

       if(PatternHasMatched(image, bottom_row_mid_id,mid_row_mid_id,top_row_mid_id))
       {
          pattern_matches.push_back(PatternMatchIds{
                                    bottom_row_left_id,
                                    bottom_row_right_id,
                                    mid_row_left_id,
                                    mid_row_right_id,
                                    bottom_row_mid_id,
                                    mid_row_mid_id,
                                    top_row_mid_id});

       }
    }


}

void StartOfLinesSearch::CountPatternMatches(
vector<PatternMatchIds> pattern_matches,
int &pattern_matches_count)
{
    pattern_matches_count = pattern_matches.size();
}


/*! collects correct patterns which have the wright distance to the camera mid point of the car */
void StartOfLinesSearch::CheckPatternToCarMatch(
vector<PatternMatchIds> pattern_matches,
vector<PatternMatchIds> &pattern_to_car_matches,
const int kCarPositionInFrame,
const int kMinLeftLineToCarDistance,
const int kMaxLeftLineToCarDistance,
const int kMinRightLineToCarDistance,
                                            const int kMaxRightLineToCarDistance)
{

    for ( auto &match_it : pattern_matches)
    {

        int bottom_row_left_id  = match_it.bottom_row_left_id;
        int bottom_row_right_id = match_it.bottom_row_right_id;

        if(HasCorrectPatternToCarDistance(
           bottom_row_left_id,
           bottom_row_right_id,
           kCarPositionInFrame,
           kMinLeftLineToCarDistance,
           kMaxLeftLineToCarDistance,
           kMinRightLineToCarDistance,
           kMaxRightLineToCarDistance))
        {
            pattern_to_car_matches.push_back(match_it);
        }

    }



}

void StartOfLinesSearch::CountPatternToCarMatches(
vector<PatternMatchIds> pattern_to_car_matches,
int &pattern_to_car_matches_count)
{
    pattern_to_car_matches_count = pattern_to_car_matches.size();
}


/*! Sets the found pattern as the start parameters for other algorithms */
void StartOfLinesSearch::SetStartParameters(
vector<PatternMatchIds> pattern_to_car_matches,
StartParameters &start_parameters,
int pattern_matches_count,
const int kBottomRow,
const int kMidRow)
{

    if(pattern_matches_count == 1)
    {

        int opposite =  kBottomRow - kMidRow;

        int bottom_row_left_id  =  pattern_to_car_matches[0].bottom_row_left_id;
        int bottom_row_right_id =  pattern_to_car_matches[0].bottom_row_right_id;
        int mid_row_left_id     =  pattern_to_car_matches[0].mid_row_left_id;
        int mid_row_right_id    =  pattern_to_car_matches[0].mid_row_right_id;

        int left_adjacent  = mid_row_left_id  - bottom_row_left_id;
        int right_adjacent = mid_row_right_id - bottom_row_right_id;

        float left_angle = CalculateAngle4Quadrants(opposite, left_adjacent);
        float right_angle = CalculateAngle4Quadrants(opposite, right_adjacent);

        start_parameters =  StartParameters{
                            bottom_row_left_id,
                            kBottomRow,
                            left_angle,
                            true,
                            bottom_row_right_id,
                            kBottomRow,
                            right_angle,
                            true};
    }
    //else { /* TODO: implement a case for more than one match */
    //    cout << "pattern_matches_count_ > 1" << endl;}

}

void StartOfLinesSearch::ClearMemory()
{
    has_found_start_parameters_ = false;
    found_adjacent_row_pairs_ = false;
    row_segments_raw_.clear();
    row_segments_true_line_width_.clear();
    row_segments_true_track_width_.clear();
    row_segments_true_adjacent_track_width_.clear();

    pattern_matches_.clear();
    pattern_to_car_matches_.clear();

    start_parameters_.reset();

}


void StartOfLinesSearch::DrawStartParameters(
Mat &rgb)
{

    circle(rgb, Point(start_parameters_.left_x,start_parameters_.left_y), 7, Scalar(255, 0, 0),CV_FILLED);
    circle(rgb, Point(start_parameters_.right_x,start_parameters_.right_y), 7, Scalar(255, 0, 0),CV_FILLED);
    line( rgb,Point(start_parameters_.left_x,start_parameters_.left_y),  Point(start_parameters_.right_x,start_parameters_.right_y), Scalar(255,0,0), 3, CV_AA);

}

StartParameters StartOfLinesSearch::GetStartParameters()
{
    return start_parameters_;
}


void StartOfLinesSearch::ClearRowFilterActivations(
vector<int> rows_to_search_for_lines,
map<int,vector<int>> &row_filter_activations)
{
    for ( auto &row : rows_to_search_for_lines )
    {
         std::fill(row_filter_activations[row].begin(), row_filter_activations[row].end(), 0);
    }
}

int StartOfLinesSearch::GetPixelValue(
Mat image,
int x,
int y)
{
    return (int)image.at<uchar>(Point(x,y));
}


void StartOfLinesSearch::SetRowFilterActivation(
map<int,vector<int>> &row_filter_activations,
int row,
int index)
{
    row_filter_activations[row].at(index) = 1;
}

bool StartOfLinesSearch::RowFilterIndexActivated(
map<int,vector<int>> row_filter_activations,
int row,
int index)
{
    if(row_filter_activations[row].at(index)==1) return true;
    else return false;
}


void StartOfLinesSearch::MeasureSegment(
map<int,vector<int>> row_filter_activations,
const int kImageWidth,
int row,
int &index,
int &start_id,
int &width)
{
   start_id = index;

    while(RowFilterIndexActivated(row_filter_activations, row,index) && index<kImageWidth)
    {
      width++;
      index++;
    }
}

bool StartOfLinesSearch::HasCorrectLineWidth(
int width,
const int kMinLineWidth,
const int kMaxLineWidth)
{
   if(width >= kMinLineWidth && width <= kMaxLineWidth) return true;
    else return false;
}

bool StartOfLinesSearch::IsPermuted(
int it1_pos,
int &it2_pos,
vector<string> &used_permutations)
{
    if(it1_pos==it2_pos){ it2_pos++; return true;}

    string permutation_hash12 = to_string(it1_pos) + to_string(it2_pos);

    auto it = find (used_permutations.begin(), used_permutations.end(), permutation_hash12);

    if (it != used_permutations.end())
    {
        it2_pos++;
        return true;
    }
    else
    {
      string permutation_hash21 = std::to_string(it2_pos) + std::to_string(it1_pos);
      used_permutations.push_back(permutation_hash12);
      used_permutations.push_back(permutation_hash21);
      return false;
    }
}

bool StartOfLinesSearch::HasCorrectTrackWidth(
int id1,
int id2,
const int kMinTrackWidth,
const int kMaxTrackWidth)
{
    int distance = abs(id2 - id1);
    if(distance>=kMinTrackWidth && distance<=kMaxTrackWidth) return true;
    else return false;
}

bool StartOfLinesSearch::CheckIfFoundAdjacentRowPairs(
multimap<int,TrueTrackWidthRowPairIds> row_segments_true_track_width,
bool &found_adjacent_row_pairs,
const int kBottomRow,
const int kMidRow)
{
    if(row_segments_true_track_width.count(kBottomRow)>0 && row_segments_true_track_width.count(kMidRow)>0){ found_adjacent_row_pairs = true; return true; }
    else{ found_adjacent_row_pairs = false; return false; }
}

bool StartOfLinesSearch::HasCorrectAlignment(
int bottom_row_left_id,
int bottom_row_right_id,
int mid_row_left_id,
int mid_row_right_id,
const int kMaxDistanceBetweenAdjacentRowPairs)
{
    int distance_left  = bottom_row_left_id - mid_row_left_id;
    int distance_right = bottom_row_right_id - mid_row_right_id;

    if(abs(distance_left) < kMaxDistanceBetweenAdjacentRowPairs && abs(distance_right) < kMaxDistanceBetweenAdjacentRowPairs) return true;
    else return false;

}

int StartOfLinesSearch::GetMidId(
int left_row_id,
int right_row_id)
{
    return (left_row_id + right_row_id)/2;
}

int StartOfLinesSearch::GetTopRowMidId(
int bottom_row_mid_id,
int mid_row_mid_id)
{
    int top_offset = mid_row_mid_id - bottom_row_mid_id;

    if(top_offset != 0)
    {
      return mid_row_mid_id + top_offset;
    }
    else {
        return mid_row_mid_id;
    }
}


bool StartOfLinesSearch::PatternHasMatched(
Mat image,
int bottom_row_mid_id,
int mid_row_mid_id,
int top_row_mid_id)
{

    bool bottom_row_matched = false;
    bool mid_row_matched = false;
    bool top_row_matched = false;

    int filter_start = -((kWindowSizeForMidLineSearch_-1)/2);
    int filter_end   =  ((kWindowSizeForMidLineSearch_-1)/2);

    for (int k=filter_start; k<=filter_end; k++)
    {

        if(GetPixelValue(image,bottom_row_mid_id+k,kBottomRow_)<= kMidLineThreshold_) bottom_row_matched = true;
        if(GetPixelValue(image,mid_row_mid_id+k,kMidRow_)>= kMidLineThreshold_) mid_row_matched = true;
        if(GetPixelValue(image,top_row_mid_id+k,kTopRow_)<= kMidLineThreshold_) top_row_matched = true;
    }

    if(bottom_row_matched && mid_row_matched && top_row_matched) return true;
    else return false;

}

bool StartOfLinesSearch::HasCorrectPatternToCarDistance(
int bottom_row_left_id,
int bottom_row_right_id,
const int kCarPositionInFrame,
const int kMinLeftLineToCarDistance,
const int kMaxLeftLineToCarDistance,
const int kMinRightLineToCarDistance,
const int kMaxRightLineToCarDistance)
{

    /* TODO: implement driving on other side of track */

    int left_id_to_car_distance  = kCarPositionInFrame - bottom_row_left_id;
    int right_id_to_car_distance = bottom_row_right_id - kCarPositionInFrame;

    bool has_correct_distance = left_id_to_car_distance  > kMinLeftLineToCarDistance  &&
                                left_id_to_car_distance  < kMaxLeftLineToCarDistance  &&
                                right_id_to_car_distance > kMinRightLineToCarDistance &&
                                right_id_to_car_distance < kMaxRightLineToCarDistance ;

    return has_correct_distance;
}




