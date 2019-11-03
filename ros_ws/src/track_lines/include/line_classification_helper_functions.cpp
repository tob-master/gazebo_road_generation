#include "line_classification.h"


void StartOfLinesSearch::ClearRowFilterActivations()
{
    for ( auto &row : rows_to_search_for_lines_ )
    {
         std::fill(row_filter_activations_[row].begin(), row_filter_activations_[row].end(), 0);
    }
}

int StartOfLinesSearch::GetPixelValue(int x, int y)
{
    return (int)image_.at<uchar>(Point(x,y));
}


void StartOfLinesSearch::SetRowFilterActivation(int row, int index)
{
    row_filter_activations_[row].at(index) = 1;
}

bool StartOfLinesSearch::RowFilterIndexActivated(int row, int index)
{
    if(row_filter_activations_[row].at(index)==1) return true;
    else return false;
}


void StartOfLinesSearch::MeasureSegment(int row, int &index, int &start_id, int &width)
{
   start_id = index;

    while(RowFilterIndexActivated(row,index) && index<kImageWidth_)
    {
      width++;
      index++;
    }
}

bool StartOfLinesSearch::HasCorrectLineWidth(int width)
{
   if(width >= kMinLineWidth_ && width <= kMaxLineWidth_) return true;
    else return false;
}

bool StartOfLinesSearch::IsPermuted(int it1_pos, int &it2_pos, vector<string> &used_permutations)
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

bool StartOfLinesSearch::HasCorrectTrackWidth(int id1, int id2)
{
    int distance = abs(id2 - id1);
    if(distance>=kMinTrackWidth_ && distance<=kMaxTrackWidth_) return true;
    else return false;
}

bool StartOfLinesSearch::FoundAdjacentRowPairs()
{
    if(row_segments_true_track_width_.count(kBottomRow_)>0 && row_segments_true_track_width_.count(kMidRow_)>0) return true;
    else return false;
}

bool StartOfLinesSearch::HasCorrectAlignment(int bottom_row_left_id, int bottom_row_right_id, int mid_row_left_id, int mid_row_right_id)
{
    int distance_left  = bottom_row_left_id - mid_row_left_id;
    int distance_right = bottom_row_right_id - mid_row_right_id;

    if(abs(distance_left) < kMaxDistanceBetweenAdjacentRowPairs_ && abs(distance_right) < kMaxDistanceBetweenAdjacentRowPairs_) return true;
    else return false;

}

int StartOfLinesSearch::GetMidId(int left_row_id, int right_row_id)
{
    return (left_row_id + right_row_id)/2;
}

int StartOfLinesSearch::GetTopRowMidId(int bottom_row_mid_id, int mid_row_mid_id)
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


bool StartOfLinesSearch::PatternHasMatched(int bottom_row_mid_id, int mid_row_mid_id, int top_row_mid_id)
{

    bool bottom_row_matched = false;
    bool mid_row_matched = false;
    bool top_row_matched = false;

    int filter_start = -((kWindowSizeForMidLineSearch_-1)/2);
    int filter_end   =  ((kWindowSizeForMidLineSearch_-1)/2);

    for (int k=filter_start; k<=filter_end; k++)
    {

        if(GetPixelValue(bottom_row_mid_id+k,kBottomRow_)<= kMidLineThreshold_) bottom_row_matched = true;
        if(GetPixelValue(mid_row_mid_id+k,kMidRow_)>= kMidLineThreshold_) mid_row_matched = true;
        if(GetPixelValue(top_row_mid_id+k,kTopRow_)<= kMidLineThreshold_) top_row_matched = true;
    }

    if(bottom_row_matched && mid_row_matched && top_row_matched) return true;
    else return false;

}

bool StartOfLinesSearch::HasCorrectPatternToCarDistance(int bottom_row_left_id, int bottom_row_right_id)
{

    /* TODO: implement driving on other side of track */

    int left_id_to_car_distance  = kCarPositionInFrame_ - bottom_row_left_id;
    int right_id_to_car_distance = bottom_row_right_id - kCarPositionInFrame_;

    bool has_correct_distance = left_id_to_car_distance  > kMinLeftLineToCarDistance_  &&
                                left_id_to_car_distance  < kMaxLeftLineToCarDistance_  &&
                                right_id_to_car_distance > kMinRightLineToCarDistance_ &&
                                right_id_to_car_distance < kMaxRightLineToCarDistance_ ;

    return has_correct_distance;
}
