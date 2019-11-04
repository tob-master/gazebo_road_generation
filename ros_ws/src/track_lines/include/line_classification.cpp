#include "line_classification.h"



StartOfLinesSearch::StartOfLinesSearch(int image_height,int image_width,StartOfLinesSearchInitializationParameters init):
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




bool StartOfLinesSearch::FindStartParameters(Mat image)
{
    SetImage(image);
    FilterRowsForActivations();
    FindStartAndWidthOfRowSegments();

    CheckMidRowMatch();

    if(mid_row_is_matched_)
    {
        RejectFalseLineWidth();
        RejectFalseTrackWidth();
        RejectFalseAlignedAdjacentRowPairs();

        CheckPatternMatch();
        CountPatternMatches();

        if(pattern_matches_count_ >= 1)
        {
            // TODO: implement case when driving on right site
            CheckPatternToCarMatch();
            CountPatternToCarMatches();

            if(pattern_to_car_matches_count_ >= 1)
            {
               SetStartParameters();
               ClearMemory();
               return true;
            }
        }
    }
    ClearMemory();
    return false;
}


void StartOfLinesSearch::SetImage(Mat image)
{
    image_ = image;
}

void StartOfLinesSearch::FilterRowsForActivations()
{
    ClearRowFilterActivations();

    int filtering_start = ((kWindowSizeForLineSearch_-1)/2);
    int filtering_end   = kImageWidth_-((kWindowSizeForLineSearch_-1)/2);

    int filter_start = -((kWindowSizeForLineSearch_-1)/2);
    int filter_end   = ((kWindowSizeForLineSearch_-1)/2);

    for ( auto &row : rows_to_search_for_lines_ )
    {
      for (int i=filtering_start; i<filtering_end; i++)
      {
        for (int k=filter_start; k<=filter_end; k++)
        {
          int x = i+k;
          int y = row;

          if( GetPixelValue(x,y) >= kLineThreshold_)
          {
            SetRowFilterActivation(row,i);
          }
        }
      }
    }
  }

void StartOfLinesSearch::FindStartAndWidthOfRowSegments()
{
    for ( auto &row : rows_to_search_for_lines_ )
    {
        for (int i=0;i<kImageWidth_;i++)
        {
          if(RowFilterIndexActivated(row,i))
          {
            int width = 0;
            int start_id = 0;

            MeasureSegment(row,i,start_id,width);

            row_segments_raw_.insert(pair<int,StartAndWidth>(row, StartAndWidth{start_id,width}));

          }
        }
    }
}

bool StartOfLinesSearch::CheckMidRowMatch()
{
    if(row_segments_raw_.count(kMidRow_) >= 3)
    {
        mid_row_is_matched_ = true;
    }
    else {
        mid_row_is_matched_ = false;
    }
}

void StartOfLinesSearch::RejectFalseLineWidth()
{

    for ( auto &row : rows_to_search_for_lines_ )
    {
        //pair<multimap<int,StartAndWidth>::iterator,multimap<int,StartAndWidth>::iterator> ret;
        auto ret = row_segments_raw_.equal_range(row);

        for (auto it=ret.first; it!=ret.second; ++it)
        {
              int start_id = it->second.start_id;
              int width    = it->second.width;

              if(HasCorrectLineWidth(width))
              {
                int mid_of_segment = start_id+width/2;
                row_segments_true_line_width_.insert(pair<int,TrueLineWidthRowId>(row, TrueLineWidthRowId{mid_of_segment}));
              }
        }
    }
}

void StartOfLinesSearch::RejectFalseTrackWidth()
{
    for ( auto &row : rows_to_search_for_lines_ )
    {
        vector<string> used_permutations;

        auto ret = row_segments_true_line_width_.equal_range(row);

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

                if(HasCorrectTrackWidth(row_id1,row_id2))
                {
                  row_segments_true_track_width_.insert(pair<int,TrueTrackWidthRowPairIds>(row, TrueTrackWidthRowPairIds{row_id1,row_id2}));
                }
                it2_pos++;
            }
            it1_pos++;
        }
    }
}

void StartOfLinesSearch::RejectFalseAlignedAdjacentRowPairs()
{
    if(FoundAdjacentRowPairs())
    {
        auto ret_bottom_row = row_segments_true_track_width_.equal_range(kBottomRow_);
        auto ret_mid_row = row_segments_true_track_width_.equal_range(kMidRow_);

        for (auto itb=ret_bottom_row.first; itb!=ret_bottom_row.second; ++itb)
        {
            for (auto itm=ret_mid_row.first; itm!=ret_mid_row.second; ++itm)
            {
                int bottom_row_left_id = itb->second.row_id1;
                int bottom_row_right_id = itb->second.row_id2;

                int mid_row_left_id = itm->second.row_id1;
                int mid_row_right_id = itm->second.row_id2;

                if(HasCorrectAlignment(bottom_row_left_id,bottom_row_right_id,mid_row_left_id,mid_row_right_id))
                {
                    row_segments_true_adjacent_track_width_.push_back(TrueAdjacentTrackWidthRowPairIds{bottom_row_left_id,
                                                                                                       bottom_row_right_id,
                                                                                                       mid_row_left_id,
                                                                                                       mid_row_right_id});
                }
            }
        }
    }
}

void StartOfLinesSearch::CheckPatternMatch()
{
    for ( auto &pattern_it : row_segments_true_adjacent_track_width_ )
    {
        int bottom_row_left_id  = pattern_it.bottom_row_left_id;
        int bottom_row_right_id = pattern_it.bottom_row_right_id;

        int mid_row_left_id  = pattern_it.mid_row_left_id;
        int mid_row_right_id = pattern_it.mid_row_right_id;

        int bottom_row_mid_id = GetMidId(bottom_row_left_id,bottom_row_right_id);
        int mid_row_mid_id    = GetMidId(mid_row_left_id,mid_row_right_id);
        int top_row_mid_id    = GetTopRowMidId(bottom_row_mid_id,mid_row_mid_id);

       if(PatternHasMatched(bottom_row_mid_id,mid_row_mid_id,top_row_mid_id))
       {
          pattern_matches_.push_back(PatternMatchIds{bottom_row_left_id,
                                                      bottom_row_right_id,
                                                      mid_row_left_id,
                                                      mid_row_right_id,
                                                      bottom_row_mid_id,
                                                      mid_row_mid_id,
                                                      top_row_mid_id});

       }
    }


}

void StartOfLinesSearch::CountPatternMatches()
{
    pattern_matches_count_ = pattern_matches_.size();

}

void StartOfLinesSearch::CheckPatternToCarMatch()
{

    for ( auto &match_it : pattern_matches_ )
    {

        int bottom_row_left_id  = match_it.bottom_row_left_id;
        int bottom_row_right_id = match_it.bottom_row_right_id;

        if(HasCorrectPatternToCarDistance(bottom_row_left_id,bottom_row_right_id))
        {
            pattern_to_car_matches_.push_back(match_it);
        }

    }



}

void StartOfLinesSearch::CountPatternToCarMatches()
{
    pattern_to_car_matches_count_ = pattern_to_car_matches_.size();
}

void StartOfLinesSearch::SetStartParameters()
{

    if(pattern_matches_count_ == 1)
    {

        int opposite =  kBottomRow_ - kMidRow_;

        int bottom_row_left_id  =  pattern_to_car_matches_[0].bottom_row_left_id;
        int bottom_row_right_id =  pattern_to_car_matches_[0].bottom_row_right_id;
        int mid_row_left_id     =  pattern_to_car_matches_[0].mid_row_left_id;
        int mid_row_right_id    =  pattern_to_car_matches_[0].mid_row_right_id;

        int left_adjacent  = mid_row_left_id  - bottom_row_left_id;
        int right_adjacent = mid_row_right_id - bottom_row_right_id;

        float left_angle = CalculateAngle4Quadrants(opposite, left_adjacent);
        float right_angle = CalculateAngle4Quadrants(opposite, right_adjacent);

        start_parameters_ = StartParameters{mid_row_left_id,
                                            kMidRow_,
                                            left_angle,
                                            mid_row_right_id,
                                            kMidRow_,
                                            right_angle};
    }
    else {

        /* TODO: implement a case for more than one match */
        cout << "pattern_matches_count_ > 1" << endl;
    }




}

void StartOfLinesSearch::ClearMemory()
{

    row_segments_raw_.clear();
    row_segments_true_line_width_.clear();
    row_segments_true_track_width_.clear();
    row_segments_true_adjacent_track_width_.clear();

    pattern_matches_.clear();
    pattern_to_car_matches_.clear();

}


void StartOfLinesSearch::DrawStartParameters(Mat &rgb)
{

    circle(rgb, Point(start_parameters_.left_x,start_parameters_.left_y), 7, Scalar(0, 255, 255));
    circle(rgb, Point(start_parameters_.right_x,start_parameters_.right_y), 7, Scalar(255, 255, 0));

}

StartParameters StartOfLinesSearch::GetStartParameters()
{
    return start_parameters_;
}




