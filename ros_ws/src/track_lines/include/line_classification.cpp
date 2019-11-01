#include "line_classification.h"



LineClassification::LineClassification():
  bottom_row_(380),
  mid_row_(360),
  top_row_(340),
  kMinLineWidth_(3),
  kMaxLineWidth_(9),
  kMinTrackWidth_(117),
  kMaxTrackWidth_(143),
  kWindowSizeForLineSearch_(5),
  kLineThreshold_(200),
  kMidLineThreshold_(170),
  kImageHeight_(417),
  kImageWidth_(1280),
  kWindowSizeForMidLineSearch_(7),
  kMaxDistanceBetweenAdjacentRowPairs_(15),
  kCarPositionInFrame_(640),
  kRoadModelLeftLine_(551),
  kRoadModelRightLine_(675),
  kLineToCarDistanceThreshold_(50),
  kLeftLineToCarDistance_(kCarPositionInFrame_ - kRoadModelLeftLine_),
  kRightLineToCarDistance_(kRoadModelRightLine_ - kCarPositionInFrame_),
  kMinLeftLineToCarDistance_(kLeftLineToCarDistance_ - kLineToCarDistanceThreshold_),
  kMaxLeftLineToCarDistance_(kLeftLineToCarDistance_ + kLineToCarDistanceThreshold_),
  kMinRightLineToCarDistance_(kRightLineToCarDistance_ - kLineToCarDistanceThreshold_),
  kMaxRightLineToCarDistance_(kRightLineToCarDistance_ + kLineToCarDistanceThreshold_)




{
row_filter_activations_.insert(make_pair(bottom_row_,vector<int>(kImageWidth_)));
row_filter_activations_.insert(make_pair(mid_row_,vector<int>(kImageWidth_)));
row_filter_activations_.insert(make_pair(top_row_,vector<int>(kImageWidth_)));
//row_filter_activations_.insert(make_pair(320,vector<int>(kImageWidth_)));
//row_filter_activations_.insert(make_pair(300,vector<int>(kImageWidth_)));
//row_filter_activations_.insert(make_pair(280,vector<int>(kImageWidth_)));
found_mid_row_match_ = false;


mid_row_is_matched_ = false;


}


bool LineClassification::CheckMidRowMatch()
{
    if(row_segments_raw_.count(mid_row_) >= 3)
    {
        mid_row_is_matched_ = true;
    }
    else {
        mid_row_is_matched_ = false;
    }
}

void LineClassification::SetImage(Mat image)
{
    image_ = image;
}


void LineClassification::ClearRowFilterActivations()
{
    for ( auto &row : rows_to_search_for_lines_ )
    {
         std::fill(row_filter_activations_[row].begin(), row_filter_activations_[row].end(), 0);
    }
}

int LineClassification::GetPixelValue(int x, int y)
{
    return (int)image_.at<uchar>(Point(x,y));
}


void LineClassification::SetRowFilterActivation(int row, int index)
{
    row_filter_activations_[row].at(index) = 1;
}




void LineClassification::FilterRowsForActivations()
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


bool LineClassification::RowFilterIndexActivated(int row, int index)
{
    if(row_filter_activations_[row].at(index)==1) return true;
    else return false;
}


void LineClassification::MeasureSegment(int row, int &index, int &start_id, int &width)
{

    start_id = index;

    while(RowFilterIndexActivated(row,index) && index<kImageWidth_)
    {
      width++;
      index++;
    }


}

void LineClassification::FindStartAndWidthOfRowSegments()
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

bool LineClassification::HasCorrectLineWidth(int width)
{
    if(width >= kMinLineWidth_ && width <= kMaxLineWidth_) return true;
    else return false;
}

void LineClassification::RejectFalseLineWidth()
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


bool LineClassification::IsPermuted(int it1_pos, int &it2_pos, vector<string> &used_permutations)
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

bool LineClassification::HasCorrectTrackWidth(int id1, int id2)
{
    int distance = abs(id2 - id1);
    if(distance>=kMinTrackWidth_ && distance<=kMaxTrackWidth_) return true;
    else return false;
}

void LineClassification::RejectFalseTrackWidth()
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


bool LineClassification::FoundAdjacentRowPairs()
{
    if(row_segments_true_track_width_.count(bottom_row_)>0 && row_segments_true_track_width_.count(mid_row_)>0) return true;
    else return false;
}

bool LineClassification::HasCorrectAlignment(int bottom_row_left_id, int bottom_row_right_id, int mid_row_left_id, int mid_row_right_id)
{
    int distance_left  = bottom_row_left_id - mid_row_left_id;
    int distance_right = bottom_row_right_id - mid_row_right_id;

    if(abs(distance_left) < kMaxDistanceBetweenAdjacentRowPairs_ && abs(distance_right) < kMaxDistanceBetweenAdjacentRowPairs_) return true;
    else return false;

}

void LineClassification::RejectFalseAlignedAdjacentRowPairs()
{
    if(FoundAdjacentRowPairs())
    {
        auto ret_bottom_row = row_segments_true_track_width_.equal_range(bottom_row_);
        auto ret_mid_row = row_segments_true_track_width_.equal_range(mid_row_);

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

int LineClassification::GetMidId(int left_row_id, int right_row_id)
{
    return (left_row_id + right_row_id)/2;
}

int LineClassification::GetTopRowMidId(int bottom_row_mid_id, int mid_row_mid_id)
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

bool LineClassification::PatternHasMatched(int bottom_row_mid_id, int mid_row_mid_id, int top_row_mid_id)
{

    bool bottom_row_matched = false;
    bool mid_row_matched = false;
    bool top_row_matched = false;

    int filter_start = -((kWindowSizeForMidLineSearch_-1)/2);
    int filter_end   =  ((kWindowSizeForMidLineSearch_-1)/2);

    for (int k=filter_start; k<=filter_end; k++)
    {

        if(GetPixelValue(bottom_row_mid_id+k,bottom_row_)<= kMidLineThreshold_) bottom_row_matched = true;
        if(GetPixelValue(mid_row_mid_id+k,mid_row_)>= kMidLineThreshold_) mid_row_matched = true;
        if(GetPixelValue(top_row_mid_id+k,top_row_)<= kMidLineThreshold_) top_row_matched = true;
    }

    if(bottom_row_matched && mid_row_matched && top_row_matched) return true;
    else return false;

}

void LineClassification::CheckPatternMatch()
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






void LineClassification::SetStartParameters()
{

    if(pattern_matches_count_ == 1)
    {

        int opposite =  bottom_row_ - mid_row_;

        int bottom_row_left_id  =  pattern_to_car_matches_[0].bottom_row_left_id;
        int bottom_row_right_id =  pattern_to_car_matches_[0].bottom_row_right_id;
        int mid_row_left_id     =  pattern_to_car_matches_[0].mid_row_left_id;
        int mid_row_right_id    =  pattern_to_car_matches_[0].mid_row_right_id;

        int left_adjacent  = mid_row_left_id  - bottom_row_left_id;
        int right_adjacent = mid_row_right_id - bottom_row_right_id;

        float left_angle = CalculateAngle4Quadrants(opposite, left_adjacent);
        float right_angle = CalculateAngle4Quadrants(opposite, right_adjacent);

        start_parameters_ = StartParameters{mid_row_left_id,
                                            mid_row_,
                                            left_angle,
                                            mid_row_right_id,
                                            mid_row_,
                                            right_angle};
    }
    else {

        /* TODO: implement a case for more than one match */
        cout << "pattern_matches_count_ > 1" << endl;
    }




}


void LineClassification::DrawStartParameters(Mat &rgb)
{

    circle(rgb, Point(start_parameters_.left_x,start_parameters_.left_y), 7, Scalar(0, 255, 255));
    circle(rgb, Point(start_parameters_.right_x,start_parameters_.right_y), 7, Scalar(255, 255, 0));

}

/*
Mat LineClassification::DrawMatches()
{
    Mat rgb;
    cv::cvtColor(image_, rgb, CV_GRAY2BGR);

    for ( auto &match_it : pattern_matches_ )
    {

        circle(rgb, Point(get<0>(match_it),bottom_row_), 7, Scalar(0, 255, 255));
        circle(rgb, Point(get<1>(match_it),bottom_row_), 7, Scalar(0, 0, 255));
        circle(rgb, Point(get<4>(match_it),bottom_row_), 7, Scalar(255, 255, 0));

        circle(rgb, Point(get<2>(match_it),mid_row_), 7, Scalar(0, 255, 0));
        circle(rgb, Point(get<3>(match_it),mid_row_), 7, Scalar(0, 255, 0));
        circle(rgb, Point(get<5>(match_it),mid_row_), 7, Scalar(255, 255, 0));

        circle(rgb, Point(get<6>(match_it),top_row_), 7, Scalar(255, 255, 0));




   }

    return rgb;
}
*/

void LineClassification::ClearMemory()
{

    row_segments_raw_.clear();
    row_segments_true_line_width_.clear();
    row_segments_true_track_width_.clear();
    row_segments_true_adjacent_track_width_.clear();

    pattern_matches_.clear();
    pattern_to_car_matches_.clear();

}


void LineClassification::CountPatternMatches()
{
    pattern_matches_count_ = pattern_matches_.size();

}


void LineClassification::CountPatternToCarMatches()
{
    pattern_to_car_matches_count_ = pattern_to_car_matches_.size();
}




bool LineClassification::HasCorrectPatternToCarDistance(int bottom_row_left_id, int bottom_row_right_id)
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

void LineClassification::CheckPatternToCarMatch()
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


StartParameters LineClassification::GetStartParametersForLineSearch()
{
    return start_parameters_;
}

bool LineClassification::FindStartParametersForLineSearch(Mat image)
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

/*
      for(int i=0; i<correct_features_row0.size();i++)
      {
        for(int j=0; j<correct_features_row1.size();j++)
        {

          //int g = bottom_row_ - mid_row_;

          int a1 = correct_features_row0.at(i).first - correct_features_row1.at(j).first;

          int a2 = correct_features_row0.at(i).second - correct_features_row1.at(j).second;


          if(abs(a1) < kMaxDistanceBetweenAdjacentRowPairs_ && abs(a2) < kMaxDistanceBetweenAdjacentRowPairs_)
          {
            correct_features.push_back(make_tuple(correct_features_row0.at(i).first,
                                                correct_features_row0.at(i).second,
                                                correct_features_row1.at(j).first,
                                                correct_features_row1.at(j).second));
          }



        }
      }

}

    for(int i=0; i<row_segments_true_line_width_.size(); i++)
    {
      for(int j=0; j<row_segments_true_line_width_.size(); j++)
      {
          if(i==j) continue;


          string permutation_ij = std::to_string(i) + std::to_string(j);

          it = find (used_permutations.begin(), used_permutations.end(), permutation_ij);

          if (it != used_permutations.end())
          {
            //std::cout << "Element found in myvector: " << *it << '\n';
            continue;
          }
          else
          {
            //std::cout << "Element not found in myvector\n";
            string permutation_ji = std::to_string(j) + std::to_string(i);
            used_permutations.push_back(permutation_ij);
            used_permutations.push_back(permutation_ji);
          }

          int distance = abs(row_segments_true_width_ids.at(j)-row_segments_true_width_ids.at(i));
          //cout <<"row: "<< row_id<< " " << w << " " << row_points.at(j)<< " " << row_points.at(i)<< endl;
          if(distance>=kMinTrackWidth_ && distance<=kMaxTrackWidth_)
          {
            row_segments_true_width_and_distance_ids.insert(pair<int,std::tuple<int,int>>(row, make_tuple(row_segments_true_width_ids.at(i),row_segments_true_width_ids.at(j))));

            correct_features.push_back(make_pair(row_segments_true_width_ids.at(i),row_segments_true_width_ids.at(j)));
          }
      }
    }



void LineClassification::CheckWidthAndDistancesOfRowSegments()
{

        for ( auto &row_id : rows_to_search_for_lines_ )
        {

            pair<multimap<int,tuple<int,int>>::iterator,multimap<int,tuple<int,int>>::iterator> ret;
            ret = row_segments_raw_.equal_range(row_id);


            std::vector<int> row_points;

            for (multimap<int,tuple<int, int>>::iterator it=ret.first; it!=ret.second; ++it)
            {
                //std::cout << "row: " << it->first << " id: " << std::get<0>(it->second) << " length: " << std::get<1>(it->second) << endl;

                  int start_id = std::get<0>(it->second);
                  int width    = std::get<1>(it->second);

                  if(width >= kMinLineWidth_ && width <= kMaxLineWidth_)
                  {
                    row_points.push_back(start_id+width/2);
                  }
                 else {
                    row_points.push_back(idx);


            }


          vector<string> used_permutations;
          std::vector<string>::iterator it;

          for(int i=0; i<row_points.size(); i++)
          {
            for(int j=0; j<row_points.size(); j++)
            {
                if(i==j) continue;


                string permutation_ij = std::to_string(i) + std::to_string(j);

                it = find (used_permutations.begin(), used_permutations.end(), permutation_ij);

                if (it != used_permutations.end())
                {
                  //std::cout << "Element found in myvector: " << *it << '\n';
                  continue;
                }
                else
                {
                  //std::cout << "Element not found in myvector\n";
                  string permutation_ji = std::to_string(j) + std::to_string(i);
                  used_permutations.push_back(permutation_ij);
                  used_permutations.push_back(permutation_ji);
                }

                int distance = abs(row_points.at(j)-row_points.at(i));
                //cout <<"row: "<< row_id<< " " << w << " " << row_points.at(j)<< " " << row_points.at(i)<< endl;
                if(distance>=kMinTrackWidth_ && distance<=kMaxTrackWidth_)
                {
                  correct_features.push_back(make_pair(row_points.at(i),row_points.at(j)));
                }
            }
          }
   }
}


vector<tuple<int,int,int,int,int,int>> LineClassification::SearchLineFeatures(Mat image)
{
  artefacts_count.emplace_back(bottom_row_,0);
  artefacts_count.emplace_back(mid_row_,0);
  artefacts_count.emplace_back(top_row_,0);
  artefacts_info.clear();
   correct_features.clear();

        matched_pattern_coordinates.clear();




  for ( auto &row : rows_to_search_for_lines_ )
  {

    fill(row_spikes.begin(), row_spikes.end(), 0);


    for (int i=((kWindowSizeForLineSearch_-1)/2); i<kImageWidth_-((kWindowSizeForLineSearch_-1)/2); i++)
    {
      for (int k= -((kWindowSizeForLineSearch_-1)/2); k<=((kWindowSizeForLineSearch_-1)/2); k++)
      {

        if((int)image.at<uchar>(row,i+k) >= kLineThreshold_)
        {
          row_spikes.at(i) = 1;
        }

      }
    }

    int artefact_length=0;


    for (int i=0;i<kImageWidth_;i++)
    {
      if(row_spikes.at(i)==1)
      {

        int artefact_start_id = i;

        while(row_spikes.at(i) == 1 && i<kImageWidth_)
        {
          artefact_length++;
          i++;
        }

        artefacts_info.insert(pair<int,std::tuple<int,int>>(row, make_tuple(artefact_start_id,artefact_length)));
        //cout << "row: " << row << "  id: " << artefact_start_id << "  lenght: " << artefact_length << endl;
        artefact_length=0;

      }

    }




  }




  if (artefacts_info.count(mid_row_) >= 3)
  {

    correct_features_row0.clear();
    correct_features_row1.clear();
    correct_features_row2.clear();

    CheckThicknessAndDistancesPerRow(bottom_row_,artefacts_info,correct_features_row0);
    CheckThicknessAndDistancesPerRow(mid_row_,artefacts_info,correct_features_row1);
    CheckThicknessAndDistancesPerRow(top_row_,artefacts_info,correct_features_row2);








     if(!correct_features_row0.empty() && !correct_features_row1.empty())
     {


       for(int i=0; i<correct_features_row0.size();i++)
       {
         for(int j=0; j<correct_features_row1.size();j++)
         {

           //int g = bottom_row_ - mid_row_;

           int a1 = correct_features_row0.at(i).first - correct_features_row1.at(j).first;

           int a2 = correct_features_row0.at(i).second - correct_features_row1.at(j).second;


           if(abs(a1) < kMaxDistanceBetweenAdjacentRowPairs_ && abs(a2) < kMaxDistanceBetweenAdjacentRowPairs_)
           {
             correct_features.push_back(make_tuple(correct_features_row0.at(i).first,
                                                 correct_features_row0.at(i).second,
                                                 correct_features_row1.at(j).first,
                                                 correct_features_row1.at(j).second));
           }



         }
       }




      // cout << "tpsize: " << true_points_row0.at(0).first << endl;




          for ( auto &t : correct_features_row0 )
          {
              std::cout <<"r0: " << t.first << " " << t.second << endl;
          }



      for ( auto &t : correct_features_row1 )
      {
          std::cout <<"r1: " << t.first << " " << t.second << endl;
      }








      for ( auto &t : correct_features )
      {


        int mid_row0 = 0;
        int mid_row1 = 0;


        int k1_row0 = get<0>(t);
        int k2_row0 = get<1>(t);
        mid_row0 = (k1_row0 + k2_row0)/2;

        int k1_row1 = get<2>(t);
        int k2_row1 = get<3>(t);
        mid_row1 = (k1_row1 + k2_row1)/2;

        bool mid_pattern_row0 = false;
        bool mid_pattern_row1 = false;
        bool mid_pattern_row2 = false;

        for (int k= -((kWindowSizeForMidLineSearch_-1)/2); k<=((kWindowSizeForMidLineSearch_-1)/2); k++)
        {
            if((int)image.at<uchar>(bottom_row_,mid_row0+k)<= kMidLineThreshold_) mid_pattern_row0 = true;
            if((int)image.at<uchar>(mid_row_,mid_row1+k)>= kMidLineThreshold_) mid_pattern_row1 = true;
            if((int)image.at<uchar>(top_row_,mid_row1+k)<= kMidLineThreshold_) mid_pattern_row2 = true;
        }

         //cout << "bottom_row_: " << mid_pattern_row0 << " " << "mid_row_: " << mid_pattern_row1 << " " << "top_row_: " << mid_pattern_row2 << endl;
         if(mid_pattern_row0 && mid_pattern_row1 && mid_pattern_row2)
         {

            matched_pattern_coordinates.push_back(make_tuple(get<0>(t),get<1>(t),get<2>(t),get<3>(t), mid_row0, mid_row1));




         }

      }



  }


}

  return matched_pattern_coordinates;

}


void LineClassification::CheckThicknessAndDistancesPerRow(int row_id, std::multimap<int,std::tuple<int, int>> &row_elements, std::vector<pair<int,int>> &correct_features)
{
  std::pair < std::multimap<int,std::tuple<int, int>>::iterator,  std::multimap<int,std::tuple<int, int>>::iterator> ret;
  ret = row_elements.equal_range(row_id);


  std::vector<int> row_points;

  for (std::multimap<int,std::tuple<int, int>>::iterator it=ret.first; it!=ret.second; ++it)
  {
    //std::cout << "row: " << it->first << " id: " << std::get<0>(it->second) << " length: " << std::get<1>(it->second) << endl;

    int row = it->first;
    int idx = std::get<0>(it->second);
    int thickness = std::get<1>(it->second);


    bool is_in_range = CheckLineThickness(thickness);

    if(is_in_range)
    {
      row_points.push_back(idx+thickness/2);
    }
    else {
      row_points.push_back(idx);
    }

  }


    vector<string> used_permutations;
    std::vector<string>::iterator it;

    for(int i=0; i<row_points.size(); i++)
    {
      for(int j=0; j<row_points.size(); j++)
      {
          if(i==j) continue;


          string permutation_ij = std::to_string(i) + std::to_string(j);

          it = find (used_permutations.begin(), used_permutations.end(), permutation_ij);

          if (it != used_permutations.end())
          {
            //std::cout << "Element found in myvector: " << *it << '\n';
            continue;
          }
          else
          {
            //std::cout << "Element not found in myvector\n";
            string permutation_ji = std::to_string(j) + std::to_string(i);
            used_permutations.push_back(permutation_ij);
            used_permutations.push_back(permutation_ji);
          }

          int distance = abs(row_points.at(j)-row_points.at(i));
          //cout <<"row: "<< row_id<< " " << w << " " << row_points.at(j)<< " " << row_points.at(i)<< endl;
          if(distance>=kMinTrackWidth_ && distance<=kMaxTrackWidth_)
          {
            correct_features.push_back(make_pair(row_points.at(i),row_points.at(j)));
          }
      }
    }

   // cout << "finished TP" << endl;
}


*/

bool LineClassification::CheckRowSegmentWidth(int width)
{
  if(width >= kMinLineWidth_ && width <= kMaxLineWidth_) { return true; }
  else { return false; }
}



