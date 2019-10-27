#include "line_classification.h"



LineClassification::LineClassification():
  bottom_row_(380),
  mid_row_(360),
  top_row_(340),
  kMinLineWidth_(3),
  kMaxLineWidth_(9),
  kMinLaneWidth_(117),
  kMaxLaneWidth_(143),
  kWindowSizeForLineSearch_(5),
  kLineThreshold_(200),
  kMidLineThreshold_(200),
  kImageHeight_(417),
  kImageWidth_(1280),
  row_spikes(kImageWidth_,3),
  kWindowSizeForMidLineSearch_(7),
  kMaxColumnDistanceForBottomAndMidPoints_(15)
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
    if(row_segments_start_and_width_.count(mid_row_) >= 3) mid_row_is_matched_ = true;
    else mid_row_is_matched_ = false;
}


void LineClassification::SetImage(Mat image)
{
    current_image_ = image;
}

void LineClassification::FilterRowSegments()
{



    for ( auto &row : rows_to_search_for_lines_ )
    {

      std::fill(row_filter_activations_[row].begin(), row_filter_activations_[row].end(), 0);

      for (int i=((kWindowSizeForLineSearch_-1)/2); i<kImageWidth_-((kWindowSizeForLineSearch_-1)/2); i++)
      {
        for (int k= -((kWindowSizeForLineSearch_-1)/2); k<=((kWindowSizeForLineSearch_-1)/2); k++)
        {

          if((int)current_image_.at<uchar>(row,i+k) >= kLineThreshold_)
          {
            row_filter_activations_[row].at(i) = 1;
          }

        }
      }

    }    

}


void LineClassification::FindStartAndWidthOfRowSegments()
{



    for ( auto &row : rows_to_search_for_lines_ )
    {
        int segment_width = 0;

        for (int x=0;x<kImageWidth_;x++)
        {
          if(row_filter_activations_[row].at(x)==1)
          {

            int segment_start_id = x;

            while(row_filter_activations_[row].at(x) == 1 && x<kImageWidth_)
            {
              segment_width++;
              x++;
            }


            row_segments_start_and_width_.insert(pair<int,tuple<int,int>>(row, make_tuple(segment_start_id,segment_width)));
            //cout << "row: " << row << "  id: " << segment_start_id << "  width: " << segment_width << endl;
            //cout << row_segments_start_and_width_.size() << endl;
            segment_width=0;

          }

        }
    }

}


void LineClassification::RejectFalseWidthRowSegments()
{


    for ( auto &row_id : rows_to_search_for_lines_ )
    {

        pair<multimap<int,tuple<int,int>>::iterator,multimap<int,tuple<int,int>>::iterator> ret;
        ret = row_segments_start_and_width_.equal_range(row_id);



        for (multimap<int,tuple<int, int>>::iterator it=ret.first; it!=ret.second; ++it)
        {
           // std::cout << "row: " << it->first << " id: " << std::get<0>(it->second) << " length: " << std::get<1>(it->second) << endl;

              int start_id = std::get<0>(it->second);
              int width    = std::get<1>(it->second);

              if(width >= kMinLineWidth_ && width <= kMaxLineWidth_)
              {

                row_segments_true_width_ids_.insert(pair<int,int>(row_id, start_id+width/2));
                //cout << "row: "<< row_id<< " " << start_id+width/2 << endl;
                //row_segments_true_width_ids.push_back(start_id+width/2);
              }
              /*else {
                row_points.push_back(idx);
              }*/

         }
    }

}

void LineClassification::RejectFalseDistantRowSegments()
{

    for ( auto &row_id : rows_to_search_for_lines_ )
    {

        vector<string> used_permutations;
        std::vector<string>::iterator it;

        pair<multimap<int,int>::iterator,multimap<int,int>::iterator> ret;
        ret = row_segments_true_width_ids_.equal_range(row_id);

        int hash1 = 0;
        int hash2 = 0;

        for (multimap<int,int>::iterator it_row1=ret.first; it_row1!=ret.second; ++it_row1)
        {
            hash2=0;
            for (multimap<int,int>::iterator it_row2=ret.first; it_row2!=ret.second; ++it_row2)
            {
                //cout << "id1: " << row_id << " r1: " << it_row1->second << " r2:" << it_row2->second
                  //   << " h1: " << hash1 << " h2: " << hash2 << endl;
                //cout << "k: " <<  it_row1._M_node << endl;

                if(hash1==hash2){ hash2++; continue;}


                string permutation_hash12 = to_string(hash1) + to_string(hash2);

                it = find (used_permutations.begin(), used_permutations.end(), permutation_hash12);

                if (it != used_permutations.end())
                {
                  //std::cout << "Element found in myvector: " << *it << '\n';
                    hash2++;
                  continue;
                }
                else
                {
                  //std::cout << "Element not found in myvector\n";
                  string permutation_hash21 = std::to_string(hash2) + std::to_string(hash1);
                  used_permutations.push_back(permutation_hash12);
                  used_permutations.push_back(permutation_hash21);
                }

                int distance = abs(it_row2->second - it_row1->second);
                //cout <<"row: "<< row_id<< " " << w << " " << row_points.at(j)<< " " << row_points.at(i)<< endl;
               // cout << "id2: " << row_id << " r1: " << it_row1->second << " r2:" << it_row2->second << " d: " << distance
                 //    << " h1: " << hash1 << " h2: " << hash2 << endl;
                if(distance>=kMinLaneWidth_ && distance<=kMaxLaneWidth_)
                {


                   // cout << "id3: " << row_id << " r1: " << it_row1->second << " r2:" << it_row2->second << " d: " << distance
                     //    << " h1: " << hash1 << " h2: " << hash2 << endl;
                  row_segments_true_width_and_distance_ids_.insert(pair<int,pair<int,int>>(row_id, make_pair(it_row1->second,it_row2->second)));

                  //correct_features.push_back(make_pair(row_segments_true_width_ids.at(i),row_segments_true_width_ids.at(j)));
                }


                hash2++;
            }
            hash1++;
        }

        //cout << endl;
    }
    //cout << "----------" << endl;
}

void LineClassification::RejectFalseDistantMidAndBottomRowSegments()
{


    if(row_segments_true_width_and_distance_ids_.count(bottom_row_)>0 && row_segments_true_width_and_distance_ids_.count(mid_row_)>0)
    {
        pair<multimap<int,pair<int,int>>::iterator,multimap<int,pair<int,int>>::iterator> ret_bottom_row;
        ret_bottom_row = row_segments_true_width_and_distance_ids_.equal_range(bottom_row_);

        pair<multimap<int,pair<int,int>>::iterator,multimap<int,pair<int,int>>::iterator> ret_mid_row;
        ret_mid_row = row_segments_true_width_and_distance_ids_.equal_range(mid_row_);

        for (multimap<int,pair<int,int>>::iterator it_bottom_row=ret_bottom_row.first; it_bottom_row!=ret_bottom_row.second; ++it_bottom_row)
        {

            for (multimap<int,pair<int,int>>::iterator it_mid_row=ret_mid_row.first; it_mid_row!=ret_mid_row.second; ++it_mid_row)
            {

                int column_distance1 = get<0>(it_bottom_row->second) - get<0>(it_mid_row->second);
                int column_distance2 = get<1>(it_bottom_row->second) - get<1>(it_mid_row->second);

                if(abs(column_distance1) < kMaxColumnDistanceForBottomAndMidPoints_ && abs(column_distance2) < kMaxColumnDistanceForBottomAndMidPoints_)
                {


                  //cout << " " << get<0>(it_bottom_row->second) << " " << get<1>(it_bottom_row->second) <<
                  //        " " << get<0>(it_mid_row->second) << " " << get<1>(it_mid_row->second) << endl;
                  row_segments_true_mid_and_bottom_.push_back(make_tuple(get<0>(it_bottom_row->second),
                                                                         get<1>(it_bottom_row->second),
                                                                         get<0>(it_mid_row->second),
                                                                         get<1>(it_mid_row->second)));
                }
            }
        }
        //cout << "______" << endl;
    }


}

void LineClassification::RejectFalseMidLineSegments()
{





    for ( auto &segment_it : row_segments_true_mid_and_bottom_ )
    {


      int mid_line_id_bottom_row = 0;
      int mid_line_id_mid_row = 0;


      int left_line_id_bottom_row = get<0>(segment_it);
      int right_line_id_bottom_row = get<1>(segment_it);

      mid_line_id_bottom_row = (left_line_id_bottom_row + right_line_id_bottom_row)/2;

      int left_line_id_mid_row = get<2>(segment_it);
      int right_line_id_mid_row = get<3>(segment_it);

      mid_line_id_mid_row = (left_line_id_mid_row + right_line_id_mid_row)/2;


      //int m = (bottom_row_ - mid_row_) /

      int mid_line_id_top_row;

      int top_offset = mid_line_id_mid_row - mid_line_id_bottom_row;



      if(top_offset != 0)
      {
        mid_line_id_top_row = mid_line_id_mid_row + top_offset;
      }
      else {
          mid_line_id_top_row = mid_line_id_mid_row;
      }



      bool mid_pattern_bottom_row = false;
      bool mid_pattern_mid_row = false;
      bool mid_pattern_top_row = false;

      for (int k= -((kWindowSizeForMidLineSearch_-1)/2); k<=((kWindowSizeForMidLineSearch_-1)/2); k++)
      {
          if((int)current_image_.at<uchar>(bottom_row_,mid_line_id_bottom_row+k)<= kMidLineThreshold_) mid_pattern_bottom_row = true;
          if((int)current_image_.at<uchar>(mid_row_,mid_line_id_mid_row+k)>= kMidLineThreshold_) mid_pattern_mid_row = true;
          if((int)current_image_.at<uchar>(top_row_,mid_line_id_top_row+k)<= kMidLineThreshold_) mid_pattern_top_row = true;
      }

       //cout << "bottom_row_: " << mid_pattern_row0 << " " << "mid_row_: " << mid_pattern_row1 << " " << "top_row_: " << mid_pattern_row2 << endl;
       if(mid_pattern_bottom_row && mid_pattern_mid_row && mid_pattern_top_row)
       {

          matched_pattern_positions.push_back(make_tuple(get<0>(segment_it),get<1>(segment_it),get<2>(segment_it),get<3>(segment_it), mid_line_id_bottom_row, mid_line_id_mid_row,mid_line_id_top_row));




       }
    }
}

float LineClassification::CalculateAngle(int opposite, int adjacent)
{

    float angle = 0;

    if(adjacent != 0 && opposite != 0)
    {
        angle = atan(float(abs(opposite)/abs(adjacent)));
        angle = angle * 180/PI;

        if (adjacent > 0 && opposite > 0)
        ;
        else if (adjacent < 0 && opposite > 0)
        {
            angle = 180 - angle;
        }
        else if (adjacent < 0 && opposite < 0)
        {
            angle = 180 + angle;
        }
        else if (adjacent > 0 && opposite < 0)
        {
            angle = 360 - angle;
        }
        else {
            cout << "something went wrong??" << endl;
        }
    }
    else if(adjacent > 0 && opposite == 0)
    {
            angle = 0;
    }
    else if(adjacent < 0 && opposite == 0)
    {
            angle = 180;
    }
    else if(adjacent == 0 && opposite > 0)
    {
            angle = 90;
    }
    else if(adjacent == 0 && opposite < 0)
    {
            angle = 270;
    }
    else
    {
        angle = 0;
    }

    return angle;


    /*
    float angle = 0;

    if(adjacent != 0)
    {
        angle =atan(float(opposite/adjacent));
        angle = angle * 180/PI;

        if (adjacent > 0)
        {
            angle = 90 - angle;
        }
        else {
            angle = -90 - angle;
        }
    }
    else
    {
        angle = 0;
    }

    return angle;

    */
}








void LineClassification::GetStartPointsAndAngles(vector<LineSearchStartParameters> &line_search_start_parameters)
{



    for ( auto &match_it : matched_pattern_positions )
    {
        int opposite =  bottom_row_ - mid_row_;

        int left_adjacent =  get<2>(match_it) - get<0>(match_it);
        int right_adjacent = get<3>(match_it) - get<1>(match_it);

        float left_angle = CalculateAngle(opposite, left_adjacent);
        float right_angle = CalculateAngle(opposite, right_adjacent);

        line_search_start_parameters.push_back(LineSearchStartParameters{get<2>(match_it),
                                                                          mid_row_,
                                                                          left_angle,
                                                                          get<3>(match_it),
                                                                          mid_row_,
                                                                          right_angle});



    }

}

void LineClassification::DrawStartParameters(Mat &rgb, vector<LineSearchStartParameters> &line_search_start_parameters)
{
    //Mat rgb;
    //cv::cvtColor(grey, rgb, CV_GRAY2BGR);

    for ( auto &start_parameters_it : line_search_start_parameters )
    {

        circle(rgb, Point(start_parameters_it.left_x,start_parameters_it.left_y), 7, Scalar(0, 255, 255));
        circle(rgb, Point(start_parameters_it.right_x,start_parameters_it.right_y), 7, Scalar(255, 255, 0));

    }




}


Mat LineClassification::DrawMatches()
{
    Mat rgb;
    cv::cvtColor(current_image_, rgb, CV_GRAY2BGR);

    for ( auto &match_it : matched_pattern_positions )
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


void LineClassification::ClearMemory()
{
    matched_pattern_positions.clear();
    row_segments_start_and_width_.clear();

    row_segments_true_width_ids_.clear();

    row_segments_true_width_and_distance_ids_.clear();
    row_segments_true_mid_and_bottom_.clear();
}

void LineClassification::FindStartParametersForLineTracking(Mat image,
                                                            vector<LineSearchStartParameters> &line_search_start_parameters)
{
    SetImage(image);
    FilterRowSegments();
    FindStartAndWidthOfRowSegments();
    CheckMidRowMatch();
    if(mid_row_is_matched_)
    {
        RejectFalseWidthRowSegments();
        RejectFalseDistantRowSegments();
        RejectFalseDistantMidAndBottomRowSegments();
        RejectFalseMidLineSegments();
    }
    GetStartPointsAndAngles(line_search_start_parameters);


    ClearMemory();
}

/*
      for(int i=0; i<correct_features_row0.size();i++)
      {
        for(int j=0; j<correct_features_row1.size();j++)
        {

          //int g = bottom_row_ - mid_row_;

          int a1 = correct_features_row0.at(i).first - correct_features_row1.at(j).first;

          int a2 = correct_features_row0.at(i).second - correct_features_row1.at(j).second;


          if(abs(a1) < kMaxColumnDistanceForBottomAndMidPoints_ && abs(a2) < kMaxColumnDistanceForBottomAndMidPoints_)
          {
            correct_features.push_back(make_tuple(correct_features_row0.at(i).first,
                                                correct_features_row0.at(i).second,
                                                correct_features_row1.at(j).first,
                                                correct_features_row1.at(j).second));
          }



        }
      }

}*/
/*
    for(int i=0; i<row_segments_true_width_ids_.size(); i++)
    {
      for(int j=0; j<row_segments_true_width_ids_.size(); j++)
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
          if(distance>=kMinLaneWidth_ && distance<=kMaxLaneWidth_)
          {
            row_segments_true_width_and_distance_ids.insert(pair<int,std::tuple<int,int>>(row, make_tuple(row_segments_true_width_ids.at(i),row_segments_true_width_ids.at(j))));

            correct_features.push_back(make_pair(row_segments_true_width_ids.at(i),row_segments_true_width_ids.at(j)));
          }
      }
    }
    */

/*
void LineClassification::CheckWidthAndDistancesOfRowSegments()
{

        for ( auto &row_id : rows_to_search_for_lines_ )
        {

            pair<multimap<int,tuple<int,int>>::iterator,multimap<int,tuple<int,int>>::iterator> ret;
            ret = row_segments_start_and_width_.equal_range(row_id);


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
                  *//*else {
                    row_points.push_back(idx);
                  }*/

     /*       }


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
                if(distance>=kMinLaneWidth_ && distance<=kMaxLaneWidth_)
                {
                  correct_features.push_back(make_pair(row_points.at(i),row_points.at(j)));
                }
            }
          }
   }
}
*/
/*
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


           if(abs(a1) < kMaxColumnDistanceForBottomAndMidPoints_ && abs(a2) < kMaxColumnDistanceForBottomAndMidPoints_)
           {
             correct_features.push_back(make_tuple(correct_features_row0.at(i).first,
                                                 correct_features_row0.at(i).second,
                                                 correct_features_row1.at(j).first,
                                                 correct_features_row1.at(j).second));
           }



         }
       }




      // cout << "tpsize: " << true_points_row0.at(0).first << endl;


*/
/*
          for ( auto &t : correct_features_row0 )
          {
              std::cout <<"r0: " << t.first << " " << t.second << endl;
          }



      for ( auto &t : correct_features_row1 )
      {
          std::cout <<"r1: " << t.first << " " << t.second << endl;
      }

*/




/*

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
*/
/*
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
    *//*else {
      row_points.push_back(idx);
    }*/
/*
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
          if(distance>=kMinLaneWidth_ && distance<=kMaxLaneWidth_)
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



