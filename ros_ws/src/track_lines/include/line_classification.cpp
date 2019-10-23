#include "line_classification.h"



LineClassification::LineClassification():
  bottom_row_(380),
  mid_row_(360),
  top_row_(340),
  kMinLineThickness_(3),
  kMaxLineThickness_(9),
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
row_filter_activations.insert(make_pair(bottom_row_,vector<int>(kImageWidth_)));
row_filter_activations.insert(make_pair(mid_row_,vector<int>(kImageWidth_)));
row_filter_activations.insert(make_pair(top_row_,vector<int>(kImageWidth_)));
}


void LineClassification::FilterRows(Mat image)
{



    for ( auto &row : rows_to_search_for_lines_ )
    {

      std::fill(row_filter_activations[row].begin(), row_filter_activations[row].end(), 0);

      for (int i=((kWindowSizeForLineSearch_-1)/2); i<kImageWidth_-((kWindowSizeForLineSearch_-1)/2); i++)
      {
        for (int k= -((kWindowSizeForLineSearch_-1)/2); k<=((kWindowSizeForLineSearch_-1)/2); k++)
        {

          if((int)image.at<uchar>(row,i+k) >= kLineThreshold_)
          {
            row_filter_activations[row].at(i) = 1;
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
    /*else {
      row_points.push_back(idx);
    }*/

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




bool LineClassification::CheckLineThickness(int thickness)
{
  if(thickness >= kMinLineThickness_ && thickness <= kMaxLineThickness_) { return true; }
  else { return false; }
}



