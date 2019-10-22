#include "line_classification.h"



LineClassification::LineClassification():
  row0(380),
  row1(360),
  row2(340),
  kLineThicknessMin(3),
  kLineThicknessMax(9),
  kTrackWidthMin(117),
  kTrackWidthMax(143),
  kRowStride(5),
  kIntensityThreshold(200),
  kImgRows(417),
  kImgCols(1280),
  row_spikes(kImgCols,0),
  kMidlineSearchSpace(7),
  kMaxColumnDistanceForRowPoints(15)


{



}


vector<tuple<int,int,int,int,int,int>> LineClassification::SearchLineFeatures(Mat image)
{
  artefacts_count.emplace_back(row0,0);
  artefacts_count.emplace_back(row1,0);
  artefacts_count.emplace_back(row2,0);
  artefacts_info.clear();
   correct_features.clear();

        matched_pattern_coordinates.clear();


  for ( auto &row : line_search_regions )
  {

    fill(row_spikes.begin(), row_spikes.end(), 0);


    for (int i=((kRowStride-1)/2); i<kImgCols-((kRowStride-1)/2); i++)
    {
      for (int k= -((kRowStride-1)/2); k<=((kRowStride-1)/2); k++)
      {

        if((int)image.at<uchar>(row,i+k) >= kIntensityThreshold)
        {
          row_spikes.at(i) = 1;
        }

      }
    }

    int artefact_length=0;


    for (int i=0;i<kImgCols;i++)
    {
      if(row_spikes.at(i)==1)
      {

        int artefact_start_id = i;

        while(row_spikes.at(i) == 1 && i<kImgCols)
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

  if (artefacts_info.count(row1) >= 3)
  {

    correct_features_row0.clear();
    correct_features_row1.clear();
    correct_features_row2.clear();

    CheckThicknessAndDistancesPerRow(row0,artefacts_info,correct_features_row0);
    CheckThicknessAndDistancesPerRow(row1,artefacts_info,correct_features_row1);
    CheckThicknessAndDistancesPerRow(row2,artefacts_info,correct_features_row2);








     if(!correct_features_row0.empty() && !correct_features_row1.empty())
     {


       for(int i=0; i<correct_features_row0.size();i++)
       {
         for(int j=0; j<correct_features_row1.size();j++)
         {

           //int g = row0 - row1;

           int a1 = correct_features_row0.at(i).first - correct_features_row1.at(j).first;

           int a2 = correct_features_row0.at(i).second - correct_features_row1.at(j).second;


           if(abs(a1) < kMaxColumnDistanceForRowPoints && abs(a2) < kMaxColumnDistanceForRowPoints)
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

        for (int k= -((kMidlineSearchSpace-1)/2); k<=((kMidlineSearchSpace-1)/2); k++)
        {
            if((int)image.at<uchar>(row0,mid_row0+k)<= kIntensityThreshold) mid_pattern_row0 = true;
            if((int)image.at<uchar>(row1,mid_row1+k)>= kIntensityThreshold) mid_pattern_row1 = true;
            if((int)image.at<uchar>(row2,mid_row1+k)<= kIntensityThreshold) mid_pattern_row2 = true;
        }

         //cout << "row0: " << mid_pattern_row0 << " " << "row1: " << mid_pattern_row1 << " " << "row2: " << mid_pattern_row2 << endl;
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
          if(distance>=kTrackWidthMin && distance<=kTrackWidthMax)
          {
            correct_features.push_back(make_pair(row_points.at(i),row_points.at(j)));
          }
      }
    }

   // cout << "finished TP" << endl;
}




bool LineClassification::CheckLineThickness(int thickness)
{
  if(thickness >= kLineThicknessMin && thickness <= kLineThicknessMax) { return true; }
  else { return false; }
}



