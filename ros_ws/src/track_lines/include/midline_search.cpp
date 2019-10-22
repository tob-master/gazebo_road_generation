

#include "midline_search.h"

MidLineSearch::MidLineSearch():
  kIntensityThreshold(240),
  kMaxCandidateIntensity(120),
  kSearchScalingFactor(1.5),
  kMidlineLength(30),

  kSearchRadius1((kMidlineLength * kSearchScalingFactor)/2),
  kSearchRadius2(((kMidlineLength * kSearchScalingFactor)/2) + 1)

{
  InitClusterScanners();
}



void MidLineSearch::CheckClusterConnections()
{

    auto end = --cluster_coordinates_hashmap.rend();


    for (auto it = cluster_coordinates_hashmap.rbegin(); it != end; ++it )
    {


      auto nx =  std::next(it,1);


      int y_start = it->second.begin()->first;
      int y_next_end = nx->second.rbegin()->first;

      int x_start = it->second.begin()->second;
      int x_next_end = nx->second.rbegin()->second;

      cout <<"xdist: " <<  x_start - x_next_end << endl;
      cout <<"ydist: " <<  y_start - y_next_end << endl;
      cout <<"pow: " << sqrt(pow(x_start - x_next_end,2) + pow(y_start - y_next_end,2)) << endl;

      /*
      cout <<"top " <<  it->second.begin()->first << " " << it->second.begin()->second << endl;


      cout <<"bottom " <<  nx->second.rbegin()->first << " " << nx->second.rbegin()->second << endl;

*/


      //cout << "next " << nx->second.begin()->first<< endl;






      /*
      for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2)
      {
        std::cout << it2->first << " " << it2->second << std::endl ;
      }
      */

    }
    cout << "_____________" << endl;

}



void MidLineSearch::SortClusters()
{

/*
  for (auto const& hash : cluster_coordinates_hashmap)
  {
    for (int i=0;i<(cluster_coordinates_hashmap.at(hash.first)).size();i++)
    {

      int y = ((cluster_coordinates_hashmap.at(hash.first)).at(i)).first;
      int x = ((cluster_coordinates_hashmap.at(hash.first)).at(i)).second;

      cout << x << " " << y << endl;

    }
  }

 for (auto const& hash : cluster_coordinates_hashmap)
  {
      //sort(hash.second.begin(),hash.second.end());

   vector<pair<int,int>> go(hash.second.begin(),hash.second.end());

    cout << "before" << endl;
   for (auto const& k : go) cout << k.first << " " << k.second << endl;

    sort(go.begin(),go.end());

    cout << "after" << endl;
    for (auto const& k : go) cout << k.first << " " << k.second << endl;

  }

cout << "after" << endl;

for (auto const& hash : cluster_coordinates_hashmap)
{
  for (int i=0;i<(cluster_coordinates_hashmap.at(hash.first)).size();i++)
  {

    int y = ((cluster_coordinates_hashmap.at(hash.first)).at(i)).first;
    int x = ((cluster_coordinates_hashmap.at(hash.first)).at(i)).second;

    cout << x << " " << y << endl;

  }
}
*/
}


void MidLineSearch::InitClusterScanners()
{
  float start_angle = PI;
  float end_angle   = start_angle+2*PI;

  for (float angle=start_angle; angle<end_angle; angle+=0.002)
  {
    float sin_ = sin(angle);
    float cos_ = cos(angle);

    int xC = cos_*kSearchRadius1+0.5;
    int yC = sin_*kSearchRadius1+0.5;
    int xC2 = cos_*kSearchRadius2+0.5;
    int yC2 = sin_*kSearchRadius2+0.5;

    if (!(std::find(scan.begin(), scan.end(), pair<int,int>{xC,yC}) != scan.end()))
    {
      scan.push_back({xC,yC});
    }

    if (!(std::find(rescan.begin(), rescan.end(), pair<int,int>{xC2,yC2}) != rescan.end()))
    {
      rescan.push_back({xC2,yC2});
    }

  }

}

void MidLineSearch::FindMidlinesInFrame(Mat image)
{

  candidates_count_hashmap.clear();
  candidates_xmass_hashmap.clear();
  candidates_ymass_hashmap.clear();
  cluster_coordinates_hashmap.clear();

  for (int i=kSearchRadius1; i<image.rows-kSearchRadius1; i++)
  {
    for (int j=kSearchRadius2; j<image.cols-kSearchRadius2; j++)
    {




      int M = (int)image.at<uchar>(Point(j,i));
      if (M > kIntensityThreshold)
      {
        bool is_candidate_scan = true;
        bool is_candidate_rescan = true;

        for (auto &it : scan)
        {
          int x = j + it.first;
          int y = i + it.second;
          if((int)image.at<uchar>(y,x) >= kMaxCandidateIntensity) is_candidate_scan = false;
        }
        for (auto &it : rescan)
        {
          int x = j + it.first;
          int y = i + it.second;
          if((int)image.at<uchar>(y,x) >= kMaxCandidateIntensity) is_candidate_rescan = false;
        }

        if(is_candidate_scan && is_candidate_rescan)
        {


          int i_key = i / (kSearchRadius2*2);
          int j_key = j / (kSearchRadius2*2);

          //cout << i_key << " " << j_key << endl;

          if ( candidates_count_hashmap.find(make_pair(i_key,j_key)) == candidates_count_hashmap.end() )
          {
              candidates_count_hashmap[make_pair(i_key,j_key)] = 1;

              candidates_xmass_hashmap[make_pair(i_key,j_key)] = i;
              candidates_ymass_hashmap[make_pair(i_key,j_key)] = j;


              cluster_coordinates_hashmap[make_pair(i_key,j_key)].push_back(make_pair(i,j));

              /*
              for (auto const& hash : candidates_count_hashmap)
              {
                  std::cout << "new: " << get<0>(hash.first) << " " << get<1>(hash.first) << " " << hash.second << std::endl ;
              }
              */
          }
          else
          {
            int count = candidates_count_hashmap.at(make_pair(i_key,j_key)) + 1;
            candidates_count_hashmap.at(make_pair(i_key,j_key)) = count;

            candidates_xmass_hashmap[make_pair(i_key,j_key)] += i;
            candidates_ymass_hashmap[make_pair(i_key,j_key)] += j;

            cluster_coordinates_hashmap[make_pair(i_key,j_key)].push_back(make_pair(i,j));

             /*
            for (auto const& hash : candidates_count_hashmap)
            {
                std::cout << "add: " << get<0>(hash.first)<< " " << get<1>(hash.first) << " " << hash.second << std::endl ;
            }
            */
          }

        }
     }






    }
  }
}


vector<pair<int,int>> MidLineSearch::GetMidlinePoints(int min_cluster_size)
{

  vector<pair<int,int>> cluster_coordinates;


  // cout << "AllSize "  << cluster_coordinates_hashmap.size() << " " << candidates_count_hashmap.size() << endl;

  for (auto const& hash : candidates_count_hashmap)
  {

      if(hash.second >= min_cluster_size)
      {

        int x = candidates_xmass_hashmap.at(hash.first) / hash.second;
        int y = candidates_ymass_hashmap.at(hash.first) / hash.second;




        cluster_coordinates.push_back(make_pair(x,y));


    //  cout << "OneSize "  << cluster_coordinates_hashmap.at(hash.first).size() << endl;
/*
        for (int i=0;i<(cluster_coordinates_hashmap.at(hash.first)).size();i++)
        {

            int y = ((cluster_coordinates_hashmap.at(hash.first)).at(i)).first;
            int x = ((cluster_coordinates_hashmap.at(hash.first)).at(i)).second;

     //       cout << x << " " << y << endl;
        }
*/
        //cout << xx << " " << yy << " " << candidates_xmass_hashmap.at(hash.first) << " " << candidates_ymass_hashmap.at(hash.first) << endl;

        //int i = get<0>(hash.first)*pointerLen2*2;
        //int j = get<1>(hash.first)*pointerLen2*2;
        //circle(rgb, Point(yy,xx), 10, Scalar(0, 255, 0));

      }
      else
      {
        cluster_coordinates_hashmap.erase(hash.first);
      }

  }
   //cout << "AfterSize "  << cluster_coordinates_hashmap.size() << " " << candidates_count_hashmap.size() << endl;

  return cluster_coordinates;

}
