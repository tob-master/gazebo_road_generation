#include "perceptual_grouping.h"

PerceptualGrouping::PerceptualGrouping()
{

}


void PerceptualGrouping::SetImage(Mat image)
{
    current_image_ = image;
}



void PerceptualGrouping::ApplyHoughLines()
{
    HoughLinesP(canny_image_, hough_lines_, kHoghLinesRho_, kHoughLinesTheta_, kHoughLinesMinintersections_, kHoughLinesMinLineLength_, kHoughLinesMaxLineGap_ );

    //cout << hough_lines_.size() << endl;

}

void PerceptualGrouping::ClearMemory()
{
    hough_lines_.clear();
    hough_mid_points_.clear();
    hough_line_lengths_.clear();
    hough_line_proximities_.clear();
    hough_line_proximity_thresholed_.clear();
    used_permutations_.clear();
}

void PerceptualGrouping::ApplyConnectedComponents()
{
    components_count_ = connectedComponentsWithStats(current_image_, labeled_image_, components_stats_, components_centroids_, kConnectionCount_, CV_32S);
}

void PerceptualGrouping::ApplyCannyEdge()
{
    Canny(current_image_, canny_image_, kCannyLowThreshold_, kCannyHighThreshold_, kCannyKernelSize_);
}


void PerceptualGrouping::ComputeGroupingParameters()
{
ApplyConnectedComponents();
ApplyCannyEdge();
ApplyHoughLines();
ChangeLinePointsToDriveDirection();
ComputeHoughLineMidPoints();
ComputeHoughLineLengths();

ComputeHoughLineProximity();
}

void PerceptualGrouping::ComputeHoughLineMidPoints()
{
    for(auto it : hough_lines_)
    {
        int x1 = it[0];
        int y1 = it[1];

        int x2 = it[2];
        int y2 = it[3];

        int x_mid = int(float(x1+x2) / 2);
        int y_mid = int(float(y1+y2) / 2);

        hough_mid_points_.push_back(Point(x_mid,y_mid));
    }
}

void PerceptualGrouping::ComputeHoughLineLengths()
{
    for(auto it : hough_lines_)
    {
        int x1 = it[0];
        int y1 = it[1];

        int x2 = it[2];
        int y2 = it[3];

        float length = sqrt(pow(x1+x2,2) + pow(y1+y2,2));

        hough_line_lengths_.push_back(length);

    }
}


bool PerceptualGrouping::IsPermuted(int i, int j, vector<string> &used_permutations)
{
    if(i==j){ return true;}

    string permutation_hash12 = to_string(i) + to_string(j);

    auto it = find (used_permutations.begin(), used_permutations.end(), permutation_hash12);

    if (it != used_permutations.end())
    {
        return true;
    }
    else
    {
      string permutation_hash21 = std::to_string(j) + std::to_string(i);
      used_permutations.push_back(permutation_hash12);
      used_permutations.push_back(permutation_hash21);
      return false;
    }
}


void PerceptualGrouping::ComputeHoughLineProximity()
{



    for (int i=0;i<hough_lines_.size(); i++)
    {

        for (int j=0;j<hough_lines_.size(); j++)
        {

            if(IsPermuted(i,j,used_permutations_)) continue;

            float l1 = hough_line_lengths_[i];
            float l2 = hough_line_lengths_[j];

            Point end1 = Point(hough_lines_[i][2],hough_lines_[i][3]);
            Point start1 = Point(hough_lines_[i][0],hough_lines_[i][1]);

            Point end2 = Point(hough_lines_[j][2],hough_lines_[j][3]);
            Point start2 = Point(hough_lines_[j][0],hough_lines_[j][1]);

            float g = sqrt(pow(end1.x+start2.x,2) + pow(end1.y+start2.y,2));

            float proximity = 0;

            if(l1>l2)
            {
                proximity = pow((l2 / g),2);
            }
            else if(l1<l2)
            {

                proximity = pow((l1 / g),2);
            }
            else
            {
                proximity = pow((l2 / g),2);
            }


           // cout << "prox: " << proximity << endl;

            //if(proximity < 0.05)
            hough_line_proximity_thresholed_.push_back(make_pair(i,j));


            hough_line_proximities_.push_back(proximity);

        }

    }






}


void PerceptualGrouping::DrawHoughLineProximityThresholded(Mat &rgb)
{
    /*
    for(auto it : hough_line_proximity_thresholed_)
    {
        int id1 = it.first;
        int id2 = it.second;

        int x1_first = hough_lines_[id1][0];
        int y1_first = hough_lines_[id1][1];
        int x2_first = hough_lines_[id1][2];
        int y2_first = hough_lines_[id1][3];

        int x1_second = hough_lines_[id2][0];
        int y1_second = hough_lines_[id2][1];
        int x2_second = hough_lines_[id2][2];
        int y2_second = hough_lines_[id2][3];

         line( rgb,Point(x1_first,y1_first), Point(x2_first,y2_first), Scalar(0,0,255), 1, CV_AA);
         line( rgb,Point(x1_second,y1_second), Point(x2_second,y2_second), Scalar(0,255,0), 1, CV_AA);
    }
    */
    int id1 = hough_line_proximity_thresholed_[0].first;
    int id2 = hough_line_proximity_thresholed_[0].second;

    cout << "id " << id1 << " " << id2 << endl;

    float l1 = hough_line_lengths_[id1];
    float l2 = hough_line_lengths_[id2];

    if(l1>l2)
    {   cout << "l1>l2" << " " << l1 << " " << l2 << endl;

    }
    else if(l1<l2)
    {
        cout << "l1<l2" << " " << l1 << " " << l2 << endl;

    }
    else
    {   cout << "same" << " " << l1 << " " << l2 << endl;

    }

    cout << "proxi: " <<  hough_line_proximities_[0] << endl;

    int x1_first = hough_lines_[id1][0];
    int y1_first = hough_lines_[id1][1];
    int x2_first = hough_lines_[id1][2];
    int y2_first = hough_lines_[id1][3];

    int x1_second = hough_lines_[id2][0];
    int y1_second = hough_lines_[id2][1];
    int x2_second = hough_lines_[id2][2];
    int y2_second = hough_lines_[id2][3];

     line( rgb,Point(x1_first,y1_first), Point(x2_first,y2_first), Scalar(0,0,255), 8, CV_AA);
     line( rgb,Point(x1_second,y1_second), Point(x2_second,y2_second), Scalar(0,255,0), 3, CV_AA);

}

void PerceptualGrouping::DrawHoughLineMidPoints(Mat &rgb)
{

    for(auto it : hough_mid_points_)
    {
         circle(rgb, it, 3, Scalar(0,255,0),CV_FILLED);
    }

}

void PerceptualGrouping::ChangeLinePointsToDriveDirection()
{
    for( size_t i = 0; i < hough_lines_.size(); i++ )
    {
      int x1 = hough_lines_[i][0];
      int y1 = hough_lines_[i][1];
      int x2 = hough_lines_[i][2];
      int y2 = hough_lines_[i][3];

      if(y2 > y1)
      {
          int tmp;
          tmp = x1;
          x1  = x2;
          x2  = tmp;
          tmp = y1;
          y1  = y2;
          y2 = tmp;


          hough_lines_[i][0] = x1;
          hough_lines_[i][1] = y1;
          hough_lines_[i][2] = x2;
          hough_lines_[i][3] = y2;
      }

    }
}

void PerceptualGrouping::DrawHoughLines(Mat &rgb)
{
    for(auto it : hough_lines_)
    {
        int x1 = it[0];
        int y1 = it[1];

        int x2 = it[2];
        int y2 = it[3];

        line( rgb,Point(x1,y1), Point(x2,y2), Scalar(0,0,255), 1, CV_AA);
    }
}
