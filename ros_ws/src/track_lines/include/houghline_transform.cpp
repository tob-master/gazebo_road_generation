#include "houghline_transform.h"

HoughLineTransform::HoughLineTransform()
{

}

 void HoughLineTransform::FindVanashingPoint(Mat image)
 {
     current_image_ = image;
 }


void HoughLineTransform::ApplyCannyEdge()
{
    Canny(current_image_, canny_image_, kLowThreshold_, kHighThreshold_, kKernelSize_);
}

void HoughLineTransform::ShowCannyEdgeImage()
{
    imshow("Canny Imgae", canny_image_);
    waitKey(0);
}

void HoughLineTransform::ApplyHoughLines()
{

    cv::Rect myROI(0, 350, 1280, 67);

    hough_image_ = canny_image_(myROI);

    //hough_image_ = current_image_;

    HoughLinesP(hough_image_, found_lines_, kRho_, kTheta_, kMinIntersections, kMinLineLength, kMaxLineGap );
}

#define pdd pair<double, double>

pdd lineLineIntersection(pdd A, pdd B, pdd C, pdd D)
{
    // Line AB represented as a1x + b1y = c1
    double a1 = B.second - A.second;
    double b1 = A.first - B.first;
    double c1 = a1*(A.first) + b1*(A.second);

    // Line CD represented as a2x + b2y = c2
    double a2 = D.second - C.second;
    double b2 = C.first - D.first;
    double c2 = a2*(C.first)+ b2*(C.second);

    double determinant = a1*b2 - a2*b1;

    if (determinant == 0)
    {
        // The lines are parallel. This is simplified
        // by returning a pair of FLT_MAX
        return make_pair(FLT_MAX, FLT_MAX);
    }
    else
    {
        double x = (b2*c1 - b1*c2)/determinant;
        double y = (a1*c2 - a2*c1)/determinant;
        return make_pair(x, y);
    }
}



void HoughLineTransform::ComputeIntersections()
{






    if(found_lines_.size()>= 2)
    {

        for(int i=0; i<found_lines_.size()-1;i++)
        {
             pdd A = make_pair(found_lines_[i][0], found_lines_[i][1]);
             pdd B = make_pair(found_lines_[i][2], found_lines_[i][3]);
             pdd C = make_pair(found_lines_[i+1][0], found_lines_[i+1][1]);
             pdd D = make_pair(found_lines_[i+1][2], found_lines_[i+1][3]);

             pdd intersection = lineLineIntersection(A, B, C, D);


             if (intersection.first == FLT_MAX &&
                 intersection.second==FLT_MAX)
             {
                 cout << i << " The given lines AB and CD are parallel.\n";
             }

             else
             {
                 // NOTE: Further check can be applied in case
                 // of line segments. Here, we have considered AB
                 // and CD as lines
                 cout << "The intersection of the given lines AB "
                         "and CD is: ";
                 cout << i << "(" << intersection.first << ", " << intersection.second<< ")" << endl;
             }
        }
    }
}

void HoughLineTransform::ShowHoughLines()
{

    cv::cvtColor(hough_image_, hough_image_, CV_GRAY2BGR);


    int min_angle =  87;

    for( size_t i = 0; i < found_lines_.size(); i++ )
    {
      Vec4i l = found_lines_[i];


      line( hough_image_, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);




        /*
      if(l[2]-l[0] == 0)
      {
          cout << "90" << endl;
          line( hough_image_, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
      }
      else
      {
          float angle = abs(atan((l[3]-l[1])/(l[2]-l[0])) * 180/PI);
          if(angle > min_angle)
          {
            cout << angle << endl;
            line( hough_image_, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
          }
      }


*/

    }


    imshow("Hough Lines Imgae", hough_image_);
    waitKey(0);
}


