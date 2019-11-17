#include "line_points_reducer.h"

LinePointsReducer::LinePointsReducer()
{

}

double LinePointsReducer::GetPerpendicularDistance(const RamerDouglasPeuckerTypePoint &pt, const RamerDouglasPeuckerTypePoint &lineStart, const RamerDouglasPeuckerTypePoint &lineEnd)
{
    double dx = lineEnd.first - lineStart.first;
    double dy = lineEnd.second - lineStart.second;

    //Normalise
    double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
    if(mag > 0.0)
    {
        dx /= mag; dy /= mag;
    }

    double pvx = pt.first - lineStart.first;
    double pvy = pt.second - lineStart.second;

    //Get dot product (project pv onto normalized direction)
    double pvdot = dx * pvx + dy * pvy;

    //Scale line direction vector
    double dsx = pvdot * dx;
    double dsy = pvdot * dy;

    //Subtract this from pv
    double ax = pvx - dsx;
    double ay = pvy - dsy;

    return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}

void LinePointsReducer::ApplyRamerDouglasPeucker(const vector<RamerDouglasPeuckerTypePoint> &pointList, double epsilon, vector<RamerDouglasPeuckerTypePoint> &out)
{
    if(pointList.size()<2)
        throw invalid_argument("Not enough points to simplify");

    // Find the point with the maximum distance from line between start and end
    double dmax = 0.0;
    size_t index = 0;
    size_t end = pointList.size()-1;
    for(size_t i = 1; i < end; i++)
    {
        double d = GetPerpendicularDistance(pointList[i], pointList[0], pointList[end]);
        if (d > dmax)
        {
            index = i;
            dmax = d;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if(dmax > epsilon)
    {
        // Recursive call
        vector<RamerDouglasPeuckerTypePoint> recResults1;
        vector<RamerDouglasPeuckerTypePoint> recResults2;
        vector<RamerDouglasPeuckerTypePoint> firstLine(pointList.begin(), pointList.begin()+index+1);
        vector<RamerDouglasPeuckerTypePoint> lastLine(pointList.begin()+index, pointList.end());
        ApplyRamerDouglasPeucker(firstLine, epsilon, recResults1);
        ApplyRamerDouglasPeucker(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end()-1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
        if(out.size()<2)
            throw runtime_error("Problem assembling output");
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList[0]);
        out.push_back(pointList[end]);
    }
}

void LinePointsReducer::SetMaxDistance(double max_distance)
{
    max_distance_ = max_distance;
}

void LinePointsReducer::SetContainers(vector<line_follower::PointAndDirection>left_line, vector<line_follower::PointAndDirection>right_line)
{
    for(auto &it: left_line)
    {
        left_line_points_.push_back(RamerDouglasPeuckerTypePoint(double(it.x),double(it.y)));
    }

    for(auto &it: right_line)
    {
        right_line_points_.push_back(RamerDouglasPeuckerTypePoint(double(it.x),double(it.y)));
    }
}

void LinePointsReducer::ClearMemory()
{

    left_line_is_reduced_ = false;
    right_line_is_reduced_ = false;

    left_line_points_.clear();
    right_line_points_.clear();
    left_line_points_reduced_.clear();
    right_line_points_reduced_.clear();

    left_line_points_reduced_length_direction_.clear();
    right_line_points_reduced_length_direction_.clear();
}

void LinePointsReducer::DrawReducedLinePoints(Mat &rgb, int line)
{
    if(line == LEFT_LINE)
    {
        for(auto &it: left_line_points_reduced_)
        {
            circle(rgb, Point((int)it.first,(int)it.second), 7, Scalar(0, 0, 255));
        }
    }

    if(line == RIGHT_LINE)
    {
        for(auto &it: right_line_points_reduced_)
        {
            circle(rgb, Point((int)it.first,(int)it.second), 7, Scalar(0, 0, 255));
        }
    }
}


LinePointsReducerReturnInfo LinePointsReducer::GetReturnInfo()
{
    return LinePointsReducerReturnInfo{left_line_is_reduced_,
                                       left_line_points_reduced_.size(),
                                       right_line_is_reduced_,
                                       right_line_points_reduced_.size()};
}

LinePointsReducerReturnInfo LinePointsReducer::ReduceLinePoints(vector<line_follower::PointAndDirection> left_line, vector<line_follower::PointAndDirection> right_line, double max_distance)
{

    ClearMemory();
    SetContainers(left_line,right_line);
    SetMaxDistance(max_distance);



    if(left_line_points_.size() >= 2)
    {
        ApplyRamerDouglasPeucker(left_line_points_, max_distance_, left_line_points_reduced_);
        ComputeLengthAndDirectionFromConsecutiveReducedLinePoints(LEFT_LINE);
        left_line_is_reduced_ = true;
    }
    else
    {
       left_line_is_reduced_ = false;
    }

    if(right_line_points_.size() >= 2)
    {
         ApplyRamerDouglasPeucker(right_line_points_, max_distance_, right_line_points_reduced_);
         ComputeLengthAndDirectionFromConsecutiveReducedLinePoints(RIGHT_LINE);
         right_line_is_reduced_ = true;
    }
    else
    {
       right_line_is_reduced_ = false;
    }


    return GetReturnInfo();

}

void LinePointsReducer::GetReducedLinePoints(vector<ReducedPoints> &line_points_reduced, int line)
{
    if(line == LEFT_LINE)
    {
        for(auto &it: left_line_points_reduced_)
        {
            line_points_reduced.push_back(ReducedPoints{int(it.first),int(it.second)});
        }
    }

    if(line == RIGHT_LINE)
    {
        for(auto &it: right_line_points_reduced_)
        {
            line_points_reduced.push_back(ReducedPoints{int(it.first),int(it.second)});
        }
    }
}

void LinePointsReducer::GetLengthAndDirectionFromConsecutiveReducedLinePoints(vector<PointInDirection> &line_points_reduced_length_direction, int line)
{
    if(line == LEFT_LINE)
    {
        for(auto &it: left_line_points_reduced_length_direction_)
        {
            line_points_reduced_length_direction.push_back(it);
        }
    }

    if(line == RIGHT_LINE)
    {
        for(auto &it: right_line_points_reduced_length_direction_)
        {
            line_points_reduced_length_direction.push_back(it);
        }
    }
}

void LinePointsReducer::ComputeLengthAndDirectionFromConsecutiveReducedLinePoints(int line)
{

    if(line == LEFT_LINE)
    {
        for(auto i=0; i<left_line_points_reduced_.size()-1; i++)
        {
            int x = left_line_points_reduced_[i].first;
            int y = left_line_points_reduced_[i].second;

            int x_next = left_line_points_reduced_[i+1].first;
            int y_next = left_line_points_reduced_[i+1].second;

            int opposite =  y - y_next;
            int adjacent =  x_next - x;

            int length = sqrt(pow(adjacent,2)+pow(opposite,2));

            float angle =  CalculateAngle4Quadrants(opposite, adjacent) ;

            left_line_points_reduced_length_direction_.push_back(PointInDirection{x,y,length, angle});
        }
    }

    if(line == RIGHT_LINE)
    {
        for(auto i=0; i<right_line_points_reduced_.size()-1; i++)
        {
            int x = right_line_points_reduced_[i].first;
            int y = right_line_points_reduced_[i].second;

            int x_next = right_line_points_reduced_[i+1].first;
            int y_next = right_line_points_reduced_[i+1].second;

            int opposite =  y - y_next;
            int adjacent =  x_next - x;

            int length = sqrt(pow(adjacent,2)+pow(opposite,2));

            float angle =  CalculateAngle4Quadrants(opposite, adjacent) ;

            right_line_points_reduced_length_direction_.push_back(PointInDirection{x,y,length, angle});
        }
    }
}


void LinePointsReducer::CoutLengthAndDirectionFromConsecutiveReducedLinePoints()
{
    cout << "___LinePointsReducer LengthAndDirectionFromConsecutiveReducedLinePoints___" << endl;
    for (auto &it : left_line_points_reduced_length_direction_)
    {
        cout <<"left: \tPoint("<< it.x << "," << it.y << ") \tDirection: " << it.angle << "° \tLength: " << it.length <<"px" << endl;
    }


    for (auto &it : right_line_points_reduced_length_direction_)
    {
        cout <<"right: \tPoint("<< it.x << "," << it.y << ") \tDirection: " << it.angle  << "° \tLength: " << it.length <<"px" << endl;
    }
    cout << "#######################################" << endl;
}
