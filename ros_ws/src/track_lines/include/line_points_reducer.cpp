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
    left_line_points_.clear();
    right_line_points_.clear();
    left_line_points_reduced_.clear();
    right_line_points_reduced_.clear();

    left_line_points_reduced_length_direction_.clear();
    right_line_points_reduced_length_direction_.clear();
}

void LinePointsReducer::DrawReducedLinePoints(Mat &rgb)
{
    for(auto &it: left_line_points_reduced_)
    {
        circle(rgb, Point((int)it.first,(int)it.second), 7, Scalar(0, 0, 255));
    }

    for(auto &it: right_line_points_reduced_)
    {
        circle(rgb, Point((int)it.first,(int)it.second), 7, Scalar(0, 0, 255));
    }
}


void LinePointsReducer::ReduceLinePoints(vector<line_follower::PointAndDirection> left_line, vector<line_follower::PointAndDirection> right_line, double max_distance)
{

    ClearMemory();
    SetContainers(left_line,right_line);
    SetMaxDistance(max_distance);

    ApplyRamerDouglasPeucker(left_line_points_, max_distance_, left_line_points_reduced_);
    ApplyRamerDouglasPeucker(right_line_points_, max_distance_, right_line_points_reduced_);

    ComputeLengthAndDirectionFromConsecutiveReducedLinePoints();

}

void LinePointsReducer::GetReducedLinePoints(vector<ReducedPoints> &left_line_points_reduced, vector<ReducedPoints> &right_line_points_reduced)
{
    for(auto &it: left_line_points_reduced_)
    {
        left_line_points_reduced.push_back(ReducedPoints{int(it.first),int(it.second)});
    }

    for(auto &it: right_line_points_reduced_)
    {
        right_line_points_reduced.push_back(ReducedPoints{int(it.first),int(it.second)});
    }
}

void LinePointsReducer::GetLengthAndDirectionFromConsecutiveReducedLinePoints(vector<LengthAndDirectionFromConsecutiveReducedLinePoints> &left_line_points_reduced_length_direction,
                                                           vector<LengthAndDirectionFromConsecutiveReducedLinePoints> &right_line_points_reduced_length_direction)
{
    for(auto &it: left_line_points_reduced_length_direction_)
    {
        left_line_points_reduced_length_direction.push_back(it);
    }

    for(auto &it: right_line_points_reduced_length_direction_)
    {
        right_line_points_reduced_length_direction.push_back(it);
    }
}

void LinePointsReducer::ComputeLengthAndDirectionFromConsecutiveReducedLinePoints()
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

        float angle =  CalculateAngle4Quadrants(opposite, adjacent) * (PI/180);

        left_line_points_reduced_length_direction_.push_back(LengthAndDirectionFromConsecutiveReducedLinePoints{x,y,length, angle});
    }

    for(auto i=0; i<right_line_points_reduced_.size()-1; i++)
    {
        int x = right_line_points_reduced_[i].first;
        int y = right_line_points_reduced_[i].second;

        int x_next = right_line_points_reduced_[i+1].first;
        int y_next = right_line_points_reduced_[i+1].second;

        int opposite =  y - y_next;
        int adjacent =  x_next - x;

        int length = sqrt(pow(adjacent,2)+pow(opposite,2));

        float angle =  CalculateAngle4Quadrants(opposite, adjacent) * (PI/180);

        right_line_points_reduced_length_direction_.push_back(LengthAndDirectionFromConsecutiveReducedLinePoints{x,y,length, angle});
    }

}


void LinePointsReducer::CoutLengthAndDirectionFromConsecutiveReducedLinePoints()
{
    cout << "___LengthAndDirectionFromConsecutiveReducedLinePoints___" << endl;
    for (auto &it : left_line_points_reduced_length_direction_)
    {
        cout <<"left: ("<< it.x << "," << it.y << ") " << it.angle * (180/PI) << "° " << it.length <<"px" << endl;
    }


    for (auto &it : right_line_points_reduced_length_direction_)
    {
        cout <<"right: ("<< it.x << "," << it.y << ") " << it.angle * (180/PI) << "° " << it.length <<"px" << endl;
    }
    cout << "______" << endl;
}
