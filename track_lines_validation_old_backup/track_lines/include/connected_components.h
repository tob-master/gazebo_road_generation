#ifndef CONNECTED_COMPONENTS_H
#define CONNECTED_COMPONENTS_H


#include <iostream>
#include <stdio.h>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "depth_first_search.h"
#include "utils.h"
#include "datatypes.h"
using namespace connected_components_search;

using namespace std;
using namespace cv;


class ConnectedComponentsSearch
{


private:


    enum {ROI_X_COLUMN, ROI_Y_COLUMN, ROI_WIDTH_COLUMN, ROI_HEIGHT_COLUMN, COMPONENT_SIZE_COLUMN};
    enum {CENTROIDS_X_COLUMN, CENTROIDS_Y_COLUMN};

    Mat current_image_;

    Mat labeled_image_;
    Mat components_stats_;
    Mat components_centroids_;

    const int kConnectionCount_;

    const int kMaxMidLineComponentSize_;
    const int kMinMidLineComponentSize_;

    const int kMaxMidLineComponentVolume_;
    const int kMinMidLineComponentVolume_;

    const float kMaxROICenterToCentroidDistance_;

    const int kMinMidLineComponentsDistance_;
    const int kMaxMidLineComponentsDistance_;

    const int kEndOfLinkageMarker_;

    const int kImageHeight_;
    const int kImageWidth_;
    const Size kImageSize_;
    const Point kCarPosition_;


    int components_count_;

    vector<ConnectedComponent> mid_line_components_;

    vector<string> used_permutations_;

    vector<vector<int>> grouped_mid_line_components_;

    vector<Point> cluster_centroids_;
    vector<vector<Point>> mid_line_cluster_groups_;

    vector<LengthAndDirectionFromConnectedComponents> connected_mid_line_clusters_length_and_direction_;
    vector<vector<LengthAndDirectionFromConnectedComponents>> grouped_connected_mid_line_clusters_length_and_direction_;


    bool has_found_mid_line_components_;
    bool has_found_mid_line_group_;

    void SetImage(Mat image);
    void ClearMemory();
    void ApplyConnectedComponents();
    void FilterMidLineComponents();
    void ComputeLengthAndDirectionOfConnectedMidLineComponents();
    void GroupMidLineComponents();
    bool IdAlreadyConnected(int id);
    vector<int> LinkInDistanceComponents();
    bool IsPermuted(int i, int j, vector<string> &used_permutations);

    double Distance2d(const Point& lhs, const Point& rhs);
    vector<pair<int,int>> FindClusterConnections(vector<pair<int,int>> &cluster_connections);

    bool HasFoundMidLineComponents();
    bool HasFoundGroup();

    ConnectedComponentsSearchReturnInfo GetReturnInfo();

public:
    ConnectedComponentsSearch(int image_height, int image_width, ConnectedComponentsSearchInitializationParameters init);
    ConnectedComponentsSearchReturnInfo FindConnectedComponents(Mat image);
    void DrawConnectedComponents(Mat &rgb);
    void DrawMidLineComponentsRect(Mat &rgb);
    void DrawGroupedMidLineComponents(Mat &rgb);
    void DrawGroupedMidLineComponentsDirections(Mat &rgb);
};

#endif // CONNECTED_COMPONENTS_H
