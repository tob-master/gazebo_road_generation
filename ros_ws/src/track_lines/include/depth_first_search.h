#ifndef DEPTH_FIRST_SEARCH_H
#define DEPTH_FIRST_SEARCH_H


#include <list>
#include <iostream>
#include <vector>

//using namespace std;

// Graph class represents a undirected graph
// using adjacency list representation
class DepthFirstSearch
{
    int V;    // No. of vertices

    std::vector<int> connected_components;
    std::vector<std::vector<int>> graphs;

    // Pointer to an array containing adjacency lists
    std::list<int> *adj;

    // A function used by DFS
    void DFSUtil(int v, bool visited[]);

public:
    DepthFirstSearch(int V);   // Constructor
    void addEdge(int v, int w);
    std::vector<std::vector<int>> connectedComponents();
};



#endif // DEPTH_FIRST_SEARCH_H
