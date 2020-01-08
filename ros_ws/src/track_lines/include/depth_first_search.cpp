#include "depth_first_search.h"

// Method to print connected components in an
// undirected graph
std::vector<std::vector<int>> DepthFirstSearch::connectedComponents()
{
    // Mark all the vertices as not visited
    bool *visited = new bool[V];
    for(int v = 0; v < V; v++)
        visited[v] = false;

    for (int v=0; v<V; v++)
    {
        if (visited[v] == false)
        {
            // print all reachable vertices
            // from v
            DFSUtil(v, visited);


            graphs.push_back(connected_components);
            connected_components.clear();
            //cout << "\n";
        }

    }
    return graphs;
}

void DepthFirstSearch::DFSUtil(int v, bool visited[])
{
    // Mark the current node as visited and print it
    visited[v] = true;
    //cout << v << " ";
    connected_components.push_back(v);
    // Recur for all the vertices
    // adjacent to this vertex
    std::list<int>::iterator i;
    for(i = adj[v].begin(); i != adj[v].end(); ++i)
        if(!visited[*i])
            DFSUtil(*i, visited);
}

DepthFirstSearch::DepthFirstSearch(int V)
{
    this->V = V;
    adj = new std::list<int>[V];
}

// method to add an undirected edge
void DepthFirstSearch::addEdge(int v, int w)
{
    adj[v].push_back(w);
    adj[w].push_back(v);
}
