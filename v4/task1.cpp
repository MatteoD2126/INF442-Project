#include <iostream>
#include <vector>
#include <queue> // for priority queue
#include <limits>
#include <stack>
#include <string>
// for input form the file
#include <sstream>
#include <fstream>
#include <omp.h>

#define INF std::numeric_limits<double>::infinity()
// two chose a sequential or a parallelized version
#define DIJKSTRAOPTION 1
#define FILETYPE 1

// the structure for edges in the graph
struct Arc
{
    int start;
    int end;
    double cost;

    // constructor
    Arc(int start, int end, double cost) : start(start), end(end), cost(cost) {}
};

// class describing the structure of graph
class Graph
{
    int V;                                 // number of vertices
    std::vector<std::vector<Arc>> adjList; // adjacency list to describe a set of neighbours of vertices in the graph

public:
    // constructor
    Graph(int V) : V(V), adjList(V) {}

    void addArc(int start, int end, double cost)
    {
        // this line means:"vertex start is associated to vertex end within the edge of weiht cost"
        adjList[start].push_back(Arc(start, end, cost));
    }

    std::vector<double> dijkstra(int source, std::vector<int> &parent)
    {
        if (DIJKSTRAOPTION == 0)
        {
            return regularDijkstra(source, parent);
        }
        else
            return parallelDijkstra(source, parent);
    }

    // returns the vector of distances of the shortest paths for all vertices
    std::vector<double> regularDijkstra(int source, std::vector<int> &parent)
    {
        // std::vector<int> parent(V, -1);
        std::vector<double> dist(V, INF);
        // build the priority queue to choose the closest vertex
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                            std::greater<std::pair<double, int>>>
            pq;
        dist[source] = 0.0;
        pq.push(std::make_pair(0.0, source));

        while (!pq.empty())
        {
            // top pair readout
            int u = pq.top().second;
            double currDist = pq.top().first;
            pq.pop();

            if (currDist > dist[u])
            {
                continue; // skip if a shorter path to u has already been found
            }

            // going through all the edges starting from u
            for (const Arc &arc : adjList[u])
            {
                int v = arc.end;
                double cost = arc.cost;

                if (dist[u] + cost < dist[v])
                {
                    dist[v] = dist[u] + cost;
                    parent[v] = u; // update parent to reconstruct the path
                    pq.push(std::make_pair(dist[v], v));
                }
            }
        }

        std::cout << "I've just executed the regular version of Dijkstra" << std::endl;

        return dist;
    }

    std::vector<double> parallelDijkstra(int source, std::vector<int> &parent)
    {
        std::vector<double> dist(V, INF);
        std::vector<bool> visited(V, false);

        dist[source] = 0.0;
        parent[source] = -1;

        for (int i = 0; i < V - 1; ++i)
        {
            int u = -1;
            double minDist = INF;

// Find the vertex with the minimum distance among the unvisited vertices
#pragma omp parallel for
            for (int j = 0; j < V; ++j)
            {
                if (!visited[j] && dist[j] < minDist)
                {
                    u = j;
                    minDist = dist[j];
                }
            }

            if (u == -1)
                break;

            visited[u] = true;

// Update the distances of the neighboring vertices in parallel
#pragma omp parallel for
            for (const Arc &arc : adjList[u])
            {
                int v = arc.end;
                double cost = arc.cost;

                if (!visited[v] && dist[u] + cost < dist[v])
                {
                    dist[v] = dist[u] + cost;
                    parent[v] = u;
                }
            }
        }

        std::cout << "I've just executed the parallel version of Dijkstra" << std::endl;

        return dist;
    }

    // the function that prints the shortes path to a given vertex "target"
    void printShortestPath(const std::vector<int> &parent, int target)
    {
        std::stack<int> path;
        int curr = target;

        while (curr != -1)
        {
            path.push(curr);
            curr = parent[curr];
        }

        std::cout << "Shortest path from source to target: ";

        while (!path.empty())
        {
            std::cout << path.top() << " ";
            path.pop();
        }

        std::cout << std::endl;
    }
};

void processInputFile(const std::string &filename, Graph &graph)
{
    std::ifstream is(filename);
    std::string line;
    
    

    if (FILETYPE == 0)
    {
        int nb_points, nb_arcs;
        std::getline(is, line, '\n');
        std::stringstream ls;
        ls << line;
        ls >> nb_points >> nb_arcs;
        int v1, v2;
        double weight;
        int res;
        int v1, v2;
        double weight;
        int res;
        for (int j = 0; j < nb_arcs; j++)
        {
            std::getline(is, line, '\n');
            ls.str(line);
            ls.clear();
            ls >> v1 >> v2 >> weight >> res;
            graph.addArc(v1, v2, weight);
        }
    }
    else {
        int nb_points, nb_arcs, nb_res;
        std::getline(is, line, '\n');
        std::stringstream ls;
        ls << line;
        ls >> nb_points >> nb_arcs >> nb_res;

        // skip some lines
        // TODO
        // skip the lower limit on the resources consumed on the chosen path
        for (int i = 0; i < nb_res; i++){
            std::getline(is, line, '\n');
        }
        // skip the upper limit on the resources consumed on the chosen path
        for (int i = 0; i < nb_res; i++){
            std::getline(is, line, '\n');
        }
        // skip the amount of each resource k (k=1,...,K) consumed in passing through each vertex
        for (int i = 0; i < nb_points * nb_res; i++){
            std::getline(is, line, '\n');
        }
        
        int v1, v2, res;
        double weight;
        for (int j = 0; j < nb_arcs; j++)
        {
            std::getline(is, line, '\n');
            ls.str(line);
            ls.clear();
            ls >> v1 >> v2 >> weight >> res;
            graph.addArc(v1, v2, weight);
        }
    }
}


int main()
{
    int nb_points, nb_arcs;
    Graph graph(nb_points);
    // processInputFile("testtest.txt", graph);
    processInputFile("rcsp1.txt", graph);

    int source, target;
    std::cout << "Enter the source vertex and target vertex: ";
    std::cin >> source >> target;

    std::vector<int> parent(nb_points, -1);
    std::vector<double> dist = graph.dijkstra(source, parent);
    // std::vector<double> dist = graph.dijkstra(source);

    std::cout << "Shortest distance from source to target: " << dist[target] << std::endl;

    graph.printShortestPath(parent, target);

    return 0;
}