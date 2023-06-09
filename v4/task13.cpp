#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <stack>
#include <string>
#include <sstream>
#include <fstream>
#include <omp.h>

#define INF std::numeric_limits<double>::infinity()
#define DIJKSTRAOPTION 1
// #define FILETYPE 0

struct Arc
{
    int start;
    int end;
    double cost;

    Arc(int start, int end, double cost) : start(start), end(end), cost(cost) {}
};

struct Path
{
    std::vector<Arc> arcs;
    double cost;

    Path() : cost(0.0) {}
    Path(const std::vector<Arc> &arcs, double cost) : arcs(arcs), cost(cost) {}

    double addArc(const Arc &arc)
    {
        arcs.push_back(arc);
        return cost += arc.cost;
    }
};

bool ComparePaths(const Path &p1, const Path &p2)
{
    double p1Cost = 0.0;
    for (const Arc &arc : p1.arcs)
        p1Cost += arc.cost;

    double p2Cost = 0.0;
    for (const Arc &arc : p2.arcs)
        p2Cost += arc.cost;

    return p1Cost > p2Cost;
}

class Graph
{
    int V; // number of vertices
    std::vector<std::vector<Arc>> adjList;

public:
    Graph(int V) : V(V), adjList(V) {}

    void addArc(int start, int end, double cost)
    {
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

    std::vector<double> regularDijkstra(int source, std::vector<int> &parent)
    {
        std::vector<double> dist(V, INF);
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                            std::greater<std::pair<double, int>>>
            pq;
        dist[source] = 0.0;
        pq.push(std::make_pair(0.0, source));

        while (!pq.empty())
        {
            int u = pq.top().second;
            double currDist = pq.top().first;
            pq.pop();

            if (currDist > dist[u])
            {
                continue; // Skip if a shorter path to u has already been found
            }

            for (const Arc &arc : adjList[u])
            {
                int v = arc.end;
                double cost = arc.cost;

                if (dist[u] + cost < dist[v])
                {
                    dist[v] = dist[u] + cost;
                    parent[v] = u; // Update parent to reconstruct the path
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

    void kShortestPaths(int source, int k, std::vector<int> &parent)
    {
        std::priority_queue<Path, std::vector<Path>, decltype(&ComparePaths)> pq(&ComparePaths);
        std::vector<bool> visited(std::vector<bool>(V, false));

        // Initialize with a path containing only the source vertex
        Path initialPath;
        initialPath.addArc(Arc(source, source, 0.0));
        pq.push(initialPath);

        int pathCount = -1;

        while (!pq.empty() && pathCount < k)
        {
            Path path = pq.top();
            pq.pop();

            int u = path.arcs.back().end;

            if (!visited[u])
            {
                visited[u] = true;
                pathCount++;
                printPath(path);
            }

            for (const Arc &arc : adjList[u])
            {
                Path newPath = path;
                newPath.addArc(arc);
                pq.push(newPath);
            }
        }
    }

    void printPath(const Path &path)
    {
        std::cout << "Path: ";
        for (const Arc &arc : path.arcs)
        {
            std::cout << arc.start << "->" << arc.end << " ";
        }
        std::cout << std::endl;
    }

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

void processInputFile(const std::string &filename, Graph &graph, int &nb_points)
{
    std::ifstream is(filename);
    std::string line;
    std::getline(is, line, '\n');
    std::stringstream ls;
    ls << line;
    int nb_arcs;
    ls >> nb_points >> nb_arcs;

    graph = Graph(nb_points); // Update the graph with the correct number of vertices

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

bool endPointsCheck(int source, int target, int nb_points)
{
    // Check if source and target vertices are valid
    if (source < 0 || source >= nb_points || target < 0 || target >= nb_points)
    {
        std::cout << "Invalid source or target vertex." << std::endl;
        return false;
    }

    return true;
}

bool pathCheck(int target, std::vector<double> &dist)
{
    // Check if a path exists between the source and target vertices
    if (dist[target] == INF)
    {
        std::cout << "There is no path between the source and target vertices." << std::endl;
        return false;
    }

    return true;
};

int main()
{
    int nb_points, nb_arcs;
    Graph graph(0);

    processInputFile("testtest.txt", graph, nb_points);

    int source, target;
    std::cout << "Enter the source vertex and target vertex: ";
    std::cin >> source >> target;

    if (endPointsCheck(source, target, nb_points) == false)
    {
        return -1;
    }

    std::vector<int> parent(nb_points, -1);
    std::vector<double> dist = graph.dijkstra(source, parent);

    if (pathCheck(target, dist) == false)
    {
        return -1;
    }

    std::cout << "Shortest distance from source to target: " << dist[target] << std::endl;

    graph.printShortestPath(parent, target);

    int k;
    std::cout << "Enter the value of k: ";
    std::cin >> k;

    graph.kShortestPaths(source, k, parent);

    return 0;
}