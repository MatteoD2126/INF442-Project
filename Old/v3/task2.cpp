#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <stack>
#include <string>
#include <sstream>
#include <fstream>
#include <functional> // Include this header for std::function


#define INF std::numeric_limits<double>::infinity()

struct Arc {
    int start;
    int end;
    double cost;

    Arc(int start, int end, double cost) : start(start), end(end), cost(cost) {}
};

class Graph {
    int V; // number of vertices
    std::vector<std::vector<Arc> > adjList;

public:
    Graph(int V) : V(V), adjList(V) {}

    void addArc(int start, int end, double cost) {
        adjList[start].push_back(Arc(start, end, cost));
    }

    std::vector<double> dijkstra(int source, std::vector<int>& parent, double (*delayFunc)(double), double bound) {
        std::vector<double> dist(V, INF);
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int> >,
                            std::greater<std::pair<double, int> > > pq;
        dist[source] = 0.0;
        pq.push(std::make_pair(0.0, source));

        while (!pq.empty()) {
            int u = pq.top().second;
            double currDist = pq.top().first;
            pq.pop();

            if (currDist > dist[u]) {
                continue; // Skip if a shorter path to u has already been found
            }

            for (const Arc& arc : adjList[u]) {
                int v = arc.end;
                double cost = arc.cost;
                double delay = delayFunc(cost);

                if (dist[u] + delay < dist[v] && dist[u] + cost <= bound) {
                    dist[v] = dist[u] + delay;
                    parent[v] = u; // Update parent to reconstruct the path
                    pq.push(std::make_pair(dist[v], v));
                }
            }
        }

        return dist;
    }

    void printShortestPath(std::vector<int>& parent, int target) {
        std::stack<int> path;
        int curr = target;

        while (curr != -1) {
            path.push(curr);
            curr = parent[curr];
        }

        std::cout << "Shortest path from source to target: ";

        while (!path.empty()) {
            std::cout << path.top() << " ";
            path.pop();
        }

        std::cout << std::endl;
    }
};

int main() {
    int nb_points, nb_arcs;

    std::ifstream is("testtest.txt");
    std::string line;
    std::getline(is, line, '\n');
    std::stringstream ls;
    ls << line;
    ls >> nb_points >> nb_arcs;

    Graph graph(nb_points);

    int v1, v2;
    double weight;
    for (int j = 0; j < nb_arcs; j++) {
        std::getline(is, line, '\n');
        ls << line;
        ls >> v1 >> v2 >> weight;
        graph.addArc(v1, v2, weight);
        ls.clear();
    }

    int source, target;
    double bound;
    std::cout << "Enter the source vertex, target vertex, and bound: ";
    std::cin >> source >> target >> bound;

    std::vector<int> parent(nb_points, -1);
    std::vector<double> dist = graph.dijkstra(source, parent, [](double cost) { return cost * cost; }, bound);

    std::cout << "Shortest distance from source to target: " << dist[target] << std::endl;
    graph.printShortestPath(parent, target);

    return 0;
}
