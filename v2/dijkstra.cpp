#include "dijkstra.hpp"
#include <queue>
#include <vector>
#include <limits>

struct Vertex {
    int id;
    double distance;

    Vertex(int id, double distance) : id(id), distance(distance) {}

    bool operator>(const Vertex& other) const {
        return distance > other.distance;
    }
};

Dijkstra::Dijkstra(const Graph& graph) : graph(graph) {}

std::vector<double> Dijkstra::shortestPath(int source) {
    const int numVertices = graph.getNumVertices();
    std::vector<double> distances(numVertices, std::numeric_limits<double>::infinity());
    distances[source] = 0.0;

    std::priority_queue<Vertex, std::vector<Vertex>, std::greater<Vertex>> pq;
    pq.emplace(source, 0.0);

    while (!pq.empty()) {
        Vertex current = pq.top();
        pq.pop();

        if (current.distance > distances[current.id]) {
            continue;
        }

        for (const auto& neighbor : graph.getAdjacencyList()[current.id]) {
            double newDistance = current.distance + neighbor.second;
            if (newDistance < distances[neighbor.first]) {
                distances[neighbor.first] = newDistance;
                pq.emplace(neighbor.first, newDistance);
            }
        }
    }

    return distances;
}
