#include "graph.hpp"
#include "dijkstra.hpp"
#include <iostream>

int main() {
    // Create a graph
    Graph graph(5);

    // Add edges
    graph.addEdge(0, 1, 4);
    graph.addEdge(0, 2, 2);
    graph.addEdge(1, 2, 1);
    graph.addEdge(1, 3, 5);
    graph.addEdge(2, 3, 8);
    graph.addEdge(2, 4, 10);
    graph.addEdge(3, 4, 2);

    // Run Dijkstra's algorithm
    Dijkstra dijkstra(graph);
    std::vector<double> shortestDistances = dijkstra.shortestPath(0);

    // Print shortest distances
    std::cout << "Shortest distances from vertex 0:\n";
    for (int i = 0; i < shortestDistances.size(); ++i) {
        std::cout << "Vertex " << i << ": " << shortestDistances[i] << '\n';
    }

    return 0;
}
