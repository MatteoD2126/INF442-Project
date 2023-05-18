#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include "graph.hpp"
#include <vector>

class Dijkstra {
public:
    Dijkstra(const Graph& graph);
    std::vector<double> shortestPath(int source);

private:
    const Graph& graph;
};

#endif  // DIJKSTRA_HPP
