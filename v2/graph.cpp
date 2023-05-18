#include "graph.hpp"

Graph::Graph(int numVertices) : numVertices(numVertices) {
    adjacencyList.resize(numVertices);
}

void Graph::addEdge(int source, int destination, double weight) {
    adjacencyList[source].emplace_back(destination, weight);
}

std::vector<std::vector<std::pair<int, double>>> Graph::getAdjacencyList() const {
    return adjacencyList;
}

int Graph::getNumVertices() const {
    return numVertices;
}
