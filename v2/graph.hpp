#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>

class Graph {
public:
    Graph(int numVertices);
    void addEdge(int source, int destination, double weight);
    std::vector<std::vector<std::pair<int, double>>> getAdjacencyList() const;
    int getNumVertices() const;

private:
    std::vector<std::vector<std::pair<int, double>>> adjacencyList;
    int numVertices;
};

#endif  // GRAPH_HPP
