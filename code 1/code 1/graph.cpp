#include "graph.hpp" // This is the header for the class implemented here

#include "cloud.hpp" // Used in the constructor
#include "edge.hpp"  // Used in almost all methods

#include <algorithm> // This provides the sort() method for the constructor
#include <cassert>
#include <iostream>
#include <sstream>
#include <limits>
#include <vector>                   // For vector container
#include <queue>                    // For priority_queue container
#include <functional>               // For std::greater



/* graph -- method implementations */

graph::graph(const cloud &_c, int _size) {
    n = _c.get_n(); // the number of vertices(nodes).
    //size = n * (n - 1) / 2; // the number of edges.
    size = _size;
    // TODO: Exercise 2
    node_names = new std::string[n];
    /*for (int i = 0; i < n; i++){
        node_names[i] = _c.get_point(i).name;
    }*/
    edges = new edge*[size];
    //create an array
    int count = 0;
    for (int i = 0; i < n; i++){
        for (int j = 0; j < i; j++){
            edges[count] = new edge(i, j, _c.get_point(i).dist(_c.get_point(j)));
            count++;
        }
    }
    //std::sort(edges, edges + size, edge::compare); // requires #include <algorithm>
    iterator_pos = 0;
}

graph::graph(long _n, const std::string _node_names[], double **dist_matrix) {
    n = _n; // the number of vertices(nodes).
    size = n * (n - 1) / 2; // the number of edges.
    // TODO: Exercise 2
    node_names = new std::string[n];
    for (int i = 0; i < n; i++){
        node_names[i] = _node_names[i];
    }
    edges = new edge*[size];
    int count = 0;
    for (int i = 0; i < n; i++){
        for (int j = i + 1; j < n; j++){
            edges[count] = new edge(i, j, dist_matrix[i][j]);
            count++;
        }
    }
    std::sort(edges, edges + size, edge::compare); // requires #include <algorithm>
    iterator_pos = 0;
}

graph::~graph() {
    // TODO: Exercise 2
    for (int i = 0; i < size; i++){
        delete edges[i];
    }//done
    
    delete[] edges;
    delete[] node_names;
}

long graph::get_num_edges() const {
    // TODO: Exercise 2

    return size; //done
   // return 0;
}

std::string &graph::get_name(int i) const {
    assert(i >= 0 && i < n);
    return node_names[i];
}

edge *graph::get_edge(long i) {
    return edges[i];
}

edge **graph::get_edges() {
    return edges;
}

long graph::get_num_nodes() const {
    return n;
}

void graph::start_iteration() {
    // TODO: Exercise 3
    iterator_pos = 0;
}

edge *graph::get_next() {
    // TODO: Exercise 3
    long temp = iterator_pos;
    if (iterator_pos + 1 == size)
        return NULL;
    else{
        iterator_pos++;
        return edges[temp];
    }
}

graph *graph::load_matrix(std::ifstream &is) {
    assert(is.is_open());

    std::string buffer;
    getline(is, buffer, '\n');
    int n = std::stoi(buffer);
    std::string node_names[n];
    for (size_t i = 0; i < n; ++i) {
	getline(is, node_names[i], '\n');
    }

    std::cout << "Names read" << std::endl;
    double **dist_matrix = new double*[n];
    for (int i = 0; i < n; ++i) {
        dist_matrix[i] = new double[n];
	std::getline(is, buffer, '\n');
        std::stringstream ls;
	ls << buffer;
	std::string dist_str;
	for (int j = 0; j < n; ++j) {
	    std::getline(ls, dist_str, ',');
	    dist_matrix[i][j] = std::stod(dist_str);
	}
    }

    graph *g = new graph(n, node_names, dist_matrix);
    for (size_t i = 0; i < n; ++i) {
        delete[] dist_matrix[i];
    }
    delete[] dist_matrix;

    return g;
}

void graph::load(std::ifstream &is) {
    assert(is.is_open());
    
    // Read the first line containing the number of vertices, arcs, and resources
    std::string line;
    std::getline(is, line, '\n');
    std::stringstream ls(line);
    
    int numResources;
    // Read the number of vertices, arcs, and resources
    ls >> n >> size >> numResources;
    
    // Read/Skip the lower and upper limits on resources consumed on the chosen path
    std::getline(is, line, '\n');
    std::getline(is, line, '\n');
    
    // Read/Skip the amount of each resource consumed in passing through each vertex
    for (int i = 0; i < n; i++){
        for (int k = 0; k < numResources; k++){
            std::getline(is, line, '\n');
        }
    }
    
    // Resize the edges array to accommodate the given number of arcs
    edges = new edge*[size];
    int v1, v2;
    double weight;
    for (int j = 0; j < size; j++){
        std::getline(is, line, '\n');
        ls.clear(); // Clear the stringstream before reading a new line
        ls.str(line);
        // Read the vertex at the start of the arc, vertex at the end of the arc, and the cost of the arc
        ls >> v1 >> v2 >> weight;
        edges[j] = new edge(v1, v2, weight);
    }
}



// void graph::bellmanFord(int source) {
//     // Step 1: Initialize distances and predecessor
//     std::vector<double> distance(n, std::numeric_limits<double>::infinity());
//     std::vector<int> predecessor(n, -1);
//     distance[source] = 0;

//     // Step 2: Relax edges repeatedly
//     for (int i = 0; i < n - 1; ++i) {
//         for (int j = 0; j < size; ++j) {
//             int u = edges[j]->get_p1();
//             int v = edges[j]->get_p2();
//             double weight = edges[j]->get_weight();
//             if (distance[u] + weight < distance[v]) {
//                 distance[v] = distance[u] + weight;
//                 predecessor[v] = u;
//             }
//         }
//     }

//     // Step 3: Check for negative-weight cycles
//     for (int i = 0; i < size; ++i) {
//         int u = edges[i]->get_p1();
//         int v = edges[i]->get_p2();
//         double weight = edges[i]->get_weight();
//         if (distance[u] + weight < distance[v]) {
//             std::cout << "Negative-weight cycle detected!" << std::endl;
//             return;
//         }
//     }

//     // Step 4: Print the shortest paths
//     std::cout << "Shortest paths from vertex " << node_names[source] << ":" << std::endl;
//     for (int i = 0; i < n; ++i) {
//         if (i != source) {
//             std::cout << "To vertex " << node_names[i] << ": ";
//             printPath(predecessor, i);
//             std::cout << " (Distance: " << distance[i] << ")" << std::endl;
//         }
//     }
// }

// Function to find the shortest path using Dijkstra's algorithm
// void graph::dijkstraShortestPath(vector<vector<Edge>>& graph, int numVertices, int startVertex, int endVertex) {
//     // Create a vector to store the distances from the start vertex to all other vertices
//     vector<double> distances(numVertices, std::numeric_limits<double>::infinity());

//     // Create a priority queue to store the vertices and their distances
//     priority_queue<Vertex, vector<Vertex>, greater<Vertex>> pq;

//     // Set the distance of the start vertex to 0 and push it into the priority queue
//     distances[startVertex] = 0;
//     pq.push(Vertex(startVertex, 0));

//     // Process vertices until the priority queue becomes empty
//     while (!pq.empty()) {
//         // Get the vertex with the minimum distance from the priority queue
//         Vertex current = pq.top();
//         pq.pop();

//         int currentVertex = current.id;
//         double currentDistance = current.distance;

//         // If the current distance is greater than the recorded distance, skip the vertex
//         if (currentDistance > distances[currentVertex]) {
//             continue;
//         }

//         // Traverse through all the adjacent vertices of the current vertex
//         for (const auto& edge : graph[currentVertex]) {
//             int adjacentVertex = edge.target;
//             double edgeWeight = edge.weight;

//             // Calculate the new distance
//             double newDistance = currentDistance + edgeWeight;

//             // If the new distance is shorter, update the distance and push the vertex into the priority queue
//             if (newDistance < distances[adjacentVertex]) {
//                 distances[adjacentVertex] = newDistance;
//                 pq.push(Vertex(adjacentVertex, newDistance));
//             }
//         }
//     }

//     // Check if a path from startVertex to endVertex exists
//     if (distances[endVertex] == std::numeric_limits<double>::infinity()) {
//         std::cout << "No path exists from the start vertex to the end vertex." << std::endl;
//     } else {
//         std::cout << "Shortest distance from the start vertex to the end vertex using Dijkstra's algorithm: " << distances[endVertex] << std::endl;
//     }
// }



void graph::printPath(const std::vector<int>& predecessor, int vertex) {
    if (predecessor[vertex] != -1) {
        printPath(predecessor, predecessor[vertex]);
        std::cout << " -> ";
    }
    std::cout << node_names[vertex];
}