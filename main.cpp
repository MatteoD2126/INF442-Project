#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

// Structure to represent an edge in the graph
struct Edge {
    int target;
    int weight;

    Edge(int tgt, int w) : target(tgt), weight(w) {}
};

// Structure to represent a vertex and its distance from the source vertex
struct Vertex {
    int id;
    int distance;

    Vertex(int i, int d) : id(i), distance(d) {}

    // Overload the comparison operator for the priority queue
    bool operator>(const Vertex& other) const {
        return distance > other.distance;
    }
};

// Function to find the shortest path using Dijkstra's algorithm
void dijkstraShortestPath(vector<vector<Edge>>& graph, int numVertices, int startVertex, int endVertex) {
    // Create a vector to store the distances from the start vertex to all other vertices
    vector<int> distances(numVertices, numeric_limits<int>::max());

    // Create a priority queue to store the vertices and their distances
    priority_queue<Vertex, vector<Vertex>, greater<Vertex>> pq;

    // Set the distance of the start vertex to 0 and push it into the priority queue
    distances[startVertex] = 0;
    pq.push(Vertex(startVertex, 0));

    // Process vertices until the priority queue becomes empty
    while (!pq.empty()) {
        // Get the vertex with the minimum distance from the priority queue
        Vertex current = pq.top();
        pq.pop();

        int currentVertex = current.id;
        int currentDistance = current.distance;

        // If the current distance is greater than the recorded distance, skip the vertex
        if (currentDistance > distances[currentVertex]) {
            continue;
        }

        // Traverse through all the adjacent vertices of the current vertex
        for (const auto& edge : graph[currentVertex]) {
            int adjacentVertex = edge.target;
            int edgeWeight = edge.weight;

            // Calculate the new distance
            int newDistance = currentDistance + edgeWeight;

            // If the new distance is shorter, update the distance and push the vertex into the priority queue
            if (newDistance < distances[adjacentVertex]) {
                distances[adjacentVertex] = newDistance;
                pq.push(Vertex(adjacentVertex, newDistance));
            }
        }
    }

    // Check if a path from startVertex to endVertex exists
    if (distances[endVertex] == numeric_limits<int>::max()) {
        cout << "No path exists from start vertex to end vertex." << endl;
    } else {
        cout << "Shortest distance from start vertex to end vertex using Dijkstra's algorithm: " << distances[endVertex] << endl;
    }
}

// Function to find the shortest path using the Bellman-Ford algorithm
void bellmanFordShortestPath(vector<vector<Edge>>& graph, int numVertices, int startVertex, int endVertex) {
    // Create an array to store the distances from the start vertex to all other vertices
    vector<int> distances(numVertices, numeric_limits<int>::max());

    // Set the distance of the start vertex to 0
    distances[startVertex] = 0;

    // Traverse through all the vertices
    for (int i = 0; i < numVertices - 1; i++) {
        // Traverse through all the edges
        for (int u = 0; u < numVertices; u++) {
            for (const auto& edge : graph[u]) {
                int v = edge.target;
                int weight = edge.weight;

                // Relax the edge
                if (distances[u] != numeric_limits<int>::max() && distances[u] + weight < distances[v]) {
                    distances[v] = distances[u] + weight;
                }
            }
        }
    }

    // Check for negative-weight cycles
    for (int u = 0; u < numVertices; u++) {
        for (const auto& edge : graph[u]) {
            int v = edge.target;
            int weight = edge.weight;

            // If a shorter path exists, a negative-weight cycle is present
            if (distances[u] != numeric_limits<int>::max() && distances[u] + weight < distances[v]) {
                cout << "Negative-weight cycle detected. Unable to find shortest path." << endl;
                return;
            }
        }
    }

    // Check if a path from startVertex to endVertex exists
    if (distances[endVertex] == numeric_limits<int>::max()) {
        cout << "No path exists from start vertex to end vertex." << endl;
    } else {
        cout << "Shortest distance from start vertex to end vertex using Bellman-Ford algorithm: " << distances[endVertex] << endl;
    }
}

int main() {
    // Example usage
    int numVertices = 5;
    vector<vector<Edge>> graph(numVertices);

    graph[0].emplace_back(1, 4); // Edge from vertex 0 to vertex 1 with weight 4
    graph[0].emplace_back(2, 3); // Edge from vertex 0 to vertex 2 with weight 3
    graph[1].emplace_back(3, 2); // Edge from vertex 1 to vertex 3 with weight 2
    graph[2].emplace_back(1, 1); // Edge from vertex 2 to vertex 1 with weight 1
    graph[2].emplace_back(3, 4); // Edge from vertex 2 to vertex 3 with weight 4
    graph[3].emplace_back(4, 5); // Edge from vertex 3 to vertex 4 with weight 5

    int startVertex = 0;
    int endVertex = 4;

    int choice;
    cout << "Choose the algorithm to find the shortest path:" << endl;
    cout << "1. Dijkstra's algorithm" << endl;
    cout << "2. Bellman-Ford algorithm" << endl;
    cin >> choice;

    if (choice == 1) {
        dijkstraShortestPath(graph, numVertices, startVertex, endVertex);
    } else if (choice == 2) {
        bellmanFordShortestPath(graph, numVertices, startVertex, endVertex);
    } else {
        cout << "Invalid choice. Exiting..." << endl;
        return 0;
    }

    return 0;
}
