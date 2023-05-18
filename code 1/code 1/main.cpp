#include "point.hpp"
#include "cloud.hpp"
#include "edge.hpp"
#include "graph.hpp"

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>


using namespace std;

int main(){
    int nb_points, nb_arcs;
    
    ifstream is("rcsp1.txt");
    string line;
    getline(is, line, '\n');
    stringstream ls;
    ls << line;
    ls >> nb_points >> nb_arcs;
    cout << "Number of points: "<< nb_points << ". ";
    cout << "Number of arcs: "<< nb_arcs << endl;
    
    cloud* c = new cloud(nb_points);
    graph* G = new graph(*c, nb_arcs);
    
    cout << "number of points according to cloud: " << c->get_n() << endl;
    cout << "number of points according to graph: " << G->get_num_nodes() << endl;
    cout << "number of edges according to graph: " << G->get_num_edges() << endl;
    
    for (int i = 0; i < c->get_n(); i++){
        c->get_point(i).print();
    }

    cout << "Shortest path calculated with Bellman Ford algorithm: " << G->bellmanFord() << endl << "\n";
    
    return 0;
}