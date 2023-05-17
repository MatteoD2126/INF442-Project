#pragma once

#include "cloud.hpp"
#include "edge.hpp"

/*  graph -- an array of edge pointers arranged by
 *  increasing length.  Allows iteration:
 *
 *  init_iteration() places current at
 *    the start of the array
 *  get_next() returns the current edge
 *    and advances to the next one
 */
class graph {
    edge **edges;

    long n; // the number of vertices(nodes).
    long size; // the number of edges.
    long iterator_pos; //the position of the iterator

    std::string *node_names; //an array of the names of the nodes in the graph.
public:
    graph(const cloud &_c);
    graph(const cloud &_c, int _size);
    graph(long _n, const std::string _node_names[], double **dist_matrix);
    ~graph();

    void start_iteration();
    edge *get_next();

    long get_num_edges() const;
    long get_num_nodes() const;
    std::string &get_name(int i) const;
    edge *get_edge(long i);
    edge **get_edges();

    void load(std::ifstream &is);
    static graph *load_matrix(std::ifstream &is);
};
