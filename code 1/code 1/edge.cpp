#include "edge.hpp"  // This is the header for the class implemented here

#include <cassert> // This provides the assert methods
#include <cstddef> // This provides NULL

/* edge -- method implementations */

int edge::count_compare = 0;

edge::edge(int _p1, int _p2, double _weight) {
    assert(_p1 >= 0);
    assert(_p2 >= 0);

    p1 = _p1;
    p2 = _p2;
    weight = _weight;
}

edge::~edge() {
    edge::count_compare = 0;
}

double edge::get_weight() {
    return weight;
}

bool edge::compare(edge *e1, edge *e2) {
    count_compare++; // for testing only
    return e1->get_weight() < e2->get_weight();
}

int edge::get_count_compare() {
    return count_compare;
}

int edge::get_p1() {
    return p1;
}

int edge::get_p2() {
    return p2;
}

