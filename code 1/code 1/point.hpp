#pragma once // ensures that this header file is only included once

#include <string>

// a declaration of the class
// implementation is in a separate file: point.cpp
class point {
    static int d;//dimension - same for all points -> static
    
public:
    double *coords; //set of point's coordinates
    std::string name; //label

    static bool set_dim(int _d);
    static int get_dim();

    point();
    ~point();

    void print() const;
    double dist(const point &q) const;
};
