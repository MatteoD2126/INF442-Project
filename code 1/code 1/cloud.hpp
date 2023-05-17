#pragma once // single inclusion

#include <fstream>

#include "point.hpp"

//this class describes the dataset, all the points from dataset are included in the cloud
class cloud {
    int n; //number of points

    int nmax; // Maximum possible number of points

    point *points; //the array of points
    
    int* nb_resources; //the array of 

public:
    cloud(int _d, int _nmax);
    cloud(int _n);
    ~cloud();

    // Getters
    int get_n() const;
    const point &get_point(int i) const;

    // Helper methods
    void add_point(point &p);
    void load(std::ifstream &is);
};
