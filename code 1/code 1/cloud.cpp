#include "cloud.hpp" // The header for the class implemented here
#include "point.hpp" // Used in all methods

#include <cassert> // This provides the assert() method
#include <iostream>
#include <sstream>

//constructor
cloud::cloud(int _d, int _nmax) {
    point::set_dim(_d);

    nmax = _nmax;
    n = 0;

    points = new point[nmax];
}

//another constructor for d = 1;
cloud::cloud(int _n){
    point::set_dim(1);
    n = _n;
    nmax = _n;
    
    points = new point[nmax];
    for(int i = 0; i < n; i++){
        points[i].coords[0] = i;
    }
}

//destructor
cloud::~cloud() {
    delete[] points;
}

//get number of points
int cloud::get_n() const {
    return n;
}

//get ith point
const point &cloud::get_point(int i) const {
    return points[i];
}

//add point p to the cloud
void cloud::add_point(point &p) {
    assert(n < nmax);

    for (int m = 0; m < point::get_dim(); m++) {
        points[n].coords[m] = p.coords[m];
        points[n].name = p.name;
    }
    n++;
}

//load dataset from the file
void cloud::load(std::ifstream &is) {
    assert(is.is_open());
    
    //NEW CODE
    int numResources;
    std::string line;
    std::getline(is, line, '\n');
    std::stringstream ls;
    ls << line;
    ls >> n >> n >> numResources;
    
    std::getline(is, line, '\n');
    std::getline(is, line, '\n');
    
    for (int i = 0; i < n; i++){
        for (int k = 0; k < numResources; k++){
            std::getline(is, line, '\n');
        }
    }
    //NEW CODE FINISH

    // Point to read into
    point p;

    //std::string line;
    // Skipping header
    std::getline(is, line, '\n');
    while (std::getline(is, line, '\n')) {
        std::stringstream ls;
	ls << line;
	std::string scoord;
	for (int i = 0; i < point::get_dim(); ++i) {
	    std::getline(ls, scoord, ',');
	    p.coords[i] = std::stod(scoord);
	}
        std::getline(ls, p.name, ',');
	add_point(p);
    }
}
