#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

typedef pair<int, int> pii;

const int INF = numeric_limits<int>::max();

vector<vector<pii>> adj_list; // adjacency list of weighted edges
vector<int> dist_from_source; // vector to store the distance from the source to each vertex

