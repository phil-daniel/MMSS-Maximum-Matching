#ifndef TYPES_H
#define TYPES_H

// Required for hashing pairs
#include <boost/container_hash/hash.hpp>

#include <set>

using namespace std;

typedef int Vertex;
typedef pair<Vertex, Vertex> Edge;
typedef pair<vector<Edge>, vector<Edge>> AugmentingPath;

#endif //TYPES_H
