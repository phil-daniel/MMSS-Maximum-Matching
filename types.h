#ifndef TYPES_H
#define TYPES_H

// Required for hashing pairs
#include <set>
#include <boost/container_hash/hash.hpp>

using namespace std;

typedef int Vertex;
typedef pair<Vertex, Vertex> Edge;
typedef unordered_map<Edge, int, boost::hash<Edge>> MatchingToLabel;

#endif //TYPES_H
