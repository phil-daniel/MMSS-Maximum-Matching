#ifndef TYPES_H
#define TYPES_H

// Required for hashing pairs
#include <boost/container_hash/hash.hpp>

#include <set>

using namespace std;

typedef int Vertex;
typedef pair<Vertex, Vertex> Edge;
typedef pair<vector<Edge>, vector<Edge>> AugmentingPath;

enum ProgressReport {
    NO_OUTPUT = 0, // No progress output
    SCALE = 1, // Outputs when new scale
    PHASE = 2, // Outputs when new phase or scale
    PASS_BUNDLE = 3 // Outputs when new phase, scale or pass_bundle
};

enum OptimisationLevel {
    NO_OPTIMISATION = 0, // No optimisations, as described in MMSS
    ALG_SKIP = 1, // Enables the Algorithm Skip optimisation
    SCALE_SKIP = 2, // Enables the Scale Skip optimisation
    PHASE_SKIP = 3, // Enables the Phase Skip optimisation
    APPROX_MET = 4 // Enables the Approximation Met optimisation
};

struct Config {
    ProgressReport progress;
    OptimisationLevel optimisation_level;
};

#endif //TYPES_H
