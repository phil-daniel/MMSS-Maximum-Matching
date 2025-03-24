#ifndef TYPES_H
#define TYPES_H

// Required for hashing pairs
#include <boost/container_hash/hash.hpp>

using namespace std;

typedef int Vertex;
typedef pair<Vertex, Vertex> Edge;
typedef pair<vector<Edge>, vector<Edge>> AugmentingPath;

// Enums used to decide what level of output is reported throughout the algorithm's execution.
enum ProgressReport {
    NO_OUTPUT = 0, // No progress output
    SCALE = 1, // Outputs when new scale
    PHASE = 2, // Outputs when new phase or scale
    PASS_BUNDLE = 3, // Outputs when new phase, scale or pass_bundle
    VERBOSE = 4, // Outputs above in addition to all operations which take place.
};

// Enums used to set what optimisations are used throughout the algorithm's execution.
enum OptimisationLevel {
    NO_OPTIMISATION = 0, // No optimisations, as described in MMSS
    ALG_SKIP = 1, // Enables the Algorithm Skip optimisation
    SCALE_SKIP = 2, // Enables the Scale Skip optimisation
    PHASE_SKIP = 3, // Enables the Phase Skip optimisation
};

// Structure of the configuration values, set the optimisations used and debug outputs.
struct Config {
    ProgressReport progress_report;
    OptimisationLevel optimisation_level;
};

#endif //TYPES_H
