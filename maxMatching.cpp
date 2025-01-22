#include <fstream>
#include <iostream>
#include <set>

// Required for hashing pairs
#include <boost/container_hash/hash.hpp>

#include "Stream/Stream.h"
#include "Stream/StreamFromFile.h"
#include "Stream/StreamFromMemory.h"

#include "Structures/FreeNodeStructure.h"

using namespace std;

typedef int Vertex;
typedef pair<Vertex, Vertex> Edge;
typedef set<Edge> Matching;
typedef unordered_map<Edge, int, boost::hash<Edge>> MatchingToLabel;

void extend_active_path(
    Stream* stream,
    Matching matching,
    float epsilon,
    vector<FreeNodeStructure> free_node_structs,
    vector<vector<Edge>> disjoint_augmenting_paths,
    set<Vertex> removed_vertices
) {

    unordered_map<Vertex, FreeNodeStructure*> vertex_to_free_node_struct;

    Edge edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {
        // If we have "removed" one of the vertices from the graph, we skip this edge.
        if (
            removed_vertices.find(edge.first) != removed_vertices.end()
            || removed_vertices.find(edge.second) != removed_vertices.end()
        ) {
            continue;
        }

        // TODO: Case 2 - to do with blossoms

        // If the first vertex is in a "marked" or "on hold" structure, we skip this edge.
        if (
            vertex_to_free_node_struct.count(edge.first)
            && (
                vertex_to_free_node_struct[edge.first]->modified
                || vertex_to_free_node_struct[edge.first]->on_hold
            )
        ) {
            continue;
        }


        // Reading next edge
        edge = stream->readStream();
    }

}


vector<vector<Edge>> alg_phase(
    Stream* stream,
    Matching matching,
    float epsilon,
    float scale
) {
    vector<vector<Edge>> disjoint_augmenting_paths = {};

    set<Vertex> removed_vertices = {};

    MatchingToLabel matching_to_label;

    int path_limit = static_cast<int>(6 / scale) + 1;
    int pass_bundles_max = static_cast<int>(72 / (scale * epsilon));

    vector<FreeNodeStructure> free_node_structs;

    // Setting the current distance for each matched edge to infinity.
    for (Edge edge : matching) {
        matching_to_label[edge] = numeric_limits<int>::max();
    }

    for (int pass_bundle = 0; pass_bundle < pass_bundles_max; pass_bundle++) {
        // TODO: CHECK EACH OF THE FREE VERTEX STRUCTURES HERE
        for (FreeNodeStructure free_node_struct : free_node_structs) {
            if (free_node_struct.vertices_count >= path_limit) free_node_struct.on_hold = true;
            else free_node_struct.on_hold = false;
            free_node_struct.modified = false;
        }

        // TODO: IMPLEMENT ALGORITHM HERE!
    }


    return disjoint_augmenting_paths;
}

Matching augment_matching(
    Matching matching,
    vector<vector<Edge>> disjoint_augmenting_paths
) {
    // Takes a vector (list) of disjoint augmenting paths and adds them to the matching.

    for (vector<Edge> augmenting_path : disjoint_augmenting_paths) {
        for (Edge edge : augmenting_path) {
            // If the edge isn't in the matching, we add it to the matching.
            // Otherwise we remove it from the matching.
            if (matching.find(edge) == matching.end()) {
                matching.insert(edge);
            } else {
                matching.erase(edge);
            }
        }
    }

    return matching;
}

Matching algorithm(
    Stream* stream,
    Matching matching,
    float epsilon
) {

    float scale_limit = (epsilon * epsilon) / 64;

    for (float scale = 1/2; scale <= scale_limit; scale *= 1/2) {
        float phase_limit = 144 / (scale * epsilon);

        for (float phase = 1; phase <= phase_limit; phase++) {
            vector<vector<Edge>> disjoint_augmenting_paths = {};
            matching = augment_matching(matching, disjoint_augmenting_paths);
        }
    }

    return matching;
}

Matching get_2_approximate_matching(
    Stream* stream
) {
    std::set<int> involved_in_matching = set<int>();
    Matching matching;

    Edge edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {

        // 2-approximation within 1 pass, only adds to the matching if both vertices are currently not in the matching.
        if (
            involved_in_matching.find(edge.first) == involved_in_matching.end()
            && involved_in_matching.find(edge.second) == involved_in_matching.end()
        ) {
            involved_in_matching.insert(edge.first);
            involved_in_matching.insert(edge.second);
            matching.insert(edge);
        }

        // Reading next edge
        edge = stream->readStream();
    }

    return matching;
}

int main() {

    //Stream* stream = new StreamFromFile("example.txt");
    Stream* stream = new StreamFromMemory("example.txt");

    Matching matching = get_2_approximate_matching(stream);

    std::cout << "Matching size: " << matching.size() << std::endl;

    return 0;
}
