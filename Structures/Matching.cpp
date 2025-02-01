#include "Matching.h"

void Matching::augmentMatching(vector<vector<Edge>>* disjoint_augmenting_paths) {
    for (vector<Edge> augmenting_path : (*disjoint_augmenting_paths)) {
        for (Edge edge : augmenting_path) {
            // If the edge isn't in the matching, we add it to the matching.
            // Otherwise we remove it from the matching.
            if (matched_edges.find(edge) == matched_edges.end()) {
                matched_edges.insert(edge);
                vertex_to_matched_edge[edge.first] = edge;
                vertex_to_matched_edge[edge.second] = edge;
            } else {
                matched_edges.erase(edge);
                matched_edge_to_label.erase(edge);
                vertex_to_matched_edge.erase(edge.first);
                vertex_to_matched_edge.erase(edge.second);
            }
        }
    }
}

void Matching::addEdge(Edge edge) {
    matched_edges.insert(edge);
    vertex_to_matched_edge[edge.first] = edge;
    vertex_to_matched_edge[edge.second] = edge;
}

void Matching::removeEdge(Edge edge) {
    matched_edges.erase(edge);
    vertex_to_matched_edge.erase(edge.first);
    vertex_to_matched_edge.erase(edge.second);
}

bool Matching::isInMatching(Edge edge) {
    if (matched_edges.find(edge) == matched_edges.end()) {
        return false;
    }
    return true;
}

Edge Matching::getMatchedEdgeFromVertex(Vertex vertex) {
    // TODO: not in dict protection?
    if (vertex_to_matched_edge.find(vertex) == vertex_to_matched_edge.end()) {
        // If it's not involved in a matching return (-1, -1), i.e. an impossible edge.
        return make_pair(-1, -1);
    }
    return vertex_to_matched_edge[vertex];
}

int Matching::getLabelFromMatchedEdge(Edge edge) {
    // TODO: not in dict protection?
    return matched_edge_to_label[edge];
}

void Matching::resetLabels() {
    // Setting the current distance for each matched edge to infinity.
    for (Edge edge : matched_edges) {
        matched_edge_to_label[edge] = numeric_limits<int>::max();
    }
}

