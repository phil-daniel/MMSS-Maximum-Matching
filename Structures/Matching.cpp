#include "Matching.h"

#include <iostream>

Edge Matching::getStandardEdge(Edge edge) {
    int min = (edge.first < edge.second) ? edge.first : edge.second;
    int max = (edge.first > edge.second) ? edge.first : edge.second;
    return Edge(min, max);
}

void Matching::augmentMatching(vector<AugmentingPath>* disjoint_augmenting_paths) {
    for (AugmentingPath augmenting_path : (*disjoint_augmenting_paths)) {
        for (Edge edge : augmenting_path.first) {
            addEdge(edge);
        }
        for (Edge edge : augmenting_path.second) {
            removeEdge(edge);
        }
    }
}

void Matching::addEdge(Edge edge) {
    Edge std_edge = getStandardEdge(edge);
    matched_edges.insert(std_edge);
    vertex_to_matched_edge[edge.first] = std_edge;
    vertex_to_matched_edge[edge.second] = std_edge;
}

void Matching::removeEdge(Edge edge) {
    Edge std_edge = getStandardEdge(edge);
    matched_edges.erase(std_edge);
    matched_edge_to_label.erase(std_edge);
}

void Matching::removeEdgeAndItsVertices(Edge edge) {
    Edge std_edge = getStandardEdge(edge);
    matched_edges.erase(std_edge);
    matched_edge_to_label.erase(std_edge);
    vertex_to_matched_edge.erase(std_edge.first);
    vertex_to_matched_edge.erase(std_edge.second);
}

bool Matching::isInMatching(Edge edge) {
    Edge std_edge = getStandardEdge(edge);
    if (matched_edges.find(std_edge) == matched_edges.end()) {
        return false;
    }
    return true;
}

Edge Matching::getMatchedEdgeFromVertex(Vertex vertex) {
    if (vertex_to_matched_edge.find(vertex) == vertex_to_matched_edge.end()) {
        // If it's not involved in a matching return (-1, -1), i.e. an impossible edge.
        return make_pair(-1, -1);
    }
    return vertex_to_matched_edge[vertex];
}

int Matching::getLabel(Edge edge) {
    Edge std_edge = getStandardEdge(edge);
    if (matched_edge_to_label.find(std_edge) == matched_edge_to_label.end()) {
        return -1;
    }
    return matched_edge_to_label[std_edge];
}

void Matching::setLabel(Edge edge, int label) {
    Edge std_edge = getStandardEdge(edge);
    matched_edge_to_label[std_edge] = label;
}

void Matching::resetLabels() {
    // Emptying the map.
    matched_edge_to_label.clear();
    // Setting the current distance for each matched edge to infinity.
    for (Edge edge : matched_edges) {
        matched_edge_to_label[edge] = numeric_limits<int>::max();
    }
}

void Matching::verifyMatching() {
    set<Vertex> used_vertices = {};
    for (Edge edge : matched_edges) {
        if (used_vertices.find(edge.first) != used_vertices.end()) {
            Edge used_in = vertex_to_matched_edge[edge.first];
            std::cout << "ERROR: Vertex " << edge.first << " of " << edge.first << "->" << edge.second << " already used in " << std::endl;
            exit(1);
        }
        if (used_vertices.find(edge.second) != used_vertices.end()) {
            Edge used_in = vertex_to_matched_edge[edge.second];
            std::cout << "ERROR: Vertex " << edge.second << " of " << edge.first << "->" << edge.second << " already used in " << std::endl;
            exit(1);
        }

        used_vertices.insert(edge.first);
        used_vertices.insert(edge.second);
    }

    std::cout << "Matching verified, size: " << matched_edges.size() << std::endl;
}


std::ostream &operator<<(std::ostream &os, Matching &matching) {
    os << "Matching:\n\t(Matched edge) : Label";
    for (Edge edge : matching.matched_edges) {
        os << "\n\t(" << edge.first << "->" << edge.second << ") : " << matching.getLabel(edge);
    }
    os << "\nMatching size: " << matching.matched_edges.size() << std::endl;
    return os;
}