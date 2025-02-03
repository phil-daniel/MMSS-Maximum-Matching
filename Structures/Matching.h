#ifndef MATCHING_H
#define MATCHING_H

#include "../types.h"

class Matching {
    // Variables
    public:
        set<Edge> matched_edges;
        unordered_map<Vertex, Edge> vertex_to_matched_edge;
        unordered_map<Edge, int, boost::hash<Edge>> matched_edge_to_label;

    // Functions
    public:
        void augmentMatching(vector<vector<Edge>>* disjoint_augmenting_paths);
        void resetLabels();
        void addEdge(Edge edge);
        void removeEdge(Edge edge);
        bool isInMatching(Edge edge);
        Edge getMatchedEdgeFromVertex(Vertex vertex);
        int getLabel(Edge edge);
        void setLabel(Edge edge, int label);
        friend std::ostream &operator<<(std::ostream &os, Matching &matching);
};

#endif //MATCHING_H
