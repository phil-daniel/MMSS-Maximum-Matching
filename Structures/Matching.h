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
        void augmentMatching(vector<AugmentingPath>* disjoint_augmenting_paths);
        void resetLabels();
        void addEdge(Edge edge);
        void removeEdge(Edge edge);
        void removeEdgeAndItsVertices(Edge edge);
        bool isInMatching(Edge edge);
        bool isVertexUsedInMatching(Vertex vertex);
        Edge getMatchedEdgeFromVertex(Vertex vertex);
        int getLabel(Edge edge);
        void setLabel(Edge edge, int label);
        void verifyMatching();
        friend std::ostream &operator<<(std::ostream &os, Matching &matching);
    private:
        Edge getStandardEdge(Edge edge);
};

#endif //MATCHING_H
