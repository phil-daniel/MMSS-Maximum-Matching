#ifndef GRAPHBLOSSOM_H
#define GRAPHBLOSSOM_H

#include <ostream>

#include "GraphNode.h"
#include "../Matching.h"

class GraphBlossom : public GraphNode {
    // Variables
    public:
        set<GraphNode*> nodesInBlossom;
    // TODO: Change this into a map? -> vertices to nodes within the blossom?
        set<Vertex> verticesInBlossom;
        vector<GraphNode*> nodesInOrder;
        vector<Vertex> verticesInOrder;
        // An unordered map which takes an input of a node of the parent of the blossom and outputs the node within the blossom
        // that it is connected to.
        unordered_map<GraphNode*, Vertex> outsideBlossomToIn;

    // Functions
    private:
        void printHelper(std::ostream &os, int depth) const;
        void print(std::ostream& os) const override;
    public:
        explicit GraphBlossom();
        void addGraphNodeToBlossom(GraphNode* node);
        void deleteContents();
        AugmentingPath getBlossomAugmentation(
            GraphNode* incoming_matched_node,
            Vertex incoming_matched_vertex,
            Vertex in_blossom_matched,
            GraphNode* incoming_unmatched_node,
            Vertex incoming_unmatched_vertex,
            Vertex in_blossom_unmatched,
            Matching* matching
        );
        Vertex getOutsideBlossomToInValue(GraphNode* node);
        friend std::ostream &operator<<(std::ostream &os, const GraphBlossom &blossom);
};

#endif //GRAPHBLOSSOM_H
