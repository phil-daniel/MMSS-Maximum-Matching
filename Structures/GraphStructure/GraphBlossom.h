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
        // An unordered map which takes an input of a node of the parent of the blossom and outputs the node within the blossom
        // that it is connected to.
        unordered_map<GraphNode*, Vertex> outsideBlossomToIn;
        // An unordered map which takes input of a vertex in the blossom and outputs the node in which that vertex is in.
        unordered_map<Vertex, GraphNode*> nodeOfVertexInBlossom;

    // Functions
    private:
        void printHelper(std::ostream &os, int depth) const;
        void print(std::ostream& os) const override;
    public:
        explicit GraphBlossom();
        Vertex getVertexInsideConnectedByEdge(GraphNode* node) override;
        void addGraphNodeToBlossom(GraphNode* node);
        void deleteContents();
        void recursivelyAddOutsideBlossomToIn(GraphNode* node, Vertex vertex);
        AugmentingPath getBlossomAugmentation(
            Vertex in_blossom,
            Vertex out_blossom,
            bool to_match,
            Matching* matching
        );
        friend std::ostream &operator<<(std::ostream &os, const GraphBlossom &blossom);
};

#endif //GRAPHBLOSSOM_H
