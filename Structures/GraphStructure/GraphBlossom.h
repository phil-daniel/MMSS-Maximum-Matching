#ifndef GRAPHBLOSSOM_H
#define GRAPHBLOSSOM_H

#include <ostream>

#include "GraphNode.h"
#include "../Matching.h"

class Matching;

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
        // TODO: Replace vertices in blossom w/ this
        unordered_map<Vertex, GraphNode*> vertexToNodeInBlossom;
        // TODO: Need to sort out which ones we need to keep
        // TODO: Need to set items in nodeToInteriorBlossom

    // Functions
    private:
        void printHelper(std::ostream &os, int depth) const;
        void print(std::ostream& os) const override;
    public:
        explicit GraphBlossom();
        void addGraphNodeToBlossom(GraphNode* node);
        void deleteContents();
        AugmentingPath getBlossomAugmentation(Vertex from_matched, Vertex from_unmatched, Matching* matching);
        friend std::ostream &operator<<(std::ostream &os, const GraphBlossom &blossom);
};

#endif //GRAPHBLOSSOM_H
