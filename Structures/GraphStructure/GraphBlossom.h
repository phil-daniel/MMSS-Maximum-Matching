#ifndef GRAPHBLOSSOM_H
#define GRAPHBLOSSOM_H

#include <unordered_map>

#include "GraphNode.h"

class GraphBlossom : public GraphNode {
    // Variables
    public:
        vector<GraphNode*> nodesInBlossom;
        unordered_map<int, int> child_to_blossom_vertex;
        int vertexToParent;


    // Functions
    public:
        explicit GraphBlossom();
};

#endif //GRAPHBLOSSOM_H
