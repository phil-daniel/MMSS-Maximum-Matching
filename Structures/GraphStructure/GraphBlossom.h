#ifndef GRAPHBLOSSOM_H
#define GRAPHBLOSSOM_H

#include <unordered_map>

#include "GraphNode.h"

class GraphBlossom : public GraphNode {
    // Variables
    public:
        vector<GraphNode*> nodesInBlossom;
        unordered_map<GraphNode*, int> child_to_blossom_vertex;
        int vertexToParent;


    // Functions
    private:
        void ostreamHelper(std::ostream &os, int depth) const;
    public:
        explicit GraphBlossom();
        void addGraphNodeToBlossom(GraphNode* node);
        friend std::ostream &operator<<(std::ostream &os, const GraphBlossom &blossom);
};

#endif //GRAPHBLOSSOM_H
