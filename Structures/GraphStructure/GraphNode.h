#ifndef GRAPHNODE_H
#define GRAPHNODE_H

#include <set>

#include "../types.h"

using namespace std;

class GraphNode {
    // Variables
    public:
        GraphNode* parent = nullptr;
        int parent_index = -1;
        Vertex vertex_id; // For blossoms, this value holds the "root" of the blossom
        set<GraphNode*> children = {};
        bool isBlossom = false;
        bool isOuterVertex = true;

    // Functions
    public:
        virtual ~GraphNode(void){};
        virtual Vertex getVertexInsideConnectedByEdge(GraphNode* node) = 0;
        virtual void print(std::ostream& os) const = 0;
        friend std::ostream &operator<<(std::ostream &os, const GraphNode &node) {
            node.print(os);
            return os;
        };
};

#endif //GRAPHNODE_H
