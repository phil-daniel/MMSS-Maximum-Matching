#ifndef GRAPHNODE_H
#define GRAPHNODE_H

#include <set>

using namespace std;

class GraphNode {
    // Variables
    public:
        GraphNode* parent = nullptr;
        set<GraphNode*> children = {};
        bool isBlossom = false;
        bool isOuterVertex = true;


    // Functions
    public:
        virtual ~GraphNode(void){};
        virtual void print(std::ostream& os) const = 0;
        friend std::ostream &operator<<(std::ostream &os, const GraphNode &node) {
            node.print(os);
            return os;
        };
};

#endif //GRAPHNODE_H
