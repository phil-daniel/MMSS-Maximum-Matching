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
};

#endif //GRAPHNODE_H
