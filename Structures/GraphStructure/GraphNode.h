#ifndef GRAPHNODE_H
#define GRAPHNODE_H

#include <vector>

using namespace std;

class GraphNode {
    // Variables
    public:
        GraphNode* parent = nullptr;
        vector<GraphNode*> children = {};
        bool isBlossom = false;
        bool isOuterVertex = true;


    // Funcations
    public:
        virtual ~GraphNode(void){};
};

#endif //GRAPHNODE_H
