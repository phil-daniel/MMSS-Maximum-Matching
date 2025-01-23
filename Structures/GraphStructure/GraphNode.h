#ifndef GRAPHNODE_H
#define GRAPHNODE_H

#include <vector>

using namespace std;

class GraphNode {
    public:
        GraphNode* parent = nullptr;
        vector<GraphNode*> Children = {};
        bool isBlossom = false;

        virtual ~GraphNode(void){};
};

#endif //GRAPHNODE_H
