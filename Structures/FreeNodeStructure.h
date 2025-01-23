#ifndef FREENODESTRUCTURE_H
#define FREENODESTRUCTURE_H

#include <unordered_map>

#include "GraphStructure/GraphNode.h"

using namespace std;

class FreeNodeStructure {
    // Variables
    public:
        bool on_hold = false;
        bool modified = false;
        int vertices_count = 0;
        GraphNode* working_vertex;
    private:
        unordered_map<int, GraphNode*> vertex_to_graph_node;

    // Functions
    public:
        GraphNode* getGraphNodeFromVertex(int vertex);

};

#endif //FREENODESTRUCTURE_H
