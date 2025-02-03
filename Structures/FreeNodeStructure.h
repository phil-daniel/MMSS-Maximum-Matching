#ifndef FREENODESTRUCTURE_H
#define FREENODESTRUCTURE_H

#include <unordered_map>
#include <ostream>

#include "GraphStructure/GraphNode.h"
#include "GraphStructure/GraphVertex.h"
#include "GraphStructure/GraphBlossom.h"

#include "../types.h"


using namespace std;

class FreeNodeStructure {
    // Variables
    public:
        bool on_hold = false;
        bool modified = false;
        GraphNode* working_node;
        GraphNode* free_node_root;
        unordered_map<Vertex, GraphNode*> vertex_to_graph_node;

    // Functions
    public:
        GraphNode* getGraphNodeFromVertex(Vertex vertex);
        void addVertexToStruct(Vertex vertex, GraphNode* node);
        void removeVertexFromStruct(Vertex vertex);
        void contract(Edge unmatched_arc);
        void backtrack();
        void deleteStructure();
        friend std::ostream &operator<<(std::ostream &os, const FreeNodeStructure &structure);
};

#endif //FREENODESTRUCTURE_H
