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
        void addGraphNodeToVertex(Vertex vertex, GraphNode* node);
        void removeGraphNodeFromStructure(GraphNode* node);
        void contract(Edge unmatched_arc);
        friend std::ostream &operator<<(std::ostream &os, const FreeNodeStructure &structure);
    private:
        void removeBlossomFromStructure(GraphBlossom* blossom);
        void removeVertexFromStructure(GraphVertex* vertex);
};

#endif //FREENODESTRUCTURE_H
