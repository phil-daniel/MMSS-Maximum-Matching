#ifndef FREENODESTRUCTURE_H
#define FREENODESTRUCTURE_H

#include <unordered_map>
#include <ostream>

#include "GraphStructure/GraphNode.h"
#include "GraphStructure/GraphVertex.h"
#include "GraphStructure/GraphBlossom.h"


using namespace std;

class FreeNodeStructure {
    // Variables
    public:
        // TODO: ADD FREE NODE?
        bool on_hold = false;
        bool modified = false;
        int vertices_count = 0;
        GraphNode* working_vertex;
        GraphNode* free_node_root;
    private:
        unordered_map<int, GraphNode*> vertex_to_graph_node;

    // Functions
    public:
        GraphNode* getGraphNodeFromVertex(int vertex);
        void addGraphNodeToVertex(int vertex, GraphNode* node);
        void removeGraphNodeFromStructure(GraphNode* node);
        void contract(pair<int, int> unmatched_arc);
        friend std::ostream &operator<<(std::ostream &os, const FreeNodeStructure &structure);
    private:
        void removeBlossomFromStructure(GraphBlossom* blossom);
        void removeVertexFromStructure(GraphVertex* vertex);
};

#endif //FREENODESTRUCTURE_H
