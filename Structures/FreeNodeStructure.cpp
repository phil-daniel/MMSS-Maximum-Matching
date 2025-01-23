#include "FreeNodeStructure.h"

GraphNode *FreeNodeStructure::getGraphNodeFromVertex(int vertex) {
    // If the vertex is not stored here return a null pointer
    if (vertex_to_graph_node.count(vertex) == 0) {
        return nullptr;
    }

    return vertex_to_graph_node.at(vertex);
}
