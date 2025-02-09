#include "GraphVertex.h"

GraphVertex::GraphVertex(Vertex vertex_id) {
    isBlossom = false;
    this->vertex_id = vertex_id;
}

Vertex GraphVertex::getVertexInsideConnectedByEdge(GraphNode* node){
    return vertex_id;
}

void GraphVertex::print(std::ostream& os) const {
    os << "Vertex " << vertex_id;
}

std::ostream &operator<<(std::ostream &os, const GraphVertex &vertex) {
    vertex.print(os);
    return os;
}