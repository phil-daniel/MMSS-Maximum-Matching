#include "GraphVertex.h"

GraphVertex::GraphVertex(int vertex_id) {
    isBlossom = false;
    this->vertex_id = vertex_id;
}

std::ostream &operator<<(std::ostream &os, const GraphVertex &vertex) {
    os << "Vertex " << vertex.vertex_id;
    return os;
}