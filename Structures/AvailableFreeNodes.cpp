#include "AvailableFreeNodes.h"

FreeNodeStructure* AvailableFreeNodes::getFreeNodeStructFromVertex(Vertex vertex) {
    if (vertex_to_struct.find(vertex) == vertex_to_struct.end()) {
        return nullptr;
    }

    return vertex_to_struct[vertex];
}

void AvailableFreeNodes::setFreeNodeStructFromVertex(Vertex vertex, FreeNodeStructure* structure) {
    vertex_to_struct[vertex] = structure;
}

FreeNodeStructure* AvailableFreeNodes::createNewStruct(GraphVertex* vertex) {
    // Making the assumption that we are only going to be creating new structures which with vertices and not blossoms.
    FreeNodeStructure* new_struct = new FreeNodeStructure();

    new_struct->addGraphNodeToVertex(vertex->vertex_id, vertex);
    new_struct->free_node_root = vertex;
    new_struct->working_node = vertex;
    vertex->isOuterVertex = true;

    free_node_structures.emplace_back(new_struct);

    vertex_to_struct[vertex->vertex_id] = new_struct;

    return new_struct;
}

void AvailableFreeNodes::deleteStructures() const {
    for (FreeNodeStructure* free_node : free_node_structures) {
        free_node->deleteStructure();
        delete free_node;
    }
}
