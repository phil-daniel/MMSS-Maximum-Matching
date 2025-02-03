#include "AvailableFreeNodes.h"

void AvailableFreeNodes::addBlossomToStruct(GraphBlossom *blossom, GraphNode* main_node, FreeNodeStructure *structure) {
    for (GraphNode* node : blossom->nodesInBlossom) {
        addNodeToStruct(node, main_node, structure);
    }
}

void AvailableFreeNodes::addNodeToStruct(GraphNode *node, GraphNode* main_node, FreeNodeStructure *structure) {
    // If we have a blossom we'd like to point to the "main_node", i.e. the blossom
    // rather than the individual GraphVertex
    if (node->isBlossom) {
        GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(node);
        addBlossomToStruct(blossom, main_node, structure);
    } else {
        structure->addVertexToStruct(node->vertex_id, main_node);
        setFreeNodeStructFromVertex(node->vertex_id, structure);
    }

    // Recursively adding every child from the structure as well
    for (GraphNode* child : node->children) {
        addNodeToStruct(child, child, structure);
    }
}

void AvailableFreeNodes::removeBlossomFromStruct(GraphBlossom* blossom, FreeNodeStructure* structure) {
    for (GraphNode* node : blossom->nodesInBlossom) {
        removeNodeFromStruct(node, structure);
    }
}

void AvailableFreeNodes::removeNodeFromStruct(GraphNode* node, FreeNodeStructure* structure) {
    if (node->isBlossom) {
        GraphBlossom* blossom = dynamic_cast<GraphBlossom*>(node);
        removeBlossomFromStruct(blossom, structure);
    } else {
        GraphVertex* vertex = dynamic_cast<GraphVertex*>(node);
        structure->vertex_to_graph_node.erase(vertex->vertex_id);
        vertex_to_struct.erase(vertex->vertex_id);
    }

    // Recursively removing every child from the structure as well
    for (GraphNode* child : node->children) {
        removeNodeFromStruct(child, structure);
    }
}

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

    new_struct->addVertexToStruct(vertex->vertex_id, vertex);
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
