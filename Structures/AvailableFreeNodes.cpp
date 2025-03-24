#include "AvailableFreeNodes.h"

void AvailableFreeNodes::addNodeToStruct(GraphNode *node, GraphNode* main_node, FreeNodeStructure *structure) {
    // Used when we are adding a node to a structure, this involves:
    // - updating the AvailableFreeNodes vertex_to_struct dictionary to point each vertex in the node to the new structure.
    // - updating the corresponding FreeNodeStructure's vertex_to_graph_node to point each vertex in the node to the node.

    if (node->isBlossom) {
        // If we have a blossom we recursively call addNodeToStruct for each node within the blossom.
        GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(node);
        for (GraphNode* node : blossom->nodesInBlossom) {
            addNodeToStruct(node, main_node, structure);
        }
    } else {
        // Setting the vertex_to_struct[vertex_id] = structure, so we know which structure the vertex belongs to.
        setFreeNodeStructFromVertex(node->vertex_id, structure);
    }

    // If we have a blossom we'd like the vertex_id to point to the "main_node", i.e. the root blossom, in the vertex_to_graph_node
    // of the structure rather than the individual GraphVertex or a blossom contained within the root blossom.
    if (node == main_node) {
        // If the node is the root blossom, we update the structures vertex_to_graph_node[vertex_id] main_node
        // so we know that the vertex_id belongs to the root blossom specified.
        if (node->isBlossom) {
            // If the node is a non-trivial blossom (i.e. not a vertex) we need to add each vertex involved in
            // the blossom, rather than just the vertex_id of the node.
            GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(node);
            for (Vertex vertex : blossom->verticesInBlossom) {
                structure->addVertexToStruct(vertex, main_node);
            }
        } else {
            structure->addVertexToStruct(node->vertex_id, main_node);
        }
    }

    // Recursively adding every child of the node to the structure as well
    for (GraphNode* child : node->children) {
        addNodeToStruct(child, child, structure);
    }
}

void AvailableFreeNodes::removeNodeFromStruct(GraphNode* node, FreeNodeStructure* structure) {
    if (node->isBlossom) {
        // If the node is a blossom, we recursively remove each item within the blossom.
        GraphBlossom* blossom = dynamic_cast<GraphBlossom*>(node);
        for (GraphNode* node : blossom->nodesInBlossom) {
            removeNodeFromStruct(node, structure);
        }
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
