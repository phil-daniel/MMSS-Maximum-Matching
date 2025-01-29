#include "FreeNodeStructure.h"

GraphNode *FreeNodeStructure::getGraphNodeFromVertex(int vertex) {
    // If the vertex is not stored here return a null pointer
    if (vertex_to_graph_node.count(vertex) == 0) {
        return nullptr;
    }

    return vertex_to_graph_node.at(vertex);
}

void FreeNodeStructure::addGraphNodeToVertex(int vertex, GraphNode* node) {
    vertex_to_graph_node.insert({vertex, node});
}

void FreeNodeStructure::removeGraphNodeFromStructure(GraphNode* node) {
    if (node->isBlossom) {
        GraphBlossom* blossom = dynamic_cast<GraphBlossom*>(node);
        removeBlossomFromStructure(blossom);
    } else {
        GraphVertex* vertex = dynamic_cast<GraphVertex*>(node);
        removeVertexFromStructure(vertex);
    }

    // Recursively removing every child from the structure as well
    for (GraphNode* child : node->children) {
        removeGraphNodeFromStructure(child);
    }
}

void FreeNodeStructure::removeVertexFromStructure(GraphVertex* vertex) {
    vertex_to_graph_node.erase(vertex->vertex_id);
}

void FreeNodeStructure::removeBlossomFromStructure(GraphBlossom* blossom) {
    for (GraphNode* node : blossom->nodesInBlossom) {
        removeGraphNodeFromStructure(node);
    }
}

void FreeNodeStructure::contract(pair<int, int> unmatched_arc) {
    GraphNode* node_of_u = getGraphNodeFromVertex(unmatched_arc.first);
    GraphNode* node_of_v = getGraphNodeFromVertex(unmatched_arc.second);

    // TODO: Some kind of validation checking?

    // Finding the Lowest Common Ancestor of u and v.
    // Getting a set of all the nodes on the path from u to the root.
    set<GraphNode*> u_to_root_path = {};
    GraphNode* current_pos = node_of_u;
    while (current_pos != nullptr) {
        u_to_root_path.insert(current_pos);
        current_pos = current_pos->parent;
    }

    current_pos = node_of_v;
    while (u_to_root_path.find(current_pos) == u_to_root_path.end()) {
        current_pos = current_pos->parent;
    }

    // current_pos now holds the LCA of u and v.
    GraphNode* lca = current_pos;

    // TODO: need to ensure deletion
    // TODO: need to link children to blossom
    GraphBlossom* new_blossom = new GraphBlossom();
    new_blossom->nodesInBlossom.emplace_back(lca);
    current_pos = node_of_v;
    while (current_pos != lca && current_pos != nullptr) {
        new_blossom->addGraphNodeToBlossom(current_pos);
        current_pos = current_pos->parent;
    }
    current_pos = node_of_u;
    while (current_pos != lca && current_pos != nullptr) {
        new_blossom->addGraphNodeToBlossom(current_pos);
        current_pos = current_pos->parent;
    }

    new_blossom->parent = lca->parent;
    lca->parent->children.erase(lca);
    lca->parent->children.insert(new_blossom);
}