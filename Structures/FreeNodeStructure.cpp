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

void FreeNodeStructure::contract(
    Edge unmatched_arc
) {
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
    // While path from v_to_root and path from u_to_root are separated
    while (u_to_root_path.find(current_pos) == u_to_root_path.end()) {
        current_pos = current_pos->parent;
    }

    // current_pos now holds the LCA of u and v.
    GraphNode* lca = current_pos;
    // TODO: need to ensure deletion
    // TODO: need to link children to blossom
    GraphBlossom* new_blossom = new GraphBlossom();
    new_blossom->nodesInBlossom.insert(lca);
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
    new_blossom->parent_index = lca->parent_index;
    // If the parent of the LCA is nullptr, then the LCA is the root and we need to update the root node.
    if (lca->parent != nullptr) {
        lca->parent->children.erase(lca);
        lca->parent->children.insert(new_blossom);
    } else {
        free_node_root = new_blossom;
    }

    new_blossom->vertex_id = lca->vertex_id;

    // Updating vertex_to_graph_node to link all of the vertices in the blossom to the new GraphBlossom structure.
    for (Vertex vertex_id : new_blossom->verticesInBlossom) {
        vertex_to_graph_node[vertex_id] = new_blossom;
    }
}

std::ostream &operator<<(std::ostream &os, const FreeNodeStructure &structure) {
    os << "Free Node Structure:\nOn Hold: " << structure.on_hold << "\nModified: " << structure.modified;
    os << "\nContents:";

    // Does a BFS printing out the contents of each node.
    int blossom_number = 1;
    vector<GraphNode*> curr_level = {structure.free_node_root};
    while (!curr_level.empty()) {
        vector<GraphNode*> new_level;
        for (GraphNode* node : curr_level) {
            if (node->isBlossom) {
                os << "\nB" << blossom_number << " " << *node;;
                blossom_number++;
            } else {
                os << "\n" << *node;
            }
            for (GraphNode* child : node->children) {
                new_level.emplace_back(child);
            }
        }
        curr_level = new_level;
    }

    os << "\n\nStructure - Node(Parent):";
    blossom_number = 1;
    vector<pair<GraphNode*, string>> level = {make_pair(structure.free_node_root, "N/A")};
    while (!level.empty()) {
        vector<pair<GraphNode*, string>> new_level;
        os << "\n";
        for (pair<GraphNode*, string> pair : level) {
            string name;
            if (pair.first->isBlossom) {
                name = "B" + to_string(blossom_number);
                blossom_number++;
            } else {
                GraphVertex* vertex = dynamic_cast<GraphVertex*>(pair.first);
                name = "V" + to_string(vertex->vertex_id);
            }
            os << " " << name << "(" << pair.second << ")";
            for (GraphNode* child : pair.first->children) {
                new_level.emplace_back(child, name);
            }
        }
        level = new_level;
    }

    return os;
}