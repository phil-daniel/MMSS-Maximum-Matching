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