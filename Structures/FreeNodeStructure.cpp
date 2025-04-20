#include "FreeNodeStructure.h"

#include <iostream>

GraphNode* FreeNodeStructure::getGraphNodeFromVertex(int vertex) {
    // If the vertex is not stored here return a null pointer

    if (vertex_to_graph_node.find(vertex) == vertex_to_graph_node.end()) {
        return nullptr;
    }

    return vertex_to_graph_node.at(vertex);
}

void FreeNodeStructure::addVertexToStruct(Vertex vertex, GraphNode *node) {
    vertex_to_graph_node[vertex] = node;
}

void FreeNodeStructure::removeVertexFromStruct(Vertex vertex) {
    vertex_to_graph_node.erase(vertex);
}

void FreeNodeStructure::contract(
    Edge unmatched_arc
) {

    GraphNode* node_of_u = getGraphNodeFromVertex(unmatched_arc.first);
    GraphNode* node_of_v = getGraphNodeFromVertex(unmatched_arc.second);

    // Connecting these two nodes if they are blossoms as we are adding an edge between them.
    if (node_of_u->isBlossom) {
        GraphBlossom* blossom_u = dynamic_cast<GraphBlossom*>(node_of_u);
        blossom_u->outside_blossom_to_in[node_of_v] = unmatched_arc.first;
    }

    if (node_of_v->isBlossom) {
        GraphBlossom* blossom_v = dynamic_cast<GraphBlossom*>(node_of_v);
        blossom_v->outside_blossom_to_in[node_of_u] = unmatched_arc.second;
    }

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
    while (current_pos != nullptr && u_to_root_path.find(current_pos) == u_to_root_path.end()) {
        current_pos = current_pos->parent;
    }
    // current_pos now holds the LCA of u and v.
    GraphNode* lca = current_pos;

    GraphBlossom* new_blossom = new GraphBlossom();
    new_blossom->addGraphNodeToBlossom(lca);
    lca->children = {};
    if (lca->isBlossom) {
        GraphBlossom* blossom_node = dynamic_cast<GraphBlossom*>(lca);
        for (Vertex vertex : blossom_node->vertices_in_blossom) {
            addVertexToStruct(vertex, new_blossom);
        }
    } else {
        addVertexToStruct(lca->vertex_id, new_blossom);
    }

    current_pos = node_of_v;
    while (current_pos != lca && current_pos != nullptr && current_pos != new_blossom) {
        new_blossom->addGraphNodeToBlossom(current_pos);
        current_pos->children = {};
        new_blossom->nodes_in_order.emplace_back(current_pos);

        if (current_pos->isBlossom) {
            GraphBlossom* blossom_node = dynamic_cast<GraphBlossom*>(current_pos);
            for (Vertex vertex : blossom_node->vertices_in_blossom) {
                addVertexToStruct(vertex, new_blossom);
            }
        } else {
            addVertexToStruct(current_pos->vertex_id, new_blossom);
        }

        if (current_pos == working_node) {
            working_node = new_blossom;
        }

        current_pos = current_pos->parent;
    }
    current_pos = node_of_u;

    new_blossom->nodes_in_order.emplace_back(lca);

    // Reversing the list, this is so we can get the cycle stored
    stack<GraphNode*> lca_to_u_path;
    while (current_pos != lca && current_pos != nullptr && current_pos != new_blossom) {
        lca_to_u_path.push(current_pos);

        current_pos = current_pos->parent;
    }

    while (!lca_to_u_path.empty()) {
        GraphNode* node = lca_to_u_path.top();

        new_blossom->addGraphNodeToBlossom(node);
        node->children = {};
        new_blossom->nodes_in_order.emplace_back(node);

        if (node->isBlossom) {
            GraphBlossom* blossom_node = dynamic_cast<GraphBlossom*>(node);
            for (Vertex vertex : blossom_node->vertices_in_blossom) {
                addVertexToStruct(vertex, new_blossom);
            }
        } else {
            addVertexToStruct(node->vertex_id, new_blossom);
        }

        if (node == working_node) {
            working_node = new_blossom;
        }

        lca_to_u_path.pop();
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

    // Updating vertex_to_graph_node to link all the vertices in the blossom to the new GraphBlossom structure.
    // for (Vertex vertex_id : new_blossom->vertices_in_blossom) {
    //     vertex_to_graph_node[vertex_id] = new_blossom;
    // }

    if (new_blossom->nodes_in_blossom.find(working_node) != new_blossom->nodes_in_blossom.end()) {
        working_node = new_blossom;
    }

    // Blossoms can only occur at outer vertices.
    new_blossom->isOuterVertex = true;

    this->modified = true;

}

void FreeNodeStructure::backtrack() {
    // If the structure is on hold or has been modified then it isn't stuck and hence doesn't
    // need modifying.
    if (on_hold || modified || used) {
        return;
    }

    // Updating the working node to the previous outer vertex (i.e. parent of the parent of the current).
    GraphNode* new_working_node = working_node;
    if (new_working_node != nullptr && new_working_node->parent != nullptr) {
        new_working_node = new_working_node->parent->parent;
        working_node = new_working_node;
    } else {
        working_node = nullptr;
    }

}

void FreeNodeStructure::deleteStructure() {
    // Deleting all the GraphNodes within the structure
    queue<GraphNode*> to_delete;
    to_delete.push(free_node_root);

    while (! to_delete.empty()) {
        GraphNode* item = to_delete.front();
        for (GraphNode* child : item->children) {
            to_delete.push(child);
        }

        // If it is a blossom we need to remove everything within the blossom
        if (item->isBlossom) {
            GraphBlossom* blossom = dynamic_cast<GraphBlossom*>(item);
            blossom->deleteContents();
        }
        to_delete.pop();

        delete item;
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

            // Adding a label for the working node.
            if (pair.first == structure.working_node) {
                name = "W-";
            }
            if (pair.first->isBlossom) {
                name += "B" + to_string(blossom_number);
                blossom_number++;
            } else {
                GraphVertex* vertex = dynamic_cast<GraphVertex*>(pair.first);
                name += "V" + to_string(vertex->vertex_id);
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