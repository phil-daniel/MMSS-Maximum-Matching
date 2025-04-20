#include "GraphBlossom.h"

#include <iostream>

#include "GraphVertex.h"

GraphBlossom::GraphBlossom() {
    isBlossom = true;
}

void GraphBlossom::recursivelyAddOutsideBlossomToIn(GraphNode* node, Vertex vertex) {
    outside_blossom_to_in[node] = vertex;

    GraphNode* node_of_vertex = vertex_to_node_in_blossom[vertex];
    if (node_of_vertex->isBlossom) {
        GraphBlossom* blossom = dynamic_cast<GraphBlossom*>(node_of_vertex);
        if (blossom->getVertexInsideConnectedByEdge(node) != vertex) {
            blossom->recursivelyAddOutsideBlossomToIn(node, vertex);
        }
    }
}

Vertex GraphBlossom::getVertexInsideConnectedByEdge(GraphNode* node) {
    if (outside_blossom_to_in.find(node) == outside_blossom_to_in.end()) {
        return -1;
    }
    return outside_blossom_to_in[node];
}

void GraphBlossom::addGraphNodeToBlossom(GraphNode* node) {
    // Adding the node to nodes_in_blossom, a set holding all the nodes in the blossom.
    nodes_in_blossom.insert(node);

    // Since this node is now inside the blossom, it shouldn't be referenced in outside_blossom_to_in
    outside_blossom_to_in.erase(node);

    // In case it has previously been added as a child.
    children.erase(node);

    // Adding each vertex index present in the blossom into the set vertices_in_blossom
    if (node->isBlossom) {
        GraphBlossom* node_blossom = dynamic_cast<GraphBlossom *>(node);
        for (Vertex vertex : node_blossom->vertices_in_blossom) {
            vertices_in_blossom.insert(vertex);
            vertex_to_node_in_blossom[vertex] = node;
        }

        for (pair<GraphNode*, Vertex> pair: node_blossom->outside_blossom_to_in) {
            outside_blossom_to_in[pair.first] = pair.second;
        }
    } else {
        vertices_in_blossom.insert(node->vertex_id);
        vertex_to_node_in_blossom[node->vertex_id] = node;
    }

    for (GraphNode* child_node : node->children) {
        // If the child node is not in the blossom, we need to add the node as a child of the blossom
        // We also need to update the parent of the child node.
        if (nodes_in_blossom.find(child_node) == nodes_in_blossom.end()) {
            children.insert(child_node);
            child_node->parent = this;

            // We also need to add that there is an edge from this child node to the parent node in the blossom
            outside_blossom_to_in[child_node] = child_node->parent_index;
        }
    }

    // We also need to add that there is an edge from the parent node to this node in the blossom

    if (node->parent != nullptr && node->parent != this &&
        this->nodes_in_blossom.find(node->parent) == this->nodes_in_blossom.end()
    ) {
        outside_blossom_to_in[node->parent] = node->vertex_id;
    }
}

void GraphBlossom::deleteContents() {
    for (GraphNode* node : nodes_in_blossom) {
        if (node->isBlossom) {
            GraphBlossom* blossom = dynamic_cast<GraphBlossom*>(node);
            blossom->deleteContents();
        }
        delete node;
    }
}

AugmentingPath GraphBlossom::getBlossomAugmentation(
    Vertex in_vertex,
    Vertex out_vertex,
    bool add_to_matching,
    Matching* matching
) {
    if (in_vertex == out_vertex) {
        return {{}, {}};
    }

    vector<Edge> to_match;
    vector<Edge> to_unmatch;

    GraphNode* in_blossom_node = vertex_to_node_in_blossom[in_vertex];
    GraphNode* out_blossom_node = vertex_to_node_in_blossom[out_vertex];

    // Finding the position of the nodes in the nodes_in_order array.
    int in_pos = 0, out_pos = 0;
    for (int i = 0; i < nodes_in_order.size(); i++) {
        if (nodes_in_order[i] == in_blossom_node) in_pos = i;
        if (nodes_in_order[i] == out_blossom_node) out_pos = i;
    }

    // Both of the vertices are within an inner blossom
    if (in_pos == out_pos) {
        GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(nodes_in_order[in_pos]);

        return blossom->getBlossomAugmentation(
            in_vertex,
            out_vertex,
            add_to_matching,
            matching
        );
    }

    // Finding which direction around the blossom cycle gives us an even-length cycle
    int direction = 1;
    int forward_dist;
    if (in_pos > out_pos) forward_dist = out_pos + nodes_in_order.size() - in_pos;
    else forward_dist = out_pos - in_pos;
    if (forward_dist % 2 != 0) direction = -1;

    // Handling if the first node is a blossom and we have to add the nodes in the matching into the blossom.
    if (nodes_in_order[in_pos]->isBlossom) {
        GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(nodes_in_order[in_pos]);

        int next_pos = (in_pos + direction + nodes_in_order.size()) % nodes_in_order.size();
        int inner_out_vertex = blossom->getVertexInsideConnectedByEdge(nodes_in_order[next_pos]);

        AugmentingPath inner_augmentation = blossom->getBlossomAugmentation(
            in_vertex,
            inner_out_vertex,
            add_to_matching,
            matching
        );
        for (Edge edge : inner_augmentation.first) {
            to_match.emplace_back(edge);
        }
        for (Edge edge : inner_augmentation.second) {
            to_unmatch.emplace_back(edge);
        }

    }

    int current_pos = in_pos;

    while (current_pos != out_pos) {
        // Adding overflow protection using nodes_in_order.size() being added/modulus-ed.
        int next_pos = (current_pos + direction + nodes_in_order.size()) % nodes_in_order.size();

        GraphNode* curr_node = nodes_in_order[current_pos];
        GraphNode* next_node = nodes_in_order[next_pos];

        Vertex curr_vertex_from_next_node = curr_node->getVertexInsideConnectedByEdge(next_node);
        Vertex next_vertex_from_curr_node = next_node->getVertexInsideConnectedByEdge(curr_node);

        if (add_to_matching) {
            to_match.emplace_back(curr_vertex_from_next_node, next_vertex_from_curr_node);
        } else {
            to_unmatch.emplace_back(curr_vertex_from_next_node, next_vertex_from_curr_node);
        }

        if (nodes_in_order[next_pos]->isBlossom) {
            GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(nodes_in_order[next_pos]);

            int next_next_pos = (next_pos + direction + nodes_in_order.size()) % nodes_in_order.size();

            Vertex inner_in_vertex = blossom->getVertexInsideConnectedByEdge(curr_node);
            Vertex inner_out_vertex = blossom->getVertexInsideConnectedByEdge(nodes_in_order[next_next_pos]);
            if (next_pos == out_pos) {
                inner_out_vertex = out_vertex;
            }

            AugmentingPath inner_augmentation = blossom->getBlossomAugmentation(
                inner_in_vertex,
                inner_out_vertex,
                ! add_to_matching,
                matching
            );
            for (Edge edge : inner_augmentation.first) {
                to_match.emplace_back(edge);
            }
            for (Edge edge : inner_augmentation.second) {
                to_unmatch.emplace_back(edge);
            }
        }

        current_pos = (current_pos + direction + nodes_in_order.size()) % nodes_in_order.size();
        add_to_matching = ! add_to_matching;
    }
    return {to_match, to_unmatch};
}

// Extracts the relevant information from the GraphBlossom to be outputed
void GraphBlossom::printHelper(std::ostream &os, int depth) const {
    os << string(depth, '\t') << "Blossom:";
    for (GraphNode* node : nodes_in_blossom) {
        if (node->isBlossom) {
            GraphBlossom* child_blossom = dynamic_cast<GraphBlossom*>(node);
            os << "\n";
            child_blossom->printHelper(os, depth+1);
        } else {
            GraphVertex* child_vertex = dynamic_cast<GraphVertex*>(node);
            os << "\n" << string(depth+1, '\t') << *child_vertex;
        }
    }
}

// Overwrites the print function inherited from GraphNode, allowing the contents of a GraphBlossom
// to be printed when it is of type GraphNode.
void GraphBlossom::print(std::ostream& os) const {
    printHelper(os, 0);
}

// Allows us to output the contents of the GraphBlossom, i.e. using std::cout.
ostream &operator<<(std::ostream &os, const GraphBlossom &blossom) {
    blossom.print(os);
    return os;
}
