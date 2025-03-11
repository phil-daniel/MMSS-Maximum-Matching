#include "GraphBlossom.h"

#include <iostream>

#include "GraphVertex.h"

GraphBlossom::GraphBlossom() {
    isBlossom = true;
}

void GraphBlossom::recursivelyAddOutsideBlossomToIn(GraphNode* node, Vertex vertex) {
    outsideBlossomToIn[node] = vertex;

    GraphNode* node_of_vertex = nodeOfVertexInBlossom[vertex];
    if (node_of_vertex->isBlossom) {
        GraphBlossom* blossom = dynamic_cast<GraphBlossom*>(node_of_vertex);
        if (blossom->getVertexInsideConnectedByEdge(node) == -1) {
            blossom->recursivelyAddOutsideBlossomToIn(node, vertex);
        }
    }
}

Vertex GraphBlossom::getVertexInsideConnectedByEdge(GraphNode* node) {
    if (outsideBlossomToIn.find(node) == outsideBlossomToIn.end()) {
        return -1;
    }
    return outsideBlossomToIn[node];
}

void GraphBlossom::addGraphNodeToBlossom(GraphNode* node) {
    // Adding the node to nodesInBlossom, a set holding all the nodes in the blossom.
    nodesInBlossom.insert(node);

    // TODO: This cleans up the bit at the bottom, would prefer to remove this somehow though
    outsideBlossomToIn.erase(node);

    // TODO: need to handle adding new vertices connecting to edges

    // In case it has previously been added as a child.
    children.erase(node);

    // Adding each vertex index present in the blossom into the set verticesInBlossom
    if (node->isBlossom) {
        GraphBlossom* node_blossom = dynamic_cast<GraphBlossom *>(node);
        for (Vertex vertex : node_blossom->verticesInBlossom) {
            verticesInBlossom.insert(vertex);
            nodeOfVertexInBlossom[vertex] = node;
        }

        for (pair<GraphNode*, Vertex> pair: node_blossom->outsideBlossomToIn) {
            outsideBlossomToIn[pair.first] = pair.second;
        }
    } else {
        verticesInBlossom.insert(node->vertex_id);
        nodeOfVertexInBlossom[node->vertex_id] = node;
    }

    for (GraphNode* child_node : node->children) {
        // If the child node is not in the blossom, we need to add the node as a child of the blossom
        // We also need to update the parent of the child node.
        if (nodesInBlossom.find(child_node) == nodesInBlossom.end()) {
            children.insert(child_node);
            child_node->parent = this;

            // We also need to add that there is an edge from this child node to the parent node in the blossom
            // TODO: Fix this by checking if node is a blossom and if so using th outsideBlossomToIn of the blossom
            outsideBlossomToIn[child_node] = child_node->parent_index;
        }
    }

    // TODO: what if multiple nodes have same parent?
    // We also need to add that there is an edge from the parent node to this node in the blossom

    // Not sure this is needed. Only occurs if LCA?
    if (node->parent != nullptr && node->parent != this) {
        outsideBlossomToIn[node->parent] = node->vertex_id;
    }
}

void GraphBlossom::deleteContents() {
    for (GraphNode* node : nodesInBlossom) {
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

    GraphNode* in_blossom_node = nodeOfVertexInBlossom[in_vertex];
    GraphNode* out_blossom_node = nodeOfVertexInBlossom[out_vertex];

    // Finding the position of the nodes in the nodesInOrder array.
    int in_pos = 0, out_pos = 0;
    for (int i = 0; i < nodesInOrder.size(); i++) {
        if (nodesInOrder[i] == in_blossom_node) in_pos = i;
        if (nodesInOrder[i] == out_blossom_node) out_pos = i;
    }

    // Both of the vertices are within an inner blossom
    if (in_pos == out_pos) {
        GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(nodesInOrder[in_pos]);

        return blossom->getBlossomAugmentation2(
            in_vertex,
            out_vertex,
            add_to_matching,
            matching
        );
    }


    int in_pos_plus_one = (in_pos + 1) % nodesInOrder.size();
    int in_pos_minus_one = (in_pos - 1 + nodesInOrder.size()) % nodesInOrder.size();

    Vertex in_plus_one_vertex = nodesInOrder[in_pos_plus_one]->getVertexInsideConnectedByEdge(nodesInOrder[in_pos]);
    Vertex in_minus_one_vertex = nodesInOrder[in_pos_minus_one]->getVertexInsideConnectedByEdge(nodesInOrder[in_pos]);

    // If our start node is a blossom, we need to find the correct vertices in and out of the blossom for the forward and backwards edges.
    Vertex forwards_start_edge_vertex = nodesInOrder[in_pos]->getVertexInsideConnectedByEdge(nodesInOrder[in_plus_one_vertex]);
    Vertex backwards_start_edge_vertex = nodesInOrder[in_pos]->getVertexInsideConnectedByEdge(nodesInOrder[in_minus_one_vertex]);

    Edge forwards_edge = make_pair(forwards_start_edge_vertex, in_plus_one_vertex);
    Edge backwards_edge = make_pair(backwards_start_edge_vertex, in_plus_one_vertex);

    int direction = 1;
    // If we want to match the first edge, but the forwards edge is in the matching, we go backwards
    if (add_to_matching && matching->isInMatching(forwards_edge)) {
        direction = -1;
    }
    // If we want to unmatch the first edge, but the forwards edge is not in the matching, we go backwards
    else if (! add_to_matching && ! matching->isInMatching(forwards_edge)) {
        direction = -1;
    }

    // If both the forwards and backwards edges are unmatched and we want to match, we need to find the even length path.
    if (! matching->isInMatching(forwards_edge) && ! matching->isInMatching(backwards_edge) && add_to_matching) {
        int forwards_dist;
        if (in_pos > out_pos) forwards_dist = out_pos + nodesInOrder.size() - in_pos;
        else forwards_dist = out_pos - in_pos;
        if (forwards_dist % 2 != 0) direction = -1;
    }

    // Handling if the first node is a blossom and we have to add the nodes in the matching into the blossom.
    if (nodesInOrder[in_pos]->isBlossom) {
        GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(nodesInOrder[in_pos]);

        int next_pos = (in_pos + direction + nodesInOrder.size()) % nodesInOrder.size();
        int inner_out_vertex = blossom->getVertexInsideConnectedByEdge(nodesInOrder[next_pos]);

        AugmentingPath inner_augmentation = blossom->getBlossomAugmentation2(
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
        // Adding overflow protection using nodesInOrder.size() being added/modulus-ed.
        int next_pos = (current_pos + direction + nodesInOrder.size()) % nodesInOrder.size();

        GraphNode* curr_node = nodesInOrder[current_pos];
        GraphNode* next_node = nodesInOrder[next_pos];

        Vertex curr_vertex_from_next_node = curr_node->getVertexInsideConnectedByEdge(next_node);
        Vertex next_vertex_from_curr_node = next_node->getVertexInsideConnectedByEdge(curr_node);

        if (add_to_matching) {
            to_match.emplace_back(curr_vertex_from_next_node, next_vertex_from_curr_node);
        } else {
            to_unmatch.emplace_back(curr_vertex_from_next_node, next_vertex_from_curr_node);
        }

        if (nodesInOrder[next_pos]->isBlossom) {
            GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(nodesInOrder[next_pos]);

            int next_next_pos = (next_pos + direction + nodesInOrder.size()) % nodesInOrder.size();

            Vertex inner_in_vertex = blossom->getVertexInsideConnectedByEdge(curr_node);
            Vertex inner_out_vertex = blossom->getVertexInsideConnectedByEdge(nodesInOrder[next_next_pos]);
            if (next_pos == out_pos) {
                inner_out_vertex = out_vertex;
            }

            AugmentingPath inner_augmentation = blossom->getBlossomAugmentation2(
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

        current_pos = (current_pos + direction + nodesInOrder.size()) % nodesInOrder.size();
        add_to_matching = ! add_to_matching;
    }

    return {to_match, to_unmatch};
}

// Extracts the relevant information from the GraphBlossom to be outputed
void GraphBlossom::printHelper(std::ostream &os, int depth) const {
    os << string(depth, '\t') << "Blossom:";
    for (GraphNode* node : nodesInBlossom) {
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
