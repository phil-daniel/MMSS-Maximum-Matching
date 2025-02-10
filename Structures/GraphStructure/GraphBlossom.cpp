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
    GraphNode* incoming_matched_node,
    Vertex incoming_matched_vertex,
    Vertex in_blossom_matched,
    GraphNode* incoming_unmatched_node,
    Vertex incoming_unmatched_vertex,
    Vertex in_blossom_unmatched,
    Matching* matching
) {
    vector<Edge> to_match;
    vector<Edge> to_unmatch;

    // Finding which node corresponds to the incoming to_be_matched and to_be_unmatched edges.
    GraphNode* in_blossom_matched_node = nodeOfVertexInBlossom[in_blossom_matched];
    GraphNode* in_blossom_unmatched_node = nodeOfVertexInBlossom[in_blossom_unmatched];

    // Finding the position in the nodesInOrder and nodesInOrder array of our nodes.
    int from_matched_pos = 0, from_unmatched_pos = 0;
    for (int i = 0; i < nodesInOrder.size(); i++) {
        if (nodesInOrder[i] == in_blossom_matched_node) from_matched_pos = i;
        if (nodesInOrder[i] == in_blossom_unmatched_node) from_unmatched_pos = i;
    }

    int matched_pos_plus_one = (from_matched_pos+1) % nodesInOrder.size();
    int matched_pos_minus_one = (from_matched_pos-1 + nodesInOrder.size()) % nodesInOrder.size();

    Vertex matched_pos_plus_one_vertex = nodesInOrder[matched_pos_plus_one]->getVertexInsideConnectedByEdge(nodesInOrder[from_matched_pos]);
    Vertex matched_pos_minus_one_vertex = nodesInOrder[matched_pos_minus_one]->getVertexInsideConnectedByEdge(nodesInOrder[from_matched_pos]);

    // If our start node is a blossom, we need to find the correct vertices in and out of the blossom for the forward and backwards edges.
    Vertex forwards_start_edge_vertex = nodesInOrder[from_matched_pos]->getVertexInsideConnectedByEdge(nodesInOrder[matched_pos_plus_one]);
    Vertex backwards_start_edge_vertex = nodesInOrder[from_matched_pos]->getVertexInsideConnectedByEdge(nodesInOrder[matched_pos_minus_one]);


    Edge forwards_edge = make_pair(forwards_start_edge_vertex, matched_pos_plus_one_vertex);
    Edge backwards_edge = make_pair(backwards_start_edge_vertex, matched_pos_minus_one_vertex);
    if (! (matching->isInMatching(forwards_edge) || matching->isInMatching(backwards_edge))) {
        if (nodesInOrder[from_matched_pos]->isBlossom) {
            GraphBlossom* blossom = dynamic_cast<GraphBlossom*>(nodesInOrder[from_matched_pos]);
            AugmentingPath interior_augmentation = blossom->getBlossomAugmentation(
                incoming_matched_node,
                incoming_matched_vertex,
                blossom->getVertexInsideConnectedByEdge(incoming_matched_node),
                incoming_unmatched_node,
                incoming_unmatched_vertex,
                blossom->getVertexInsideConnectedByEdge(incoming_unmatched_node),
                matching
            );
            return interior_augmentation;
        }
        return {to_match, to_unmatch};
    }

    // Since from_matched is going to become matched with the incoming edge, we know that it is the outgoing edge of from_matched that must
    // be changed to an unmatched edge
    // So we need to work out if we are going forwards or backwards.
    int direction = -1;
    // We will check whether this edge is matched to decide which direction we will cycle through the array.
    // Using modulus to prevent overflow.

    // Handling direction
    if (matching->isInMatching(forwards_edge)) {
        // If this edge is matched we know we need to go forwards, otherwise we go backwards.
        direction = 1;
    }

    int current_pos = from_matched_pos;

    bool add_to_matching = false;

    // Handling if the first node is a blossom and we have to add the nodes in the matching into the blossom.
    if (nodesInOrder[current_pos]->isBlossom) {
        GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(nodesInOrder[current_pos]);

        int next_pos = (current_pos + direction + nodesInOrder.size()) % nodesInOrder.size();

        Vertex inside_blossom_to_match = in_blossom_matched;
        Vertex inside_blossom_to_unmatch = blossom->getVertexInsideConnectedByEdge(nodesInOrder[next_pos]);

        AugmentingPath inner_augmentation = blossom->getBlossomAugmentation(
            incoming_matched_node,
            incoming_matched_vertex,
            inside_blossom_to_match,
            nodesInOrder[next_pos],
            nodesInOrder[next_pos]->getVertexInsideConnectedByEdge(nodesInOrder[current_pos]),
            inside_blossom_to_unmatch,
            matching
        );
        for (Edge edge : inner_augmentation.first) {
            to_match.emplace_back(edge);
        }
        for (Edge edge : inner_augmentation.second) {
            to_unmatch.emplace_back(edge);
        }
    }

    if (incoming_matched_vertex == incoming_unmatched_vertex) {
        // Adding one less edge, which we add back at the end
        // this prevents us from adding the blossom multiple times if incoming_matched_vertex/incoming_unmatched_vertex is a blossom.
        from_unmatched_pos = (from_unmatched_pos - direction + nodesInOrder.size()) % nodesInOrder.size();
    }

    while (current_pos != from_unmatched_pos) {

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

            // "How do we get into this blossom from GraphNode x"
            Vertex vertex_in_blossom_connected_with_curr_pos = blossom->getVertexInsideConnectedByEdge(nodesInOrder[current_pos]);
            Vertex vertex_in_blossom_connected_with_next_next_pos = blossom->getVertexInsideConnectedByEdge(nodesInOrder[next_next_pos]);

            GraphNode* from_matched_node = nodesInOrder[current_pos];
            GraphNode* from_unmatched_node = nodesInOrder[next_next_pos];
            Vertex inside_to_match = vertex_in_blossom_connected_with_curr_pos;
            Vertex inside_to_unmatch = vertex_in_blossom_connected_with_next_next_pos;

            if (! add_to_matching) {
                // If the current edge is matched, and we want to set it to unmatched,
                // we want the edge after it in the blossom to be unmatched.
                from_matched_node = nodesInOrder[next_next_pos];
                from_unmatched_node = nodesInOrder[current_pos];
                inside_to_match = vertex_in_blossom_connected_with_next_next_pos;
                inside_to_unmatch = vertex_in_blossom_connected_with_curr_pos;
            }

            Vertex from_matched_vertex = from_matched_node->getVertexInsideConnectedByEdge(blossom);
            Vertex from_unmatched_vertex = from_unmatched_node->getVertexInsideConnectedByEdge(blossom);

            // TODO: Ensure this is correct
            // Handling if final node is in a blossom
            if (next_pos == from_unmatched_pos) {
                from_unmatched_node = incoming_unmatched_node;
                from_unmatched_vertex = incoming_unmatched_vertex;
            }

            AugmentingPath inner_augmentation = blossom->getBlossomAugmentation(
                from_matched_node,
                from_matched_vertex,
                inside_to_match,
                from_unmatched_node,
                from_unmatched_vertex,
                inside_to_unmatch,
                matching
            );
            for (Edge edge : inner_augmentation.first) {
                to_match.emplace_back(edge);
            }
            for (Edge edge : inner_augmentation.second) {
                to_unmatch.emplace_back(edge);
            }
        }

        add_to_matching = ! add_to_matching;
        current_pos = next_pos;
    }

    if (incoming_matched_vertex == incoming_unmatched_vertex) {
        // Readding the edge we removed from the loop previously
        Vertex matched_pos_vertex = nodesInOrder[from_matched_pos]->getVertexInsideConnectedByEdge(nodesInOrder[from_unmatched_pos]);
        Vertex unmatched_pos_vertex = nodesInOrder[from_unmatched_pos]->getVertexInsideConnectedByEdge(nodesInOrder[from_matched_pos]);
        to_unmatch.emplace_back(matched_pos_vertex, unmatched_pos_vertex);
    } else {
        // Only need to add this edge if we aren't entering and exitting from the same node.
        Vertex vertex_connected_to_incoming_unmatched = nodesInOrder[from_unmatched_pos]->getVertexInsideConnectedByEdge(incoming_unmatched_node);
        to_unmatch.emplace_back(incoming_unmatched_vertex, vertex_connected_to_incoming_unmatched);
    }
    to_match.emplace_back(incoming_matched_vertex, in_blossom_matched);

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
