#include "GraphBlossom.h"
#include "GraphVertex.h"

GraphBlossom::GraphBlossom() {
    isBlossom = true;
}

void GraphBlossom::addGraphNodeToBlossom(GraphNode* node) {
    // Adding the node to nodesInBlossom, a set holding all the nodes in the blossom.
    nodesInBlossom.insert(node);

    // TODO: This cleans up the bit at the bottom, would prefer to remove this somehow though
    outsideBlossomToIn.erase(node);

    // TODO: need to handle adding new vertices connecting to edges

    // Adding each vertex index present in the blossom into the set verticesInBlossom
    if (node->isBlossom) {
        GraphBlossom* node_blossom = dynamic_cast<GraphBlossom *>(node);
        for (Vertex vertex : node_blossom->verticesInBlossom) {
            verticesInBlossom.insert(vertex);
        }
    } else {
        GraphVertex* node_vertex = dynamic_cast<GraphVertex *>(node);
        verticesInBlossom.insert(node_vertex->vertex_id);
    }

    for (GraphNode* child_node : node->children) {
        // If the child node is not in the blossom, we need to add the node as a child of the blossom
        // We also need to update the parent of the child node.
        if (nodesInBlossom.find(child_node) == nodesInBlossom.end()) {
            children.insert(child_node);
            child_node->parent = this;

            // We also need to add that there is an edge from this child node to the parent node in the blossom
            outsideBlossomToIn[child_node] = this;
        }
    }

    // TODO: what if multiple nodes have same parent?
    // We also need to add that there is an edge from the parent node to this node in the blossom
    if (node->parent != nullptr) {
        outsideBlossomToIn[node->parent] = this;
    }
}

void GraphBlossom::deleteContents() {
    for (GraphNode* node : nodesInBlossom) {
        delete node;
    }
}

AugmentingPath GraphBlossom::getBlossomAugmentation(Vertex from_matched, Vertex from_unmatched, Matching* matching) {
    vector<Edge> to_match;
    vector<Edge> to_unmatch;


    // TODO: Handle if this is inside a blossom (get the augmentation inside the blossom as well)
    GraphNode* node_from_matched = vertexToNodeInBlossom[from_matched];
    GraphNode* node_from_unmatched = vertexToNodeInBlossom[from_unmatched];

    // Finding the position in the nodesInOrder and verticesInOrder array of our nodes.
    int from_matched_pos = 0, from_unmatched_pos = 0;
    for (int i = 0; i < nodesInOrder.size(); i++) {
        if (nodesInOrder[i] == node_from_matched) from_matched_pos = i;
        if (nodesInOrder[i] == node_from_unmatched) from_unmatched_pos = i;
    }

    // Since from_matched is going to become matched with the incoming edge, we know that it is the outgoing edge of from_matched that must
    // be changed to an unmatched edge
    // So we need to work out if we are going forwards or backwards.
    int direction = -1;
    // We will check whether this edge is matched to decide which direction we will cycle through the array.
    // Using modulus to prevent overflow.
    // TODO: eventually get rid of using verticesInOrder in favour of the dictionary vertexToNodeInBlossom
    Edge forwards_edge = make_pair(from_matched, verticesInOrder[(from_unmatched_pos+1) % verticesInOrder.size()]);
    Edge backwards_edge = make_pair(verticesInOrder[(from_unmatched_pos-1 + verticesInOrder.size()) % verticesInOrder.size()], from_matched);

    // Handling direction
    if (matching->isInMatching(forwards_edge)) {
        // If this edge is matched we know we need to go forwards, otherwise we go backwards.
        direction = 1;
    } else if (! matching->isInMatching(backwards_edge)) {
        // If neither the forwards or backwards edge is in the matching then no augmentation to the blossom is required.
        return {};
    }

    int current_pos = from_matched_pos;

    // TODO: Need to set this
    bool add_to_matching = false;
    vector<Edge> edges_to_match, edges_to_unmatch;

    if (from_matched == from_unmatched) {
        // TODO: Handle doing the same - do one step first (entrance == exit)
    }

    while (current_pos != from_unmatched_pos) {
        // Adding overflow protection using verticesInOrder.size() being added/modulus-ed.
        int next_pos = (current_pos + direction + verticesInOrder.size()) % verticesInOrder.size();

        // TODO: IDK if verticesInOrder would work, need to eventually replace.
        if (add_to_matching) {
            to_match.emplace_back(verticesInOrder[current_pos], verticesInOrder[next_pos]);
        } else {
            to_unmatch.emplace_back(verticesInOrder[current_pos], verticesInOrder[next_pos]);
        }

        if (nodesInOrder[next_pos]->isBlossom) {

            GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(nodesInOrder[next_pos]);

            int next_next_pos = (next_pos + direction + verticesInOrder.size()) % verticesInOrder.size();
            if (next_pos == from_unmatched_pos) {
                // If we've reached the end of the cycle and the last vertex is a blossom.
                next_next_pos = current_pos;
            }

            // "How do we get into this blossom from GraphNode x"
            Vertex vertex_in_blossom_connected_with_curr_pos = blossom->outsideBlossomToIn[nodesInOrder[current_pos]];
            Vertex vertex_in_blossom_connected_with_next_next_pos = blossom->outsideBlossomToIn[nodesInOrder[next_next_pos]];

            Vertex from_matched_inner = vertex_in_blossom_connected_with_curr_pos;
            Vertex from_unmatched_inner = vertex_in_blossom_connected_with_next_next_pos;
            if (add_to_matching) {
                // If the current edge is matched, and we want to set it to unmatched,
                // we want the edge after it in the blossom to be unmatched.
                from_matched_inner = vertex_in_blossom_connected_with_next_next_pos;
                from_unmatched_inner = vertex_in_blossom_connected_with_curr_pos;
            }

            AugmentingPath inner_augmentation = blossom->getBlossomAugmentation(from_matched_inner, from_unmatched_inner, matching);
            // TODO: Add this to the augmentation
        }


        add_to_matching = ! add_to_matching;
        current_pos += next_pos;
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
