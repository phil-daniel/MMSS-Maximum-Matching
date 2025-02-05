#include "GraphBlossom.h"
#include "GraphVertex.h"

GraphBlossom::GraphBlossom() {
    isBlossom = true;
}

void GraphBlossom::addGraphNodeToBlossom(GraphNode* node) {
    // Adding the node to nodesInBlossom, a set holding all the nodes in the blossom.
    nodesInBlossom.insert(node);

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
        }
    }
}

void GraphBlossom::deleteContents() {
    for (GraphNode* node : nodesInBlossom) {
        delete node;
    }
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
