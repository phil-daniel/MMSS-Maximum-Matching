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
        verticesInBlossom.insert(
            node_blossom->verticesInBlossom.begin(), node_blossom->verticesInBlossom.end()
        );
    } else {
        GraphVertex* node_vertex = dynamic_cast<GraphVertex *>(node);
        verticesInBlossom.insert(node_vertex->vertex_id);
    }

    /*
    for (GraphNode* child : node->children) {
        // TODO: Work out why this bit
        if (child->isBlossom) {
            GraphBlossom* blossom_child = dynamic_cast<GraphBlossom*>(child);
            child_to_blossom_vertex[child] = blossom_child->vertexToParent;
        } else {
            GraphVertex* vertex_child = dynamic_cast<GraphVertex*>(child);
            child_to_blossom_vertex[child] = vertex_child->vertex_id;
        }
    }
    */
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
