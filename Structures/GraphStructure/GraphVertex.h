#ifndef GRAPHVERTEX_H
#define GRAPHVERTEX_H

#include <vector>
#include <ostream>

#include "GraphNode.h"


using namespace std;

class GraphVertex : public GraphNode {
    // Variables
    public:
        int vertex_id;
        explicit GraphVertex(int vertex_id);

    // Functions
    public:
        friend std::ostream &operator<<(std::ostream &os, const GraphVertex &vertex);
};

#endif //GRAPHVERTEX_H
