#ifndef GRAPHVERTEX_H
#define GRAPHVERTEX_H

#include <vector>

#include "GraphNode.h"

using namespace std;

class GraphVertex : public GraphNode {
    public:
        int vertex_id;
        explicit GraphVertex(int vertex_id);
};

#endif //GRAPHVERTEX_H
