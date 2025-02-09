#ifndef GRAPHVERTEX_H
#define GRAPHVERTEX_H

#include <vector>
#include <ostream>

#include "GraphNode.h"


using namespace std;

class GraphVertex : public GraphNode {
    // Variables
    public:


    // Functions
    private:
        void print(std::ostream& os) const override;
    public:
        explicit GraphVertex(Vertex vertex_id);
        Vertex getVertexInsideConnectedByEdge(GraphNode* node) override;
        friend std::ostream &operator<<(std::ostream &os, const GraphVertex &vertex);
};

#endif //GRAPHVERTEX_H
