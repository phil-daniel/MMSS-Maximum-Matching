#ifndef AVAILABLEFREENODES_H
#define AVAILABLEFREENODES_H

#include "FreeNodeStructure.h"
#include "../types.h"

using namespace std;

class AvailableFreeNodes {
    // Variables
    public:
        vector<FreeNodeStructure*> free_node_structures;
    private:
        // Uses public getter/setter methods to prevent problems if vertex isn't part of a structure.
        unordered_map<Vertex, FreeNodeStructure*> vertex_to_struct;


    // Functions
    public:
        FreeNodeStructure* getFreeNodeStructFromVertex(Vertex vertex);
        FreeNodeStructure* createNewStruct(GraphVertex* vertex);
        void deleteStructures();
};

#endif //AVAILABLEFREENODES_H
