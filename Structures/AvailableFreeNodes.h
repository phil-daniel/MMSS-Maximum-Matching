#ifndef AVAILABLEFREENODES_H
#define AVAILABLEFREENODES_H

#include "FreeNodeStructure.h"
#include "types.h"

using namespace std;

class AvailableFreeNodes {
    // Variables
    public:
        // List containing all the current free node structures.
        vector<FreeNodeStructure*> free_node_structures;
    private:
        // Dictionary matching vertex_ids to their corresponding free node structures.
        unordered_map<Vertex, FreeNodeStructure*> vertex_to_struct;

    // Functions
    public:
        // Uses public getter/setter methods for vertex_to_struct to prevent problems if vertex isn't part of a structure.
        FreeNodeStructure* getFreeNodeStructFromVertex(Vertex vertex);
        void setFreeNodeStructFromVertex(Vertex vertex, FreeNodeStructure* structure);
        void removeNodeFromStruct(GraphNode* node, FreeNodeStructure* structure);
        void addNodeToStruct(GraphNode* node, FreeNodeStructure* structure);
        FreeNodeStructure* createNewStruct(GraphVertex* vertex);
        void deleteStructures() const;
};

#endif //AVAILABLEFREENODES_H
