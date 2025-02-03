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
        void setFreeNodeStructFromVertex(Vertex vertex, FreeNodeStructure* structure);
        void removeNodeFromStruct(GraphNode* node, FreeNodeStructure* structure);
        void addNodeToStruct(GraphNode* node, GraphNode* main_node, FreeNodeStructure* structure);
        FreeNodeStructure* createNewStruct(GraphVertex* vertex);
        void deleteStructures() const;
    private:
        void removeBlossomFromStruct(GraphBlossom* blossom, FreeNodeStructure* structure);
        void addBlossomToStruct(GraphBlossom* blossom, GraphNode* main_node, FreeNodeStructure* structure);
};

#endif //AVAILABLEFREENODES_H
