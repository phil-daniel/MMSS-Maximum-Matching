#include "MMSS-Maximum-Matching.h"


/*
Example Usage:
int main() {
    // Creates a new edge stream, using StreamFromFile or StreamFromMemory depending on emulation requirements.
    //Stream* stream = new StreamFromFile("example.txt");
    Stream* stream = new StreamFromMemory("example.txt");

    // Call the getMMSSApproxMaximumMatching() function with the relevant parameters.
    Matching matching = getMMSSApproxMaximumMatching(stream, 0.25, 3, true, false);
    // A matching is returned.
    std::cout << matching << std::endl;
    // The number of passes can be seen by checking the stream.
    std::cout << "Total number of passes: " << stream->number_of_passes << std::endl;

    delete stream;

    return 0;
}
*/

void updateChildLabels(
    GraphNode* parent_matched_vertex,
    int new_label,
    Matching* matching
) {
    // Conducts a breadth-first search to update all the matched edge labels below parent_matched_vertex
    vector<GraphNode*> current_level = {parent_matched_vertex};
    while (! current_level.empty()) {
        // Need to go two levels down each time, otherwise we will be labelling unmatched edges
        vector<GraphNode*> new_level = {};
        for (GraphNode* node : current_level) {
            for (GraphNode* child : node->children) {

                Edge matched_edge = make_pair(child->parent_index, child->vertex_id);

                matching->setLabel(matched_edge, new_label);

                for (GraphNode* grandchild : child->children) {
                    new_level.emplace_back(grandchild);
                }
            }
        }
        new_label++;
        current_level = new_level;
    }
}

AugmentingPath getAugmentationInTree(
    Vertex in_vertex,
    GraphNode* in_node,
    Matching* matching
) {
    GraphNode* curr_node = in_node;

    // Getting the path in the tree from in_node to the root.
    vector<GraphNode*> path_to_root;
    while (curr_node != nullptr) {
        path_to_root.emplace_back(curr_node);
        curr_node = curr_node->parent;
    }

    vector<Edge> to_match, to_unmatch;
    bool add_to_matching = false;

    int pos = 0;

    if (in_node->isBlossom) {
        Vertex out_blossom_vertex;

        if (in_node->parent == nullptr) {
            out_blossom_vertex = in_node->vertex_id;
        }
        else {
            GraphNode* next_node = path_to_root[1];
            out_blossom_vertex = in_node->getVertexInsideConnectedByEdge(next_node);
        }

        GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(in_node);

        AugmentingPath augmentation = blossom->getBlossomAugmentation(
            in_vertex,
            out_blossom_vertex,
            add_to_matching,
            matching
        );
        for (Edge edge : augmentation.first) {
            to_match.emplace_back(edge);
        }
        for (Edge edge : augmentation.second) {
            to_unmatch.emplace_back(edge);
        }
    }

    while (pos < path_to_root.size()-1) {
        curr_node = path_to_root[pos];
        GraphNode* next_node = path_to_root[pos + 1];

        Vertex vertex_from_next_to_curr = curr_node->getVertexInsideConnectedByEdge(next_node);
        Vertex vertex_from_curr_to_next = next_node->getVertexInsideConnectedByEdge(curr_node);

        if (add_to_matching) {
            to_match.emplace_back(vertex_from_curr_to_next, vertex_from_next_to_curr);
        }
        else {
            to_unmatch.emplace_back(vertex_from_curr_to_next, vertex_from_next_to_curr);
        }

        if (next_node->isBlossom) {
            Vertex in_blossom_vertex = next_node->getVertexInsideConnectedByEdge(curr_node);
            Vertex out_blossom_vertex;

            if (next_node->parent == nullptr) {
                out_blossom_vertex = next_node->vertex_id;
            }
            else {
                GraphNode* next_next_node = path_to_root[pos + 2];
                out_blossom_vertex = next_node->getVertexInsideConnectedByEdge(next_next_node);
            }

            GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(next_node);

            AugmentingPath augmentation = blossom->getBlossomAugmentation(
                in_blossom_vertex,
                out_blossom_vertex,
                !add_to_matching,
                matching
            );
            for (Edge edge : augmentation.first) {
                to_match.emplace_back(edge);
            }
            for (Edge edge : augmentation.second) {
                to_unmatch.emplace_back(edge);
            }
        }

        add_to_matching = ! add_to_matching;
        pos += 1;
    }
    return {to_match, to_unmatch};
}

AugmentingPath getAugmentation(
    GraphNode* node_in_struct_u,
    GraphNode* node_in_struct_v,
    Vertex vertex_in_u_joining,
    Vertex vertex_in_v_joining,
    Matching* matching
) {
    AugmentingPath augmenting_path_in_u = getAugmentationInTree(vertex_in_u_joining, node_in_struct_u, matching);
    AugmentingPath augmenting_path_in_v = getAugmentationInTree(vertex_in_v_joining, node_in_struct_v, matching);

    vector<Edge> to_match, to_unmatch;
    for (Edge edge : augmenting_path_in_u.first) {
        to_match.emplace_back(edge);
    }
    for (Edge edge : augmenting_path_in_u.second) {
        to_unmatch.emplace_back(edge);
    }
    for (Edge edge : augmenting_path_in_v.first) {
        to_match.emplace_back(edge);
    }
    for (Edge edge : augmenting_path_in_v.second) {
        to_unmatch.emplace_back(edge);
    }
    to_match.emplace_back(vertex_in_u_joining, vertex_in_v_joining);

    return {to_match, to_unmatch};
}

void augment(
    vector<AugmentingPath>* disjoint_augmenting_paths,
    Edge unmatched_arc,
    AvailableFreeNodes* available_free_nodes,
    Matching* matching
) {

    FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(unmatched_arc.first);
    FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(unmatched_arc.second);

    GraphNode* graph_node_of_u = struct_of_u->getGraphNodeFromVertex(unmatched_arc.first);
    GraphNode* graph_node_of_v = struct_of_v->getGraphNodeFromVertex(unmatched_arc.second);
    AugmentingPath new_augmentation = getAugmentation(graph_node_of_u, graph_node_of_v, unmatched_arc.first, unmatched_arc.second, matching);

    struct_of_u->used = true;
    struct_of_v->used = true;

    disjoint_augmenting_paths->emplace_back(new_augmentation);
}

void contractAndAugment(
    Stream* stream,
    AvailableFreeNodes* available_free_nodes,
    vector<AugmentingPath>* disjoint_augmenting_paths,
    Matching* matching,
    Config config,
    int* operations_completed
) {

    unordered_map<FreeNodeStructure*, vector<Edge>> edges_in_structures;

    // Contraction Step
    Edge edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {

        FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(edge.first);
        FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(edge.second);

        // If the two vertices are in the same non-used structure.
        if (
            struct_of_u != nullptr && struct_of_u == struct_of_v && (! struct_of_u->used)
        ) {
            GraphNode* node_of_u = struct_of_u->getGraphNodeFromVertex(edge.first);
            GraphNode* node_of_v = struct_of_u->getGraphNodeFromVertex(edge.second);

            // If the two vertices are not in the same root blossom.
            if (node_of_u != node_of_v) {
                // Adding the edge to the list of edges connecting vertices in the structure
                if (edges_in_structures.find(struct_of_u) == edges_in_structures.end()) {
                    edges_in_structures[struct_of_u] = {edge};
                } else {
                    edges_in_structures[struct_of_u].emplace_back(edge);
                }
            }
        }

        // Reading next edge
        edge = stream->readStream();
    }

    for (pair<FreeNodeStructure*, vector<Edge>> pair : edges_in_structures) {
        if (edge.first == -1 || edge.second == -1) {
            continue;
        }
        int contractions_in_last_iteration = -1;
        while (contractions_in_last_iteration != 0) {
            contractions_in_last_iteration = 0;
            for (Edge edge_in_struct : pair.second) {
                GraphNode* node_of_u = pair.first->getGraphNodeFromVertex(edge_in_struct.first);
                GraphNode* node_of_v = pair.first->getGraphNodeFromVertex(edge_in_struct.second);

                if (node_of_u != node_of_v &&  node_of_u->isOuterVertex && node_of_v->isOuterVertex) {
                    pair.first->contract(edge_in_struct);
                    GraphNode* new_blossom = pair.first->getGraphNodeFromVertex(edge_in_struct.first);

                    int blossom_parent_id = new_blossom->parent_index;
                    int parent_label = matching->getLabel(matching->getMatchedEdgeFromVertex(blossom_parent_id));
                    for (GraphNode* child : new_blossom->children) {
                        updateChildLabels(child, parent_label, matching);
                    }
                    contractions_in_last_iteration += 1;

                    *operations_completed += 1;
                    if (config.progress_report >= VERBOSE) {
                        std::cout << "ContractAndAugment - Contract: Struct " << pair.first->free_node_root->vertex_id;
                        std::cout << " on edge " << edge_in_struct.first << "->" << edge_in_struct.second << std::endl;
                    }
                }
            }
        }
    }

    // Augmentation Step
    edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {

        FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(edge.first);
        FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(edge.second);

        if (
            struct_of_u != nullptr && struct_of_v != nullptr &&
            ! struct_of_u->used && ! struct_of_v->used &&
            struct_of_u != struct_of_v
        ) {

            GraphNode* node_of_u = struct_of_u->getGraphNodeFromVertex(edge.first);
            GraphNode* node_of_v = struct_of_v->getGraphNodeFromVertex(edge.second);
            if (node_of_u->isOuterVertex && node_of_v->isOuterVertex && ! (struct_of_u->used || struct_of_v->used)) {
                augment(disjoint_augmenting_paths, edge, available_free_nodes, matching);

                *operations_completed += 1;

                if (config.progress_report >= VERBOSE) {
                    std::cout << "ContractAndAugment - Augment: Struct " << struct_of_u->free_node_root->vertex_id;
                    std::cout << " and Struct "<< struct_of_v->free_node_root->vertex_id;
                    std::cout << " on edge " << edge.first << "->" << edge.second << std::endl;
                }
            }
        }

        // Reading next edge
        edge = stream->readStream();
    }
}

void backtrackStuckStructures(
    AvailableFreeNodes* available_free_nodes,
    Config config,
    int* operations_completed
) {
    for (FreeNodeStructure* structure : available_free_nodes->free_node_structures) {
        if (structure->on_hold || structure->modified || structure->used || structure->working_node == nullptr) {
            continue;
        }

        structure->backtrack();

        *operations_completed += 1;
        if (config.progress_report >= VERBOSE) {
            std::cout << "Backtracking: Struct " << structure->free_node_root->vertex_id << std::endl;
            if (structure->working_node == nullptr) std::cout << "Struct " << structure->free_node_root->vertex_id << " now inactive." << std::endl;
        }
    }
}

void overtake(
    Edge unmatched_arc, // (u,v)
    Edge matched_arc, // (v,t)
    AvailableFreeNodes* available_free_nodes,
    Matching* matching,
    Config config
) {

    FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(unmatched_arc.first);
    FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(unmatched_arc.second);
    FreeNodeStructure* struct_of_t = available_free_nodes->getFreeNodeStructFromVertex(matched_arc.second);

    Edge matched_arc_using_u = matching->getMatchedEdgeFromVertex(unmatched_arc.first);
    int current_label = matching->getLabel(matched_arc_using_u);
    // int current_label = 0;
    // if (matched_arc_using_u.first != -1) {
    //     // If the edge doesn't exist them getMatchedEdgeFromVertex will return (-1,-1)
    //     current_label = matching->getLabel(matched_arc_using_u);
    // }
    if (unmatched_arc.second != matched_arc.first) {
        matched_arc = make_pair(matched_arc.second, matched_arc.first);
    }

    // Case 1: Our matched_arc is not currently in a structure
    if (struct_of_v == nullptr && struct_of_t == nullptr) {
        GraphVertex* vertex_v = new GraphVertex(unmatched_arc.second);
        GraphVertex* vertex_t = new GraphVertex(matched_arc.second);

        if (struct_of_u->working_node->isBlossom) {
            GraphBlossom* blossom = dynamic_cast<GraphBlossom *>(struct_of_u->working_node);
            blossom->recursivelyAddOutsideBlossomToIn(vertex_v, unmatched_arc.first);
        }

        vertex_v->parent = struct_of_u->working_node;
        vertex_v->parent_index = unmatched_arc.first;
        vertex_t->parent = vertex_v;
        vertex_t->parent_index = matched_arc.first;
        vertex_v->children.insert(vertex_t);
        struct_of_u->working_node->children.insert(vertex_v);
        struct_of_u->working_node = vertex_t;

        // By the algorithm we know that v will be an outer vertex and t will be an inner vertex.
        vertex_v->isOuterVertex = false;
        vertex_t->isOuterVertex = true;

        available_free_nodes->addNodeToStruct(vertex_v, struct_of_u);

        matching->setLabel(matched_arc, current_label+1);

        struct_of_u->modified = true;

        if (config.progress_report >= VERBOSE) {
            std::cout << "Overtake Case 1: Struct " << struct_of_u->free_node_root->vertex_id;
            std::cout << ", edge " << unmatched_arc.first << "->" << unmatched_arc.second << std::endl;
        }
    }

    // Case 2: If our matched_arc is currently in a structure.
    else {
        // Case 2.1: If the matched arc is in the same structure as the vertex u, with the matched arc joining them.
        // I.e. overtaking within a single structure.
        if (struct_of_u == struct_of_t) {
            // Here we know struct_of_u == struct_of_v == struct_of_v
            GraphNode* vertex_u = struct_of_t->getGraphNodeFromVertex(unmatched_arc.first);
            GraphNode* vertex_v = struct_of_t->getGraphNodeFromVertex(matched_arc.first);
            GraphNode* vertex_t = struct_of_t->getGraphNodeFromVertex(matched_arc.second);

            GraphNode* current_parent_of_v = vertex_v->parent;
            // Removing vertex v from the set of it's parent's children.
            current_parent_of_v->children.erase(vertex_v);

            if (current_parent_of_v->isBlossom) {
                GraphBlossom* parent_blossom = dynamic_cast<GraphBlossom *>(current_parent_of_v);
                parent_blossom->outside_blossom_to_in.erase(vertex_v);
            }

            if (vertex_v->isBlossom) {
                GraphBlossom* blossom_v = dynamic_cast<GraphBlossom *>(vertex_v);
                blossom_v->recursivelyAddOutsideBlossomToIn(vertex_u, unmatched_arc.second);
                //blossom_v->outside_blossom_to_in[vertex_u] = unmatched_arc.second;
                blossom_v->outside_blossom_to_in.erase(current_parent_of_v);
            }

            if (vertex_u->isBlossom) {
                GraphBlossom* blossom_u = dynamic_cast<GraphBlossom *>(vertex_u);
                blossom_u->recursivelyAddOutsideBlossomToIn(vertex_v, unmatched_arc.first);
                //blossom_u->outside_blossom_to_in[vertex_v] = unmatched_arc.first;
            }

            // Updating vertex v to now be parented by vertex u
            vertex_u->children.insert(vertex_v);
            vertex_v->parent = vertex_u;
            vertex_v->parent_index = unmatched_arc.first;

            // Updating the working vertex
            struct_of_t->working_node = vertex_t;

            struct_of_t->modified = true;
            updateChildLabels(vertex_v, current_label+1, matching);

            if (config.progress_report >= VERBOSE) {
                std::cout << "Overtake Case 2.1: Struct " << struct_of_u->free_node_root->vertex_id;
                std::cout << " on itself, edge " << unmatched_arc.first << "->" << unmatched_arc.second << std::endl;
            }
        }
        // Case 2.2: If the matched arc is in a different structure to u, with the unmatched arc (u,v) joining the two structures.
        // I.e. overtaking between two structures.
        else {
            GraphNode* vertex_u = struct_of_u->getGraphNodeFromVertex(unmatched_arc.first);
            GraphNode* vertex_v = struct_of_v->getGraphNodeFromVertex(unmatched_arc.second);

            GraphNode* parent_of_v_in_struct_v = vertex_v->parent;
            parent_of_v_in_struct_v->children.erase(vertex_v);

            if (parent_of_v_in_struct_v->isBlossom) {
                GraphBlossom* parent_blossom = dynamic_cast<GraphBlossom *>(parent_of_v_in_struct_v);
                parent_blossom->outside_blossom_to_in.erase(vertex_v);
            }

            if (vertex_v->isBlossom) {
                GraphBlossom* blossom_v = dynamic_cast<GraphBlossom *>(vertex_v);
                blossom_v->outside_blossom_to_in.erase(parent_of_v_in_struct_v);
                blossom_v->recursivelyAddOutsideBlossomToIn(vertex_u, unmatched_arc.second);
            }

            if (vertex_u->isBlossom) {
                GraphBlossom* blossom_u = dynamic_cast<GraphBlossom *>(vertex_u);
                blossom_u->recursivelyAddOutsideBlossomToIn(vertex_v, unmatched_arc.first);
            }


            vertex_v->parent = vertex_u;
            vertex_v->parent_index = unmatched_arc.first;

            vertex_u->children.insert(vertex_v);

            available_free_nodes->removeNodeFromStruct(vertex_v, struct_of_v);
            available_free_nodes->addNodeToStruct(vertex_v, struct_of_u);

            GraphNode* old_working_node = struct_of_v->working_node;
            if (old_working_node != nullptr && struct_of_v->getGraphNodeFromVertex(old_working_node->vertex_id) == nullptr) {
                struct_of_v->working_node = parent_of_v_in_struct_v;
                struct_of_u->working_node = old_working_node;
            }

            struct_of_u->modified = true;
            struct_of_v->modified = true;

            updateChildLabels(vertex_v, current_label+1, matching);

            if (config.progress_report >= VERBOSE) {
                std::cout << "Overtake Case 2.2: Struct " << struct_of_u->free_node_root->vertex_id;
                std::cout << " on Struct "<< struct_of_v->free_node_root->vertex_id;
                std::cout << ", edge " << unmatched_arc.first << "->" << unmatched_arc.second << std::endl;
            }

        }
    }
}

void extendActivePath(
    Stream* stream,
    Matching* matching,
    AvailableFreeNodes* available_free_nodes,
    vector<AugmentingPath>* disjoint_augmenting_paths,
    Config config,
    int* operations_completed
) {
    Edge edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {

        // Checking whether you need to create a new FreeNodeStructure each vertex in the edge.
        // Requirements to create a new FreeNodeStructure:
        // - The vertex is not involved in the current matching
        // - The vertex does not belong to any current FreeNodeStructures.
        if (
            ! matching->isVertexUsedInMatching(edge.first) &&
            available_free_nodes->getFreeNodeStructFromVertex(edge.first) == nullptr
        ) {
            GraphVertex* new_vertex_u = new GraphVertex(edge.first);
            available_free_nodes->createNewStruct(new_vertex_u);
        }
        if (
            ! matching->isVertexUsedInMatching(edge.second) &&
            available_free_nodes->getFreeNodeStructFromVertex(edge.second) == nullptr
        ) {
            GraphVertex* new_vertex_v = new GraphVertex(edge.second);
            available_free_nodes->createNewStruct(new_vertex_v);
        }

        // We will define "u" as the first edge and "v" as the second edge.

        // Identifying the structures of each of the vertices in the edge.
        FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(edge.first);
        FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(edge.second);

        // If u does not belong to a structure, we move to the next edge in the stream.
        if (struct_of_u == nullptr) {
            edge = stream->readStream();
            continue;
        }
        // Case 1 - If we have "used" one of the vertices from the graph, we skip this edge.
        if (struct_of_u->used || (struct_of_v != nullptr && struct_of_v->used) ) {
            edge = stream->readStream();
            continue;
        }

        // Case 2: If blossom1 is in the same root blossom as blossom2, vertex1 isn't a working vertex
        // or the edge is already matched, we skip this edge.
        if (
            (
                struct_of_u == struct_of_v &&
                struct_of_u->getGraphNodeFromVertex(edge.first) == struct_of_u->getGraphNodeFromVertex(edge.second)
            ) ||
            (struct_of_u->working_node != struct_of_u->getGraphNodeFromVertex(edge.first)) ||
            matching->isInMatching(edge)
        ) {
            edge = stream->readStream();
            continue;
        }

        // Case 3: If the first vertex is in a "marked" or "on hold" structure, we skip this edge.
        if (struct_of_u->modified || struct_of_u->on_hold) {
            edge = stream->readStream();
            continue;
        }

        // Case 4: If blossom of v is an outer vertex we can either contract or augment it.
        if (struct_of_u != nullptr && struct_of_v != nullptr &&
            struct_of_u->getGraphNodeFromVertex(edge.first)->isOuterVertex &&
            struct_of_v->getGraphNodeFromVertex(edge.second)->isOuterVertex
        ) {
            if (struct_of_u == struct_of_v) {
                // u and v belong to the same structure
                if (struct_of_u->getGraphNodeFromVertex(edge.first) != struct_of_u->getGraphNodeFromVertex(edge.second)) {
                    // u and v do not belong to the same root blossom.

                    // Contracting the blossom created by the adding the edge to the structure.
                    struct_of_u->contract(edge);

                    GraphNode* new_blossom = struct_of_u->getGraphNodeFromVertex(edge.first);
                    int blossom_parent_id = new_blossom->parent_index;
                    int parent_label = matching->getLabel(matching->getMatchedEdgeFromVertex(blossom_parent_id));
                    for (GraphNode* child : new_blossom->children) {
                        updateChildLabels(child, parent_label, matching);
                    }

                    *operations_completed += 1;
                    if (config.progress_report >= VERBOSE) {
                        std::cout << "Contract: Struct " << struct_of_u->free_node_root->vertex_id;
                        std::cout << " on edge" << edge.first << "->" << edge.second << std::endl;
                    }

                }
            } else {
                // u and v belong to different structures

                // Creating a augmenting path from the root of u to the root of v, through the edge.
                augment(disjoint_augmenting_paths, edge, available_free_nodes, matching);

                *operations_completed += 1;
                if (config.progress_report >= VERBOSE) {
                    std::cout << "Augment: Struct " << struct_of_u->free_node_root->vertex_id;
                    std::cout << " and Struct "<< struct_of_v->free_node_root->vertex_id;
                    std::cout << " on edge " << edge.first << "->" << edge.second << std::endl;
                }
            }
        }

        // Case 5: Otherwise we attempt to overtake and add the matched edge to the structure.
        else {

            // Ensuring v is not an ancestor of u in any structure, this is a requirement of overtake.
            if (struct_of_u == struct_of_v) {
                GraphNode* current = struct_of_u->getGraphNodeFromVertex(edge.first);
                GraphNode* v_node = struct_of_u->getGraphNodeFromVertex(edge.second);
                bool v_is_ancestor_of_u = false;
                while (current != nullptr) {
                    if (current == v_node) {
                        v_is_ancestor_of_u = true;
                        break;
                    }
                    current = current->parent;
                }
                if (v_is_ancestor_of_u) {
                    edge = stream->readStream();
                    continue;
                }
            }

            int distance_to_u = 0;
            // Getting the matched edge whose has a vertex of u.
            Edge matching_using_u = matching->getMatchedEdgeFromVertex(edge.first);
            // If u is a free vertex it won't have a matching connecting to it.
            if (matching_using_u.first != -1) {
                distance_to_u = matching->getLabel(matching_using_u);
            }

            // Getting the matched edge whose has a vertex of u.
            Edge matching_using_v = matching->getMatchedEdgeFromVertex(edge.second);
            int distance_to_v = matching->getLabel(matching_using_v);

            // If the matched edge using v exists we can overtake from edge {u,v}
            if (matching_using_v.first != -1 && distance_to_u + 1 < distance_to_v) {
                overtake(edge, matching_using_v, available_free_nodes, matching, config);
                *operations_completed += 1;

            }
        }

        // Reading next edge
        edge = stream->readStream();
    }
}


pair<bool, vector<AugmentingPath>> algPhase(
    Stream* stream,
    Matching* matching,
    float epsilon,
    float scale,
    Config config
) {
    /* Completes a single phase of the MMSS algorithm */

    vector<AugmentingPath> disjoint_augmenting_paths = {};

    int path_limit = static_cast<int>(6 / scale) + 1;
    int pass_bundles_max = static_cast<int>(72 / (scale * epsilon));

    AvailableFreeNodes available_free_nodes = AvailableFreeNodes();

    // Setting the value of all the labels back to the maximum value.
    matching->resetLabels();

    int pass_bundle;

    for (pass_bundle = 1; pass_bundle <= pass_bundles_max; pass_bundle++) {
        // Used to count the number of operations completed in a pass bundle, part of the Phase Skip optimisation
        int operations_completed = 0;

        if (config.progress_report >= PASS_BUNDLE) std::cout << "Pass bundle: " << pass_bundle << "/" << pass_bundles_max << std::endl;

        // Resetting any free node structures whenever required.
        for (FreeNodeStructure* free_node_struct : available_free_nodes.free_node_structures) {
            if (free_node_struct->vertex_to_graph_node.size() >= path_limit) free_node_struct->on_hold = true;
            else free_node_struct->on_hold = false;
            free_node_struct->modified = false;
        }

        // Attempts to increase the active path of each free node structure in a single pass over the edge stream.
        extendActivePath(stream, matching, &available_free_nodes, &disjoint_augmenting_paths, config, &operations_completed);
        // Contracts any blossoms in free node structures and checks for any augmenting paths between them.
        contractAndAugment(stream, &available_free_nodes, &disjoint_augmenting_paths, matching, config, &operations_completed);
        // Backtracks any structures which have not be used.
        backtrackStuckStructures(&available_free_nodes, config, &operations_completed);

        // std::cout << "Overtakes: " << overtake_count << " Contracts: " << contract_count << " Backtracks: " << backtrack_count << " Augments: " << augment_count  << std::endl;

        // Phase Skip optimisation - If we have not completed any overtake, contract, augment or backtrack operations,
        // skip the remaining pass bundles of the current phase.
        if (config.optimisations && operations_completed == 0) {
            if (config.progress_report >= PASS_BUNDLE) std::cout << "PHASE SKIP: No operations completed in the current pass bundle, skipping the remainder of the phase." << std::endl;
            break;
        }

    }

    // End check signifies whether we continue with the algorithm after completion of this phase. If true we carry on.
    bool end_check = true;

    // Algorithm Skip Optimisation
    bool node_on_hold_exists = false;
    for (FreeNodeStructure* free_node_struct : available_free_nodes.free_node_structures) {
        node_on_hold_exists = node_on_hold_exists || free_node_struct->on_hold;
    }
    if (config.optimisations && ! node_on_hold_exists && disjoint_augmenting_paths.size() == 0) {
        std::cout << "ALGORITHM SKIP: DFS has completed with no nodes marked as on hold, finishing the algorithm early" << std::endl;
        end_check = false;
    }

    // Early Finish optimisation -
    if (config.early_finish && end_check) {
        int minimum_matching_size_required = ceil(
            (matching->matched_edges.size() + floor(available_free_nodes.free_node_structures.size() / 2)) / (1 + epsilon)
        );
        if (matching->matched_edges.size() + disjoint_augmenting_paths.size() >= minimum_matching_size_required) {
            std::cout << "Early finish check passed:" << std::endl;
            std::cout << "\tMinimum Matching size required: " << minimum_matching_size_required << std::endl;
            std::cout << "\tCurrent Matching size: " << matching->matched_edges.size() + disjoint_augmenting_paths.size() << std::endl;
            // Set to false so we finish early and don't complete another phase
            end_check = false;
        }
    }

    available_free_nodes.deleteStructures();

    // Returns true so we continue with the next phases
    return make_pair(end_check, disjoint_augmenting_paths);
}

Matching get2ApproximateMatching(
    Stream* stream
) {
    /* Greedily produces a 2-approximate maximum matching in a single pass over the graph's edge stream. */
    Matching matching;

    Edge edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {
        // The edge is only added to the matching if both vertices are currently not in the matching.
        if (
            ! matching.isVertexUsedInMatching(edge.first) &&
            ! matching.isVertexUsedInMatching(edge.second)
        ) {
            matching.addEdge(edge);
        }
        // Reading next edge
        edge = stream->readStream();
    }

    return matching;
}

Matching getMMSSApproxMaximumMatching(
    Stream* stream,
    float epsilon,
    int progress_report,
    bool optimisations,
    bool early_finish
) {

    // Setting up the config structure.
    Config config;
    if (progress_report < NO_OUTPUT) config.progress_report = NO_OUTPUT;
    else if (progress_report > VERBOSE) config.progress_report = VERBOSE;
    else config.progress_report = static_cast<ProgressReport>(progress_report);
    config.optimisations = optimisations;
    config.early_finish = early_finish;

    // Greedy matching, giving a 2 approximation
    Matching matching = get2ApproximateMatching(stream);

    // Outputting relevant information about the initial matching if required.
    if (config.progress_report >= SCALE) std::cout << "2 approximation size: " << matching.matched_edges.size() << std::endl;
    if (config.progress_report >= PASS_BUNDLE) std::cout << matching << std::endl;

    // Finding the limit, rounding up to get the exact value.
    int scale_limit_count = ceil(log2f(64 / (epsilon * epsilon)));
    float scale_limit = 1 / (pow(2, scale_limit_count));

    // Iterating through each scale up to the limit.
    for (float scale = 0.5f; scale >= scale_limit; scale = scale * 0.5f) {
        if (config.progress_report >= SCALE) std::cout << "Scale change: " << scale << "/" << scale_limit << std::endl;

        // Iterating through each phase in the scale.
        float phase_limit = std::ceil(144.f / (scale * epsilon));
        for (float phase = 1; phase <= phase_limit; phase++) {
            if (config.progress_report >= PHASE) std::cout << "Scale: " << scale << "/" << scale_limit << " Phase: " << phase << "/" << phase_limit << std::endl;

            // Running a single phase of the algorithm to find disjoint augmenting paths.
            // The algPhase function returns true if it is to continue
            std::pair<bool, vector<AugmentingPath>> phase_response = algPhase(stream, &matching, epsilon, scale, config);
            vector<AugmentingPath> disjoint_augmenting_paths = phase_response.second;

            // Outputting relevant information about the augmenting paths found if required
            if (config.progress_report >= VERBOSE && ! disjoint_augmenting_paths.empty()) {
                std::cout << "Augmenting paths found:" << std::endl;
                for (AugmentingPath path : disjoint_augmenting_paths) {
                    std::cout << "Path: " << std::endl;
                    std::cout << "\tTo match: ";
                    for (Edge edge : path.first) {
                        std::cout << edge.first << "->" << edge.second << " ";
                    }
                    std::cout << std::endl;
                    std::cout << "\tTo unmatch: ";
                    for (Edge edge : path.second) {
                        std::cout << edge.first << "->" << edge.second << " ";
                    }
                    std::cout << std::endl;
                }
            }

            // Augmenting the current matching with the augmenting paths found.
            matching.augmentMatching(&disjoint_augmenting_paths);
            // Checking the matching is valid
            matching.verifyMatching();

            // Ending if the phase that has been run has told us to finish early
            if (! phase_response.first) {
                return matching;
            }

            // Scale Skip optimisation - if we find no disjoint augmenting paths after a phase, we skip the current scale.
            if (optimisations && disjoint_augmenting_paths.empty()) {
                if (config.progress_report >= SCALE) std::cout << "SCALE SKIP: No augmenting paths found in phase, skipping the remainder of the scale." << std::endl;
                break;
            }
        }
    }

    return matching;
}