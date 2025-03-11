#include <fstream>
#include <iostream>
#include <set>

#include "types.h"

#include "Stream/Stream.h"
#include "Stream/StreamFromFile.h"
#include "Stream/StreamFromMemory.h"
#include "Structures/AvailableFreeNodes.h"

#include "Structures/FreeNodeStructure.h"
#include "Structures/GraphStructure/GraphBlossom.h"
#include "Structures/GraphStructure/GraphVertex.h"
#include "Structures/Matching.h"

using namespace std;

vector<Edge> getLeafToRootPath(
    GraphNode* leaf
) {
    vector<Edge> path = {};
    GraphNode* current = leaf;
    GraphNode* parent = leaf->parent;

    while (parent != nullptr) {
        int current_value, parent_value;

        current_value = current->vertex_id;

        if (! parent->isBlossom) {
            parent_value = parent->vertex_id;
        } else {
            parent_value = current->parent_index;
        }

        path.emplace_back(current_value, parent_value);

        current = parent;
        parent = parent->parent;
    }

    return path;
}

void updateChildLabels(GraphNode* parent_matched_vertex, int new_label, Matching* matching) {
    vector<GraphNode*> current_level = {parent_matched_vertex};
    while (!current_level.empty()) {
        // Need to go two levels down each time, otherwise we will be labelling unmatched edges
        vector<GraphNode*> new_level = {};
        for (GraphNode* node : current_level) {
            for (GraphNode* child : node->children) {
                // TODO: Should only have 1 child in for a matched edge.

                // TODO: double check this logic is correct especially for blossoms
                Edge matched_edge = make_pair(child->parent_index, child->vertex_id);
                matching->setLabel(matched_edge, new_label);
                new_label++;

                for (GraphNode* grandchild : child->children) {
                    new_level.emplace_back(grandchild);
                }
            }
        }
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

    GraphNode *graph_node_of_u = struct_of_u->getGraphNodeFromVertex(unmatched_arc.first);
    GraphNode *graph_node_of_v = struct_of_v->getGraphNodeFromVertex(unmatched_arc.second);
    AugmentingPath new_augmentation = getAugmentation(graph_node_of_u, graph_node_of_v, unmatched_arc.first, unmatched_arc.second, matching);

    struct_of_u->removed = true;
    struct_of_v->removed = true;

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

        // If the two vertices are in the same non-removed structure.
        if (
            struct_of_u != nullptr && struct_of_u == struct_of_v &&
            ! struct_of_u->removed && ! struct_of_v->removed
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
        int contractions_in_last_iteration = -1;
        while (contractions_in_last_iteration != 0) {
            contractions_in_last_iteration = 0;
            for (Edge edge_in_struct : pair.second) {
                GraphNode* node_of_u = pair.first->getGraphNodeFromVertex(edge_in_struct.first);
                GraphNode* node_of_v = pair.first->getGraphNodeFromVertex(edge_in_struct.second);

                if (node_of_u != node_of_v &&  node_of_u->isOuterVertex && node_of_v->isOuterVertex) {
                    pair.first->contract(edge_in_struct);
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

        // TODO: Error handling for nullptr
        FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(edge.first);
        FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(edge.second);

        // TODO: temp added nullptr handling here
        if (
            struct_of_u != nullptr && struct_of_v != nullptr &&
            ! struct_of_u->removed && ! struct_of_v->removed &&
            struct_of_u != struct_of_v
        ) {

            GraphNode* node_of_u = struct_of_u->getGraphNodeFromVertex(edge.first);
            GraphNode* node_of_v = struct_of_v->getGraphNodeFromVertex(edge.second);
            if (node_of_u->isOuterVertex && node_of_v->isOuterVertex && ! (struct_of_u->removed || struct_of_v->removed)) {
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
        if (structure->on_hold || structure->modified || structure->removed || structure->working_node == nullptr) {
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
    // TODO: Add input check?

    FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(unmatched_arc.first);
    FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(unmatched_arc.second);
    FreeNodeStructure* struct_of_t = available_free_nodes->getFreeNodeStructFromVertex(matched_arc.second);

    Edge matched_arc_using_u = matching->getMatchedEdgeFromVertex(unmatched_arc.first);
    int current_label = 0;
    if (matched_arc_using_u.first != -1) {
        // If the edge doesn't exist them getMatchedEdgeFromVertex will return (-1,-1)
        current_label = matching->getLabel(matched_arc_using_u);
    }
    if (unmatched_arc.second != matched_arc.first) {
        matched_arc = make_pair(matched_arc.second, matched_arc.first);
    }
    // TODO: needs nullptr check and merge with below

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

        vertex_v->isOuterVertex = ! vertex_v->parent->isOuterVertex;
        vertex_t->isOuterVertex = ! vertex_v->isOuterVertex;

        available_free_nodes->addNodeToStruct(vertex_v, vertex_v, struct_of_u);

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
            // TODO: Need nullptr checks?
            // Here we know struct_of_u == struct_of_v == struct_of_v
            GraphNode* vertex_u = struct_of_t->getGraphNodeFromVertex(unmatched_arc.first);
            GraphNode* vertex_v = struct_of_t->getGraphNodeFromVertex(matched_arc.first);
            GraphNode* vertex_t = struct_of_t->getGraphNodeFromVertex(matched_arc.second);

            GraphNode* current_parent_of_v = vertex_v->parent;
            // Removing vertex v from the set of it's parent's children.
            current_parent_of_v->children.erase(vertex_v);

            // TODO: Check these are correct
            if (current_parent_of_v->isBlossom) {
                GraphBlossom* parent_blossom = dynamic_cast<GraphBlossom *>(current_parent_of_v);
                parent_blossom->outsideBlossomToIn.erase(vertex_v);
            }

            if (vertex_v->isBlossom) {
                GraphBlossom* blossom_v = dynamic_cast<GraphBlossom *>(vertex_v);
                blossom_v->recursivelyAddOutsideBlossomToIn(vertex_u, unmatched_arc.second);
                //blossom_v->outsideBlossomToIn[vertex_u] = unmatched_arc.second;
                blossom_v->outsideBlossomToIn.erase(current_parent_of_v);
            }

            if (vertex_u->isBlossom) {
                GraphBlossom* blossom_u = dynamic_cast<GraphBlossom *>(vertex_u);
                blossom_u->recursivelyAddOutsideBlossomToIn(vertex_v, unmatched_arc.first);
                //blossom_u->outsideBlossomToIn[vertex_v] = unmatched_arc.first;
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

            // TODO: Check these correct
            if (parent_of_v_in_struct_v->isBlossom) {
                GraphBlossom* parent_blossom = dynamic_cast<GraphBlossom *>(parent_of_v_in_struct_v);
                parent_blossom->outsideBlossomToIn.erase(vertex_v);
            }

            if (vertex_v->isBlossom) {
                GraphBlossom* blossom_v = dynamic_cast<GraphBlossom *>(vertex_v);
                blossom_v->recursivelyAddOutsideBlossomToIn(vertex_u, unmatched_arc.second);
                //blossom_v->outsideBlossomToIn[vertex_u] = unmatched_arc.second;
                blossom_v->outsideBlossomToIn.erase(parent_of_v_in_struct_v);
            }

            if (vertex_u->isBlossom) {
                GraphBlossom* blossom_u = dynamic_cast<GraphBlossom *>(vertex_u);
                blossom_u->recursivelyAddOutsideBlossomToIn(vertex_v, unmatched_arc.first);
                //blossom_u->outsideBlossomToIn[vertex_v] = unmatched_arc.first;
            }


            vertex_v->parent = vertex_u;
            vertex_v->parent_index = unmatched_arc.first;

            vertex_u->children.insert(vertex_v);

            available_free_nodes->removeNodeFromStruct(vertex_v, struct_of_u);
            available_free_nodes->addNodeToStruct(vertex_v, vertex_v, struct_of_u);

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
        // TODO: Do we need to do each direction of an arc?

        // Checking whether you need to create a new FreeNodeStructure for these two vertices. Requirements:
        // - Is the vertex not involved in a matching
        // - Does the vertex have an existing structure
        if (
            matching->getMatchedEdgeFromVertex(edge.first).first == -1 &&
            available_free_nodes->getFreeNodeStructFromVertex(edge.first) == nullptr
        ) {
            GraphVertex* new_vertex_u = new GraphVertex(edge.first);
            available_free_nodes->createNewStruct(new_vertex_u);
        }
        if (
            matching->getMatchedEdgeFromVertex(edge.second).first == -1 &&
            available_free_nodes->getFreeNodeStructFromVertex(edge.second) == nullptr
        ) {
            GraphVertex* new_vertex_v = new GraphVertex(edge.second);
            available_free_nodes->createNewStruct(new_vertex_v);
        }

        FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(edge.first);
        FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(edge.second);
        if (struct_of_u == nullptr) {
            edge = stream->readStream();
            continue;
        }
        // Case 1 - If we have "removed" one of the vertices from the graph, we skip this edge.
        if (struct_of_u->removed || (struct_of_v != nullptr && struct_of_v->removed) ) {
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

        // Case 4: If blossom of u is an outer vertex we contract it.
        if (struct_of_u != nullptr && struct_of_v != nullptr &&
            struct_of_u->getGraphNodeFromVertex(edge.first)->isOuterVertex &&
            struct_of_v->getGraphNodeFromVertex(edge.second)->isOuterVertex
        ) {
            if (struct_of_u == struct_of_v) {
                // in the same structure
                if (struct_of_u->getGraphNodeFromVertex(edge.first) != struct_of_u->getGraphNodeFromVertex(edge.second)) {
                    // are not the same node
                    if (struct_of_v->getGraphNodeFromVertex(edge.second)->isOuterVertex) {
                        // both are outer vertices
                        struct_of_u->contract(edge);

                        *operations_completed += 1;
                        if (config.progress_report >= VERBOSE) {
                            std::cout << "Contract: Struct " << struct_of_u->free_node_root->vertex_id;
                            std::cout << " on edge" << edge.first << "->" << edge.second << std::endl;
                        }
                    }
                }
            } else {
                if (struct_of_v->getGraphNodeFromVertex(edge.second)->isOuterVertex) {
                    augment(disjoint_augmenting_paths, edge, available_free_nodes, matching);

                    *operations_completed += 1;
                    if (config.progress_report >= VERBOSE) {
                        std::cout << "Augment: Struct " << struct_of_u->free_node_root->vertex_id;
                        std::cout << " and Struct "<< struct_of_v->free_node_root->vertex_id;
                        std::cout << " on edge" << edge.first << "->" << edge.second << std::endl;
                    }
                }
            }
        }

        // Case 5: Otherwise we attempt to overtake and add the matched edge to the structure.
        else {
            // Getting the edge which is the parent to u.
            Edge matching_using_u = matching->getMatchedEdgeFromVertex(edge.first);
            int distance_to_u = 0;
            // If u is a free vertex it won't have a matching connecting to it.
            if (matching_using_u.first != -1) {
                distance_to_u = matching->getLabel(matching_using_u);
            }

            Edge matching_using_v = matching->getMatchedEdgeFromVertex(edge.second);
            int distance_to_v = matching->getLabel(matching_using_v);

            // TODO: Move check of corrected matched edge somewhere else?
            if (distance_to_u + 1 < distance_to_v && matching_using_v.first != -1) {
                overtake(edge, matching_using_v, available_free_nodes, matching, config);

                *operations_completed += 1;
            }
        }

        // Reading next edge
        edge = stream->readStream();
    }

}


vector<AugmentingPath> algPhase(
    Stream* stream,
    Matching* matching,
    float epsilon,
    float scale,
    Config config
) {
    vector<AugmentingPath> disjoint_augmenting_paths = {};

    set<Vertex> removed_vertices = {};

    int path_limit = static_cast<int>(6 / scale) + 1;
    int pass_bundles_max = static_cast<int>(72 / (scale * epsilon));

    AvailableFreeNodes available_free_nodes = AvailableFreeNodes();

    matching->resetLabels();

    for (int pass_bundle = 0; pass_bundle < pass_bundles_max; pass_bundle++) {
        int operations_completed = 0;

        if (config.progress_report >= PASS_BUNDLE) std::cout << "Pass bundle: " << pass_bundle << "/" << pass_bundles_max << std::endl;
        for (FreeNodeStructure* free_node_struct : available_free_nodes.free_node_structures) {
            if (free_node_struct->vertex_to_graph_node.size() >= path_limit) free_node_struct->on_hold = true;
            else free_node_struct->on_hold = false;
            free_node_struct->modified = false;
        }

        extendActivePath(stream, matching, &available_free_nodes, &disjoint_augmenting_paths, config, &operations_completed);
        contractAndAugment(stream, &available_free_nodes, &disjoint_augmenting_paths, matching, config, &operations_completed);
        backtrackStuckStructures(&available_free_nodes, config, &operations_completed);

        if (config.optimisation_level >= PHASE_SKIP && config.progress_report >= PASS_BUNDLE && operations_completed == 0) {
            std::cout << "PHASE SKIP: No operations completed in the current pass bundle, skipping the remainder of the phase." << std::endl;
            break;
        }

    }

    return disjoint_augmenting_paths;
}

Matching get2ApproximateMatching(
    Stream* stream
) {
    std::set<int> involved_in_matching = set<int>();
    Matching matching;

    Edge edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {

        // 2-approximation within 1 pass, only adds to the matching if both vertices are currently not in the matching.
        if (
            involved_in_matching.find(edge.first) == involved_in_matching.end() &&
            involved_in_matching.find(edge.second) == involved_in_matching.end()
        ) {
            involved_in_matching.insert(edge.first);
            involved_in_matching.insert(edge.second);
            matching.addEdge(edge);
        }

        // Reading next edge
        edge = stream->readStream();
    }

    return matching;
}

Matching algorithm(
    Stream* stream,
    float epsilon,
    int progress_report = 3,
    int optimisation_level = 3
) {
    // Setting up the config structure.
    Config config;
    if (progress_report < NO_OUTPUT) config.progress_report = NO_OUTPUT;
    else if (progress_report > VERBOSE) config.progress_report = VERBOSE;
    else config.progress_report = static_cast<ProgressReport>(progress_report);

    if (optimisation_level < NO_OUTPUT) config.optimisation_level = NO_OPTIMISATION;
    else if (optimisation_level > PHASE_SKIP) config.optimisation_level = PHASE_SKIP;
    else config.optimisation_level = static_cast<OptimisationLevel>(optimisation_level);

    // Computing a 2-approximate matching for the graph
    Matching matching = get2ApproximateMatching(stream);
    // Outputting the information about the matching if required
    if (config.progress_report >= SCALE) std::cout << "2 approximation size: " << matching.matched_edges.size() << std::endl;
    if (config.progress_report >= PHASE) std::cout << matching << std::endl;

    float scale_limit = (epsilon * epsilon) / 64;

    for (float scale = 1.f/2.f; scale >= scale_limit; scale = scale * 1.f / 2.f) {
        if (config.progress_report >= SCALE) std::cout << "Scale change: " << scale << "/" << scale_limit << std::endl;
        float phase_limit = 144.f / (scale * epsilon);

        for (float phase = 1; phase <= phase_limit; phase++) {

            if (config.progress_report >= PHASE) std::cout << "Scale: " << scale << "/" << scale_limit << " Phase: " << phase << "/" << phase_limit << std::endl;
            vector<AugmentingPath> disjoint_augmenting_paths = algPhase(stream, &matching, epsilon, scale, config);

            if (config.progress_report >= PHASE && ! disjoint_augmenting_paths.empty()) {
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

            if (config.optimisation_level >= SCALE_SKIP && config.progress_report >= SCALE && disjoint_augmenting_paths.empty()) {
                std::cout << "SCALE SKIP: No augmenting paths found in phase, skipping the remainder of the scale." << std::endl;
                break;
            }

            matching.augmentMatching(&disjoint_augmenting_paths);
            matching.verifyMatching();
        }
    }

    return matching;
}

int main() {

    //Stream* stream = new StreamFromFile("example.txt");
    Stream* stream = new StreamFromMemory("test_graph.txt");

    Matching matching = algorithm(stream, 0.25, 3, 3);
    std::cout << matching << std::endl;
    std::cout << "Total number of passes: " << stream->number_of_passes << std::endl;

    delete stream;

    return 0;
}
