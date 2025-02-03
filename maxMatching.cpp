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

void augment(
    vector<vector<Edge>>* disjoint_augmenting_paths,
    Edge unmatched_arc,
    AvailableFreeNodes* available_free_nodes
) {
    FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(unmatched_arc.first);
    FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(unmatched_arc.second);

    // Creating the augmenting path
    vector<Edge> augmenting_path = {unmatched_arc};
    vector<Edge> root_of_u_to_u = {};
    vector<Edge> v_to_root_of_v = {};

    if (struct_of_u != nullptr) {
        GraphNode *graph_node_of_u = struct_of_u->getGraphNodeFromVertex(unmatched_arc.first);
        root_of_u_to_u = getLeafToRootPath(graph_node_of_u);
        reverse(root_of_u_to_u.begin(), root_of_u_to_u.end());
        struct_of_u->on_hold = true;
    }
    if (struct_of_v != nullptr) {
        GraphNode *graph_node_of_v = struct_of_v->getGraphNodeFromVertex(unmatched_arc.second);
        v_to_root_of_v = getLeafToRootPath(graph_node_of_v);
        struct_of_v->on_hold = true;
    }

    // TODO: Do we actually have to order the augmenting path? Doesn't necessarily seem needed.
    augmenting_path.insert(augmenting_path.begin(), root_of_u_to_u.begin(), root_of_u_to_u.end());
    augmenting_path.insert(augmenting_path.end(), v_to_root_of_v.begin(), v_to_root_of_v.end());

    // TODO: Need to remove vertices? - Does making them on hold do the same thing?

    disjoint_augmenting_paths->emplace_back(augmenting_path);
}


void contractAndAugment(
    Stream* stream,
    AvailableFreeNodes* available_free_nodes,
    vector<vector<Edge>>* disjoint_augmenting_paths
) {
    // TODO: Can both of these steps be completed in one pass?
    // Contraction Step
    Edge edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {

        FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(edge.first);
        FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(edge.second);

        // TODO: Error handling for nullptr

        if (struct_of_u == struct_of_v) {
            // Contract
            struct_of_u->contract(edge);
        }

        // Reading next edge
        edge = stream->readStream();
    }

    // Augmentation Step
    edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {

        // TODO: Error handling for nullptr
        FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(edge.first);
        FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(edge.second);

        if (struct_of_u != struct_of_v) {
            GraphNode* node_of_u = struct_of_u->getGraphNodeFromVertex(edge.first);
            GraphNode* node_of_v = struct_of_v->getGraphNodeFromVertex(edge.second);

            if (node_of_u->isOuterVertex && node_of_v->isOuterVertex) {
                augment(disjoint_augmenting_paths, edge, available_free_nodes);
            }
        }

        // Reading next edge
        edge = stream->readStream();
    }
}

void backtrackStuckStructures(
    AvailableFreeNodes* available_free_nodes
) {
    for (FreeNodeStructure* structure : available_free_nodes->free_node_structures) {
        structure->backtrack();
    }
}

void overtake(
    Edge unmatched_arc, // (u,v)
    Edge matched_arc, // (v,t)
    AvailableFreeNodes* available_free_nodes,
    Matching* matching
) {
    // TODO: Add input check?

    FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(unmatched_arc.first);
    FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(unmatched_arc.second);
    FreeNodeStructure* struct_of_t = available_free_nodes->getFreeNodeStructFromVertex(matched_arc.second);

    Edge matched_arc_using_u = matching->getMatchedEdgeFromVertex(unmatched_arc.first);
    int current_label = matching->getLabel(matched_arc_using_u);

    // Case 1: Our matched_arc is not currently in a structure
    if (struct_of_t == nullptr && struct_of_v == nullptr) {
        GraphVertex vertex_v = GraphVertex(unmatched_arc.second);
        GraphVertex vertex_t = GraphVertex(matched_arc.second);
        vertex_v.parent = struct_of_u->working_node;
        vertex_t.parent = &vertex_v;
        vertex_v.children.insert(&vertex_t);
        struct_of_u->working_node->children.insert(&vertex_v);
        struct_of_u->working_node = &vertex_v;

        vertex_v.isOuterVertex = ! vertex_v.parent->isOuterVertex;
        vertex_t.isOuterVertex = ! vertex_v.isOuterVertex;

        struct_of_u->addGraphNodeToStructure(&vertex_v, &vertex_v);
        struct_of_u->addGraphNodeToStructure(&vertex_t, &vertex_t);

        available_free_nodes->setFreeNodeStructFromVertex(unmatched_arc.second, struct_of_u);
        available_free_nodes->setFreeNodeStructFromVertex(matched_arc.second, struct_of_u);

        matching->setLabel(matched_arc, current_label+1);
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
            // Updating vertex v to now be parented by vertex u
            vertex_u->children.insert(vertex_v);
            vertex_v->parent = vertex_u;

            // Updating the working vertex
            struct_of_t->working_node = vertex_t;

            struct_of_t->modified = true;

            int curr_label;
            if (vertex_u->parent == nullptr) {
                // Vertex_u is the root of the structure;
                curr_label = 0;
            } else {
                Edge parent_matched_edge =  matching->getMatchedEdgeFromVertex(vertex_u->vertex_id);
                // Here we can't just use unmatched_edge.first as it could be part of a blossom.
                curr_label = matching->getLabel(parent_matched_edge);
            }
            updateChildLabels(vertex_v, curr_label+1, matching);

        }
        // Case 2.2: If the matched arc is in a different structure to u, with the unmatched arc (u,v) joining the two structures.
        // I.e. overtaking between two structures.
        else {
            GraphNode* vertex_u = struct_of_u->getGraphNodeFromVertex(unmatched_arc.first);
            GraphNode* vertex_v = struct_of_v->getGraphNodeFromVertex(unmatched_arc.second);

            GraphNode* parent_of_v_in_struct_v = vertex_v->parent;
            parent_of_v_in_struct_v->children.erase(vertex_v);

            // TODO: Remove each node from vertex_to_graph_node -> do we want to move this out?

            vertex_v->parent = vertex_u;
            vertex_v->parent_index = unmatched_arc.first;
            // TODO: clean up if blossom structure

            // NEED TO REMOVE/ADD THE NEW VERTICES TO STRUCTURES

            vertex_u->children.insert(vertex_v);
            // TODO: need to update the vertex_to_children dictionary, can do length measurements at the same time
            // TODO: also need to check if the working vertex has been changed by the overtake

            // TODO: need to clean up if anything linking
            // TODO: update length measurements
        }
    }
}

void extendActivePath(
    Stream* stream,
    Matching* matching,
    AvailableFreeNodes* available_free_nodes,
    vector<vector<Edge>> disjoint_augmenting_paths,
    set<Vertex> removed_vertices
) {
    Edge edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {

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

        // Case 1 - If we have "removed" one of the vertices from the graph, we skip this edge.
        if (
            removed_vertices.find(edge.first) != removed_vertices.end() ||
            removed_vertices.find(edge.second) != removed_vertices.end()
        ) {
            edge = stream->readStream();
            continue;
        }

        // Case 2: If blossom1 is in the same node as blossom2, vertex1 isn't a working vertex
        // or the edge is already matched, we skip this edge.
        FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(edge.first);
        FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(edge.second);
        if (
            (
                struct_of_u != nullptr && struct_of_v != nullptr &&
                struct_of_u->getGraphNodeFromVertex(edge.first) == struct_of_v->getGraphNodeFromVertex(edge.second)
            ) ||
            (struct_of_u != nullptr && struct_of_u->working_node != struct_of_u->getGraphNodeFromVertex(edge.first)) ||
            matching->isInMatching(edge)
        ) {
            edge = stream->readStream();
            continue;
        }

        // Case 3: If the first vertex is in a "marked" or "on hold" structure, we skip this edge.
        if (
            struct_of_u != nullptr &&
            (
                struct_of_u->modified ||
                struct_of_u->on_hold
            )
        ) {
            edge = stream->readStream();
            continue;
        }

        // Case 4: If blossom of u is an outer vertex we contract it.
        if (struct_of_u != nullptr && struct_of_u->getGraphNodeFromVertex(edge.first)->isOuterVertex) {
            if (struct_of_u == struct_of_v) {
                struct_of_u->contract(edge);
            } else {
                augment(&disjoint_augmenting_paths, edge, available_free_nodes);
            }
        }

        // Case 5: Otherwise we attempt to overtake and add the matched edge to the structure.
        else {
            // Getting the edge which is the parent to u.
            Edge matching_using_u = matching->getMatchedEdgeFromVertex(edge.first);
            int distance_to_u = matching->getLabel(matching_using_u);

            Edge matching_using_v = matching->getMatchedEdgeFromVertex(edge.second);
            int distance_to_v = matching->getLabel(matching_using_v);

            if (distance_to_u + 1 < distance_to_v) {
                overtake(edge, matching_using_v, available_free_nodes, matching);
            }
        }

        // Reading next edge
        edge = stream->readStream();
    }

}


vector<vector<Edge>> algPhase(
    Stream* stream,
    Matching* matching,
    float epsilon,
    float scale
) {
    vector<vector<Edge>> disjoint_augmenting_paths = {};

    set<Vertex> removed_vertices = {};

    int path_limit = static_cast<int>(6 / scale) + 1;
    int pass_bundles_max = static_cast<int>(72 / (scale * epsilon));

    AvailableFreeNodes available_free_nodes = AvailableFreeNodes();

    matching->resetLabels();

    for (int pass_bundle = 0; pass_bundle < pass_bundles_max; pass_bundle++) {
        for (FreeNodeStructure* free_node_struct : available_free_nodes.free_node_structures) {
            if (free_node_struct->vertex_to_graph_node.size() >= path_limit) free_node_struct->on_hold = true;
            else free_node_struct->on_hold = false;
            free_node_struct->modified = false;
        }

        extendActivePath(stream, matching, &available_free_nodes, disjoint_augmenting_paths, removed_vertices);
        contractAndAugment(stream, &available_free_nodes, &disjoint_augmenting_paths);
        backtrackStuckStructures(&available_free_nodes);
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
    float epsilon
) {

    Matching matching = get2ApproximateMatching(stream);

    float scale_limit = (epsilon * epsilon) / 64;

    for (float scale = 1.f/2.f; scale <= scale_limit; scale *= 1.f/2.f) {
        float phase_limit = 144.f / (scale * epsilon);

        for (float phase = 1; phase <= phase_limit; phase++) {
            vector<vector<Edge>> disjoint_augmenting_paths = algPhase(stream, &matching, epsilon, scale);
            matching.augmentMatching(&disjoint_augmenting_paths);
        }
    }

    return matching;
}

void testing() {
    GraphVertex* zero = new GraphVertex(0);
    GraphVertex* one = new GraphVertex(1);
    GraphVertex* two = new GraphVertex(2);
    GraphVertex* three = new GraphVertex(3);
    GraphVertex* four = new GraphVertex(4);
    GraphVertex* five = new GraphVertex(5);
    GraphVertex* six = new GraphVertex(6);
    GraphVertex* seven = new GraphVertex(7);
    GraphVertex* eight = new GraphVertex(8);
    GraphVertex* nine = new GraphVertex(9);

    zero->children.insert(one);
    one->parent = zero;
    one->parent_index = 0;
    zero->children.insert(two);
    two->parent = zero;
    one->children.insert(three);
    three->parent = one;
    three->parent_index = 3;
    two->children.insert(four);
    four->parent = two;
    four->parent_index = 2;
    three->children.insert(five);
    five->parent = three;
    five->parent_index = 3;
    four->children.insert(six);
    six->parent = four;
    six->parent_index = 4;
    five->children.insert(seven);
    seven->parent = five;
    seven->parent_index = 5;
    six->children.insert(eight);
    eight->parent = six;
    eight->parent_index = 6;
    seven->children.insert(nine);
    nine->parent = seven;
    nine->parent_index = 7;

    AvailableFreeNodes available_free_nodes;
    FreeNodeStructure* structure = available_free_nodes.createNewStruct(zero);

    std::cout << *structure << std::endl;

    // available_free_nodes.setFreeNodeStructFromVertex(0, structure);
    // available_free_nodes.setFreeNodeStructFromVertex(1, structure);
    // available_free_nodes.setFreeNodeStructFromVertex(2, structure);
    // available_free_nodes.setFreeNodeStructFromVertex(3, structure);
    // available_free_nodes.setFreeNodeStructFromVertex(4, structure);
    // available_free_nodes.setFreeNodeStructFromVertex(5, structure);
    // available_free_nodes.setFreeNodeStructFromVertex(6, structure);
    // available_free_nodes.setFreeNodeStructFromVertex(7, structure);
    // available_free_nodes.setFreeNodeStructFromVertex(8, structure);
    // available_free_nodes.setFreeNodeStructFromVertex(9, structure);

    // structure->free_node_root = zero;
    // structure->working_node = nine;
    //
    // Matching matching;
    // matching.addEdge(make_pair(0, 1));
    // matching.addEdge(make_pair(3, 5));
    // matching.addEdge(make_pair(7, 9));
    // matching.setLabel(make_pair(0, 1), 1);
    //
    // updateChildLabels(zero, 4, &matching);

    //std::cout << *structure << std::endl;
    // std::cout << matching << std::endl;

    // overtake(make_pair(4,5), make_pair(5,7), &available_free_nodes, &matching);
    //
    // std::cout << *structure << std::endl;
    //
    // vector<Edge> path = getLeafToRootPath(nine);
    //
    // for (int i = 0; i < path.size(); i++) {
    //     std::cout << path[i].first << " -> " << path[i].second << std::endl;
    // }
    //
    // available_free_nodes.deleteStructures();

}

int main() {

    //Stream* stream = new StreamFromFile("example.txt");
    Stream* stream = new StreamFromMemory("example.txt");

    testing();

    delete stream;

    return 0;
}
