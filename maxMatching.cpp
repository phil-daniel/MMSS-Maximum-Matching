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
    // TODO: Need to check if matched?
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
        // TODO: Can this be moved into FreeNodeStructure?
        // If the structure is on hold or has been modified then it isn't stuck and hence doesn't
        // need modifying.
        if (structure->on_hold || structure->modified) {
            continue;
        }

        // Updating the working node to the previous outer vertex (i.e. parent of the parent of the current).
        GraphNode* new_working_node = structure->working_node;
        if (new_working_node->parent != nullptr) {
            new_working_node = new_working_node->parent->parent;
        }
        // TODO: Make structure inactive? This should already be done by setting working node to nullptr

        structure->working_node = new_working_node;
    }
}

void extendActivePath(
    Stream* stream,
    Matching* matching,
    float epsilon,
    AvailableFreeNodes* available_free_nodes,
    vector<vector<Edge>> disjoint_augmenting_paths,
    set<Vertex> removed_vertices
) {
    // TODO: Need to create new FreeNodeStructures if needed.

    Edge edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {
        // Case 1 - If we have "removed" one of the vertices from the graph, we skip this edge.
        if (
            removed_vertices.find(edge.first) != removed_vertices.end() ||
            removed_vertices.find(edge.second) != removed_vertices.end()
        ) {
            edge = stream->readStream();
            continue;
        }

        // Case 2: If vertex1 is in the same structure as vertex2, vertex1 isn't a working vertex
        // or the edge is already matched, we skip this edge.
        FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(edge.first);
        FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(edge.second);

        // TODO: Double check "struct_of_u->getGraphNodeFromVertex(edge.first) == struct_of_v->getGraphNodeFromVertex(edge.second)"

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
        // TODO: Can this be simplified since we already know the struct.
        // TODO: nullptr check?
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

        // TODO: CASE 4 OUTER VERTEX
        if (struct_of_u != nullptr && struct_of_u->getGraphNodeFromVertex(edge.first)->isOuterVertex) {
            if (struct_of_u == struct_of_v) {
                // TODO: CONTRACT();
            } else {
                augment(&disjoint_augmenting_paths, edge, available_free_nodes);
            }
        }

        // TODO: CASE 5 - add if case
        else {
            // Getting the edge which is the parent to u.
            Edge matching_using_u = matching->getMatchedEdgeFromVertex(edge.first);
            int distance_to_u = matching->getLabelFromMatchedEdge(matching_using_u);

            Edge matching_using_v = matching->getMatchedEdgeFromVertex(edge.second);
            int distance_to_v = matching->getLabelFromMatchedEdge(matching_using_v);

            if (distance_to_u + 1 < distance_to_v) {
                // TODO: OVERTAKE();
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
            if (free_node_struct->vertices_count >= path_limit) free_node_struct->on_hold = true;
            // TODO: Need to be updating the vertex count
            else free_node_struct->on_hold = false;
            free_node_struct->modified = false;
        }

        // TODO: IMPLEMENT ALGORITHM HERE!
        //extendActivePath(stream, matching, epsilon, &available_free_nodes, disjoint_augmenting_paths, removed_vertices);
        //contractAndAugment(stream, &available_free_nodes, &disjoint_augmenting_paths);
        //backtrackStuckStructures(&available_free_nodes);
    }

    return disjoint_augmenting_paths;
}

Matching augmentMatching(
    Matching matching,
    vector<vector<Edge>>* disjoint_augmenting_paths
) {
    // Takes a vector (list) of disjoint augmenting paths and adds them to the matching.

    // TODO: change to pointer rather than returning

    for (vector<Edge> augmenting_path : (*disjoint_augmenting_paths)) {
        for (Edge edge : augmenting_path) {
            // If the edge isn't in the matching, we add it to the matching.
            // Otherwise we remove it from the matching.
            if (matching.isInMatching(edge)) {
                matching.removeEdge(edge);
            } else {
                matching.addEdge(edge);
            }
        }
    }

    return matching;
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
        float phase_limit = 144 / (scale * epsilon);

        for (float phase = 1; phase <= phase_limit; phase++) {
            vector<vector<Edge>> disjoint_augmenting_paths = {};
            // TODO: Implement alg phase.
            algPhase(stream, &matching, epsilon, scale);
            matching = augmentMatching(matching, &disjoint_augmenting_paths);
        }
    }

    return matching;
}

void overtake(
    Edge unmatched_arc, // (u,v)
    Edge matched_arc, // (v,t)
    AvailableFreeNodes* available_free_nodes
) {
    // TODO: Add input check?
    // TODO: How do we know its a matched_arc?

    // TODO: Need to do length measurement updates somewhere?

    FreeNodeStructure* struct_of_u = available_free_nodes->getFreeNodeStructFromVertex(unmatched_arc.first);
    FreeNodeStructure* struct_of_v = available_free_nodes->getFreeNodeStructFromVertex(unmatched_arc.second);
    FreeNodeStructure* struct_of_t = available_free_nodes->getFreeNodeStructFromVertex(matched_arc.second);

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

        struct_of_u->addGraphNodeToVertex(unmatched_arc.second, &vertex_v);
        struct_of_u->addGraphNodeToVertex(matched_arc.second, &vertex_t);

        available_free_nodes->setFreeNodeStructFromVertex(unmatched_arc.second, struct_of_u);
        available_free_nodes->setFreeNodeStructFromVertex(matched_arc.second, struct_of_u);

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

            // TODO: need to sort out blossom overtakes children

            GraphNode* current_parent_of_v = vertex_v->parent;
            // Removing vertex v from the set of it's parent's children.
            current_parent_of_v->children.erase(vertex_v);
            // Updating vertex v to now be parented by vertex u
            vertex_u->children.insert(vertex_v);
            vertex_v->parent = vertex_u;

            // Updating the working vertex
            struct_of_t->working_node = vertex_t;

            struct_of_t->modified = true;

            // TODO: Update length measurements?

        }
        // Case 2.2: If the matched arc is in a different structure to u, with the unmatched arc (u,v) joining the two structures.
        // I.e. overtaking between two structures.
        else {
            GraphNode* vertex_u = struct_of_u->getGraphNodeFromVertex(unmatched_arc.first);
            GraphNode* vertex_v = struct_of_v->getGraphNodeFromVertex(unmatched_arc.second);

            GraphNode* parent_of_v_in_struct_v = vertex_v->parent;
            parent_of_v_in_struct_v->children.erase(vertex_v);
            // TODO: clean up if blossom structure

            vertex_u->children.insert(vertex_v);
            // TODO: need to update the vertex_to_children dictionary, can do length measurements at the same time
            // TODO: also need to check if the working vertex has been changed by the overtake

            // TODO: need to clean up if anything linking
            // TODO: update length measurements
        }
    }
}

void testing() {
    GraphVertex zero = GraphVertex(0);
    GraphVertex one = GraphVertex(1);
    GraphVertex two = GraphVertex(2);
    GraphVertex three = GraphVertex(3);
    GraphVertex four = GraphVertex(4);
    GraphVertex five = GraphVertex(5);
    GraphVertex six = GraphVertex(6);
    GraphVertex seven = GraphVertex(7);
    GraphVertex eight = GraphVertex(8);
    GraphVertex nine = GraphVertex(9);

    zero.children.insert(&one);
    one.parent = &zero;
    zero.children.insert(&two);
    two.parent = &zero;
    one.children.insert(&three);
    three.parent = &one;
    two.children.insert(&four);
    four.parent = &two;
    three.children.insert(&five);
    five.parent = &three;
    four.children.insert(&six);
    six.parent = &four;
    five.children.insert(&seven);
    seven.parent = &five;
    six.children.insert(&eight);
    eight.parent = &six;
    seven.children.insert(&nine);
    nine.parent = &seven;

    AvailableFreeNodes available_free_nodes;
    FreeNodeStructure* structure = available_free_nodes.createNewStruct(&zero);
    structure->addGraphNodeToVertex(1, &one);
    structure->addGraphNodeToVertex(2, &two);
    structure->addGraphNodeToVertex(3, &three);
    structure->addGraphNodeToVertex(4, &four);
    structure->addGraphNodeToVertex(5, &five);
    structure->addGraphNodeToVertex(6, &six);
    structure->addGraphNodeToVertex(7, &seven);
    structure->addGraphNodeToVertex(8, &eight);
    structure->addGraphNodeToVertex(9, &nine);

    available_free_nodes.setFreeNodeStructFromVertex(0, structure);
    available_free_nodes.setFreeNodeStructFromVertex(1, structure);
    available_free_nodes.setFreeNodeStructFromVertex(2, structure);
    available_free_nodes.setFreeNodeStructFromVertex(3, structure);
    available_free_nodes.setFreeNodeStructFromVertex(4, structure);
    available_free_nodes.setFreeNodeStructFromVertex(5, structure);
    available_free_nodes.setFreeNodeStructFromVertex(6, structure);
    available_free_nodes.setFreeNodeStructFromVertex(7, structure);
    available_free_nodes.setFreeNodeStructFromVertex(8, structure);
    available_free_nodes.setFreeNodeStructFromVertex(9, structure);

    structure->free_node_root = &zero;
    structure->working_node = &nine;

    overtake(make_pair(4,5), make_pair(5,7), &available_free_nodes);

    std::cout << *structure << std::endl;

    vector<Edge> path = getLeafToRootPath(&nine);

    for (int i = 0; i < path.size(); i++) {
        std::cout << path[i].first << " -> " << path[i].second << std::endl;
    }

    delete structure;

}

int main() {

    //Stream* stream = new StreamFromFile("example.txt");
    Stream* stream = new StreamFromMemory("example.txt");

    //Matching matching = get2ApproximateMatching(stream);

    //std::cout << "Matching size: " << matching.size() << std::endl;

    testing();

    delete stream;

    return 0;
}
