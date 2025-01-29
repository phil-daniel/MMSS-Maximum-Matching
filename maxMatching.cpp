#include <fstream>
#include <iostream>
#include <set>

// Required for hashing pairs
#include <boost/container_hash/hash.hpp>

#include "Stream/Stream.h"
#include "Stream/StreamFromFile.h"
#include "Stream/StreamFromMemory.h"

#include "Structures/FreeNodeStructure.h"
#include "Structures/GraphStructure/GraphBlossom.h"
#include "Structures/GraphStructure/GraphVertex.h"

using namespace std;

typedef int Vertex;
typedef pair<Vertex, Vertex> Edge;
typedef set<Edge> Matching;
typedef unordered_map<Edge, int, boost::hash<Edge>> MatchingToLabel;

void contractAndAugment(
    Stream* stream,
    unordered_map<Vertex, FreeNodeStructure*>* vertex_to_free_node_struct
) {
    // TODO: Can both of these steps be completed in one pass?
    // Contraction Step
    Edge edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {

        // TODO: Error handling for nullptr
        FreeNodeStructure* struct_of_u = vertex_to_free_node_struct->at(edge.first);
        FreeNodeStructure* struct_of_v = vertex_to_free_node_struct->at(edge.second);

        if (struct_of_u == struct_of_v) {
            // TODO: CONTRACT();
        }

        // Reading next edge
        edge = stream->readStream();
    }

    // Augmentation Step
    edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {

        // TODO: Error handling for nullptr
        FreeNodeStructure* struct_of_u = vertex_to_free_node_struct->at(edge.first);
        FreeNodeStructure* struct_of_v = vertex_to_free_node_struct->at(edge.second);

        if (struct_of_u != struct_of_v) {
            GraphNode* node_of_u = struct_of_u->getGraphNodeFromVertex(edge.first);
            GraphNode* node_of_v = struct_of_v->getGraphNodeFromVertex(edge.second);

            if (node_of_u->isOuterVertex && node_of_v->isOuterVertex) {
                // TODO: AUGMENT();
            }
        }

        // Reading next edge
        edge = stream->readStream();
    }
}

void backtrackStuckStructures(
    vector<FreeNodeStructure *> free_node_structs
) {
    for (FreeNodeStructure* structure : free_node_structs) {
        // If the structure is on hold or has been modified then it isn't stuck and hence doesn't
        // need modifying.
        if (structure->on_hold || structure->modified) {
            continue;
        }

        // Updating the working node to the previous outer vertex (i.e. parent of the parent of the current).
        GraphNode* new_working_node = structure->working_vertex;
        if (new_working_node->parent != nullptr) {
            new_working_node = new_working_node->parent->parent;
        }
        // TODO: Make structure inactive? This should already be done by setting working node to nullptr

        structure->working_vertex = new_working_node;
    }
}

vector<Edge> getLeafToRootPath(
    GraphNode* leaf
) {
    vector<Edge> path = {};
    GraphNode* current = leaf;
    GraphNode* parent = leaf->parent;

    while (parent != nullptr) {
        int current_value, parent_value;

        if (! current->isBlossom) {
            GraphVertex* vertex_pointer = dynamic_cast<GraphVertex *>(current);
            current_value = vertex_pointer->vertex_id;
        } else {
            GraphBlossom* blossom_pointer = dynamic_cast<GraphBlossom *>(current);
            current_value = blossom_pointer->vertexToParent;
        }

        if (! parent->isBlossom) {
            GraphVertex* vertex_pointer = dynamic_cast<GraphVertex *>(parent);
            parent_value = vertex_pointer->vertex_id;
        } else {
            GraphBlossom* blossom_pointer = dynamic_cast<GraphBlossom *>(parent);
            // TODO: Add some error handling here in case not in dictionary?
            parent_value = blossom_pointer->child_to_blossom_vertex[current];
        }

        path.emplace_back(make_pair(current_value, parent_value));

        current = parent;
        parent = parent->parent;
    }

    return path;
}

vector<vector<Edge>> augment(
    vector<vector<Edge>> disjoint_augmenting_paths,
    Edge unmatched_arc,
    unordered_map<Vertex, FreeNodeStructure*> vertex_to_free_node_struct
) {
    FreeNodeStructure* struct_of_u = nullptr;
    if (vertex_to_free_node_struct.count(unmatched_arc.first) > 0) {
        struct_of_u = vertex_to_free_node_struct[unmatched_arc.first];
    }
    FreeNodeStructure* struct_of_v = nullptr;
    if (vertex_to_free_node_struct.count(unmatched_arc.first) > 0) {
        struct_of_v = vertex_to_free_node_struct[unmatched_arc.first];
    }

    // TODO: Add check here to ensure augmentation only happens at the correct point

    // Creating the augmenting path
    vector<Edge> augmenting_path = {unmatched_arc};
    GraphNode* graph_node_of_u;
    GraphNode* graph_node_of_v;
    vector<Edge> root_of_u_to_u = {};
    vector<Edge> v_to_root_of_v = {};

    if (struct_of_u != nullptr) {
        graph_node_of_u = struct_of_u->getGraphNodeFromVertex(unmatched_arc.first);
        root_of_u_to_u = getLeafToRootPath(graph_node_of_u);
        reverse(root_of_u_to_u.begin(), root_of_u_to_u.end());
    }
    if (struct_of_v != nullptr) {
        graph_node_of_v = struct_of_v->getGraphNodeFromVertex(unmatched_arc.second);
        v_to_root_of_v = getLeafToRootPath(graph_node_of_v);
    }

    // Do we actually have to order the augmenting path? Doesn't necessarily seem needed.
    augmenting_path.insert(augmenting_path.begin(), root_of_u_to_u.begin(), root_of_u_to_u.end());
    augmenting_path.insert(augmenting_path.end(), v_to_root_of_v.begin(), v_to_root_of_v.end());

    // TODO: Need to remove vertices

    return disjoint_augmenting_paths;
}

void overtake(
    Edge unmatched_arc, // (u,v)
    Edge matched_arc, // (v,t)
    int k,
    unordered_map<Vertex, FreeNodeStructure*>* vertex_to_free_node_struct
) {
    // TODO: Add input check?

    // TODO: Return failure if not in a struct -> need it to return a nullptr
    FreeNodeStructure* struct_of_u = (*vertex_to_free_node_struct)[unmatched_arc.first];
    FreeNodeStructure* struct_of_v = (*vertex_to_free_node_struct)[unmatched_arc.second];
    FreeNodeStructure* struct_of_t = (*vertex_to_free_node_struct)[matched_arc.second];

    // Case 1: Our matched_arc is not currently in a structure
    if (struct_of_t == nullptr && struct_of_v == nullptr) {
        // TODO: Are we creating these graph vertexes in the correct way? I.e. do we need to do "new" and the free.
        GraphVertex vertex_v = GraphVertex(unmatched_arc.second);
        GraphVertex vertex_t = GraphVertex(matched_arc.second);
        vertex_v.parent = struct_of_u->working_vertex;
        vertex_t.parent = &vertex_v;
        vertex_v.children.insert(&vertex_t);
        struct_of_u->working_vertex->children.insert(&vertex_v);
        struct_of_u->working_vertex = &vertex_v;
        // TODO: Is this the best way of updating the outer/inner
        vertex_v.isOuterVertex = ! vertex_v.parent->isOuterVertex;
        vertex_t.isOuterVertex = ! vertex_v.isOuterVertex;

        struct_of_u->addGraphNodeToVertex(unmatched_arc.second, &vertex_v);
        struct_of_u->addGraphNodeToVertex(matched_arc.second, &vertex_t);

        (*vertex_to_free_node_struct)[unmatched_arc.second] = struct_of_u;
        (*vertex_to_free_node_struct)[matched_arc.second] = struct_of_u;
    }

    // Case 2: If our matched_arc is currently in a structure.
    else {
        // Case 2.1: If the matched arc is in the same structure as the vertex u, with the matched arc joining them.
        // I.e. overtaking within a single structure.
        if (struct_of_u == struct_of_t) {
            // TODO: Need nullptr checks?
            // Here we know struct_of_u == struct_of_v == struct_of_v
            GraphNode* vertex_u = struct_of_t->getGraphNodeFromVertex(matched_arc.first);
            GraphNode* vertex_v = struct_of_t->getGraphNodeFromVertex(matched_arc.first);
            GraphNode* vertex_t = struct_of_t->getGraphNodeFromVertex(matched_arc.first);

            GraphNode* current_parent_of_v = vertex_v->parent;
            // Removing vertex v from the set of it's parent's children.
            current_parent_of_v->children.erase(vertex_v);
            // Updating vertex v to now be parented by vertex u
            vertex_u->children.insert(vertex_v);
            vertex_v->parent = vertex_u;

            // Updating the working vertex
            struct_of_t->working_vertex = vertex_t;

            struct_of_t->modified = true;

            // TODO: Do we need to update length measurements?

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


void extendActivePath(
    Stream* stream,
    Matching matching,
    float epsilon,
    vector<FreeNodeStructure> free_node_structs,
    vector<vector<Edge>> disjoint_augmenting_paths,
    set<Vertex> removed_vertices,
    MatchingToLabel matching_to_label,
    unordered_map<int, Edge> vertexToMatchedEdge
) {

    unordered_map<Vertex, FreeNodeStructure*> vertex_to_free_node_struct;

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

        // Case 2: If vertex1 is in the same blossom as vertex2, vertex1 isn't a working vertex
        // or the edge is already matched, we skip this edge.
        FreeNodeStructure* struct_of_u = nullptr;
        if (vertex_to_free_node_struct.count(edge.first) > 0) {
            struct_of_u = vertex_to_free_node_struct[edge.first];
        }
        FreeNodeStructure* struct_of_v = nullptr;
        if (vertex_to_free_node_struct.count(edge.first) > 0) {
            struct_of_v = vertex_to_free_node_struct[edge.first];
        }

        if (
            (
                struct_of_u != nullptr && struct_of_v != nullptr &&
                struct_of_u->getGraphNodeFromVertex(edge.first) == struct_of_v->getGraphNodeFromVertex(edge.second)
            ) ||
            (struct_of_u != nullptr && struct_of_u->working_vertex != struct_of_u->getGraphNodeFromVertex(edge.first)) ||
            matching.find(edge) != matching.end()
        ) {
            edge = stream->readStream();
            continue;
        }

        // Case 3: If the first vertex is in a "marked" or "on hold" structure, we skip this edge.
        if (
            vertex_to_free_node_struct.count(edge.first) &&
            (
                vertex_to_free_node_struct[edge.first]->modified ||
                vertex_to_free_node_struct[edge.first]->on_hold
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
                disjoint_augmenting_paths = augment(disjoint_augmenting_paths, edge, vertex_to_free_node_struct);
            }
        }

        // TODO: CASE 5
        else {
            // Getting the edge which is the parent to u.
            Edge matching_using_u = vertexToMatchedEdge[edge.first];
            int distance_to_u = matching_to_label[matching_using_u];

            Edge matching_using_v = vertexToMatchedEdge[edge.second];
            int distance_to_v = matching_to_label[matching_using_v];

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
    Matching matching,
    float epsilon,
    float scale
) {
    vector<vector<Edge>> disjoint_augmenting_paths = {};

    set<Vertex> removed_vertices = {};

    MatchingToLabel matching_to_label;

    int path_limit = static_cast<int>(6 / scale) + 1;
    int pass_bundles_max = static_cast<int>(72 / (scale * epsilon));

    vector<FreeNodeStructure> free_node_structs;

    // Setting the current distance for each matched edge to infinity.
    for (Edge edge : matching) {
        matching_to_label[edge] = numeric_limits<int>::max();
    }

    for (int pass_bundle = 0; pass_bundle < pass_bundles_max; pass_bundle++) {
        for (FreeNodeStructure free_node_struct : free_node_structs) {
            if (free_node_struct.vertices_count >= path_limit) free_node_struct.on_hold = true;
            else free_node_struct.on_hold = false;
            free_node_struct.modified = false;
        }

        // TODO: IMPLEMENT ALGORITHM HERE!
        // EXTEND-ACTIVE-PATH()
        // CONTRACT-AND-AUGMENT()
        //backtrackStuckStructures(&free_node_structs);
    }

    return disjoint_augmenting_paths;
}

Matching augmentMatching(
    Matching matching,
    vector<vector<Edge>> disjoint_augmenting_paths
) {
    // Takes a vector (list) of disjoint augmenting paths and adds them to the matching.

    for (vector<Edge> augmenting_path : disjoint_augmenting_paths) {
        for (Edge edge : augmenting_path) {
            // If the edge isn't in the matching, we add it to the matching.
            // Otherwise we remove it from the matching.
            if (matching.find(edge) == matching.end()) {
                matching.insert(edge);
            } else {
                matching.erase(edge);
            }
        }
    }

    return matching;
}

Matching algorithm(
    Stream* stream,
    Matching matching,
    float epsilon
) {

    float scale_limit = (epsilon * epsilon) / 64;

    for (float scale = 1/2; scale <= scale_limit; scale *= 1/2) {
        float phase_limit = 144 / (scale * epsilon);

        for (float phase = 1; phase <= phase_limit; phase++) {
            vector<vector<Edge>> disjoint_augmenting_paths = {};
            matching = augmentMatching(matching, disjoint_augmenting_paths);
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
            matching.insert(edge);
        }

        // Reading next edge
        edge = stream->readStream();
    }

    return matching;
}

void contract(
    Edge unmatched_arc,
    FreeNodeStructure* structure
) {
    GraphNode* node_of_u = structure->getGraphNodeFromVertex(unmatched_arc.first);
    GraphNode* node_of_v = structure->getGraphNodeFromVertex(unmatched_arc.second);

    // TODO: Some kind of validation checking?

    // Finding the Lowest Common Ancestor of u and v.
    // Getting a set of all the nodes on the path from u to the root.
    set<GraphNode*> u_to_root_path = {};
    GraphNode* current_pos = node_of_u;
    while (current_pos != nullptr) {
        u_to_root_path.insert(current_pos);
        current_pos = current_pos->parent;
    }

    current_pos = node_of_v;
    // While path from v_to_root and path from u_to_root are separated
    while (u_to_root_path.find(current_pos) == u_to_root_path.end()) {
        current_pos = current_pos->parent;
    }

    // current_pos now holds the LCA of u and v.
    GraphNode* lca = current_pos;
    // TODO: need to ensure deletion
    // TODO: need to link children to blossom
    GraphBlossom* new_blossom = new GraphBlossom();
    new_blossom->nodesInBlossom.insert(lca);
    current_pos = node_of_v;
    while (current_pos != lca && current_pos != nullptr) {
        new_blossom->addGraphNodeToBlossom(current_pos);

        for (GraphNode* node : current_pos->children) {
            // If the child node is not in the blossom, we need to add the node as a child of the blossom
            // We also need to update the parent of the child node.
            if (new_blossom->nodesInBlossom.find(node) == new_blossom->nodesInBlossom.end()) {
                new_blossom->children.insert(node);
                node->parent = new_blossom;
            }
        }

        current_pos = current_pos->parent;
    }
    current_pos = node_of_u;
    while (current_pos != lca && current_pos != nullptr) {
        new_blossom->addGraphNodeToBlossom(current_pos);

        for (GraphNode* node : current_pos->children) {
            // If the child node is not in the blossom, we need to add the node as a child of the blossom
            // We also need to update the parent of the child node.
            if (new_blossom->nodesInBlossom.find(node) == new_blossom->nodesInBlossom.end()) {
                new_blossom->children.insert(node);
                node->parent = new_blossom;
            }
        }

        // TODO: need to sort out updating vertex_to_graph_node
        // maybe hold a set in each blossom of the vertices involved?

        current_pos = current_pos->parent;
    }

    new_blossom->parent = lca->parent;
    // If the parent of the LCA is nullptr, then the LCA is the root and we need to update the root node.
    if (lca->parent != nullptr) {
        lca->parent->children.erase(lca);
        lca->parent->children.insert(new_blossom);
    } else {
        structure->free_node_root = new_blossom;
    }

}

void testing() {
    GraphVertex a = GraphVertex(0);
    GraphVertex b = GraphVertex(1);
    GraphVertex c = GraphVertex(2);
    GraphVertex d = GraphVertex(3);
    GraphVertex e = GraphVertex(4);
    GraphVertex f = GraphVertex(5);
    GraphVertex g = GraphVertex(6);
    GraphVertex h = GraphVertex(7);
    GraphVertex i = GraphVertex(8);
    // GraphVertex j = GraphVertex(9);


     FreeNodeStructure* structure = new FreeNodeStructure();

     GraphBlossom blossom = GraphBlossom();
     blossom.nodesInBlossom.insert(&a);
     blossom.nodesInBlossom.insert(&b);
     GraphBlossom blossom2 = GraphBlossom();
     blossom2.nodesInBlossom.insert(&c);
     blossom.nodesInBlossom.insert(&blossom2);
     blossom.children.insert(&d);
     d.parent = &blossom;
     blossom.children.insert(&e);
     e.parent = &blossom;
     e.children.insert(&f);
     f.parent = &e;
     GraphBlossom blossom3 = GraphBlossom();
     blossom3.nodesInBlossom.insert(&g);
     blossom3.nodesInBlossom.insert(&h);
     blossom3.nodesInBlossom.insert(&i);
     d.children.insert(&blossom3);
     blossom3.parent = &d;


     structure->free_node_root = &blossom;
     std::cout << *structure << std::endl;

    structure->addGraphNodeToVertex(0, &blossom);
    structure->addGraphNodeToVertex(1, &blossom);
    structure->addGraphNodeToVertex(2, &blossom);
    structure->addGraphNodeToVertex(3, &d);
    structure->addGraphNodeToVertex(4, &e);
    structure->addGraphNodeToVertex(5, &f);
    structure->addGraphNodeToVertex(6, &blossom3);
    structure->addGraphNodeToVertex(7, &blossom3);
    structure->addGraphNodeToVertex(8, &blossom3);


    contract(make_pair(3,4), structure);

    std::cout << *structure << std::endl;

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
