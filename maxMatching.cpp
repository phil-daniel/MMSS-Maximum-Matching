#include <fstream>
#include <iostream>
#include <set>

#include "Stream/Stream.h"
#include "Stream/StreamFromFile.h"
#include "Stream/StreamFromMemory.h"

using namespace std;

int main() {

    Stream* stream = new StreamFromFile("example.txt");
    //Stream* stream = new StreamFromMemory("example.txt");

    std::set<int> involved_in_matching = set<int>();
    vector<pair<int,int>> matches;

    pair<int,int> edge = stream->readStream();
    // edges are only -1 if we have reached the end of the stream.
    while (edge.first != -1) {

        // 2-approximation within 1 pass, only adds to the matching if both vertices are currently not in the matching.
        if (
            involved_in_matching.find(edge.first) == involved_in_matching.end()
            && involved_in_matching.find(edge.second) == involved_in_matching.end()
        ) {
            involved_in_matching.insert(edge.first);
            involved_in_matching.insert(edge.second);
            matches.emplace_back(edge);
        }

        edge = stream->readStream();
    }

    std::cout << "Matching size: " << involved_in_matching.size() << std::endl;

    return 0;
}
