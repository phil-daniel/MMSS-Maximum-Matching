#ifndef MMSS_MAXIMUM_MATCHING_H
#define MMSS_MAXIMUM_MATCHING_H

#pragma once

#include <fstream>
#include <iostream>
#include <set>

#include "Structures/types.h"

#include "Stream/Stream.h"
#include "Stream/StreamFromFile.h"
#include "Stream/StreamFromMemory.h"
#include "Structures/AvailableFreeNodes.h"

#include "Structures/FreeNodeStructure.h"
#include "Structures/GraphStructure/GraphBlossom.h"
#include "Structures/GraphStructure/GraphVertex.h"
#include "Structures/Matching.h"

using namespace std;

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

Matching get2ApproximateMatching(Stream* stream);

Matching getMMSSApproxMaximumMatching(Stream* stream, float epsilon, int progress_report = 3, bool optimisations = true,
                                      bool early_finish = false);

#endif //MMSS_MAXIMUM_MATCHING_H
