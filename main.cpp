#include "MMSS-Maximum-Matching.h"

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