# MMSS-Maximum-Matchings

A implementation of a (1+ε)-approximation algorithm for maximum matchings in general graphs within a semi-streaming setting using O(1/ε^6) passes of the edge stream, based on https://arxiv.org/abs/2412.19057v2.

## Usage Instructions

This implementation can be used by including the `MMSS-Maximum-Matching.h` header file and then creating an edge stream and calling the `getMMSSApproxMaximumMatching()` function, as is shown in the example below.


```cpp
#include "MMSS-Maximum-Matching.h"

int main() {
    // Create a new edge stream, using StreamFromFile or StreamFromMemory depending on emulation requirements.
    //Stream* stream = new StreamFromFile("example.txt");
    Stream* stream = new StreamFromMemory("example.txt");

    // Call the getMMSSApproxMaximumMatching() function with the relevant parameters.
    Matching matching = getMMSSApproxMaximumMatching(stream, 0.25, 3, true, false);
    // A matching is returned, which can be outputted as normal.
    std::cout << matching << std::endl;
    // The number of passes can also be viewed by checking the stream variable.
    std::cout << "Total number of passes: " << stream->number_of_passes << std::endl;

    delete stream;

    return 0;
}
```

We have implemented two basic stream emulations, `StreamFromFile` and `StreamFromMemory`. Both of these streams take input of a `.txt` file, where each line represents an edge, written as two space separated integers, which can then be read by the algorithm.
In `StreamFromFile`, the stream is directly read from the file, however in `StreamFromMemory` the edge stream is initially saved to memory and then read from them to improve performance.
Both of these options only emulate the functionality of an edge stream and in real life usage it is unlikely either will be useful. Instead a new stream class can be declared by extending from `Stream`, which can implement its own `readStream()` functionality as required in its specific use case.

The `Matching getMMSSApproxMaximumMatching()` function takes input of the following parameters:
- `Stream *stream` - A pointer to an edge stream object. Either `StreamFromFile`, `StreamFromMemory` or a custom stream extending `Stream` with its own `readStream()` function defined.
- `float epsilon` -  The approximation factor for the algorithm.
- (OPTIONAL) `int progress_report` - Sets how much output is provided during the algorithm's execution. Takes the following integers as input and defaults to `3`.
  - `0` - No progress output
  - `1` - Outputs when a scale change has occurred.
  - `2` - Outputs when a scale change or phase change has occurred.
  - `3` - Outputs when a scale change, phase change or pass-bundle change has occurred.
  - `4` - Verbose, outputs everything above and also outputs when any free-vertex structure operations occur.
- (OPTIONAL) `bool optimisations` - Enables or disables the skip optimisations, which reduce the number of passes by skipping excessive computation. By default, this set to true.
- (OPTIONAL) `bool early_finish` - Enables or disables the early termination check which ends the algorithm once the approximation factor has been guaranteed to be met. By default, this is set to false.


