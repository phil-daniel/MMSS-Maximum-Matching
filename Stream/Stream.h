#ifndef STREAM_H
#define STREAM_H

#include <fstream>

using namespace std;

class Stream {
    public:
        int number_of_passes = 0;

    // Virtual function, any functions wanting to emulate a stream needs to implement its own readStream() function
    // which passes a single edge at a time when called.
    public:
        virtual ~Stream(void){};
        virtual pair<int, int> readStream() = 0;
};

#endif
