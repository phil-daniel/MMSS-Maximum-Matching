#ifndef STREAM_H
#define STREAM_H

#include <fstream>

using namespace std;

class Stream {
    public:
        int number_of_passes = 0;

    public:
        virtual ~Stream(void){};
        virtual pair<int, int> readStream() = 0;
};

#endif
