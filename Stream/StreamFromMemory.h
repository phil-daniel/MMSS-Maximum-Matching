#ifndef STREAMFROMMEMORY_H
#define STREAMFROMMEMORY_H
#include "Stream.h"

class StreamFromMemory : public Stream {
    int line_number = 0;
    vector<pair<int, int>> lines = {};

    public:
        explicit StreamFromMemory(string file_name);
        pair<int, int> readStream() override;

};

#endif //STREAMFROMMEMORY_H
