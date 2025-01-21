#ifndef STREAMFILE_H
#define STREAMFILE_H

#include "Stream.h"

using namespace std;

class StreamFromFile : public Stream {

    ifstream file{};

    public:
        explicit StreamFromFile(string file_name);
        pair<int, int> readStream() override;
};

#endif
