#ifndef STREAMFILE_H
#define STREAMFILE_H

#include "Stream.h"

using namespace std;

class StreamFromFile : public Stream {
    private:
        pair<int,int> last_edge = make_pair(-1,-1);
        bool show_last_edge = false;
        ifstream file{};

    // StreamFromFile provides the edge stream directly from the input text file.
    public:
        explicit StreamFromFile(string file_name);
        pair<int, int> readStream() override;
};

#endif
