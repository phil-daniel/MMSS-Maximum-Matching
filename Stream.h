#ifndef STREAM_H
#define STREAM_H

#include <fstream>

using namespace std;

class Stream {

    ifstream file{};

    public:
        pair<int, int> readStream();

        Stream(string file_name);
};



#endif
