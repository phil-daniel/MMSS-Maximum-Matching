#include "StreamFromFile.h"

#include <string>

using namespace std;

StreamFromFile::StreamFromFile(string file_name) {
    file = ifstream(file_name);
    number_of_passes = 0;
    last_edge = make_pair(-1,-1);
    show_last_edge = false;
}

pair<int, int> StreamFromFile::readStream() {
    string line;

    // Returning the second arc of the edge.
    if (show_last_edge) {
        show_last_edge = false;
        return last_edge;
    }

    // Occurs when we are at the end of the stream.
    if (! getline(file, line)) {

        // Returning to the beginning of the file in preparation for the next readStream() call
        file.clear();
        file.seekg(0, ios::beg);

        number_of_passes += 1;

        // -1 will represent the fact that we have reached the end of the stream.
        return make_pair(-1, -1);
    }

    // If the line we have just read is empty, we recursively read the next line.
    if (line.empty() || line[0] == '#') {
        return readStream();
    }

    // Splitting string by the " "
    int deliminter_position = line.find_first_of(" ");
    string v1_text = line.substr(0, deliminter_position);
    string v2_text = line.substr(deliminter_position + 1, line.length());

    int v1 = stoi(v1_text);
    int v2 = stoi(v2_text);

    last_edge = make_pair(v2, v1);
    show_last_edge = true;

    return make_pair(v1, v2);
}
