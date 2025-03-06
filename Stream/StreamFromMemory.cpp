#include "StreamFromMemory.h"

StreamFromMemory::StreamFromMemory(string file_name) {
    number_of_passes = 0;

    ifstream file = ifstream(file_name);

    string line;

    while(getline(file, line)) {
        // Skipping unimportant lines
        if (line.empty()) {
            continue;
        }
        if (line[0] == '#') {
            continue;
        }

        // Splitting string by the " "
        int deliminter_position = line.find_first_of(" ");
        string v1_text = line.substr(0, deliminter_position);
        string v2_text = line.substr(deliminter_position + 1, line.length());

        int v1 = stoi(v1_text);
        int v2 = stoi(v2_text);

        lines.push_back(make_pair(v1, v2));
        // TODO: testing both arcs?
        lines.push_back(make_pair(v2, v1));
    }
}

pair<int, int> StreamFromMemory::readStream() {

    if (line_number >= lines.size()) {
        number_of_passes += 1;
        line_number = 0;
        return make_pair(-1, -1);
    }

    pair<int, int> edge = lines.at(line_number);
    line_number++;
    return edge;
}
