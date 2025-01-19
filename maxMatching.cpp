#include <fstream>
#include <iostream>
#include <set>

using namespace std;

int main() {

    string line;
    ifstream file ("example.txt");

    std::set<string> involved_in_matching = set<string>();
    vector<pair<string,string>> matches;

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
        string v1 = line.substr(0, deliminter_position);
        string v2 = line.substr(deliminter_position + 1, line.length());

        if (
            involved_in_matching.find(v1) == involved_in_matching.end()
            && involved_in_matching.find(v2) == involved_in_matching.end()
        ) {
            involved_in_matching.insert(v1);
            involved_in_matching.insert(v2);
            matches.emplace_back(v1, v2);
        }

    }

    std::cout << "Matching size: " << involved_in_matching.size() << std::endl;

    return 0;
}
