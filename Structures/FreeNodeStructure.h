#ifndef FREENODESTRUCTURE_H
#define FREENODESTRUCTURE_H

class FreeNodeStructure {
    public:
        bool on_hold = false;
        bool modified = false;
        int vertices_count = 0;
        int working_vertex;
};

#endif //FREENODESTRUCTURE_H
