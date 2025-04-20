cmake_minimum_required(VERSION 3.29)
project(MaximumMatchings)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

set(CMAKE_CXX_STANDARD 14)

add_executable(MaximumMatchings
        MMSS-Maximum-Matching.cpp
        Stream/Stream.h
        Stream/StreamFromMemory.h
        Stream/StreamFromMemory.cpp
        Stream/StreamFromFile.h
        Stream/StreamFromFile.cpp
        Structures/FreeNodeStructure.cpp
        Structures/GraphStructure/GraphNode.h
        Structures/GraphStructure/GraphVertex.h
        Structures/GraphStructure/GraphVertex.cpp
        Structures/GraphStructure/GraphBlossom.cpp
        types.h
        Structures/AvailableFreeNodes.cpp
        Structures/AvailableFreeNodes.h
        Structures/Matching.cpp
        Structures/Matching.h
)

include_directories(~/Programs/cpp_libs/boost_1_87_0/)
