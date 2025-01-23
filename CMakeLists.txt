cmake_minimum_required(VERSION 3.29)
project(MaximumMatchings)

set(CMAKE_CXX_STANDARD 14)

add_executable(MaximumMatchings
        maxMatching.cpp
        Stream/Stream.h
        Stream/StreamFromMemory.h
        Stream/StreamFromMemory.cpp
        Stream/StreamFromFile.h
        Stream/StreamFromFile.cpp
        Structures/FreeNodeStructure.cpp
        Structures/GraphNode/GraphNode.h
        Structures/GraphNode/GraphVertex.h
        Structures/GraphNode/GraphVertex.cpp
        Structures/GraphNode/GraphBlossom.cpp
)

include_directories(~/Programs/cpp_libs/boost_1_87_0/)
