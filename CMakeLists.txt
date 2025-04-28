cmake_minimum_required(VERSION 3.29)
project(MaximumMatchings)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

set(CMAKE_CXX_STANDARD 14)

add_library(MaximumMatchingsLib
        MMSS-Maximum-Matching.cpp
        Stream/StreamFromMemory.cpp
        Stream/StreamFromFile.cpp
        Structures/FreeNodeStructure.cpp
        Structures/GraphStructure/GraphVertex.cpp
        Structures/GraphStructure/GraphBlossom.cpp
        Structures/AvailableFreeNodes.cpp
        Structures/Matching.cpp
)

add_executable(MaximumMatchings
        main.cpp
)

target_link_libraries(MaximumMatchings PRIVATE MaximumMatchingsLib)

include_directories(~/Programs/cpp_libs/boost_1_87_0/)
