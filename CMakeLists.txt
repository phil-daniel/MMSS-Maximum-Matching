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
        Structures/FreeNodeStructure.cpp)
