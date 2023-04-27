cmake_minimum_required(VERSION 3.23)
project(LINERLIB)

set(CMAKE_CXX_STANDARD 14)
find_package(Boost 1.57.0 COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(include)
include_directories(src)

add_executable(LINERLIB
        include/bm_data.hpp
        include/BM_Graph.hpp
        include/bm_output.h
        include/bm_reader.hpp
        include/bm_rotation.h
        include/cl_parameters.hpp
        src/bm_data.cpp
        src/bm_reader.cpp
        src/bm_rotation.cpp
        src/cl_parameters.cpp
        src/main_example.cpp)
target_link_libraries(LINERLIB ${Boost_LIBRARIES})
