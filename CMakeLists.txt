cmake_minimum_required(VERSION 3.17)
project(BMEA)

set(CMAKE_CXX_STANDARD 11)
set(Kmeans kmeans.h kmeans_oop.h)
set(Extract extract_cad.h)
#set(Extract Extract.h Extract.cpp)

add_subdirectory(util)

add_executable(BMEA ${Kmeans} ${Extract} main.cpp)
target_link_libraries(BMEA util)