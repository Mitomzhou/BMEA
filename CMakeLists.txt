cmake_minimum_required(VERSION 3.17)
project(BMEA)

set(CMAKE_CXX_STANDARD 11)
set(Kmeans kmeans.h kmeans_oop.h)
set(Extract extract_cad.h)
#set(Extract Extract.h Extract.cpp)

#python
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

add_subdirectory(util)

add_executable(BMEA ${Kmeans} ${Extract} main.cpp)
target_link_libraries(BMEA util)

find_package(OpenCV REQUIRED)
target_link_libraries(BMEA ${OpenCV_LIBS})
target_link_libraries(BMEA ${PYTHON_LIBRARIES})

