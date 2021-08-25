cmake_minimum_required(VERSION 3.17)
project(BMEA)

set(CMAKE_CXX_STANDARD 11)

#set(Extract Extract.h Extract.cpp)

#python
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

set(Kmeans kmeans.h kmeans_oop.h)
set(Extract extract_cad.h)
set(DXF_HEADER  dxf/dl_attributes.h
                dxf/dl_codes.h
                dxf/dl_creationadapter.h
                dxf/dl_creationinterface.h
                dxf/dl_dxf.h
                dxf/dl_dxf.cpp
                dxf/dl_entities.h
                dxf/dl_exception.h
                dxf/dl_extrusion.h
                dxf/dl_global.h
                dxf/dl_writer.h
                dxf/dl_writer_ascii.cpp
                dxf/dl_writer_ascii.h)
set(DXF_DRIVER dxftest.h dxftest.cpp)

add_subdirectory(util)
add_executable(BMEA ${Kmeans} ${Extract} main.cpp  ${DXF_HEADER} ${DXF_DRIVER})
target_link_libraries(BMEA util)

find_package(OpenCV REQUIRED)
target_link_libraries(BMEA ${OpenCV_LIBS})
target_link_libraries(BMEA ${PYTHON_LIBRARIES})

