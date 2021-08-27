cmake_minimum_required(VERSION 3.17)
project(BMEA)

include_directories("..")

set(STRINGUTIL string_util.h string_util.cpp)
set(MATHUTIL mathop.h mathop.cpp)

add_library(util ${STRINGUTIL} ${MATHUTIL})