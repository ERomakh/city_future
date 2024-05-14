cmake_minimum_required(VERSION 3.18)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20 CACHE STRING "C++ version selection")

project(city_load)

set(SOURCE_FL main.cpp io.cpp data_preparation.cpp classifications.cpp gtfs_data.cpp ptmodel.cpp)

add_definitions(-w)

add_executable(city_load ${SOURCE_FL})

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

find_package(GDAL CONFIG REQUIRED)

find_package(igraph REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(mlpack REQUIRED)
target_link_libraries(city_load PRIVATE GDAL::GDAL ${MLPACK_LIBRARY} ${ARMADILLO_LIBRARIES}  PUBLIC igraph::igraph)