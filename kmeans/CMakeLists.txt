cmake_minimum_required(VERSION 3.1)

project(pc_kmeans)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})


add_executable (kmeans kmeans.cpp)
add_executable (center_normal center_normal.cpp)
add_executable (tsp prepare_tsp.cpp)
add_executable (knn knn_4_normal.cpp)
add_executable (cec condition_ec.cpp)
target_link_libraries (kmeans ${PCL_LIBRARIES})
target_link_libraries (center_normal ${PCL_LIBRARIES})
target_link_libraries (knn ${PCL_LIBRARIES})
target_link_libraries (cec ${PCL_LIBRARIES})