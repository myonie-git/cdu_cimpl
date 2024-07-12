cmake_minimum_required(VERSION 3.0)
project(urdf_model)

# 查找Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_BUILD_TYPE "Debug")

# 手动指定TinyXML2的位置
set(TINYXML2_INCLUDE_DIR /usr/include/tinyxml2)
set(TINYXML2_LIBRARIES /usr/lib/x86_64-linux-gnu/libtinyxml2.so)

include_directories(${TINYXML2_INCLUDE_DIR})

add_executable(urdf_model urdf_model.cpp)

target_link_libraries(urdf_model ${TINYXML2_LIBRARIES} Eigen3::Eigen)
