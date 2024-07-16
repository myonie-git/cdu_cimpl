cmake_minimum_required(VERSION 3.10)
project(URDF_Parser)

# 设置默认构建类型为Debug
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找URDF库
find_package(urdfdom REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)

# 设置包含目录
include_directories(
  ${urdfdom_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(urdf_parser urdf_parser.cpp)

# 链接库
target_link_libraries(urdf_parser
  ${urdfdom_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  Eigen3::Eigen
)
